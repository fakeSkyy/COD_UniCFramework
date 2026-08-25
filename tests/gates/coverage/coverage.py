#!/usr/bin/env python3
"""Collect and gate merged GCC gcov coverage for repository production C sources."""

from __future__ import annotations

import argparse
import gzip
import html
import json
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from decimal import Decimal, InvalidOperation
from pathlib import Path
from typing import Iterable

LAYERS = ("01_application", "02_device", "03_platform", "04_impl", "06_utils")


@dataclass
class FileCoverage:
    """Merged counters for every instrumented copy of one source file."""

    lines: dict[int, int] = field(default_factory=dict)
    branches: dict[tuple[int, int], int] = field(default_factory=dict)
    functions: dict[tuple[str, int, int, int, int], int] = field(default_factory=dict)
    objects: set[str] = field(default_factory=set)


@dataclass(frozen=True)
class Metric:
    """Covered and total item counts for one metric."""

    covered: int
    total: int

    @property
    def percent(self) -> float:
        return 100.0 if self.total == 0 else self.covered * 100.0 / self.total


def parse_threshold(value: str) -> Decimal:
    """Parse one inclusive percentage threshold."""
    try:
        threshold = Decimal(value)
    except InvalidOperation as error:
        raise argparse.ArgumentTypeError(f"invalid percentage: {value}") from error
    if not threshold.is_finite() or threshold < 0 or threshold > 100:
        raise argparse.ArgumentTypeError(f"percentage must be between 0 and 100: {value}")
    return threshold


def arguments() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True, help="repository root")
    parser.add_argument("--build-dir", type=Path, required=True, help="instrumented CMake build")
    parser.add_argument("--output-dir", type=Path, required=True, help="report destination")
    parser.add_argument("--gcov", default="gcov", help="gcov executable")
    parser.add_argument("--min-lines", type=parse_threshold, default=Decimal("0"))
    parser.add_argument("--min-functions", type=parse_threshold, default=Decimal("0"))
    parser.add_argument("--min-branches", type=parse_threshold, default=Decimal("0"))
    return parser.parse_args()


def expected_sources(root: Path) -> list[Path]:
    """Return the complete owned production-source inventory."""
    sources: list[Path] = []
    for layer in LAYERS:
        layer_dir = root / layer
        if not layer_dir.is_dir():
            raise RuntimeError(f"production layer does not exist: {layer_dir}")
        sources.extend(path.resolve() for path in layer_dir.rglob("*.c"))
    return sorted(sources)


def resolve_source(name: str, working_directory: str, root: Path) -> Path:
    """Resolve paths emitted by gcov, including relative compiler paths."""
    source = Path(name)
    if source.is_absolute():
        return source.resolve()
    candidates = (Path(working_directory) / source, root / source)
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved.exists():
            return resolved
    return candidates[0].resolve()


def merge_record(coverage: FileCoverage, record: dict, object_path: Path) -> None:
    """Merge one gcov file record into stable source-level identities."""
    coverage.objects.add(str(object_path))
    for line in record.get("lines", []):
        line_number = int(line["line_number"])
        coverage.lines[line_number] = coverage.lines.get(line_number, 0) + int(line["count"])
        for index, branch in enumerate(line.get("branches", [])):
            key = (line_number, index)
            coverage.branches[key] = coverage.branches.get(key, 0) + int(branch["count"])

    for function in record.get("functions", []):
        key = (
            str(function["name"]),
            int(function["start_line"]),
            int(function.get("start_column", 0)),
            int(function["end_line"]),
            int(function.get("end_column", 0)),
        )
        coverage.functions[key] = coverage.functions.get(key, 0) + int(
            function["execution_count"]
        )


def collect(root: Path, build_dir: Path, gcov: str, sources: list[Path]) -> dict[Path, FileCoverage]:
    """Run gcov for every data file and merge records belonging to owned sources."""
    data_files = sorted(build_dir.rglob("*.gcda"))
    if not data_files:
        raise RuntimeError(f"no .gcda files found under {build_dir}; run the tests first")

    expected = set(sources)
    result = {source: FileCoverage() for source in sources}
    with tempfile.TemporaryDirectory(prefix="unicframework-gcov-") as temporary:
        temporary_dir = Path(temporary)
        for data_file in data_files:
            completed = subprocess.run(
                [gcov, "--json-format", "--branch-probabilities", str(data_file)],
                cwd=temporary_dir,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            if completed.returncode != 0:
                detail = (completed.stderr or completed.stdout).strip()
                raise RuntimeError(f"gcov failed for {data_file}: {detail}")

            json_files = sorted(temporary_dir.glob("*.gcov.json.gz"))
            if not json_files:
                raise RuntimeError(f"gcov produced no JSON for {data_file}")
            for json_file in json_files:
                with gzip.open(json_file, "rt", encoding="utf-8") as stream:
                    document = json.load(stream)
                working_directory = document.get("current_working_directory", str(root))
                for record in document.get("files", []):
                    source = resolve_source(record["file"], working_directory, root)
                    if source in expected:
                        merge_record(result[source], record, data_file)
                json_file.unlink()
    return result


def metric(counters: Iterable[int]) -> Metric:
    """Summarize positive counters as covered items."""
    values = list(counters)
    return Metric(sum(value > 0 for value in values), len(values))


def file_metrics(coverage: FileCoverage) -> dict[str, Metric]:
    """Calculate all metrics for one source file."""
    return {
        "lines": metric(coverage.lines.values()),
        "functions": metric(coverage.functions.values()),
        "branches": metric(coverage.branches.values()),
    }


def aggregate(files: Iterable[FileCoverage]) -> dict[str, Metric]:
    """Aggregate metrics without losing per-item denominators."""
    per_file = [file_metrics(coverage) for coverage in files]
    return {
        name: Metric(
            sum(metrics[name].covered for metrics in per_file),
            sum(metrics[name].total for metrics in per_file),
        )
        for name in ("lines", "functions", "branches")
    }


def compact_ranges(numbers: Iterable[int]) -> str:
    """Render sorted line numbers as compact inclusive ranges."""
    values = sorted(set(numbers))
    if not values:
        return "-"
    ranges: list[str] = []
    start = previous = values[0]
    for value in values[1:]:
        if value == previous + 1:
            previous = value
            continue
        ranges.append(str(start) if start == previous else f"{start}-{previous}")
        start = previous = value
    ranges.append(str(start) if start == previous else f"{start}-{previous}")
    return ",".join(ranges)


def metric_dict(value: Metric) -> dict[str, int | float]:
    """Convert a metric to JSON-safe values."""
    return {"covered": value.covered, "total": value.total, "percent": round(value.percent, 2)}


def threshold_met(value: Metric, threshold: Decimal) -> bool:
    """Compare without rounded percentages."""
    if value.total == 0:
        return True
    return Decimal(value.covered * 100) >= threshold * Decimal(value.total)


def relative(source: Path, root: Path) -> str:
    """Return one stable POSIX repository-relative path."""
    return source.relative_to(root).as_posix()


def build_summary(
    root: Path,
    sources: list[Path],
    coverage: dict[Path, FileCoverage],
    thresholds: dict[str, Decimal],
) -> tuple[dict, list[str]]:
    """Build the machine-readable summary and gate failures."""
    totals = aggregate(coverage.values())
    missing = [source for source in sources if not coverage[source].objects]
    failures = [f"missing coverage data: {relative(source, root)}" for source in missing]
    for name, threshold in thresholds.items():
        if not threshold_met(totals[name], threshold):
            failures.append(
                f"{name} coverage {totals[name].percent:.2f}% is below {threshold}%"
            )

    file_records = []
    for source in sources:
        values = file_metrics(coverage[source])
        file_records.append(
            {
                "path": relative(source, root),
                "observed_objects": len(coverage[source].objects),
                "metrics": {name: metric_dict(value) for name, value in values.items()},
                "missed_lines": [
                    line for line, count in sorted(coverage[source].lines.items()) if count == 0
                ],
            }
        )

    layers = {}
    for layer in LAYERS:
        layer_files = [
            coverage[source]
            for source in sources
            if relative(source, root).split("/", 1)[0] == layer
        ]
        values = aggregate(layer_files)
        layers[layer] = {name: metric_dict(value) for name, value in values.items()}

    summary = {
        "scope": list(LAYERS),
        "source_files": len(sources),
        "observed_source_files": len(sources) - len(missing),
        "thresholds": {name: float(value) for name, value in thresholds.items()},
        "totals": {name: metric_dict(value) for name, value in totals.items()},
        "layers": layers,
        "files": file_records,
        "passed": not failures,
        "failures": failures,
    }
    return summary, failures


def text_report(summary: dict) -> str:
    """Render a concise human-readable report."""
    lines = [
        "UniCFramework host production coverage",
        "=======================================",
        f"sources: {summary['observed_source_files']}/{summary['source_files']}",
        "",
        "scope                         lines          functions       branches",
    ]

    def row(name: str, values: dict) -> str:
        cells = []
        for metric_name in ("lines", "functions", "branches"):
            value = values[metric_name]
            cells.append(f"{value['percent']:6.2f}% {value['covered']:4d}/{value['total']:<4d}")
        return f"{name:<28} " + "  ".join(cells)

    lines.append(row("TOTAL", summary["totals"]))
    for layer in LAYERS:
        lines.append(row(layer, summary["layers"][layer]))

    lines.extend(("", "Per-file coverage and missed executable lines:"))
    for record in summary["files"]:
        values = record["metrics"]
        lines.append(
            f"{record['path']}: "
            f"L {values['lines']['percent']:.2f}% "
            f"F {values['functions']['percent']:.2f}% "
            f"B {values['branches']['percent']:.2f}% "
            f"missed={compact_ranges(record['missed_lines'])}"
        )

    lines.append("")
    if summary["passed"]:
        lines.append("COVERAGE GATE: PASS")
    else:
        lines.append("COVERAGE GATE: FAIL")
        lines.extend(f"- {failure}" for failure in summary["failures"])
    return "\n".join(lines) + "\n"


def html_metric(value: dict) -> str:
    """Render one metric table cell."""
    return f"{value['percent']:.2f}%<small>{value['covered']}/{value['total']}</small>"


def write_source_page(
    html_dir: Path, root: Path, source: Path, coverage: FileCoverage, output_name: str
) -> None:
    """Write line-annotated HTML for one source file."""
    branch_totals: dict[int, tuple[int, int]] = {}
    for (line_number, _), count in coverage.branches.items():
        covered, total = branch_totals.get(line_number, (0, 0))
        branch_totals[line_number] = (covered + int(count > 0), total + 1)

    rows = []
    source_lines = source.read_text(encoding="utf-8", errors="replace").splitlines()
    for line_number, content in enumerate(source_lines, 1):
        count = coverage.lines.get(line_number)
        css_class = "neutral" if count is None else ("covered" if count > 0 else "missed")
        count_text = "" if count is None else str(count)
        branch = branch_totals.get(line_number)
        branch_text = "" if branch is None else f"{branch[0]}/{branch[1]}"
        rows.append(
            f'<tr class="{css_class}"><td>{line_number}</td><td>{count_text}</td>'
            f'<td>{branch_text}</td><td><code>{html.escape(content)}</code></td></tr>'
        )

    values = file_metrics(coverage)
    title = relative(source, root)
    document = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>{html.escape(title)}</title>
<style>
body{{font:14px system-ui,sans-serif;margin:1.5rem;color:#202124}}a{{color:#1769aa}}
table{{border-collapse:collapse;width:100%}}td{{padding:1px 8px;vertical-align:top}}
td:nth-child(-n+3){{text-align:right;color:#5f6368;width:1%;white-space:nowrap}}
code{{white-space:pre;font:13px ui-monospace,monospace}}.covered{{background:#e6f4ea}}
.missed{{background:#fce8e6}}.neutral{{background:#fff}}.summary{{font-weight:600}}
</style></head><body><p><a href="index.html">← Summary</a></p>
<h1>{html.escape(title)}</h1>
<p class="summary">Lines {values['lines'].percent:.2f}% · Functions {values['functions'].percent:.2f}% · Branches {values['branches'].percent:.2f}%</p>
<table><thead><tr><th>Line</th><th>Count</th><th>Branches</th><th>Source</th></tr></thead>
<tbody>{''.join(rows)}</tbody></table></body></html>
"""
    (html_dir / output_name).write_text(document, encoding="utf-8")


def write_html(root: Path, output_dir: Path, sources: list[Path], coverage: dict, summary: dict) -> None:
    """Write the HTML index and all annotated source pages."""
    html_dir = output_dir / "html"
    html_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for index, (source, record) in enumerate(zip(sources, summary["files"])):
        output_name = f"source-{index:03d}.html"
        write_source_page(html_dir, root, source, coverage[source], output_name)
        values = record["metrics"]
        rows.append(
            f'<tr><td><a href="{output_name}">{html.escape(record["path"])}</a></td>'
            f'<td>{html_metric(values["lines"])}</td>'
            f'<td>{html_metric(values["functions"])}</td>'
            f'<td>{html_metric(values["branches"])}</td></tr>'
        )

    status = "PASS" if summary["passed"] else "FAIL"
    status_class = "pass" if summary["passed"] else "fail"
    totals = summary["totals"]
    failures = "".join(f"<li>{html.escape(item)}</li>" for item in summary["failures"])
    document = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>UniCFramework coverage</title>
<style>
body{{font:14px system-ui,sans-serif;margin:2rem;color:#202124}}table{{border-collapse:collapse;width:100%}}
th,td{{border-bottom:1px solid #ddd;padding:6px 8px;text-align:right}}th:first-child,td:first-child{{text-align:left}}
small{{display:block;color:#5f6368}}a{{color:#1769aa}}.pass{{color:#137333}}.fail{{color:#c5221f}}
.cards{{display:flex;gap:2rem;font-size:1.2rem}}.cards strong{{font-size:1.6rem}}
</style></head><body><h1>Host production coverage</h1>
<h2 class="{status_class}">Coverage gate: {status}</h2><ul>{failures}</ul>
<p>Scope: {html.escape(', '.join(summary['scope']))}; sources: {summary['observed_source_files']}/{summary['source_files']}.</p>
<div class="cards"><div>Lines<br><strong>{totals['lines']['percent']:.2f}%</strong></div>
<div>Functions<br><strong>{totals['functions']['percent']:.2f}%</strong></div>
<div>Branches<br><strong>{totals['branches']['percent']:.2f}%</strong></div></div>
<h2>Files</h2><table><thead><tr><th>Source</th><th>Lines</th><th>Functions</th><th>Branches</th></tr></thead>
<tbody>{''.join(rows)}</tbody></table></body></html>
"""
    (html_dir / "index.html").write_text(document, encoding="utf-8")


def prepare_output(root: Path, build_dir: Path, requested: Path) -> Path:
    """Validate the report path and invalidate any previous result safely."""
    cache_file = build_dir / "CMakeCache.txt"
    expected_home = f"CMAKE_HOME_DIRECTORY:INTERNAL={(root / 'tests').resolve()}"
    if not cache_file.is_file() or expected_home not in cache_file.read_text(
        encoding="utf-8", errors="replace"
    ).splitlines():
        raise RuntimeError(f"not a configured UniCFramework host-test build: {build_dir}")
    if requested.is_symlink():
        raise RuntimeError(f"output directory must not be a symbolic link: {requested}")
    output_dir = requested.resolve()
    protected_output = (root / "tests" / "coverage").resolve()
    if (
        output_dir.parent != build_dir
        or not output_dir.name.startswith("coverage")
        or output_dir == protected_output
    ):
        raise RuntimeError(
            "output directory must be a safe direct build-directory child named coverage*"
        )
    if output_dir.exists():
        if output_dir.is_dir():
            shutil.rmtree(output_dir)
        else:
            output_dir.unlink()
    output_dir.mkdir()
    (output_dir / "INCOMPLETE.txt").write_text(
        "Coverage collection did not complete; no prior PASS report is valid.\n",
        encoding="utf-8",
    )
    return output_dir


def publish_reports(
    root: Path,
    build_dir: Path,
    output_dir: Path,
    sources: list[Path],
    coverage: dict[Path, FileCoverage],
    summary: dict,
) -> str:
    """Build reports in staging and replace the incomplete marker only on success."""
    staging = Path(tempfile.mkdtemp(prefix=".coverage-report-", dir=build_dir))
    try:
        report = text_report(summary)
        (staging / "coverage.txt").write_text(report, encoding="utf-8")
        (staging / "coverage-summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        write_html(root, staging, sources, coverage, summary)
        shutil.rmtree(output_dir)
        staging.replace(output_dir)
        return report
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def main() -> int:
    """Collect counters, write reports, and enforce thresholds."""
    options = arguments()
    root = options.root.resolve()
    build_dir = options.build_dir.resolve()
    output_dir = prepare_output(root, build_dir, options.output_dir)
    sources = expected_sources(root)
    coverage = collect(root, build_dir, options.gcov, sources)
    thresholds = {
        "lines": options.min_lines,
        "functions": options.min_functions,
        "branches": options.min_branches,
    }
    summary, failures = build_summary(root, sources, coverage, thresholds)
    report = publish_reports(root, build_dir, output_dir, sources, coverage, summary)
    print(report, end="")
    print(f"HTML report: {output_dir / 'html/index.html'}")
    return 1 if failures else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, subprocess.SubprocessError, json.JSONDecodeError) as error:
        print(f"coverage: error: {error}", file=sys.stderr)
        raise SystemExit(2) from error
