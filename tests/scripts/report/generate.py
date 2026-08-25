#!/usr/bin/env python3
"""Generate deterministic current host-verification reports from machine data."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import pathlib
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from typing import Any

SCHEMA_VERSION = 1
STATUSES = {"PASS", "FAIL", "INCOMPLETE", "not_run"}
ROOT = pathlib.Path(__file__).resolve().parents[3]


class ReportError(RuntimeError):
    """Report input or output contract violation."""


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=pathlib.Path)
    parser.add_argument("--output-dir", type=pathlib.Path)
    parser.add_argument("--ctest-json", type=pathlib.Path)
    parser.add_argument("--junit", type=pathlib.Path)
    parser.add_argument("--ctest-exit-status", type=int)
    parser.add_argument("--cache", type=pathlib.Path)
    parser.add_argument("--target-help", type=pathlib.Path)
    parser.add_argument("--quality", type=pathlib.Path)
    parser.add_argument("--coverage", type=pathlib.Path)
    parser.add_argument("--performance", type=pathlib.Path)
    parser.add_argument("--sanitizer-status", type=pathlib.Path, action="append", default=[])
    parser.add_argument("--fuzz-status", type=pathlib.Path)
    parser.add_argument("--cmock-json", type=pathlib.Path, action="append", default=[])
    return parser.parse_args()


def is_within(path: pathlib.Path, parent: pathlib.Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def discover_build(requested: pathlib.Path | None) -> pathlib.Path:
    if requested is not None:
        build = requested.expanduser().resolve()
    else:
        build = None
        for candidate in (pathlib.Path.cwd(), *pathlib.Path.cwd().parents):
            if (candidate / "CMakeCache.txt").is_file():
                build = candidate.resolve()
                break
        if build is None:
            raise ReportError("could not discover a configured build; pass --build-dir")
    if not build.is_dir() or not (build / "CMakeCache.txt").is_file():
        raise ReportError(f"not a configured CMake build directory: {build}")
    return build


def safe_output(build: pathlib.Path, requested: pathlib.Path | None) -> pathlib.Path:
    output = (requested or (build / "reports")).expanduser()
    if output.is_symlink():
        raise ReportError(f"output directory must not be a symbolic link: {output}")
    output = output.resolve()
    if output != (build / "reports").resolve():
        raise ReportError("output directory must be the build directory's direct reports child")
    if is_within(output, ROOT):
        raise ReportError(f"refusing to write a report in the source tree: {output}")
    output.mkdir(parents=False, exist_ok=True)
    return output


def explicit_or_auto(
    requested: pathlib.Path | None, candidates: list[pathlib.Path]
) -> pathlib.Path | None:
    if requested is not None:
        path = requested.expanduser().resolve()
        if not path.is_file():
            raise ReportError(f"explicit input does not exist: {path}")
        return path
    return next((path for path in candidates if path.is_file()), None)


def load_json(path: pathlib.Path, description: str) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ReportError(f"malformed {description} input {path}: {error}") from error


def run_json(command: list[str], description: str) -> dict[str, Any]:
    process = subprocess.run(command, text=True, capture_output=True, check=False)
    if process.returncode != 0:
        detail = process.stderr.strip() or process.stdout.strip()
        raise ReportError(f"{description} failed with status {process.returncode}: {detail}")
    try:
        value = json.loads(process.stdout)
    except json.JSONDecodeError as error:
        raise ReportError(f"{description} returned malformed JSON: {error}") from error
    if not isinstance(value, dict):
        raise ReportError(f"{description} did not return a JSON object")
    return value


def load_ctest_metadata(build: pathlib.Path, path: pathlib.Path | None) -> dict[str, Any]:
    value = (
        load_json(path, "CTest JSON-v1")
        if path is not None
        else run_json(
            ["ctest", "--test-dir", str(build), "--show-only=json-v1"],
            "ctest --show-only=json-v1",
        )
    )
    if not isinstance(value, dict) or value.get("kind") != "ctestInfo":
        raise ReportError("CTest JSON-v1 must be a ctestInfo object")
    version = value.get("version")
    tests = value.get("tests")
    if not isinstance(version, dict) or version.get("major") != 1 or not isinstance(tests, list):
        raise ReportError("unsupported or malformed CTest JSON-v1 schema")
    names: set[str] = set()
    for test in tests:
        if not isinstance(test, dict) or not isinstance(test.get("name"), str):
            raise ReportError("CTest JSON-v1 contains a test without a string name")
        if test["name"] in names:
            raise ReportError(f"CTest JSON-v1 contains duplicate test {test['name']!r}")
        names.add(test["name"])
    return value


def labels_for(test: dict[str, Any]) -> list[str]:
    for prop in test.get("properties", []):
        if isinstance(prop, dict) and prop.get("name") == "LABELS":
            value = prop.get("value", [])
            if isinstance(value, list) and all(isinstance(item, str) for item in value):
                return sorted(set(value))
            raise ReportError(f"test {test['name']!r} has malformed LABELS")
    return []


def parse_junit(path: pathlib.Path | None) -> dict[str, str] | None:
    if path is None:
        return None
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as error:
        raise ReportError(f"malformed CTest JUnit XML {path}: {error}") from error
    cases = root.findall(".//testcase") if root.tag != "testcase" else [root]
    results: dict[str, str] = {}
    for case in cases:
        name = case.get("name")
        if not name or name in results:
            raise ReportError("CTest JUnit contains a missing or duplicate testcase name")
        if case.find("failure") is not None or case.find("error") is not None:
            result = "fail"
        elif case.find("skipped") is not None or case.get("status") in {"notrun", "disabled"}:
            result = "skip"
        else:
            result = "pass"
        results[name] = result
    return results


def aggregate_ctest(metadata: dict[str, Any], junit: dict[str, str] | None) -> dict[str, Any]:
    tests = metadata["tests"]
    label_tests: dict[str, list[str]] = {}
    for test in tests:
        for label in labels_for(test):
            label_tests.setdefault(label, []).append(test["name"])

    if junit is None:
        gate: dict[str, Any] = {
            "status": "not_run",
            "total": len(tests),
            "pass": 0,
            "fail": 0,
            "skip": 0,
            "incomplete": len(tests),
            "labels": {},
        }
    else:
        known = {test["name"] for test in tests}
        extra = sorted(set(junit) - known)
        missing = sorted(known - set(junit))
        counts = {name: sum(value == name for value in junit.values()) for name in ("pass", "fail", "skip")}
        status = "FAIL" if counts["fail"] else ("INCOMPLETE" if missing or extra else "PASS")
        gate = {
            "status": status,
            "total": len(tests),
            **counts,
            "incomplete": len(missing),
            "labels": {},
        }
        if missing:
            gate["missing_tests"] = missing
        if extra:
            gate["unexpected_tests"] = extra

    for label in sorted(label_tests):
        names = label_tests[label]
        values = {} if junit is None else {name: junit[name] for name in names if name in junit}
        gate["labels"][label] = {
            "total": len(names),
            "pass": sum(value == "pass" for value in values.values()),
            "fail": sum(value == "fail" for value in values.values()),
            "skip": sum(value == "skip" for value in values.values()),
            "incomplete": len(names) - len(values),
        }
    return gate


def parse_cache(path: pathlib.Path) -> dict[str, str]:
    values: dict[str, str] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as error:
        raise ReportError(f"cannot read CMakeCache {path}: {error}") from error
    for line in lines:
        match = re.match(r"([^/#][^:]*):[^=]+=(.*)$", line)
        if match:
            values[match.group(1)] = match.group(2)
    required = ("CMAKE_BUILD_TYPE", "CMAKE_C_COMPILER", "CMAKE_GENERATOR")
    missing = [name for name in required if not values.get(name)]
    if missing:
        raise ReportError(f"CMakeCache is missing required keys: {', '.join(missing)}")
    return values


def target_count(build: pathlib.Path, path: pathlib.Path | None) -> int:
    if path is None:
        process = subprocess.run(
            ["cmake", "--build", str(build), "--target", "help"],
            text=True,
            capture_output=True,
            check=False,
        )
        if process.returncode != 0:
            raise ReportError(f"CMake target help failed with status {process.returncode}")
        text = process.stdout
    else:
        try:
            text = path.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as error:
            raise ReportError(f"cannot read target help {path}: {error}") from error
    targets = []
    for line in text.splitlines():
        if line.startswith("... "):
            target = line[4:].split(" ", 1)[0]
            if target != "all":
                targets.append(target)
    if not targets or len(targets) != len(set(targets)):
        raise ReportError("target help contains no targets or duplicate targets")
    return len(targets)


def gate_json(path: pathlib.Path | None, kind: str) -> dict[str, Any]:
    if path is None:
        return {"status": "not_run"}
    value = load_json(path, kind)
    if not isinstance(value, dict):
        raise ReportError(f"{kind} report must be a JSON object")
    if kind == "quality":
        raw = value.get("overall_status")
        if raw not in {"PASS", "FAIL"} or not isinstance(value.get("checks"), list):
            raise ReportError("quality report has an unsupported schema")
        return {"status": raw, "checks": len(value["checks"]), "schema_version": value.get("schema_version")}
    if kind == "coverage":
        if not isinstance(value.get("passed"), bool) or not isinstance(value.get("totals"), dict):
            raise ReportError("coverage report has an unsupported schema")
        return {
            "status": "PASS" if value["passed"] else "FAIL",
            "totals": value["totals"],
            "source_files": value.get("source_files"),
            "observed_source_files": value.get("observed_source_files"),
        }
    if kind == "performance":
        raw = value.get("status")
        if raw not in {"PASS", "FAIL", "REPORT"} or not isinstance(value.get("checks"), list):
            raise ReportError("performance report has an unsupported schema")
        return {
            "status": "INCOMPLETE" if raw == "REPORT" else raw,
            "mode": value.get("mode"),
            "checks": len(value["checks"]),
            "scope": value.get("scope"),
        }
    raise AssertionError(kind)


def status_gate(paths: list[pathlib.Path], kind: str) -> dict[str, Any]:
    if not paths:
        return {"status": "not_run"}
    details = []
    for path in paths:
        try:
            text = path.read_text(encoding="utf-8").strip()
        except (OSError, UnicodeError) as error:
            raise ReportError(f"cannot read {kind} status {path}: {error}") from error
        match = re.match(r"^(PASS|FAIL|INCOMPLETE):\s+(.+)$", text)
        if not match:
            raise ReportError(f"malformed {kind} status input {path}")
        details.append({"status": match.group(1), "detail": match.group(2)})
    statuses = {item["status"] for item in details}
    overall = "FAIL" if "FAIL" in statuses else ("INCOMPLETE" if "INCOMPLETE" in statuses else "PASS")
    return {"status": overall, "runs": details}


def cmock_gate(paths: list[pathlib.Path]) -> dict[str, Any]:
    if not paths:
        return {"status": "not_run", "reports": 0, "domains": 0}
    domains = 0
    passed = True
    for path in paths:
        value = load_json(path, "CMock")
        if not isinstance(value, dict) or not isinstance(value.get("passed"), bool) or not isinstance(value.get("domains"), list):
            raise ReportError("CMock report has an unsupported schema")
        passed = passed and value["passed"]
        domains += len(value["domains"])
    return {"status": "PASS" if passed else "FAIL", "reports": len(paths), "domains": domains}


def timestamp() -> str:
    raw = os.environ.get("SOURCE_DATE_EPOCH")
    try:
        moment = dt.datetime.fromtimestamp(int(raw), dt.timezone.utc) if raw is not None else dt.datetime.now(dt.timezone.utc)
    except (ValueError, OverflowError, OSError) as error:
        raise ReportError(f"invalid SOURCE_DATE_EPOCH: {raw!r}") from error
    return moment.replace(microsecond=0).isoformat().replace("+00:00", "Z")


def markdown(report: dict[str, Any]) -> str:
    ctest = report["gates"]["ctest"]
    lines = [
        "# Current host verification report",
        "",
        f"Generated: `{report['generated_at']}` (UTC)",
        f"Overall status: **{report['overall_status']}**",
        "",
        "## Build",
        "",
        f"- Mode: `{report['build']['mode']}`",
        f"- Compiler: `{report['build']['compiler']}`",
        f"- Generator: `{report['build']['generator']}`",
        f"- Targets: **{report['build']['target_count']}**",
        "",
        "## CTest",
        "",
        f"Status: **{ctest['status']}**; total **{ctest['total']}**, pass **{ctest['pass']}**, fail **{ctest['fail']}**, skip **{ctest['skip']}**, incomplete **{ctest['incomplete']}**.",
        "",
        "| Label | Total | Pass | Fail | Skip | Incomplete |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for label, counts in ctest["labels"].items():
        lines.append(
            f"| `{label}` | {counts['total']} | {counts['pass']} | {counts['fail']} | {counts['skip']} | {counts['incomplete']} |"
        )
    lines.extend(("", "## Gates", "", "| Gate | Status |", "|---|---|"))
    for name, gate in report["gates"].items():
        lines.append(f"| `{name}` | **{gate['status']}** |")
    lines.extend(
        (
            "",
            "## Scope limitation",
            "",
            report["scope"]["statement"],
            "",
        )
    )
    return "\n".join(lines)


def atomic_write(path: pathlib.Path, content: str) -> None:
    fd, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(content)
        pathlib.Path(temporary_name).replace(path)
    except Exception:
        pathlib.Path(temporary_name).unlink(missing_ok=True)
        raise


def main() -> int:
    args = arguments()
    build = discover_build(args.build_dir)
    output = safe_output(build, args.output_dir)
    current_json = output / "current.json"
    current_md = output / "current.md"
    current_json.unlink(missing_ok=True)
    current_md.unlink(missing_ok=True)

    cache_path = explicit_or_auto(args.cache, [build / "CMakeCache.txt"])
    assert cache_path is not None
    ctest_path = explicit_or_auto(args.ctest_json, [])
    junit_path = explicit_or_auto(args.junit, [output / "ctest.xml"])
    target_help_path = explicit_or_auto(args.target_help, [])
    quality_path = explicit_or_auto(args.quality, [build / "quality-report.json"])
    coverage_path = explicit_or_auto(args.coverage, [build / "coverage" / "coverage-summary.json"])
    performance_path = explicit_or_auto(
        args.performance,
        [
            build / "reports" / "performance.json",
            build / "performance" / "reports" / "performance.json",
            build / "gates" / "performance" / "reports" / "performance.json",
        ],
    )
    fuzz_path = explicit_or_auto(args.fuzz_status, [build / "fuzz-status.txt"])

    sanitizer_paths = [path.expanduser().resolve() for path in args.sanitizer_status]
    if not sanitizer_paths and (build / "sanitizer-status.txt").is_file():
        sanitizer_paths = [(build / "sanitizer-status.txt").resolve()]
    for path in sanitizer_paths:
        if not path.is_file():
            raise ReportError(f"explicit sanitizer status does not exist: {path}")

    cmock_paths = [path.expanduser().resolve() for path in args.cmock_json]
    if not cmock_paths:
        cmock_paths = sorted(build.glob("cmock*.json")) + sorted(output.glob("cmock*.json"))
    for path in cmock_paths:
        if not path.is_file():
            raise ReportError(f"explicit CMock report does not exist: {path}")

    cache = parse_cache(cache_path)
    metadata = load_ctest_metadata(build, ctest_path)
    ctest_gate = aggregate_ctest(metadata, parse_junit(junit_path))
    if args.ctest_exit_status is not None:
        if args.ctest_exit_status < 0:
            raise ReportError("CTest exit status must be zero or positive")
        ctest_gate["process_exit_status"] = args.ctest_exit_status
        if args.ctest_exit_status != 0:
            ctest_gate["status"] = "FAIL"
            ctest_gate["reason"] = (
                f"CTest process exited with status {args.ctest_exit_status}; "
                "JUnit testcase results cannot override a failed process"
            )
    gates = {
        "ctest": ctest_gate,
        "quality": gate_json(quality_path, "quality"),
        "coverage": gate_json(coverage_path, "coverage"),
        "performance": gate_json(performance_path, "performance"),
        "sanitizer": status_gate(sanitizer_paths, "sanitizer"),
        "fuzz": status_gate([fuzz_path] if fuzz_path else [], "fuzz"),
        "cmock": cmock_gate(cmock_paths),
    }

    incomplete_markers = {
        "coverage": [build / "coverage" / "INCOMPLETE.txt"],
        "performance": [
            build / "reports" / "INCOMPLETE",
            build / "performance" / "reports" / "INCOMPLETE",
            build / "gates" / "performance" / "reports" / "INCOMPLETE",
        ],
    }
    for gate_name, markers in incomplete_markers.items():
        present = [marker.name for marker in markers if marker.is_file()]
        if gates[gate_name]["status"] == "not_run" and present:
            gates[gate_name] = {"status": "INCOMPLETE", "markers": present}

    quality_status = build / "quality-status.txt"
    if gates["quality"]["status"] == "not_run" and quality_status.is_file():
        status_evidence = status_gate([quality_status], "quality")
        gates["quality"] = {
            **status_evidence,
            "status": "FAIL" if status_evidence["status"] == "FAIL" else "INCOMPLETE",
            "reason": "terminal status exists but quality-report.json is absent",
        }

    if any(gate["status"] not in STATUSES for gate in gates.values()):
        raise ReportError("internal error: unsupported gate status")
    statuses = {gate["status"] for gate in gates.values()}
    overall = "FAIL" if "FAIL" in statuses else ("INCOMPLETE" if statuses - {"PASS"} else "PASS")
    report = {
        "schema_version": SCHEMA_VERSION,
        "generated_at": timestamp(),
        "overall_status": overall,
        "build": {
            "mode": cache["CMAKE_BUILD_TYPE"],
            "compiler": cache["CMAKE_C_COMPILER"],
            "generator": cache["CMAKE_GENERATOR"],
            "target_count": target_count(build, target_help_path),
        },
        "gates": gates,
        "scope": {
            "kind": "native_host",
            "hil_or_mcu_evidence": False,
            "statement": "This report is native-host evidence only; it is not HIL, on-target MCU, peripheral timing, waveform, or firmware resource proof.",
        },
    }
    serialized = json.dumps(report, indent=2, sort_keys=True) + "\n"
    atomic_write(current_json, serialized)
    atomic_write(current_md, markdown(report))
    print(f"Current report: {current_json}")
    print(f"Current report: {current_md}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ReportError, OSError, subprocess.SubprocessError) as error:
        print(f"report generation: error: {error}", file=sys.stderr)
        raise SystemExit(2) from error
