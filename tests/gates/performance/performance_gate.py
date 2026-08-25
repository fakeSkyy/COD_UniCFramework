#!/usr/bin/env python3
"""Host-only performance and resource reporting/gating."""

from __future__ import annotations

import argparse
import json
import math
import os
import pathlib
import re
import subprocess
import sys
from typing import Any

THRESHOLD_ENV = {
    "calibration_max_spread": "PERF_CALIBRATION_MAX_SPREAD",
    "ringbuf_max_ratio": "PERF_RINGBUF_MAX_RATIO",
    "pid_max_ratio": "PERF_PID_MAX_RATIO",
    "ahrs_max_ratio": "PERF_AHRS_MAX_RATIO",
    "max_stack_bytes": "PERF_MAX_STACK_BYTES",
    "benchmark_max_text": "PERF_BENCHMARK_MAX_TEXT",
    "benchmark_max_data": "PERF_BENCHMARK_MAX_DATA",
    "benchmark_max_bss": "PERF_BENCHMARK_MAX_BSS",
    "representative_max_text": "PERF_REPRESENTATIVE_MAX_TEXT",
    "representative_max_data": "PERF_REPRESENTATIVE_MAX_DATA",
    "representative_max_bss": "PERF_REPRESENTATIVE_MAX_BSS",
}


def atomic_write(path: pathlib.Path, content: str) -> None:
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(content, encoding="utf-8")
    temporary.replace(path)


def invalidate_previous_pass(output_dir: pathlib.Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    for name in ("PASS", "FAIL", "REPORT"):
        (output_dir / name).unlink(missing_ok=True)
    atomic_write(
        output_dir / "INCOMPLETE",
        "Performance run did not complete; no prior PASS is valid.\n",
    )


def load_thresholds(path: pathlib.Path) -> dict[str, float]:
    thresholds = json.loads(path.read_text(encoding="utf-8"))
    for key, env_name in THRESHOLD_ENV.items():
        if env_name in os.environ:
            thresholds[key] = float(os.environ[env_name])
        value = float(thresholds[key])
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"threshold {key} must be finite and positive")
        thresholds[key] = value
    return thresholds


def run_benchmark(executable: pathlib.Path) -> dict[str, Any]:
    completed = subprocess.run(
        [str(executable)],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env={**os.environ, "LC_ALL": "C"},
    )
    result = json.loads(completed.stdout)
    required = ("ringbuf_put_get", "pid_step", "ahrs_update")
    for name in required:
        metric = result["benchmarks"][name]
        for field in ("median_ns_per_op", "spread_ratio", "ratio"):
            value = float(metric[field])
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"invalid benchmark metric {name}.{field}: {value}")
    return result


def parse_stack_usage(stack_root: pathlib.Path) -> dict[str, Any]:
    entries: list[dict[str, Any]] = []
    pattern = re.compile(r"^(.*):(\d+):(\d+):(.+)$")
    usage_files = sorted(stack_root.rglob("*.su"))
    if not usage_files:
        raise RuntimeError(f"no production .su files found below {stack_root}")

    for usage_file in usage_files:
        for raw_line in usage_file.read_text(encoding="utf-8", errors="replace").splitlines():
            fields = raw_line.split("\t")
            if len(fields) < 3:
                raise RuntimeError(f"unrecognized stack-usage line: {raw_line!r}")
            location = pattern.match(fields[0])
            if location is None:
                raise RuntimeError(f"unrecognized stack-usage location: {fields[0]!r}")
            try:
                stack_bytes = int(fields[1])
            except ValueError as error:
                raise RuntimeError(f"invalid stack size in: {raw_line!r}") from error
            qualifier = " ".join(fields[2:]).strip()
            entries.append(
                {
                    "source": location.group(1),
                    "line": int(location.group(2)),
                    "function": location.group(4),
                    "bytes": stack_bytes,
                    "qualifier": qualifier,
                }
            )

    if not entries:
        raise RuntimeError("production .su files contained no function records")
    entries.sort(key=lambda item: (-item["bytes"], item["function"]))
    unbounded = [
        entry
        for entry in entries
        if "dynamic" in entry["qualifier"] and "bounded" not in entry["qualifier"]
    ]
    return {
        "scope": "host production objects compiled with -fstack-usage",
        "max_bytes": entries[0]["bytes"],
        "max_function": entries[0]["function"],
        "max_source": entries[0]["source"],
        "function_count": len(entries),
        "unbounded_dynamic_functions": unbounded,
        "largest_functions": entries[:10],
    }


def verify_gnu_size(size_tool: pathlib.Path) -> str:
    completed = subprocess.run(
        [str(size_tool), "--version"],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env={**os.environ, "LC_ALL": "C"},
    )
    first_line = completed.stdout.splitlines()[0] if completed.stdout else ""
    if "GNU" not in completed.stdout:
        raise RuntimeError(f"GNU size is required, got: {first_line}")
    return first_line


def read_size(size_tool: pathlib.Path, executable: pathlib.Path) -> dict[str, Any]:
    completed = subprocess.run(
        [str(size_tool), "--format=berkeley", str(executable)],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env={**os.environ, "LC_ALL": "C"},
    )
    lines = [line.split() for line in completed.stdout.splitlines() if line.strip()]
    if len(lines) < 2 or lines[0][:3] != ["text", "data", "bss"]:
        raise RuntimeError(f"unrecognized GNU size output for {executable}: {completed.stdout!r}")
    values = lines[-1]
    if len(values) < 3:
        raise RuntimeError(f"incomplete GNU size output for {executable}")
    return {
        "path": str(executable),
        "text": int(values[0]),
        "data": int(values[1]),
        "bss": int(values[2]),
        "note": "host ELF size; this is not STM32/MCU firmware usage",
    }


def evaluate(
    benchmark: dict[str, Any],
    stack: dict[str, Any],
    sizes: dict[str, Any],
    thresholds: dict[str, float],
) -> list[dict[str, Any]]:
    checks: list[dict[str, Any]] = []

    def add(name: str, actual: float, limit: float, unit: str) -> None:
        checks.append(
            {
                "name": name,
                "actual": actual,
                "limit": limit,
                "unit": unit,
                "pass": actual <= limit,
            }
        )

    add(
        "calibration_spread",
        float(benchmark["calibration"]["spread_ratio"]),
        thresholds["calibration_max_spread"],
        "ratio",
    )
    add(
        "ringbuf_ratio",
        float(benchmark["benchmarks"]["ringbuf_put_get"]["ratio"]),
        thresholds["ringbuf_max_ratio"],
        "ratio",
    )
    add(
        "pid_ratio",
        float(benchmark["benchmarks"]["pid_step"]["ratio"]),
        thresholds["pid_max_ratio"],
        "ratio",
    )
    add(
        "ahrs_ratio",
        float(benchmark["benchmarks"]["ahrs_update"]["ratio"]),
        thresholds["ahrs_max_ratio"],
        "ratio",
    )
    add("production_max_stack", stack["max_bytes"], thresholds["max_stack_bytes"], "bytes")

    for executable_name, threshold_prefix in (
        ("benchmark", "benchmark"),
        ("representative", "representative"),
    ):
        for section in ("text", "data", "bss"):
            add(
                f"{executable_name}_{section}",
                sizes[executable_name][section],
                thresholds[f"{threshold_prefix}_max_{section}"],
                "bytes",
            )

    checks.append(
        {
            "name": "bounded_stack_usage",
            "actual": len(stack["unbounded_dynamic_functions"]),
            "limit": 0,
            "unit": "functions",
            "pass": not stack["unbounded_dynamic_functions"],
        }
    )
    return checks


def make_text(report: dict[str, Any]) -> str:
    lines = [
        f"Host performance/resource {report['mode'].upper()}: {report['status']}",
        "Timing gate: same-process workload ratios; ns/op values are informational only.",
        "Resource gate: native host ELF/object data only; it does NOT represent MCU firmware.",
        "",
        "Performance:",
    ]
    calibration = report["performance"]["calibration"]
    lines.append(
        "  calibration: "
        f"{calibration['median_ns_per_op']:.3f} ns/op, "
        f"mad_spread={calibration['spread_ratio']:.3f}"
    )
    for name, metric in report["performance"]["benchmarks"].items():
        lines.append(
            f"  {name}: {metric['median_ns_per_op']:.3f} ns/op (report only), "
            f"ratio={metric['ratio']:.3f}, spread={metric['spread_ratio']:.3f}"
        )

    stack = report["resources"]["stack_usage"]
    lines.extend(
        [
            "",
            "Resources (host only):",
            f"  production max stack: {stack['max_bytes']} bytes in {stack['max_function']}",
        ]
    )
    for name, size in report["resources"]["elf_size"].items():
        lines.append(
            f"  {name}: text={size['text']} data={size['data']} bss={size['bss']} bytes"
        )

    lines.extend(["", "Checks:"])
    for check in report["checks"]:
        state = "PASS" if check["pass"] else "FAIL"
        lines.append(
            f"  {state} {check['name']}: {check['actual']} <= {check['limit']} {check['unit']}"
        )
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("report", "check"), required=True)
    parser.add_argument("--benchmark", type=pathlib.Path, required=True)
    parser.add_argument("--representative", type=pathlib.Path, required=True)
    parser.add_argument("--stack-root", type=pathlib.Path, required=True)
    parser.add_argument("--size-tool", type=pathlib.Path, required=True)
    parser.add_argument("--config", type=pathlib.Path, required=True)
    parser.add_argument("--output-dir", type=pathlib.Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    invalidate_previous_pass(args.output_dir)

    try:
        thresholds = load_thresholds(args.config)
        benchmark = run_benchmark(args.benchmark)
        stack = parse_stack_usage(args.stack_root)
        size_version = verify_gnu_size(args.size_tool)
        sizes = {
            "benchmark": read_size(args.size_tool, args.benchmark),
            "representative": read_size(args.size_tool, args.representative),
        }
        checks = evaluate(benchmark, stack, sizes, thresholds)
        failed = [check for check in checks if not check["pass"]]
        status = "FAIL" if args.mode == "check" and failed else (
            "PASS" if args.mode == "check" else "REPORT"
        )
        report = {
            "schema_version": 1,
            "mode": args.mode,
            "status": status,
            "scope": "native Linux host; timing and resource figures do not represent MCU behavior",
            "thresholds": thresholds,
            "performance": benchmark,
            "resources": {
                "stack_usage": stack,
                "elf_size": sizes,
                "size_tool": size_version,
            },
            "checks": checks,
        }
        atomic_write(args.output_dir / "performance.json", json.dumps(report, indent=2) + "\n")
        atomic_write(args.output_dir / "performance.txt", make_text(report))

        if args.mode == "check" and failed:
            atomic_write(
                args.output_dir / "FAIL",
                "Performance/resource check failed; inspect performance.txt.\n",
            )
            (args.output_dir / "INCOMPLETE").unlink(missing_ok=True)
            return 1

        marker = "PASS" if args.mode == "check" else "REPORT"
        atomic_write(args.output_dir / marker, f"{status}: reports are complete.\n")
        (args.output_dir / "INCOMPLETE").unlink(missing_ok=True)
        return 0
    except Exception as error:  # Keep INCOMPLETE and ensure shell/CTest fail.
        print(f"performance gate error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
