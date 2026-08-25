#!/usr/bin/env python3
"""Dependency-free static-analysis and architecture quality gate."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import uuid
from typing import Iterable
import xml.etree.ElementTree as ET

SCHEMA_VERSION = 3
PRODUCTION_LAYERS = (
    "01_application",
    "02_device",
    "03_platform",
    "04_impl",
    "06_utils",
)
SOURCE_SUFFIXES = {".c", ".h"}
INCLUDE_RE = re.compile(r'^[ \t]*#[ \t]*include[ \t]*[<"]([^>"]+)[>"]', re.MULTILINE)
FLOAT_RE = re.compile(
    r"(?<![A-Za-z0-9_.])(?:"
    r"0[xX](?:[0-9A-Fa-f]+\.[0-9A-Fa-f]*|\.[0-9A-Fa-f]+|[0-9A-Fa-f]+)"
    r"[pP][+-]?\d+|"
    r"(?:\d+\.\d*|\.\d+)(?:[eE][+-]?\d+)?|"
    r"\d+[eE][+-]?\d+"
    r")(?P<suffix>[fFlL]?)(?![A-Za-z0-9_])"
)
MATH_NAMES = (
    "acos", "asin", "atan", "atan2", "cbrt", "ceil", "cos", "cosh", "erf", "erfc",
    "exp", "fabs", "floor", "fmod", "frexp", "hypot", "ldexp", "lgamma", "log",
    "log10", "modf", "pow", "round", "sin", "sinh", "sqrt", "tan", "tanh", "tgamma",
    "trunc",
)
MATH_RE = re.compile(r"(?<![A-Za-z0-9_])(" + "|".join(MATH_NAMES) + r")\s*\(")
VENDOR_TYPE_RE = re.compile(
    r"\b(?:HAL_[A-Za-z0-9_]*|LL_[A-Za-z0-9_]*|CMSIS_[A-Za-z0-9_]*|"
    r"(?:ADC|CAN|CRC|DMA|FDCAN|FLASH|GPIO|I2C|IWDG|RCC|SPI|TIM|UART|WWDG)_HandleTypeDef|"
    r"GPIO_TypeDef|TaskHandle_t|SemaphoreHandle_t|QueueHandle_t|EventGroupHandle_t|"
    r"BaseType_t|UBaseType_t|TickType_t)\b"
)
VENDOR_HEADER_RE = re.compile(
    r"^(?:stm32[^/]*\.h|FreeRTOS\.h|task\.h|semphr\.h|queue\.h|event_groups\.h|"
    r"SEGGER_RTT[^/]*\.h|cmsis[^/]*\.h|arm_math\.h|main\.h|gpio\.h|tim\.h|"
    r"fdcan\.h|can\.h|spi\.h|i2c\.h|usart\.h|adc\.h|dma\.h)$",
    re.IGNORECASE,
)
VENDOR_PACKAGES = ("05_vender/freertos", "05_vender/segger_rtt")
TEST_PATH_PARTS = {"test", "tests", "mock", "mocks", "contract", "contracts"}

ANALYZER_CONFIG_VERSION = 1
CLANG_TIDY_NAMES = ("clang-tidy-18",)
CLANG_TIDY_CHECKS = (
    "-*,"
    "clang-analyzer-core.*,"
    "clang-analyzer-unix.Malloc,"
    "clang-analyzer-unix.MallocSizeof,"
    "bugprone-sizeof-expression,"
    "bugprone-suspicious-memory-comparison"
)
CPPCHECK_ENABLE = "warning,performance,portability"
ANALYSIS_CMAKE_ARGS = (
    "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
    "-DENABLE_QUALITY_GATES=OFF",
    "-DENABLE_PROPERTY_TESTS=OFF",
    "-DENABLE_PERFORMANCE_GATES=OFF",
)


def mask_c(text: str, mask_strings: bool = True) -> str:
    """Mask comments and optionally literals while retaining offsets and line numbers."""
    output = list(text)
    state = "normal"
    quote = ""
    index = 0
    while index < len(text):
        char = text[index]
        nxt = text[index + 1] if index + 1 < len(text) else ""
        if state == "normal":
            if char == "/" and nxt == "/":
                output[index] = output[index + 1] = " "
                index += 2
                state = "line_comment"
                continue
            if char == "/" and nxt == "*":
                output[index] = output[index + 1] = " "
                index += 2
                state = "block_comment"
                continue
            if char in {'"', "'"}:
                quote = char
                if mask_strings:
                    output[index] = " "
                state = "string"
        elif state == "line_comment":
            if char == "\n":
                state = "normal"
            else:
                output[index] = " "
        elif state == "block_comment":
            if char == "*" and nxt == "/":
                output[index] = output[index + 1] = " "
                index += 2
                state = "normal"
                continue
            if char != "\n":
                output[index] = " "
        elif state == "string":
            if mask_strings and char != "\n":
                output[index] = " "
            if char == "\\" and nxt:
                if mask_strings and nxt != "\n":
                    output[index + 1] = " "
                index += 2
                continue
            if char == quote:
                state = "normal"
        index += 1
    return "".join(output)


def line_number(text: str, offset: int) -> int:
    return text.count("\n", 0, offset) + 1


def normalized_line(text: str, line: int) -> str:
    lines = text.splitlines()
    value = lines[line - 1] if 0 < line <= len(lines) else ""
    return " ".join(value.strip().split())


def violation_id(path: str, rule: str, token: str, code_line: str, ordinal: int) -> str:
    digest = hashlib.sha256(code_line.encode("utf-8")).hexdigest()[:16]
    return f"{path}|{rule}|{token}|{digest}|{ordinal}"


def make_violation(rule: str, path: str, line: int, message: str, token: str = "") -> dict:
    return {"rule": rule, "path": path, "line": line, "token": token, "message": message}


def check_result(name: str, status: str, summary: str, violations: list[dict] | None = None) -> dict:
    return {
        "name": name,
        "status": status,
        "summary": summary,
        "violations": violations or [],
    }


class QualityRunner:
    """Run repository checks without traversing vendor source trees."""

    def __init__(
        self,
        root: Path,
        baseline_path: Path | None = None,
        architecture_baseline_path: Path | None = None,
        vendor_checksums_path: Path | None = None,
        compile_db_path: Path | None = None,
    ) -> None:
        self.root = root.resolve()
        self.quality_dir = Path(__file__).resolve().parent
        self.baseline_path = (baseline_path or self.quality_dir / "float_baseline.json").resolve()
        self.architecture_baseline_path = (
            architecture_baseline_path or self.quality_dir / "architecture_baseline.json"
        ).resolve()
        self.vendor_checksums_path = (
            vendor_checksums_path or self.quality_dir / "vendor_checksums.sha256"
        ).resolve()
        self.compile_db_path = compile_db_path.resolve() if compile_db_path is not None else None
        self.production_files = self._production_files()
        self.header_index = self._header_index()
        self.test_header_files = self._test_header_files()
        self.test_header_names = {path.name for path in self.test_header_files}

    def _production_files(self) -> list[Path]:
        files: list[Path] = []
        for layer in PRODUCTION_LAYERS:
            directory = self.root / layer
            if directory.is_dir():
                files.extend(
                    path for path in directory.rglob("*")
                    if path.is_file() and path.suffix.lower() in SOURCE_SUFFIXES
                )
        return sorted(files)

    def _header_index(self) -> dict[str, list[str]]:
        index: dict[str, list[str]] = {}
        for path in self.production_files:
            if path.suffix.lower() == ".h":
                index.setdefault(path.name, []).append(path.relative_to(self.root).as_posix())
        return index

    def _test_header_files(self) -> list[Path]:
        files: set[Path] = set()
        tests = self.root / "tests"
        if not tests.is_dir():
            return []
        for category in ("mock", "mocks", "contract", "contracts"):
            for directory in tests.rglob(category):
                if directory.is_dir():
                    files.update(path for path in directory.rglob("*.h") if path.is_file())
        return sorted(files)

    def _relative(self, path: Path) -> str:
        return path.relative_to(self.root).as_posix()

    @staticmethod
    def _layer(path: str) -> str | None:
        first = path.replace("\\", "/").lstrip("./").split("/", 1)[0]
        return first if first in {*PRODUCTION_LAYERS, "05_vender"} else None

    def _include_targets(self, source: Path, include: str) -> list[str]:
        normalized = include.replace("\\", "/").lstrip("./")
        explicit_layer = self._layer(normalized)
        if explicit_layer is not None:
            return [normalized]
        candidate = (source.parent / include).resolve()
        try:
            relative = candidate.relative_to(self.root).as_posix()
        except ValueError:
            relative = ""
        if relative and candidate.is_file() and self._layer(relative) is not None:
            return [relative]
        return self.header_index.get(Path(normalized).name, [])

    @staticmethod
    def _test_include(
        include: str, test_header_names: set[str], production_targets: list[str]
    ) -> bool:
        normalized = include.replace("\\", "/")
        parts = {part.lower() for part in normalized.split("/")}
        name = Path(normalized).name
        return (
            bool(parts & TEST_PATH_PARTS)
            or name.startswith("mock_")
            or (
                name in test_header_names
                and not production_targets
                and VENDOR_HEADER_RE.match(name) is None
            )
        )

    def _load_architecture_baseline(self) -> tuple[dict[str, dict], list[dict]]:
        """Load individually reviewed architecture findings; broad exceptions are forbidden."""
        path = self.architecture_baseline_path
        if not path.is_file():
            return {}, [make_violation(
                "ARCH_BASELINE_MISSING", path.as_posix(), 0,
                "reviewed architecture baseline is missing",
            )]
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            return {}, [make_violation(
                "ARCH_BASELINE_INVALID", path.as_posix(), 0, str(error),
            )]
        if data.get("schema_version") != 1 or not isinstance(data.get("reviewed_violations"), list):
            return {}, [make_violation(
                "ARCH_BASELINE_INVALID", path.as_posix(), 0,
                "expected schema_version 1 and reviewed_violations array",
            )]
        entries: dict[str, dict] = {}
        errors: list[dict] = []
        for entry in data["reviewed_violations"]:
            strings_valid = isinstance(entry, dict) and all(
                isinstance(entry.get(key), str) and entry[key]
                for key in ("id", "path", "rule", "token", "line_sha256", "rationale")
            )
            occurrence_valid = (
                isinstance(entry, dict)
                and isinstance(entry.get("occurrence"), int)
                and not isinstance(entry.get("occurrence"), bool)
                and entry["occurrence"] > 0
            )
            if not strings_valid or not occurrence_valid:
                errors.append(make_violation(
                    "ARCH_BASELINE_INVALID", path.as_posix(), 0,
                    "each entry requires id/path/rule/token/line_sha256/rationale and a positive "
                    "integer occurrence",
                ))
                continue
            if entry["id"] in entries:
                errors.append(make_violation(
                    "ARCH_BASELINE_INVALID", path.as_posix(), 0,
                    f"duplicate baseline id '{entry['id']}'", entry["id"],
                ))
            entries[entry["id"]] = entry
        return entries, errors

    def _review_architecture_findings(self, findings: list[dict]) -> dict:
        occurrences: dict[tuple[str, str, str, str], int] = {}
        for item in findings:
            source = self.root / item["path"]
            view = mask_c(source.read_text(encoding="utf-8", errors="replace"), mask_strings=False)
            code_line = normalized_line(view, item["line"])
            key = (item["path"], item["rule"], item["token"], code_line)
            occurrences[key] = occurrences.get(key, 0) + 1
            occurrence = occurrences[key]
            item["line_sha256"] = hashlib.sha256(code_line.encode("utf-8")).hexdigest()
            item["occurrence"] = occurrence
            item["id"] = violation_id(
                item["path"], item["rule"], item["token"], code_line, occurrence
            )

        baseline, errors = self._load_architecture_baseline()
        current_by_id = {item["id"]: item for item in findings}
        current_ids = set(current_by_id)
        new = [item for item in findings if item["id"] not in baseline]
        stale = sorted(set(baseline) - current_ids)
        mismatched: list[dict] = []
        metadata = ("path", "rule", "token", "line_sha256", "occurrence")
        for identifier in sorted(set(baseline) & current_ids):
            entry = baseline[identifier]
            finding = current_by_id[identifier]
            if any(entry[key] != finding[key] for key in metadata):
                mismatched.append(make_violation(
                    "ARCH_BASELINE_INVALID", entry["path"], 0,
                    "baseline metadata does not exactly match the current finding", identifier,
                ))
        violations = errors + new + mismatched
        for identifier in stale:
            entry = baseline[identifier]
            violations.append(make_violation(
                "ARCH_BASELINE_STALE", entry["path"], 0,
                "reviewed violation no longer exists; remove the stale baseline entry", identifier,
            ))
        reviewed = len(findings) - len(new) - len(mismatched)
        return check_result(
            "architecture.includes",
            "FAIL" if violations else "PASS",
            f"found {len(findings)} invalid direct includes: {reviewed} reviewed, "
            f"{len(new)} new, {len(stale)} stale, {len(mismatched)} metadata mismatches",
            violations,
        )

    def check_architecture(self) -> list[dict]:
        dependency_violations: list[dict] = []
        vendor_violations: list[dict] = []
        test_violations: list[dict] = []
        allowed = {
            "01_application": {"01_application", "02_device", "03_platform", "06_utils"},
            "02_device": {"02_device", "03_platform", "06_utils"},
            "03_platform": {"03_platform", "04_impl", "06_utils"},
            "04_impl": {"04_impl", "05_vender", "06_utils"},
            "06_utils": {"06_utils"},
        }
        for source in self.production_files:
            relative = self._relative(source)
            source_layer = self._layer(relative)
            raw = source.read_text(encoding="utf-8", errors="replace")
            include_view = mask_c(raw, mask_strings=False)
            for match in INCLUDE_RE.finditer(include_view):
                include = match.group(1)
                line = line_number(include_view, match.start())
                targets = self._include_targets(source, include)
                if self._test_include(include, self.test_header_names, targets):
                    test_violations.append(make_violation(
                        "TEST_HEADER_IN_PRODUCTION", relative, line,
                        f"production source includes test mock/contract header '{include}'", include,
                    ))
                target_layers = {self._layer(target) for target in targets}
                target_layers.discard(None)
                if not target_layers and VENDOR_HEADER_RE.match(Path(include).name):
                    target_layers.add("05_vender")
                board_exception = relative.startswith("01_application/board/")
                for target_layer in sorted(target_layers):
                    if board_exception:
                        continue
                    if target_layer not in allowed.get(source_layer or "", set()):
                        dependency_violations.append(make_violation(
                            "ARCH_LAYER_INCLUDE", relative, line,
                            f"{source_layer} may not directly include {target_layer}: '{include}'", include,
                        ))
                if source_layer == "03_platform" and "04_impl" in target_layers:
                    if any(not target.startswith("04_impl/common/") for target in targets):
                        dependency_violations.append(make_violation(
                            "ARCH_PLATFORM_COMMON_ONLY", relative, line,
                            f"03_platform may include only 04_impl/common interfaces: '{include}'", include,
                        ))
                if source_layer in {"02_device", "03_platform"} and "05_vender" in target_layers:
                    vendor_violations.append(make_violation(
                        "VENDOR_HEADER_OUTSIDE_IMPL", relative, line,
                        f"{source_layer} must not include vendor header '{include}'", include,
                    ))
            if source_layer in {"02_device", "03_platform"}:
                token_view = mask_c(raw, mask_strings=True)
                for match in VENDOR_TYPE_RE.finditer(token_view):
                    vendor_violations.append(make_violation(
                        "VENDOR_TYPE_OUTSIDE_IMPL", relative,
                        line_number(token_view, match.start()),
                        f"{source_layer} exposes vendor type/token '{match.group(0)}'", match.group(0),
                    ))
        dependency_violations.sort(key=lambda item: (item["path"], item["line"], item["rule"]))
        vendor_violations.sort(key=lambda item: (item["path"], item["line"], item["rule"]))
        test_violations.sort(key=lambda item: (item["path"], item["line"]))
        dependency_result = self._review_architecture_findings(dependency_violations)
        return [
            dependency_result,
            check_result(
                "architecture.vendor_neutral",
                "FAIL" if vendor_violations else "PASS",
                f"{len(vendor_violations)} vendor headers/types leaked above 04_impl",
                vendor_violations,
            ),
            check_result(
                "architecture.test_headers",
                "FAIL" if test_violations else "PASS",
                f"{len(test_violations)} test mock/contract headers included by production",
                test_violations,
            ),
        ]

    def _vendor_package_files(self) -> list[Path]:
        """Enumerate only the two immutable upstream packages covered by the baseline."""
        files: list[Path] = []
        for package in VENDOR_PACKAGES:
            directory = self.root / package
            if directory.is_dir():
                files.extend(path for path in directory.rglob("*") if path.is_file())
        return sorted(files)

    @staticmethod
    def _valid_vendor_baseline_name(name: str) -> bool:
        if "\\" in name or "//" in name or name.startswith("/"):
            return False
        raw_parts = name.split("/")
        if any(part in {"", ".", ".."} for part in raw_parts):
            return False
        path = PurePosixPath(name)
        return any(path == PurePosixPath(package) or path.is_relative_to(package)
                   for package in VENDOR_PACKAGES)

    def check_vendor_checksums(self) -> dict:
        violations: list[dict] = []
        baseline = self.vendor_checksums_path
        baseline_name = baseline.as_posix()
        if not baseline.is_file():
            return check_result(
                "vendor.checksums", "FAIL", "reviewed vendor checksum baseline is missing",
                [make_violation(
                    "VENDOR_CHECKSUM_BASELINE_MISSING", baseline_name, 0,
                    "tests/gates/quality/vendor_checksums.sha256 is required",
                )],
            )

        seen: set[str] = set()
        checked_files = 0
        try:
            lines = baseline.read_text(encoding="utf-8", errors="strict").splitlines()
        except (OSError, UnicodeError) as error:
            return check_result(
                "vendor.checksums", "FAIL", "vendor checksum baseline could not be read",
                [make_violation("VENDOR_CHECKSUM_SYNTAX", baseline_name, 0, str(error))],
            )

        for line_no, raw_line in enumerate(lines, start=1):
            if not raw_line:
                continue
            match = re.fullmatch(r"([0-9A-Fa-f]{64}) ([ *])(.+)", raw_line)
            if match is None:
                violations.append(make_violation(
                    "VENDOR_CHECKSUM_SYNTAX", baseline_name, line_no,
                    "entry is not in sha256sum format", raw_line,
                ))
                continue
            expected, _mode, name = match.groups()
            if not self._valid_vendor_baseline_name(name):
                violations.append(make_violation(
                    "VENDOR_CHECKSUM_PATH", baseline_name, line_no,
                    f"path must remain inside a reviewed vendor package: '{name}'", name,
                ))
                continue
            if name in seen:
                violations.append(make_violation(
                    "VENDOR_CHECKSUM_DUPLICATE", baseline_name, line_no,
                    f"duplicate checksum path '{name}'", name,
                ))
                continue
            seen.add(name)
            target = self.root / name
            package = next(package for package in VENDOR_PACKAGES
                           if PurePosixPath(name).is_relative_to(package))
            try:
                target.resolve().relative_to((self.root / package).resolve())
            except ValueError:
                violations.append(make_violation(
                    "VENDOR_CHECKSUM_PATH", baseline_name, line_no,
                    f"path escapes its reviewed vendor package: '{name}'", name,
                ))
                continue
            if not target.is_file():
                violations.append(make_violation(
                    "VENDOR_CHECKSUM_MISSING", baseline_name, line_no,
                    f"listed vendor file is missing: '{name}'", name,
                ))
                continue
            checked_files += 1
            digest = hashlib.sha256(target.read_bytes()).hexdigest()
            if digest.lower() != expected.lower():
                violations.append(make_violation(
                    "VENDOR_CHECKSUM_HASH", baseline_name, line_no,
                    f"checksum mismatch for '{name}'", name,
                ))

        actual = {self._relative(path) for path in self._vendor_package_files()}
        for name in sorted(actual - seen):
            violations.append(make_violation(
                "VENDOR_CHECKSUM_UNLISTED", name, 0,
                "vendor package contains a file absent from the reviewed checksum baseline", name,
            ))
        for package in VENDOR_PACKAGES:
            if not (self.root / package).is_dir():
                violations.append(make_violation(
                    "VENDOR_PACKAGE_MISSING", package, 0,
                    "reviewed vendor package directory is missing", package,
                ))

        return check_result(
            "vendor.checksums",
            "FAIL" if violations else "PASS",
            f"verified {checked_files} listed files and closed-set scanned {len(actual)} files "
            "across FreeRTOS and SEGGER RTT",
            violations,
        )

    def check_format(self) -> dict:
        executable = shutil.which("clang-format")
        if executable is None:
            return check_result(
                "format.clang-format", "FAIL",
                "clang-format is required for production C/H formatting but was not found",
                [make_violation("CLANG_FORMAT_MISSING", ".", 0, "install clang-format")],
            )
        if not self.production_files:
            return check_result("format.clang-format", "PASS", "no production C/H files found")
        process = subprocess.run(
            [executable, "--dry-run", "--Werror", "--style=file", *map(str, self.production_files)],
            cwd=self.root,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        violations: list[dict] = []
        if process.returncode != 0:
            diagnostics = (process.stderr or process.stdout).splitlines()
            diagnostic_re = re.compile(r"^(.*?):(\d+):\d+: (?:error|warning): (.*)$")
            for diagnostic in diagnostics:
                match = diagnostic_re.match(diagnostic)
                if match:
                    path_text, line_text, message = match.groups()
                    path = Path(path_text)
                    try:
                        path_text = path.resolve().relative_to(self.root).as_posix()
                    except ValueError:
                        pass
                    violations.append(make_violation(
                        "CLANG_FORMAT", path_text, int(line_text), message,
                    ))
            if not violations:
                violations.append(make_violation(
                    "CLANG_FORMAT", ".", 0,
                    "clang-format failed: " + "\n".join(diagnostics[-10:]),
                ))
        return check_result(
            "format.clang-format",
            "FAIL" if violations else "PASS",
            f"clang-format --dry-run checked {len(self.production_files)} production C/H files",
            violations,
        )

    def _load_float_baseline(self) -> tuple[dict[str, dict], list[dict]]:
        errors: list[dict] = []
        if not self.baseline_path.is_file():
            return {}, [make_violation(
                "FLOAT_BASELINE_MISSING", self.baseline_path.as_posix(), 0,
                "reviewed float baseline is missing",
            )]
        try:
            data = json.loads(self.baseline_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            return {}, [make_violation(
                "FLOAT_BASELINE_INVALID", self.baseline_path.as_posix(), 0, str(error),
            )]
        if data.get("schema_version") != 1 or not isinstance(data.get("reviewed_violations"), list):
            return {}, [make_violation(
                "FLOAT_BASELINE_INVALID", self.baseline_path.as_posix(), 0,
                "expected schema_version 1 and reviewed_violations array",
            )]
        entries: dict[str, dict] = {}
        for entry in data["reviewed_violations"]:
            if not isinstance(entry, dict) or not all(
                isinstance(entry.get(key), str) and entry[key]
                for key in ("id", "path", "rule", "token", "line_sha256", "rationale")
            ):
                errors.append(make_violation(
                    "FLOAT_BASELINE_INVALID", self.baseline_path.as_posix(), 0,
                    "each baseline entry requires non-empty id/path/rule/token/line_sha256/rationale",
                ))
                continue
            if entry["id"] in entries:
                errors.append(make_violation(
                    "FLOAT_BASELINE_INVALID", self.baseline_path.as_posix(), 0,
                    f"duplicate baseline id '{entry['id']}'", entry["id"],
                ))
            entries[entry["id"]] = entry
        return entries, errors

    def scan_float_violations(self) -> list[dict]:
        violations: list[dict] = []
        for source in self.production_files:
            relative = self._relative(source)
            raw = source.read_text(encoding="utf-8", errors="replace")
            view = mask_c(raw, mask_strings=True)
            occurrences: dict[tuple[str, str, str], int] = {}
            candidates: list[tuple[int, str, str, str]] = []
            for match in FLOAT_RE.finditer(view):
                if match.group("suffix").lower() != "f":
                    candidates.append((match.start(), "FLOAT_LITERAL_SUFFIX", match.group(0),
                                       "floating literal must have an f suffix"))
            for match in MATH_RE.finditer(view):
                candidates.append((match.start(), "FLOAT_MATH_VARIANT", match.group(1),
                                   f"use {match.group(1)}f instead of {match.group(1)}"))
            for offset, rule, token, message in sorted(candidates):
                line = line_number(view, offset)
                code_line = normalized_line(view, line)
                key = (rule, token, code_line)
                occurrences[key] = occurrences.get(key, 0) + 1
                item = make_violation(rule, relative, line, message, token)
                item["line_sha256"] = hashlib.sha256(code_line.encode("utf-8")).hexdigest()
                item["id"] = violation_id(relative, rule, token, code_line, occurrences[key])
                violations.append(item)
        return violations

    def check_float_rules(self) -> dict:
        current = self.scan_float_violations()
        baseline, errors = self._load_float_baseline()
        current_by_id = {item["id"]: item for item in current}
        current_ids = set(current_by_id)
        new = [item for item in current if item["id"] not in baseline]
        stale = sorted(set(baseline) - current_ids)
        mismatched: list[dict] = []
        for identifier in sorted(set(baseline) & current_ids):
            entry = baseline[identifier]
            finding = current_by_id[identifier]
            if any(entry[key] != finding[key] for key in ("path", "rule", "token", "line_sha256")):
                mismatched.append(make_violation(
                    "FLOAT_BASELINE_INVALID", entry["path"], 0,
                    "baseline metadata does not exactly match the current finding", identifier,
                ))
        violations = errors + new + mismatched
        for identifier in stale:
            entry = baseline[identifier]
            violations.append(make_violation(
                "FLOAT_BASELINE_STALE", entry["path"], 0,
                "reviewed violation no longer exists; remove the stale baseline entry", identifier,
            ))
        reviewed = len(current) - len(new) - len(mismatched)
        return check_result(
            "float.rules",
            "FAIL" if violations else "PASS",
            f"found {len(current)} float-rule findings: {reviewed} reviewed, "
            f"{len(new)} new, {len(stale)} stale, {len(mismatched)} metadata mismatches",
            violations,
        )

    @staticmethod
    def _find_tool(names: tuple[str, ...]) -> tuple[str | None, str]:
        for name in names:
            executable = shutil.which(name)
            if executable is not None:
                return executable, name
        return None, names[0]

    def _production_diagnostic_path(self, value: str) -> str | None:
        path = Path(value)
        if not path.is_absolute():
            path = self.root / path
        try:
            relative = path.resolve().relative_to(self.root).as_posix()
        except ValueError:
            return None
        return relative if self._layer(relative) in PRODUCTION_LAYERS else None

    @staticmethod
    def _executable_identity(label: str, executable: str) -> str:
        """Identify an executable by its resolved path and reported version."""
        resolved = Path(executable).resolve().as_posix()
        try:
            process = subprocess.run(
                [resolved, "--version"],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )
            version = process.stdout.strip()
        except OSError as error:
            version = f"EXECUTION_FAILED:{error}"
        return f"{label}:{resolved}:{version}"

    @staticmethod
    def _normalize_analysis_value(
        value: str, root: Path, database_root: Path, workspace: Path
    ) -> str:
        """Remove ephemeral roots while retaining every semantic command component."""
        normalized = value.replace("\\", "/")
        replacements = (
            (database_root.resolve().as_posix(), "${COMPILE_DB_ROOT}"),
            (workspace.resolve().as_posix(), "${ANALYSIS_WORKSPACE}"),
            (root.resolve().as_posix(), "${SOURCE_ROOT}"),
        )
        for original, replacement in sorted(replacements, key=lambda item: -len(item[0])):
            normalized = normalized.replace(original, replacement)
        return normalized

    def _compiler_identity(self, entry: dict, source_db: Path) -> str:
        """Resolve the compiler named by a selected compile command."""
        arguments = entry.get("arguments")
        if (
            isinstance(arguments, list)
            and arguments
            and all(isinstance(argument, str) for argument in arguments)
        ):
            command = arguments
        elif isinstance(entry.get("command"), str):
            try:
                command = shlex.split(entry["command"])
            except ValueError as error:
                return f"c-compiler:UNPARSEABLE:{error}"
        else:
            return "c-compiler:MISSING"
        if not command:
            return "c-compiler:MISSING"

        token = command[0]
        directory = Path(entry.get("directory", source_db.parent))
        if not directory.is_absolute():
            directory = source_db.parent / directory
        candidate = Path(token)
        if candidate.is_absolute() or "/" in token:
            executable = candidate if candidate.is_absolute() else directory / candidate
            if not executable.exists():
                return f"c-compiler:NOT_FOUND:{token}"
            resolved = executable.resolve().as_posix()
        else:
            found = shutil.which(token)
            if found is None:
                return f"c-compiler:NOT_FOUND:{token}"
            resolved = found
        return self._executable_identity("c-compiler", resolved)

    def _analysis_database_identity(
        self,
        selected: dict[str, dict],
        source_db: Path,
        workspace: Path,
        generated: bool,
        cmake: str | None,
    ) -> dict:
        """Hash the exact selected commands after stable build-root normalization."""
        records: list[dict] = []
        compiler_identities: set[str] = set()
        for source_name in sorted(selected):
            entry = selected[source_name]
            directory = str(entry.get("directory", source_db.parent))
            record = {
                "file": Path(source_name).relative_to(self.root).as_posix(),
                "directory": self._normalize_analysis_value(
                    directory, self.root, source_db.parent, workspace
                ),
            }
            if isinstance(entry.get("command"), str):
                record["command"] = self._normalize_analysis_value(
                    entry["command"], self.root, source_db.parent, workspace
                )
            if isinstance(entry.get("arguments"), list):
                record["arguments"] = [
                    self._normalize_analysis_value(
                        str(argument), self.root, source_db.parent, workspace
                    )
                    for argument in entry["arguments"]
                ]
            records.append(record)
            compiler_identities.add(self._compiler_identity(entry, source_db))

        serialized = json.dumps(records, sort_keys=True, separators=(",", ":"))
        identity = {
            "mode": "generated" if generated else "explicit",
            "normalized_compile_database_sha256": hashlib.sha256(
                serialized.encode("utf-8")
            ).hexdigest(),
            "translation_units": len(records),
            "c_compilers": sorted(compiler_identities),
        }
        if generated and cmake is not None:
            identity["cmake"] = self._executable_identity("cmake", cmake)
        return identity

    @staticmethod
    def _failed_analysis_identity(generated: bool, result: dict) -> dict:
        """Represent database-preparation failure deterministically in failed reports."""
        failure = {
            "status": result["status"],
            "rules": sorted(item["rule"] for item in result["violations"]),
        }
        serialized = json.dumps(failure, sort_keys=True, separators=(",", ":"))
        return {
            "mode": "generated" if generated else "explicit",
            "preparation": "FAILED",
            "failure_sha256": hashlib.sha256(serialized.encode("utf-8")).hexdigest(),
        }

    def _prepare_analysis_database(
        self, workspace: Path
    ) -> tuple[dict, Path | None, dict]:
        source_db = self.compile_db_path
        generated = source_db is None
        cmake: str | None = None
        if source_db is None:
            cmake = shutil.which("cmake")
            if cmake is None:
                violation = make_violation(
                    "ANALYSIS_CMAKE_MISSING",
                    ".",
                    0,
                    "cmake is required to generate the host compile database",
                )
                result = check_result(
                    "analysis.compile_database",
                    "FAIL",
                    "could not generate a production compile database",
                    [violation],
                )
                return result, None, self._failed_analysis_identity(generated, result)
            host_build = workspace / "host-build"
            process = subprocess.run(
                [
                    cmake,
                    "-S",
                    str(self.root / "tests"),
                    "-B",
                    str(host_build),
                    *ANALYSIS_CMAKE_ARGS,
                ],
                cwd=self.root,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )
            if process.returncode != 0:
                violation = make_violation(
                    "ANALYSIS_CMAKE_CONFIGURE",
                    "tests/CMakeLists.txt",
                    0,
                    "host compile database configuration failed: "
                    + "\n".join(process.stdout.splitlines()[-10:]),
                )
                result = check_result(
                    "analysis.compile_database",
                    "FAIL",
                    "could not generate a production compile database",
                    [violation],
                )
                return result, None, self._failed_analysis_identity(generated, result)
            source_db = host_build / "compile_commands.json"

        if not source_db.is_file():
            violation = make_violation(
                "ANALYSIS_COMPILE_DB_MISSING",
                source_db.as_posix(),
                0,
                "compile_commands.json does not exist",
            )
            result = check_result(
                "analysis.compile_database",
                "FAIL",
                "compile database is missing",
                [violation],
            )
            return result, None, self._failed_analysis_identity(generated, result)

        try:
            entries = json.loads(source_db.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            violation = make_violation(
                "ANALYSIS_COMPILE_DB_INVALID", source_db.as_posix(), 0, str(error)
            )
            result = check_result(
                "analysis.compile_database",
                "FAIL",
                "compile database is invalid",
                [violation],
            )
            return result, None, self._failed_analysis_identity(generated, result)
        if not isinstance(entries, list):
            violation = make_violation(
                "ANALYSIS_COMPILE_DB_INVALID",
                source_db.as_posix(),
                0,
                "compile database root must be an array",
            )
            result = check_result(
                "analysis.compile_database",
                "FAIL",
                "compile database is invalid",
                [violation],
            )
            return result, None, self._failed_analysis_identity(generated, result)

        expected = {
            path.resolve().as_posix(): path
            for path in self.production_files
            if path.suffix.lower() == ".c"
        }
        selected: dict[str, dict] = {}
        for entry in entries:
            if not isinstance(entry, dict) or not isinstance(entry.get("file"), str):
                continue
            directory = Path(entry.get("directory", self.root))
            source = Path(entry["file"])
            if not source.is_absolute():
                source = directory / source
            source_name = source.resolve().as_posix()
            if source_name in expected and source_name not in selected:
                selected[source_name] = entry

        missing = sorted(set(expected) - set(selected))
        if missing:
            violations = [
                make_violation(
                    "ANALYSIS_COMPILE_COMMAND_MISSING",
                    expected[name].relative_to(self.root).as_posix(),
                    0,
                    "production translation unit has no host compile command",
                )
                for name in missing
            ]
            result = check_result(
                "analysis.compile_database",
                "FAIL",
                f"compile database is missing {len(missing)} production translation units",
                violations,
            )
            return result, None, self._failed_analysis_identity(generated, result)

        analysis_dir = workspace / "analysis-db"
        analysis_dir.mkdir(parents=True, exist_ok=True)
        filtered_db = analysis_dir / "compile_commands.json"
        filtered_db.write_text(
            json.dumps([selected[name] for name in sorted(selected)], indent=2) + "\n",
            encoding="utf-8",
        )
        identity = self._analysis_database_identity(
            selected, source_db, workspace, generated, cmake
        )
        return (
            check_result(
                "analysis.compile_database",
                "PASS",
                f"selected {len(selected)} production C translation units from the host compile database",
            ),
            analysis_dir,
            identity,
        )

    @staticmethod
    def _analyzer_unavailable(name: str, rule: str, names: tuple[str, ...]) -> dict:
        accepted = ", ".join(names)
        return check_result(
            name, "FAIL", f"required analyzer was not found ({accepted})",
            [make_violation(rule, ".", 0, f"install one of: {accepted}")],
        )

    def check_clang_tidy(self, analysis_dir: Path | None) -> dict:
        executable, _name = self._find_tool(CLANG_TIDY_NAMES)
        if executable is None:
            return self._analyzer_unavailable(
                "static.clang-tidy", "CLANG_TIDY_MISSING", CLANG_TIDY_NAMES
            )
        if analysis_dir is None:
            return check_result(
                "static.clang-tidy", "FAIL", "clang-tidy could not run without a compile database",
                [make_violation(
                    "CLANG_TIDY_COMPILE_DB", ".", 0,
                    "production compile database preparation failed",
                )],
            )

        sources = [path for path in self.production_files if path.suffix.lower() == ".c"]
        layer_pattern = "|".join(re.escape(layer) for layer in PRODUCTION_LAYERS)
        header_filter = rf"^{re.escape(self.root.as_posix())}/(?:{layer_pattern})/"
        try:
            process = subprocess.run(
                [executable, "-p", str(analysis_dir), "--quiet",
                 f"--checks={CLANG_TIDY_CHECKS}", f"--header-filter={header_filter}",
                 *map(str, sources)],
                cwd=self.root,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )
        except OSError as error:
            return check_result(
                "static.clang-tidy", "FAIL", "clang-tidy execution failed",
                [make_violation("CLANG_TIDY_EXECUTION", ".", 0, str(error))],
            )

        diagnostic_re = re.compile(
            r"^(.*?):(\d+):(\d+): (warning|error): (.*?) \[([^]]+)]$"
        )
        violations: list[dict] = []
        for line in process.stdout.splitlines():
            match = diagnostic_re.match(line)
            if match is None:
                continue
            path_text, line_text, _column, _severity, message, check_name = match.groups()
            relative = self._production_diagnostic_path(path_text)
            if relative is None:
                continue
            violations.append(make_violation(
                f"CLANG_TIDY/{check_name}", relative, int(line_text), message, check_name,
            ))
        if process.returncode != 0 and not violations:
            violations.append(make_violation(
                "CLANG_TIDY_EXECUTION", ".", 0,
                "clang-tidy exited nonzero without a production diagnostic: "
                + "\n".join(process.stdout.splitlines()[-10:]),
            ))
        return check_result(
            "static.clang-tidy", "FAIL" if violations else "PASS",
            f"clang-tidy checked {len(sources)} production C translation units with "
            f"'{CLANG_TIDY_CHECKS}' and found {len(violations)} diagnostics",
            violations,
        )

    def check_cppcheck(self, analysis_dir: Path | None) -> dict:
        executable, _name = self._find_tool(("cppcheck",))
        if executable is None:
            return self._analyzer_unavailable(
                "static.cppcheck", "CPPCHECK_MISSING", ("cppcheck",)
            )
        if analysis_dir is None:
            return check_result(
                "static.cppcheck", "FAIL", "cppcheck could not run without a compile database",
                [make_violation(
                    "CPPCHECK_COMPILE_DB", ".", 0,
                    "production compile database preparation failed",
                )],
            )

        database = analysis_dir / "compile_commands.json"
        try:
            process = subprocess.run(
                [executable, f"--project={database}", f"--enable={CPPCHECK_ENABLE}",
                 "--std=c11", "--xml", "--xml-version=2", "--quiet"],
                cwd=self.root,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
        except OSError as error:
            return check_result(
                "static.cppcheck", "FAIL", "cppcheck execution failed",
                [make_violation("CPPCHECK_EXECUTION", ".", 0, str(error))],
            )

        violations: list[dict] = []
        try:
            xml_root = ET.fromstring(process.stderr)
        except ET.ParseError as error:
            violations.append(make_violation(
                "CPPCHECK_EXECUTION", ".", 0,
                f"cppcheck did not emit valid XML: {error}; "
                + "\n".join(process.stderr.splitlines()[-10:]),
            ))
        else:
            for error_node in xml_root.findall("./errors/error"):
                check_id = error_node.get("id", "unknown")
                message = error_node.get("verbose") or error_node.get("msg") or check_id
                location = None
                relative = None
                for candidate in error_node.findall("location"):
                    candidate_file = candidate.get("file")
                    if candidate_file is None:
                        continue
                    candidate_relative = self._production_diagnostic_path(candidate_file)
                    if candidate_relative is not None:
                        location = candidate
                        relative = candidate_relative
                        break
                if relative is None:
                    file0 = error_node.get("file0")
                    if file0 is not None:
                        relative = self._production_diagnostic_path(file0)
                    if relative is None and not error_node.findall("location"):
                        relative = "."
                    if relative is None:
                        continue
                line_text = location.get("line", "0") if location is not None else "0"
                line_number_value = int(line_text) if line_text.isdigit() else 0
                violations.append(make_violation(
                    f"CPPCHECK/{check_id}", relative, line_number_value, message, check_id,
                ))
        if process.returncode != 0 and not violations:
            violations.append(make_violation(
                "CPPCHECK_EXECUTION", ".", 0,
                f"cppcheck exited with status {process.returncode}",
            ))
        source_count = sum(path.suffix.lower() == ".c" for path in self.production_files)
        return check_result(
            "static.cppcheck", "FAIL" if violations else "PASS",
            f"cppcheck checked {source_count} production C translation units with "
            f"'{CPPCHECK_ENABLE}' and found {len(violations)} diagnostics",
            violations,
        )

    def check_external_analyzers(self) -> list[dict]:
        with tempfile.TemporaryDirectory(prefix="cod-quality-analysis-") as temporary:
            database_result, analysis_dir, _identity = self._prepare_analysis_database(
                Path(temporary)
            )
            return [
                database_result,
                self.check_clang_tidy(analysis_dir),
                self.check_cppcheck(analysis_dir),
            ]

    @staticmethod
    def _tool_identity(label: str, names: tuple[str, ...]) -> str:
        executable, _name = QualityRunner._find_tool(names)
        if executable is None:
            return f"{label}:MISSING"
        return QualityRunner._executable_identity(label, executable)

    def input_fingerprint(self, analysis_input: dict | None = None) -> str:
        if analysis_input is None:
            with tempfile.TemporaryDirectory(prefix="cod-quality-fingerprint-") as temporary:
                _result, _analysis_dir, analysis_input = self._prepare_analysis_database(
                    Path(temporary)
                )
        digest = hashlib.sha256()
        inputs = list(self.production_files)
        inputs.extend(self.test_header_files)
        inputs.extend((self.root / ".clang-format", Path(__file__).resolve()))
        inputs.extend((
            self.baseline_path,
            self.architecture_baseline_path,
            self.vendor_checksums_path,
        ))
        inputs.extend(self._vendor_package_files())
        tests_root = self.root / "tests"
        if tests_root.is_dir():
            inputs.extend(tests_root.rglob("CMakeLists.txt"))
        for path in sorted({path for path in inputs if path.is_file()}):
            try:
                relative = path.relative_to(self.root).as_posix()
            except ValueError:
                relative = path.as_posix()
            digest.update(relative.encode("utf-8") + b"\0")
            digest.update(hashlib.sha256(path.read_bytes()).digest())
        formatter = shutil.which("clang-format")
        formatter_identity = "clang-format:MISSING"
        if formatter is not None:
            version = subprocess.run(
                [formatter, "--version"], text=True, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, check=False,
            ).stdout.strip()
            formatter_identity = f"clang-format:{formatter}:{version}"
        digest.update(formatter_identity.encode("utf-8") + b"\0")
        analyzer_configuration = json.dumps({
            "version": ANALYZER_CONFIG_VERSION,
            "clang_tidy_checks": CLANG_TIDY_CHECKS,
            "cppcheck_enable": CPPCHECK_ENABLE,
            "analysis_cmake_args": ANALYSIS_CMAKE_ARGS,
            "scope": PRODUCTION_LAYERS,
        }, sort_keys=True)
        digest.update(analyzer_configuration.encode("utf-8") + b"\0")
        digest.update(self._tool_identity("clang-tidy", CLANG_TIDY_NAMES).encode("utf-8") + b"\0")
        digest.update(self._tool_identity("cppcheck", ("cppcheck",)).encode("utf-8") + b"\0")
        digest.update(
            json.dumps(analysis_input, sort_keys=True, separators=(",", ":")).encode("utf-8")
            + b"\0"
        )
        return digest.hexdigest()

    def run(self) -> dict:
        checks = self.check_architecture()
        checks.extend((
            self.check_vendor_checksums(),
            self.check_format(),
            self.check_float_rules(),
        ))
        with tempfile.TemporaryDirectory(prefix="cod-quality-analysis-") as temporary:
            database_result, analysis_dir, analysis_input = self._prepare_analysis_database(
                Path(temporary)
            )
            checks.extend((
                database_result,
                self.check_clang_tidy(analysis_dir),
                self.check_cppcheck(analysis_dir),
            ))
            overall = "FAIL" if any(item["status"] == "FAIL" for item in checks) else "PASS"
            return {
                "schema_version": SCHEMA_VERSION,
                "run_id": str(uuid.uuid4()),
                "generated_at": dt.datetime.now(dt.timezone.utc).isoformat(),
                "root": self.root.as_posix(),
                "analysis_input": analysis_input,
                "input_fingerprint": self.input_fingerprint(analysis_input),
                "pass_scope": "A PASS is valid only for this exact input_fingerprint; always rerun after changes.",
                "overall_status": overall,
                "checks": checks,
            }


def render_text(report: dict) -> str:
    lines = [
        f"QUALITY {report['overall_status']}",
        f"run_id: {report['run_id']}",
        f"input_fingerprint: {report['input_fingerprint']}",
    ]
    for check in report["checks"]:
        lines.append(f"[{check['status']}] {check['name']}: {check['summary']}")
        for violation in check["violations"]:
            location = violation["path"]
            if violation.get("line"):
                location += f":{violation['line']}"
            lines.append(f"  {location}: {violation['rule']}: {violation['message']}")
    return "\n".join(lines)


def verify_report(
    root: Path,
    baseline: Path,
    architecture_baseline: Path,
    vendor_checksums: Path,
    report_path: Path,
    compile_db: Path | None = None,
) -> int:
    try:
        report = json.loads(report_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        print(f"STALE/INVALID report: {error}", file=sys.stderr)
        return 1
    runner = QualityRunner(
        root, baseline, architecture_baseline, vendor_checksums, compile_db
    )
    with tempfile.TemporaryDirectory(prefix="cod-quality-verify-") as temporary:
        _result, _analysis_dir, analysis_input = runner._prepare_analysis_database(
            Path(temporary)
        )
        current = runner.input_fingerprint(analysis_input)
    valid = (
        report.get("schema_version") == SCHEMA_VERSION
        and report.get("overall_status") == "PASS"
        and report.get("analysis_input") == analysis_input
        and report.get("input_fingerprint") == current
    )
    print("REPORT VALID" if valid else "REPORT STALE/INVALID")
    print(f"current_input_fingerprint: {current}")
    print(f"report_input_fingerprint: {report.get('input_fingerprint', '<missing>')}")
    return 0 if valid else 1


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    default_root = Path(__file__).resolve().parents[3]
    parser.add_argument("--root", type=Path, default=default_root)
    parser.add_argument("--baseline", type=Path, default=Path(__file__).resolve().parent / "float_baseline.json")
    parser.add_argument(
        "--architecture-baseline", type=Path,
        default=Path(__file__).resolve().parent / "architecture_baseline.json",
    )
    parser.add_argument(
        "--vendor-checksums", type=Path,
        default=Path(__file__).resolve().parent / "vendor_checksums.sha256",
    )
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--verify-report", type=Path)
    parser.add_argument(
        "--compile-db", type=Path,
        help="existing compile_commands.json; otherwise a temporary host database is generated",
    )
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(argv)
    if args.verify_report is not None:
        return verify_report(
            args.root.resolve(),
            args.baseline.resolve(),
            args.architecture_baseline.resolve(),
            args.vendor_checksums.resolve(),
            args.verify_report.resolve(),
            args.compile_db.resolve() if args.compile_db is not None else None,
        )
    report = QualityRunner(
        args.root,
        args.baseline,
        args.architecture_baseline,
        args.vendor_checksums,
        args.compile_db,
    ).run()
    text = render_text(report)
    json_text = json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False)
    print(text)
    print("--- JSON ---")
    print(json_text)
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json_text + "\n", encoding="utf-8")
        print(f"JSON report written to {args.json_out}")
    return 0 if report["overall_status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
