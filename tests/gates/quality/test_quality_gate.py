#!/usr/bin/env python3
"""Negative fixture tests for every quality-gate rule family."""

from __future__ import annotations

import hashlib
import os
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
from quality_gate import QualityRunner


class FixtureRepo:
    def __init__(self, root: Path) -> None:
        self.root = root
        (root / ".clang-format").write_text(
            "BasedOnStyle: LLVM\nIndentWidth: 4\nBreakBeforeBraces: Allman\n", encoding="utf-8"
        )
        for layer in ("01_application", "02_device", "03_platform", "04_impl", "06_utils"):
            directory = root / layer
            directory.mkdir(parents=True)
            (directory / "clean.c").write_text("int clean_symbol;\n", encoding="utf-8")
        self.compile_db = root / "compile_commands.json"
        compile_entries = []
        for source in sorted(root.glob("0[1-6]_*/clean.c")):
            compile_entries.append({
                "directory": root.as_posix(),
                "command": f"/usr/bin/cc -std=c11 -c {source} -o {source}.o",
                "file": source.as_posix(),
            })
        self.compile_db.write_text(
            json.dumps(compile_entries, indent=2) + "\n", encoding="utf-8"
        )
        self.baseline = root / "float_baseline.json"
        self.architecture_baseline = root / "architecture_baseline.json"
        self.vendor_checksums = root / "vendor_checksums.sha256"
        self.write_baseline([])
        self.write_architecture_baseline([])
        for package in ("05_vender/freertos", "05_vender/segger_rtt"):
            (root / package).mkdir(parents=True)
        self.vendor_checksums.write_text("", encoding="utf-8")
        self.write(
            "tests/CMakeLists.txt",
            "cmake_minimum_required(VERSION 3.16)\n"
            "project(quality_fixture C)\n"
            "file(GLOB_RECURSE PRODUCTION_SOURCES CONFIGURE_DEPENDS\n"
            "    ${CMAKE_CURRENT_LIST_DIR}/../01_application/*.c\n"
            "    ${CMAKE_CURRENT_LIST_DIR}/../02_device/*.c\n"
            "    ${CMAKE_CURRENT_LIST_DIR}/../03_platform/*.c\n"
            "    ${CMAKE_CURRENT_LIST_DIR}/../04_impl/*.c\n"
            "    ${CMAKE_CURRENT_LIST_DIR}/../06_utils/*.c)\n"
            "add_library(fixture_objects OBJECT ${PRODUCTION_SOURCES})\n",
        )

    def write(self, relative: str, content: str) -> Path:
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
        return path

    def write_baseline(self, entries: list[dict]) -> None:
        self.baseline.write_text(
            json.dumps({"schema_version": 1, "reviewed_violations": entries}, indent=2) + "\n",
            encoding="utf-8",
        )

    def write_architecture_baseline(self, entries: list[dict]) -> None:
        self.architecture_baseline.write_text(
            json.dumps({"schema_version": 1, "reviewed_violations": entries}, indent=2) + "\n",
            encoding="utf-8",
        )

    def write_vendor_baseline(self, paths: list[str]) -> None:
        lines = []
        for relative in paths:
            digest = hashlib.sha256((self.root / relative).read_bytes()).hexdigest()
            lines.append(f"{digest}  {relative}")
        self.vendor_checksums.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")

    def runner(self) -> QualityRunner:
        return QualityRunner(
            self.root,
            self.baseline,
            self.architecture_baseline,
            self.vendor_checksums,
            self.compile_db,
        )


class QualityGateNegativeFixtures(unittest.TestCase):
    def fixture(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        return FixtureRepo(Path(temporary.name))

    @staticmethod
    def check(runner: QualityRunner, name: str) -> dict:
        if name.startswith("architecture."):
            checks = runner.check_architecture()
        elif name == "vendor.checksums":
            checks = [runner.check_vendor_checksums()]
        elif name == "format.clang-format":
            checks = [runner.check_format()]
        elif name == "float.rules":
            checks = [runner.check_float_rules()]
        else:
            checks = runner.check_external_analyzers()
        return next(item for item in checks if item["name"] == name)

    def test_each_layer_direction_violation_fails(self) -> None:
        cases = (
            ("01_application/bad.c", '#include "04_impl/common/impl_x.h"\n'),
            ("02_device/bad.c", '#include "04_impl/common/impl_x.h"\n'),
            ("03_platform/bad.c", '#include "04_impl/bsp/stm32h7/x.h"\n'),
            ("04_impl/bad.c", '#include "03_platform/bsp/x.h"\n'),
            ("06_utils/bad.c", '#include "03_platform/bsp/x.h"\n'),
        )
        for path, content in cases:
            with self.subTest(path=path):
                repo = self.fixture()
                repo.write(path, content)
                result = self.check(repo.runner(), "architecture.includes")
                self.assertEqual("FAIL", result["status"])

    def test_application_may_directly_include_platform(self) -> None:
        repo = self.fixture()
        repo.write("01_application/direct_platform.c", '#include "03_platform/bsp/x.h"\n')
        result = self.check(repo.runner(), "architecture.includes")
        self.assertEqual("PASS", result["status"])

    def test_board_is_the_composition_exception(self) -> None:
        repo = self.fixture()
        repo.write(
            "01_application/board/compose.c",
            '#include "03_platform/bsp/x.h"\n#include "04_impl/common/x.h"\n'
            '#include "05_vender/package/vendor.h"\n',
        )
        result = self.check(repo.runner(), "architecture.includes")
        self.assertEqual("PASS", result["status"])

    def test_device_vendor_header_and_type_fail(self) -> None:
        repo = self.fixture()
        repo.write(
            "02_device/bad.c",
            '#include "stm32h7xx_hal.h"\nFDCAN_HandleTypeDef *handle;\n',
        )
        result = self.check(repo.runner(), "architecture.vendor_neutral")
        self.assertEqual("FAIL", result["status"])
        self.assertEqual(
            {"VENDOR_HEADER_OUTSIDE_IMPL", "VENDOR_TYPE_OUTSIDE_IMPL"},
            {item["rule"] for item in result["violations"]},
        )

    def test_platform_vendor_type_fails(self) -> None:
        repo = self.fixture()
        repo.write("03_platform/bad.h", "TaskHandle_t leaked_handle;\n")
        result = self.check(repo.runner(), "architecture.vendor_neutral")
        self.assertEqual("FAIL", result["status"])

    def test_test_mock_or_contract_header_in_production_fails(self) -> None:
        repo = self.fixture()
        repo.write("tests/unit/device/mocks/mock_bus.h", "#pragma once\n")
        repo.write("02_device/bad.c", '#include "mock_bus.h"\n')
        result = self.check(repo.runner(), "architecture.test_headers")
        self.assertEqual("FAIL", result["status"])

    def test_precise_architecture_baseline_rejects_new_and_stale_findings(self) -> None:
        repo = self.fixture()
        legacy = repo.write("06_utils/legacy.c", '#include "03_platform/bsp/x.h"\n')
        finding = self.check(repo.runner(), "architecture.includes")["violations"][0]
        repo.write_architecture_baseline([{
            key: finding[key]
            for key in ("id", "path", "rule", "token", "line_sha256", "occurrence")
        } | {"rationale": "fixture-reviewed architecture debt"}])
        self.assertEqual("PASS", self.check(repo.runner(), "architecture.includes")["status"])

        new_source = repo.write("06_utils/new_bad.c", '#include "04_impl/common/x.h"\n')
        new_result = self.check(repo.runner(), "architecture.includes")
        self.assertEqual("FAIL", new_result["status"])
        self.assertIn("ARCH_LAYER_INCLUDE", {item["rule"] for item in new_result["violations"]})

        new_source.unlink()
        legacy.write_text("int legacy_is_clean;\n", encoding="utf-8")
        stale = self.check(repo.runner(), "architecture.includes")
        self.assertEqual("FAIL", stale["status"])
        self.assertEqual("ARCH_BASELINE_STALE", stale["violations"][0]["rule"])

    def test_vendor_checksum_tamper_fails(self) -> None:
        repo = self.fixture()
        path = "05_vender/freertos/blob.bin"
        repo.write(path, "original\n")
        repo.write_vendor_baseline([path])
        repo.write(path, "modified\n")
        result = self.check(repo.runner(), "vendor.checksums")
        self.assertEqual("FAIL", result["status"])
        self.assertEqual("VENDOR_CHECKSUM_HASH", result["violations"][0]["rule"])

    def test_vendor_unlisted_file_fails(self) -> None:
        repo = self.fixture()
        repo.write("05_vender/segger_rtt/new.bin", "unreviewed\n")
        result = self.check(repo.runner(), "vendor.checksums")
        self.assertEqual("FAIL", result["status"])
        self.assertEqual("VENDOR_CHECKSUM_UNLISTED", result["violations"][0]["rule"])

    def test_vendor_checksum_baseline_is_required(self) -> None:
        repo = self.fixture()
        repo.vendor_checksums.unlink()
        result = self.check(repo.runner(), "vendor.checksums")
        self.assertEqual("FAIL", result["status"])
        self.assertEqual("VENDOR_CHECKSUM_BASELINE_MISSING", result["violations"][0]["rule"])

    def test_vendor_checksum_rejects_escape_duplicate_and_missing(self) -> None:
        cases = (
            ("0" * 64 + "  05_vender/freertos/../segger_rtt/blob.bin\n",
             "VENDOR_CHECKSUM_PATH"),
            (None, "VENDOR_CHECKSUM_DUPLICATE"),
            ("0" * 64 + "  05_vender/freertos/missing.bin\n",
             "VENDOR_CHECKSUM_MISSING"),
        )
        for content, expected_rule in cases:
            with self.subTest(rule=expected_rule):
                repo = self.fixture()
                if expected_rule == "VENDOR_CHECKSUM_DUPLICATE":
                    path = "05_vender/freertos/blob.bin"
                    repo.write(path, "blob\n")
                    digest = hashlib.sha256((repo.root / path).read_bytes()).hexdigest()
                    content = f"{digest}  {path}\n{digest}  {path}\n"
                repo.vendor_checksums.write_text(content or "", encoding="utf-8")
                result = self.check(repo.runner(), "vendor.checksums")
                self.assertEqual("FAIL", result["status"])
                self.assertIn(expected_rule, {item["rule"] for item in result["violations"]})

    @unittest.skipUnless(shutil.which("clang-format"), "clang-format unavailable")
    def test_unformatted_production_file_fails(self) -> None:
        repo = self.fixture()
        repo.write("06_utils/bad.c", "int  badly_formatted( void ){return 1;}\n")
        result = self.check(repo.runner(), "format.clang-format")
        self.assertEqual("FAIL", result["status"])

    def test_float_literal_and_math_variant_fail_as_new_findings(self) -> None:
        repo = self.fixture()
        repo.write("06_utils/bad.c", "float bad(float x)\n{\n    return 1.0 + sqrt(x);\n}\n")
        result = self.check(repo.runner(), "float.rules")
        self.assertEqual("FAIL", result["status"])
        self.assertEqual(
            {"FLOAT_LITERAL_SUFFIX", "FLOAT_MATH_VARIANT"},
            {item["rule"] for item in result["violations"]},
        )

    def test_precise_float_baseline_allows_only_reviewed_finding_and_rejects_stale(self) -> None:
        repo = self.fixture()
        source = repo.write("06_utils/legacy.c", "float legacy = 1.0;\n")
        runner = repo.runner()
        finding = runner.scan_float_violations()[0]
        repo.write_baseline([{
            "id": finding["id"],
            "path": finding["path"],
            "rule": finding["rule"],
            "token": finding["token"],
            "line_sha256": finding["line_sha256"],
            "rationale": "fixture-reviewed legacy violation",
        }])
        self.assertEqual("PASS", self.check(repo.runner(), "float.rules")["status"])
        source.write_text("float legacy = 1.0f;\n", encoding="utf-8")
        stale = self.check(repo.runner(), "float.rules")
        self.assertEqual("FAIL", stale["status"])
        self.assertEqual("FLOAT_BASELINE_STALE", stale["violations"][0]["rule"])

    def test_missing_production_compile_command_fails(self) -> None:
        repo = self.fixture()
        entries = json.loads(repo.compile_db.read_text(encoding="utf-8"))
        repo.compile_db.write_text(json.dumps(entries[:-1]) + "\n", encoding="utf-8")
        result = self.check(repo.runner(), "analysis.compile_database")
        self.assertEqual("FAIL", result["status"])
        self.assertEqual(
            "ANALYSIS_COMPILE_COMMAND_MISSING", result["violations"][0]["rule"]
        )

    def test_missing_analyzers_are_hard_failures(self) -> None:
        repo = self.fixture()
        real_which = shutil.which

        def without_analyzers(name: str):
            if name in {"clang-tidy", "clang-tidy-18", "cppcheck"}:
                return None
            return real_which(name)

        with mock.patch("quality_gate.shutil.which", side_effect=without_analyzers):
            checks = repo.runner().check_external_analyzers()
        tidy = next(item for item in checks if item["name"] == "static.clang-tidy")
        cppcheck = next(item for item in checks if item["name"] == "static.cppcheck")
        self.assertEqual("FAIL", tidy["status"])
        self.assertEqual("CLANG_TIDY_MISSING", tidy["violations"][0]["rule"])
        self.assertEqual("FAIL", cppcheck["status"])
        self.assertEqual("CPPCHECK_MISSING", cppcheck["violations"][0]["rule"])

    @unittest.skipUnless(
        shutil.which("clang-tidy") or shutil.which("clang-tidy-18"),
        "clang-tidy unavailable",
    )
    def test_real_clang_tidy_diagnostic_is_structured_failure(self) -> None:
        repo = self.fixture()
        repo.write(
            "06_utils/clean.c",
            "int bad(void)\n{\n    int *pointer = 0;\n    return *pointer;\n}\n",
        )
        result = self.check(repo.runner(), "static.clang-tidy")
        self.assertEqual("FAIL", result["status"])
        self.assertIn(
            "CLANG_TIDY/clang-analyzer-core.NullDereference",
            {item["rule"] for item in result["violations"]},
        )
        self.assertTrue(all("path" in item and "line" in item for item in result["violations"]))

    @unittest.skipUnless(shutil.which("cppcheck"), "cppcheck unavailable")
    def test_real_cppcheck_diagnostic_is_structured_failure(self) -> None:
        repo = self.fixture()
        repo.write(
            "06_utils/clean.c",
            "int bad(void)\n{\n    int *pointer = 0;\n    return *pointer;\n}\n",
        )
        result = self.check(repo.runner(), "static.cppcheck")
        self.assertEqual("FAIL", result["status"])
        self.assertIn(
            "CPPCHECK/nullPointer", {item["rule"] for item in result["violations"]}
        )
        self.assertTrue(all("path" in item and "line" in item for item in result["violations"]))

    @unittest.skipUnless(
        all(
            shutil.which(tool)
            for tool in ("cmake", "gcc", "clang", "clang-format", "cppcheck")
        ) and (shutil.which("clang-tidy") or shutil.which("clang-tidy-18")),
        "automatic compile-database test tools unavailable",
    )
    def test_auto_compile_database_compiler_change_invalidates_pass(self) -> None:
        repo = self.fixture()
        report = repo.root / "auto-report.json"
        gate = Path(__file__).resolve().parent / "quality_gate.py"
        common = [
            sys.executable,
            str(gate),
            "--root",
            str(repo.root),
            "--baseline",
            str(repo.baseline),
            "--architecture-baseline",
            str(repo.architecture_baseline),
            "--vendor-checksums",
            str(repo.vendor_checksums),
        ]
        gcc_environment = dict(os.environ, CC=shutil.which("gcc"))
        first = subprocess.run(
            [*common, "--json-out", str(report)],
            env=gcc_environment,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        self.assertEqual(0, first.returncode, first.stdout + first.stderr)
        saved = json.loads(report.read_text(encoding="utf-8"))
        self.assertEqual("generated", saved["analysis_input"]["mode"])
        self.assertIn("normalized_compile_database_sha256", saved["analysis_input"])
        self.assertTrue(saved["analysis_input"]["c_compilers"])

        clang_environment = dict(os.environ, CC=shutil.which("clang"))
        verify = subprocess.run(
            [*common, "--verify-report", str(report)],
            env=clang_environment,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        self.assertNotEqual(0, verify.returncode, verify.stdout + verify.stderr)
        current_line = next(
            line
            for line in verify.stdout.splitlines()
            if line.startswith("current_input_fingerprint: ")
        )
        current = current_line.split(": ", 1)[1]
        self.assertNotEqual(saved["input_fingerprint"], current)

    @unittest.skipUnless(shutil.which("clang-format"), "clang-format unavailable")
    def test_failure_is_nonzero_and_old_pass_report_becomes_invalid(self) -> None:
        repo = self.fixture()
        report = repo.root / "report.json"
        gate = Path(__file__).resolve().parent / "quality_gate.py"
        command = [
            sys.executable, str(gate), "--root", str(repo.root), "--baseline", str(repo.baseline),
            "--architecture-baseline", str(repo.architecture_baseline),
            "--vendor-checksums", str(repo.vendor_checksums),
            "--compile-db", str(repo.compile_db), "--json-out", str(report),
        ]
        first = subprocess.run(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, check=False)
        self.assertEqual(0, first.returncode)
        repo.write("06_utils/new_bad.c", '#include "03_platform/bsp/x.h"\n')
        verify = subprocess.run(
            [sys.executable, str(gate), "--root", str(repo.root), "--baseline", str(repo.baseline),
             "--architecture-baseline", str(repo.architecture_baseline),
             "--vendor-checksums", str(repo.vendor_checksums),
             "--compile-db", str(repo.compile_db), "--verify-report", str(report)],
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, check=False,
        )
        second = subprocess.run(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, check=False)
        self.assertNotEqual(0, verify.returncode)
        self.assertNotEqual(0, second.returncode)


if __name__ == "__main__":
    unittest.main(verbosity=2)
