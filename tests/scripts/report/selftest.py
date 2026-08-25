#!/usr/bin/env python3
"""End-to-end self-tests for the current report generator."""

from __future__ import annotations

import json
import os
import pathlib
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

GENERATOR = pathlib.Path(__file__).with_name("generate.py")
RUNNER = GENERATOR.parents[1] / "run" / "tests.sh"
EPOCH = "1787584765"


class ReportFixture:
    def __init__(self, root: pathlib.Path) -> None:
        self.root = root
        self.reports = root / "reports"
        self.reports.mkdir()
        (root / "CMakeCache.txt").write_text(
            "CMAKE_BUILD_TYPE:STRING=Debug\n"
            "CMAKE_C_COMPILER:FILEPATH=/usr/bin/cc\n"
            "CMAKE_GENERATOR:INTERNAL=Unix Makefiles\n",
            encoding="utf-8",
        )
        self.write_ctest({"alpha": "pass", "beta": "pass"})
        (root / "target-help.txt").write_text(
            "The following are valid targets:\n... all (default)\n... alpha\n... beta\n",
            encoding="utf-8",
        )
        self.write_json("quality.json", {"schema_version": 3, "overall_status": "PASS", "checks": []})
        self.write_json(
            "coverage.json",
            {
                "passed": True,
                "source_files": 2,
                "observed_source_files": 2,
                "totals": {"lines": {"covered": 2, "total": 2, "percent": 100.0}},
            },
        )
        self.write_json(
            "performance.json",
            {"schema_version": 1, "mode": "check", "status": "PASS", "checks": [], "scope": "host"},
        )
        (root / "sanitizer-status.txt").write_text("PASS: address sanitizer\n", encoding="utf-8")
        (root / "fuzz-status.txt").write_text("PASS: bounded campaigns\n", encoding="utf-8")
        self.write_json("cmock.json", {"passed": True, "updated": False, "domains": [{"status": "PASS"}]})

    def write_json(self, name: str, value: object) -> None:
        (self.root / name).write_text(json.dumps(value), encoding="utf-8")

    def write_ctest(self, results: dict[str, str]) -> None:
        tests = []
        suite = ET.Element("testsuite", tests=str(len(results)))
        for index, (name, result) in enumerate(results.items()):
            tests.append(
                {
                    "name": name,
                    "properties": [
                        {"name": "LABELS", "value": ["host", "even" if index % 2 == 0 else "odd"]}
                    ],
                }
            )
            case = ET.SubElement(suite, "testcase", name=name, status="run")
            if result == "fail":
                ET.SubElement(case, "failure", message="fixture failure")
            elif result == "skip":
                ET.SubElement(case, "skipped")
        self.write_json("ctest.json", {"kind": "ctestInfo", "version": {"major": 1, "minor": 0}, "tests": tests})
        ET.ElementTree(suite).write(self.reports / "ctest.xml", encoding="utf-8", xml_declaration=True)

    def command(self, *, all_gates: bool = True) -> list[str]:
        command = [
            sys.executable,
            str(GENERATOR),
            "--build-dir",
            str(self.root),
            "--ctest-json",
            str(self.root / "ctest.json"),
            "--target-help",
            str(self.root / "target-help.txt"),
            "--ctest-exit-status",
            "0",
        ]
        if all_gates:
            command.extend(
                [
                    "--quality",
                    str(self.root / "quality.json"),
                    "--coverage",
                    str(self.root / "coverage.json"),
                    "--performance",
                    str(self.root / "performance.json"),
                    "--sanitizer-status",
                    str(self.root / "sanitizer-status.txt"),
                    "--fuzz-status",
                    str(self.root / "fuzz-status.txt"),
                    "--cmock-json",
                    str(self.root / "cmock.json"),
                ]
            )
        return command

    def run(self, *, all_gates: bool = True) -> subprocess.CompletedProcess[str]:
        environment = os.environ.copy()
        environment["SOURCE_DATE_EPOCH"] = EPOCH
        return subprocess.run(self.command(all_gates=all_gates), text=True, capture_output=True, env=environment)

    def report(self) -> dict:
        return json.loads((self.reports / "current.json").read_text(encoding="utf-8"))


class GenerateReportTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(prefix="cod-report-selftest-", dir="/tmp")
        self.fixture = ReportFixture(pathlib.Path(self.temporary.name))

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_all_pass_and_markdown_json_are_consistent(self) -> None:
        result = self.fixture.run()
        self.assertEqual(0, result.returncode, result.stderr)
        report = self.fixture.report()
        markdown = (self.fixture.reports / "current.md").read_text(encoding="utf-8")
        self.assertEqual("PASS", report["overall_status"])
        self.assertEqual({"total": 2, "pass": 2, "fail": 0, "skip": 0, "incomplete": 0}, {key: report["gates"]["ctest"][key] for key in ("total", "pass", "fail", "skip", "incomplete")})
        self.assertIn(f"Overall status: **{report['overall_status']}**", markdown)
        self.assertIn("total **2**, pass **2**, fail **0**, skip **0**, incomplete **0**", markdown)
        for name, gate in report["gates"].items():
            self.assertIn(f"| `{name}` | **{gate['status']}** |", markdown)

    def test_ctest_failure_is_reported(self) -> None:
        self.fixture.write_ctest({"alpha": "pass", "beta": "fail"})
        result = self.fixture.run()
        self.assertEqual(0, result.returncode, result.stderr)
        report = self.fixture.report()
        self.assertEqual("FAIL", report["overall_status"])
        self.assertEqual("FAIL", report["gates"]["ctest"]["status"])
        self.assertEqual(1, report["gates"]["ctest"]["fail"])

    def test_nonzero_ctest_process_overrides_green_junit(self) -> None:
        command = self.fixture.command()
        status_index = command.index("--ctest-exit-status") + 1
        command[status_index] = "7"
        result = subprocess.run(command, text=True, capture_output=True)
        self.assertEqual(0, result.returncode, result.stderr)
        report = self.fixture.report()
        self.assertEqual("FAIL", report["overall_status"])
        self.assertEqual("FAIL", report["gates"]["ctest"]["status"])
        self.assertEqual(7, report["gates"]["ctest"]["process_exit_status"])
        self.assertEqual(2, report["gates"]["ctest"]["pass"])

    def test_runner_preserves_nonzero_ctest_status_with_green_junit(self) -> None:
        root = pathlib.Path(self.temporary.name)
        fake_bin = root / "fake-bin"
        build = root / "runner-build"
        fake_bin.mkdir()

        cmake = fake_bin / "cmake"
        cmake.write_text(
            "#!/bin/sh\n"
            "if [ \"$1\" = \"-S\" ]; then\n"
            "  while [ \"$#\" -gt 0 ]; do\n"
            "    if [ \"$1\" = \"-B\" ]; then shift; build=$1; break; fi\n"
            "    shift\n"
            "  done\n"
            "  mkdir -p \"$build\"\n"
            "  printf '%s\\n' 'CMAKE_BUILD_TYPE:STRING=Debug' "
            "'CMAKE_C_COMPILER:FILEPATH=/usr/bin/cc' "
            "'CMAKE_GENERATOR:INTERNAL=Unix Makefiles' > \"$build/CMakeCache.txt\"\n"
            "  exit 0\n"
            "fi\n"
            "case \" $* \" in\n"
            "  *' --target help '*) printf '%s\\n' '... alpha';;\n"
            "esac\n"
            "exit 0\n",
            encoding="utf-8",
        )
        ctest = fake_bin / "ctest"
        ctest.write_text(
            "#!/bin/sh\n"
            "case \" $* \" in\n"
            "  *' --show-only=json-v1 '*)\n"
            "    printf '%s\\n' '{\"kind\":\"ctestInfo\",\"version\":{\"major\":1,\"minor\":0},\"tests\":[{\"name\":\"alpha\",\"properties\":[{\"name\":\"LABELS\",\"value\":[\"host\"]}]}]}'\n"
            "    exit 0;;\n"
            "esac\n"
            "junit=\n"
            "while [ \"$#\" -gt 0 ]; do\n"
            "  if [ \"$1\" = \"--output-junit\" ]; then shift; junit=$1; break; fi\n"
            "  shift\n"
            "done\n"
            "mkdir -p \"$(dirname \"$junit\")\"\n"
            "printf '%s\\n' '<testsuite tests=\"1\"><testcase name=\"alpha\" status=\"run\"/></testsuite>' > \"$junit\"\n"
            "exit 7\n",
            encoding="utf-8",
        )
        cmake.chmod(0o755)
        ctest.chmod(0o755)
        environment = os.environ.copy()
        environment["PATH"] = f"{fake_bin}:{environment['PATH']}"
        result = subprocess.run(
            [str(RUNNER), str(build)], text=True, capture_output=True, env=environment
        )
        self.assertEqual(7, result.returncode, result.stdout + result.stderr)
        report = json.loads((build / "reports/current.json").read_text(encoding="utf-8"))
        self.assertEqual("FAIL", report["overall_status"])
        self.assertEqual("FAIL", report["gates"]["ctest"]["status"])
        self.assertEqual(7, report["gates"]["ctest"]["process_exit_status"])
        self.assertEqual(1, report["gates"]["ctest"]["pass"])
        self.assertEqual(0, report["gates"]["ctest"]["fail"])
        self.assertTrue((build / "test-status.txt").read_text().startswith("FAIL:"))

    def test_missing_optional_gates_are_not_run(self) -> None:
        for name in ("sanitizer-status.txt", "fuzz-status.txt", "cmock.json"):
            (self.fixture.root / name).unlink()
        result = self.fixture.run(all_gates=False)
        self.assertEqual(0, result.returncode, result.stderr)
        report = self.fixture.report()
        self.assertEqual("INCOMPLETE", report["overall_status"])
        for gate in ("quality", "coverage", "performance", "sanitizer", "fuzz", "cmock"):
            self.assertEqual("not_run", report["gates"][gate]["status"])

    def test_source_date_epoch_is_byte_deterministic(self) -> None:
        first = self.fixture.run()
        self.assertEqual(0, first.returncode, first.stderr)
        json_first = (self.fixture.reports / "current.json").read_bytes()
        markdown_first = (self.fixture.reports / "current.md").read_bytes()
        second = self.fixture.run()
        self.assertEqual(0, second.returncode, second.stderr)
        self.assertEqual(json_first, (self.fixture.reports / "current.json").read_bytes())
        self.assertEqual(markdown_first, (self.fixture.reports / "current.md").read_bytes())
        self.assertEqual("2026-08-24T15:19:25Z", self.fixture.report()["generated_at"])

    def test_malformed_input_fails_and_invalidates_prior_current_report(self) -> None:
        good = self.fixture.run()
        self.assertEqual(0, good.returncode, good.stderr)
        (self.fixture.root / "quality.json").write_text("{not json", encoding="utf-8")
        bad = self.fixture.run()
        self.assertEqual(2, bad.returncode)
        self.assertIn("malformed quality input", bad.stderr)
        self.assertFalse((self.fixture.reports / "current.json").exists())
        self.assertFalse((self.fixture.reports / "current.md").exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
