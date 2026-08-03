import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
BOUNDARY_CHECKER = (
    REPOSITORY_ROOT / "scripts" / "check_colcon_build_boundary.py"
)
REPORTER = REPOSITORY_ROOT / "scripts" / "report_test_results.py"
VERIFY_SCRIPT = REPOSITORY_ROOT / "scripts" / "verify.sh"
CI_WORKFLOW = REPOSITORY_ROOT / ".github" / "workflows" / "ci.yml"
MOTION_GATE_NODE_CRITICAL_CASES = (
    (
        "voice_nav_mission.MotionGateNodeTest",
        (
            "test_journal_parameters_are_declared_read_only_and_"
            "default_off"
        ),
    ),
    (
        "voice_nav_mission.MotionGateNodeTest",
        "test_steady_fail_closed_protocol_without_clock",
    ),
    (
        "voice_nav_mission.MotionGateNodeShutdownTest",
        "test_motion_gate_exits_cleanly",
    ),
)
MOTION_GATE_NODE_JOURNAL_CRITICAL_CASES = (
    (
        "voice_nav_mission.MotionGateNodeJournalTest",
        "test_partial_configuration_exits_without_writer_claim",
    ),
    (
        "voice_nav_mission.MotionGateNodeJournalTest",
        (
            "test_full_configuration_journals_zero_output_"
            "and_survives_exit"
        ),
    ),
    (
        "voice_nav_mission.MotionGateNodeJournalShutdownTest",
        "test_exit_codes_match_configuration_contract",
    ),
)
AUTHORITY_PROCESS_DEATH_CRITICAL_CASES = (
    (
        "voice_nav_sim.AuthorityProcessDeathTest",
        "test_exact_authority_sigkill_expires_gate_to_zero",
    ),
    (
        "voice_nav_sim.AuthorityProcessDeathShutdownTest",
        "test_exact_exit_ledger_is_complete",
    ),
)
FAULT_PRODUCER_PAIR_CRITICAL_CASES = (
    (
        "voice_nav_sim.FaultProducerPairTest",
        "test_independent_helpers_arm_gate_without_parent_control",
    ),
    (
        "voice_nav_sim.FaultProducerPairShutdownTest",
        "test_all_fixture_processes_exit_cleanly",
    ),
)
SIMULATION_CONTROL_CRITICAL_CASES = (
    (
        "voice_nav_sim.LaunchStartupPolicyTest",
        "test_startup_handler_stops_after_failed_stage",
    ),
    (
        "voice_nav_sim.SimulationControlTest",
        "test_stamped_drive_odometry_tf_and_consumer_timeout",
    ),
    (
        "voice_nav_sim.SimulationControlShutdownTest",
        "test_all_launch_managed_processes_exit_cleanly",
    ),
)
SIMULATION_INTERFACES_CRITICAL_CASES = (
    (
        "voice_nav_sim.SimulationInterfacesTest",
        "test_perception_odom_tf_and_ownership_contract",
    ),
    (
        "voice_nav_sim.SimulationInterfacesShutdownTest",
        "test_all_launch_managed_processes_exit_cleanly",
    ),
)
TF_OWNERSHIP_CONFLICT_CRITICAL_CASES = (
    (
        "voice_nav_sim.TfOwnershipConflictTest",
        "test_normal_audit_rejects_and_sentinel_proves_the_conflict",
    ),
)
SIM_CRITICAL_FILES = {
    "test_test_authority_process_death.py.xunit.xml": (
        AUTHORITY_PROCESS_DEATH_CRITICAL_CASES
    ),
    "test_test_fault_producer_pair.py.xunit.xml": (
        FAULT_PRODUCER_PAIR_CRITICAL_CASES
    ),
    "test_test_simulation_control.py.xunit.xml": (
        SIMULATION_CONTROL_CRITICAL_CASES
    ),
    "test_test_simulation_interfaces.py.xunit.xml": (
        SIMULATION_INTERFACES_CRITICAL_CASES
    ),
    "test_test_tf_ownership_conflict.py.xunit.xml": (
        TF_OWNERSHIP_CONFLICT_CRITICAL_CASES
    ),
}


def write_xunit(path: Path, *, tests: int, skipped: int = 0) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        (
            f'<testsuite name="fixture" tests="{tests}" errors="0" '
            f'failures="0" skipped="{skipped}"></testsuite>\n'
        ),
        encoding="utf-8",
    )


def write_launch_xunit(
    path: Path,
    cases: tuple[tuple[str, str], ...],
    *,
    skipped_case: tuple[str, str] | None = None,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    case_xml = []
    for classname, name in cases:
        skipped = (
            "<skipped message=\"disabled\" />"
            if (classname, name) == skipped_case
            else ""
        )
        case_xml.append(
            f'<testcase classname="{classname}" name="{name}">'
            f"{skipped}</testcase>"
        )
    skipped_count = 1 if skipped_case is not None else 0
    path.write_text(
        (
            f'<testsuite name="launch" tests="{len(cases)}" errors="0" '
            f'failures="0" skipped="{skipped_count}">'
            f'{"".join(case_xml)}</testsuite>\n'
        ),
        encoding="utf-8",
    )


def write_sim_critical_inventory(
    results: Path,
    *,
    omit: frozenset[str] = frozenset(),
    skipped: tuple[str, tuple[str, str]] | None = None,
) -> None:
    for filename, cases in SIM_CRITICAL_FILES.items():
        if filename in omit:
            continue
        skipped_case = None
        if skipped is not None and skipped[0] == filename:
            skipped_case = skipped[1]
        write_launch_xunit(
            results / filename,
            cases,
            skipped_case=skipped_case,
        )


def write_mission_launch_evidence(results: Path) -> None:
    """Write the complete safety-critical MotionGate launch inventory."""
    write_launch_xunit(
        results / "test_test_motion_gate_node.py.xunit.xml",
        MOTION_GATE_NODE_CRITICAL_CASES,
    )
    write_launch_xunit(
        results / "test_test_motion_gate_node_journal.py.xunit.xml",
        MOTION_GATE_NODE_JOURNAL_CRITICAL_CASES,
    )


def write_ctest(path: Path, *, status: str = "passed") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        (
            '<Site><Testing><Test Status="'
            f'{status}"><Name>fixture</Name></Test></Testing></Site>\n'
        ),
        encoding="utf-8",
    )


def write_package_manifest(path: Path, *, package_name: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        (
            '<?xml version="1.0"?>\n'
            '<package format="3">\n'
            f"  <name>{package_name}</name>\n"
            "  <version>0.0.0</version>\n"
            "  <description>fixture</description>\n"
            "  <maintainer email=\"fixture@example.com\">Fixture</maintainer>\n"
            "  <license>Apache-2.0</license>\n"
            "</package>\n"
        ),
        encoding="utf-8",
    )


class ScopedTestResultsTest(unittest.TestCase):
    def run_boundary_check(
        self,
        build_base: Path,
        *packages: str,
    ) -> subprocess.CompletedProcess[str]:
        command = [
            sys.executable,
            str(BOUNDARY_CHECKER),
            "--build-base",
            str(build_base),
        ]
        for package in packages:
            command.extend(("--package", package))
        return subprocess.run(
            command,
            cwd=REPOSITORY_ROOT,
            check=False,
            capture_output=True,
            text=True,
        )

    def run_reporter(
        self,
        build_base: Path,
        *packages: str,
        clear: bool = False,
    ) -> subprocess.CompletedProcess[str]:
        command = [
            sys.executable,
            str(REPORTER),
            "--build-base",
            str(build_base),
        ]
        for package in packages:
            command.extend(("--package", package))
        if clear:
            command.append("--clear")
        return subprocess.run(
            command,
            cwd=REPOSITORY_ROOT,
            check=False,
            capture_output=True,
            text=True,
        )

    def test_report_excludes_stale_sibling_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            write_xunit(
                build_base / "voice_nav_audio" / "current.xml",
                tests=3,
            )
            write_xunit(
                build_base
                / "voice_nav_audio.stale-l0009"
                / "stale.xml",
                tests=74,
                skipped=4,
            )

            completed = self.run_reporter(
                build_base,
                "voice_nav_audio",
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn(
                "Summary: 3 tests, 0 errors, 0 failures, 0 skipped",
                completed.stdout,
            )
            self.assertNotIn("74 tests", completed.stdout)

    def test_report_rejects_unallowlisted_skipped_result(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            skipped_case = (
                "voice_nav_audio.FunctionalTest",
                "test_disabled",
            )
            write_launch_xunit(
                build_base / "voice_nav_audio" / "functional.xunit.xml",
                (skipped_case,),
                skipped_case=skipped_case,
            )

            completed = self.run_reporter(
                build_base,
                "voice_nav_audio",
            )

            self.assertEqual(completed.returncode, 2, completed.stdout)
            self.assertIn(
                "skipped tests are not allowlisted",
                completed.stderr,
            )

    def test_report_allows_official_cppcheck_skip(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            skipped_case = (
                "voice_nav_audio.cppcheck",
                "src/example.cpp",
            )
            write_launch_xunit(
                build_base
                / "voice_nav_audio"
                / "test_results"
                / "voice_nav_audio"
                / "cppcheck.xunit.xml",
                (skipped_case,),
                skipped_case=skipped_case,
            )

            completed = self.run_reporter(
                build_base,
                "voice_nav_audio",
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("1 skipped", completed.stdout)

    def test_report_requires_complete_critical_launch_inventory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            result = (
                build_base
                / "voice_nav_bringup"
                / "test_results"
                / "voice_nav_bringup"
                / "test_test_motion_gate_product.py.xunit.xml"
            )
            write_launch_xunit(
                result,
                (
                    (
                        "voice_nav_bringup.MotionGateProductShutdownTest",
                        "test_all_launch_managed_processes_exit_cleanly",
                    ),
                ),
            )

            completed = self.run_reporter(
                build_base,
                "voice_nav_bringup",
            )

            self.assertEqual(completed.returncode, 2)
            self.assertIn("critical launch evidence", completed.stderr)

    def test_report_rejects_skipped_critical_launch_case(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            result = (
                build_base
                / "voice_nav_bringup"
                / "test_results"
                / "voice_nav_bringup"
                / "test_test_motion_gate_product.py.xunit.xml"
            )
            active_case = (
                "voice_nav_bringup.MotionGateProductTest",
                "test_motion_gate_product_contract",
            )
            write_launch_xunit(
                result,
                (
                    active_case,
                    (
                        "voice_nav_bringup.MotionGateProductShutdownTest",
                        "test_all_launch_managed_processes_exit_cleanly",
                    ),
                ),
                skipped_case=active_case,
            )

            completed = self.run_reporter(
                build_base,
                "voice_nav_bringup",
            )

            self.assertEqual(completed.returncode, 2)
            self.assertIn("critical launch evidence", completed.stderr)

    def test_report_accepts_complete_critical_launch_inventory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            result = (
                build_base
                / "voice_nav_bringup"
                / "test_results"
                / "voice_nav_bringup"
                / "test_test_motion_gate_product.py.xunit.xml"
            )
            write_launch_xunit(
                result,
                (
                    (
                        "voice_nav_bringup.MotionGateProductTest",
                        "test_motion_gate_product_contract",
                    ),
                    (
                        "voice_nav_bringup.MotionGateProductShutdownTest",
                        "test_all_launch_managed_processes_exit_cleanly",
                    ),
                ),
            )

            completed = self.run_reporter(
                build_base,
                "voice_nav_bringup",
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("Summary: 2 tests", completed.stdout)

    def test_report_requires_motion_gate_node_launch_inventory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            result = (
                build_base
                / "voice_nav_mission"
                / "test_results"
                / "voice_nav_mission"
                / "test_test_motion_gate_node.py.xunit.xml"
            )
            write_launch_xunit(
                result,
                (("voice_nav_mission.UnrelatedTest", "test_unrelated"),),
            )

            completed = self.run_reporter(
                build_base,
                "voice_nav_mission",
            )

            self.assertEqual(completed.returncode, 2, completed.stdout)
            self.assertIn("critical launch evidence", completed.stderr)

    def test_report_requires_motion_gate_node_journal_inventory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            results = (
                build_base
                / "voice_nav_mission"
                / "test_results"
                / "voice_nav_mission"
            )
            write_launch_xunit(
                results / "test_test_motion_gate_node.py.xunit.xml",
                MOTION_GATE_NODE_CRITICAL_CASES,
            )
            write_launch_xunit(
                (
                    results
                    / "test_test_motion_gate_node_journal.py.xunit.xml"
                ),
                (("voice_nav_mission.UnrelatedTest", "test_unrelated"),),
            )

            completed = self.run_reporter(
                build_base,
                "voice_nav_mission",
            )

            self.assertEqual(completed.returncode, 2, completed.stdout)
            self.assertIn("critical launch evidence", completed.stderr)

    def test_report_requires_tf_conflict_launch_inventory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            results = (
                build_base
                / "voice_nav_sim"
                / "test_results"
                / "voice_nav_sim"
            )
            write_sim_critical_inventory(
                results,
                omit=frozenset(
                    {"test_test_tf_ownership_conflict.py.xunit.xml"}
                ),
            )

            completed = self.run_reporter(build_base, "voice_nav_sim")

            self.assertEqual(completed.returncode, 2, completed.stdout)
            self.assertIn("critical launch evidence", completed.stderr)

    def test_report_requires_simulation_startup_policy_inventory(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            results = (
                build_base
                / "voice_nav_sim"
                / "test_results"
                / "voice_nav_sim"
            )
            write_sim_critical_inventory(results)
            write_launch_xunit(
                results / "test_test_simulation_control.py.xunit.xml",
                (
                    (
                        "voice_nav_sim.SimulationControlTest",
                        "test_stamped_drive_odometry_tf_and_consumer_timeout",
                    ),
                    (
                        "voice_nav_sim.SimulationControlShutdownTest",
                        "test_all_launch_managed_processes_exit_cleanly",
                    ),
                ),
            )

            completed = self.run_reporter(build_base, "voice_nav_sim")

            self.assertEqual(completed.returncode, 2, completed.stdout)
            self.assertIn("inventory is incomplete", completed.stderr)

    def test_report_requires_fault_producer_pair_launch_inventory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            results = (
                build_base
                / "voice_nav_sim"
                / "test_results"
                / "voice_nav_sim"
            )
            write_sim_critical_inventory(
                results,
                omit=frozenset(
                    {"test_test_fault_producer_pair.py.xunit.xml"}
                ),
            )

            completed = self.run_reporter(build_base, "voice_nav_sim")

            self.assertEqual(completed.returncode, 2, completed.stdout)
            self.assertIn("critical launch evidence", completed.stderr)

    def test_report_requires_authority_process_death_inventory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            results = (
                build_base
                / "voice_nav_sim"
                / "test_results"
                / "voice_nav_sim"
            )
            write_sim_critical_inventory(
                results,
                omit=frozenset(
                    {"test_test_authority_process_death.py.xunit.xml"}
                ),
            )

            completed = self.run_reporter(build_base, "voice_nav_sim")

            self.assertEqual(completed.returncode, 2, completed.stdout)
            self.assertIn("critical launch evidence", completed.stderr)

    def test_report_rejects_skipped_authority_process_death_case(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            results = (
                build_base
                / "voice_nav_sim"
                / "test_results"
                / "voice_nav_sim"
            )
            write_sim_critical_inventory(
                results,
                skipped=(
                    "test_test_authority_process_death.py.xunit.xml",
                    AUTHORITY_PROCESS_DEATH_CRITICAL_CASES[0],
                ),
            )

            completed = self.run_reporter(build_base, "voice_nav_sim")

            self.assertEqual(completed.returncode, 2, completed.stdout)
            self.assertIn("critical launch evidence", completed.stderr)

    def test_report_rejects_skipped_fault_producer_pair_case(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            results = (
                build_base
                / "voice_nav_sim"
                / "test_results"
                / "voice_nav_sim"
            )
            write_sim_critical_inventory(
                results,
                skipped=(
                    "test_test_fault_producer_pair.py.xunit.xml",
                    FAULT_PRODUCER_PAIR_CRITICAL_CASES[0],
                ),
            )

            completed = self.run_reporter(build_base, "voice_nav_sim")

            self.assertEqual(completed.returncode, 2, completed.stdout)
            self.assertIn("critical launch evidence", completed.stderr)

    def test_report_rejects_inconsistent_critical_launch_inventory(
        self,
    ) -> None:
        critical_cases = (
            '<testcase classname="voice_nav_bringup.MotionGateProductTest" '
            'name="test_motion_gate_product_contract" />'
            '<testcase classname="voice_nav_bringup.'
            'MotionGateProductShutdownTest" '
            'name="test_all_launch_managed_processes_exit_cleanly" />'
        )
        variants = {
            "declared zero tests": (
                '<testsuite name="launch" tests="0" errors="0" '
                'failures="0" skipped="0">'
                f"{critical_cases}</testsuite>"
            ),
            "required cases outside suite": (
                '<testsuites tests="2" errors="0" failures="0">'
                '<testsuite name="launch" tests="0" errors="0" '
                'failures="0" skipped="0"></testsuite>'
                f"{critical_cases}</testsuites>"
            ),
            "failed aggregate root": (
                '<testsuites tests="2" errors="0" failures="1">'
                '<testsuite name="launch" tests="2" errors="0" '
                'failures="0" skipped="0">'
                f"{critical_cases}</testsuite></testsuites>"
            ),
        }

        for label, xml in variants.items():
            with self.subTest(label=label):
                with tempfile.TemporaryDirectory() as temporary_directory:
                    build_base = Path(temporary_directory) / "build"
                    result = (
                        build_base
                        / "voice_nav_bringup"
                        / "test_results"
                        / "voice_nav_bringup"
                        / "test_test_motion_gate_product.py.xunit.xml"
                    )
                    result.parent.mkdir(parents=True)
                    result.write_text(xml, encoding="utf-8")

                    completed = self.run_reporter(
                        build_base,
                        "voice_nav_bringup",
                    )

                    self.assertEqual(
                        completed.returncode,
                        2,
                        completed.stdout,
                    )
                    self.assertIn(
                        "critical launch evidence",
                        completed.stderr,
                    )

    def test_clear_removes_only_selected_package_results(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            selected_result = (
                build_base / "voice_nav_mission" / "current.xml"
            )
            stale_result = (
                build_base
                / "voice_nav_mission.stale-l0009"
                / "stale.xml"
            )
            write_xunit(selected_result, tests=3)
            write_xunit(stale_result, tests=74)

            completed = self.run_reporter(
                build_base,
                "voice_nav_mission",
                clear=True,
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertFalse(selected_result.exists())
            self.assertTrue(stale_result.exists())
            self.assertIn(
                "Cleared 1 selected package test-result file.",
                completed.stdout,
            )

    def test_package_name_cannot_escape_build_base(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            build_base.mkdir()

            completed = self.run_reporter(build_base, "../outside")

            self.assertEqual(completed.returncode, 2)
            self.assertIn("invalid package name", completed.stderr)

    def test_report_rejects_selected_package_without_results(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            (build_base / "voice_nav_mission").mkdir(parents=True)

            completed = self.run_reporter(
                build_base,
                "voice_nav_mission",
            )

            self.assertEqual(completed.returncode, 2)
            self.assertIn("no test results", completed.stderr)

    def test_report_rejects_symlinked_package_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            real_package = build_base / "real_package"
            real_package.mkdir(parents=True)
            (build_base / "voice_nav_mission").symlink_to(
                real_package,
                target_is_directory=True,
            )

            completed = self.run_reporter(
                build_base,
                "voice_nav_mission",
            )

            self.assertEqual(completed.returncode, 2)
            self.assertIn("non-symlinked child", completed.stderr)

    def test_report_rejects_ctest_tag_path_traversal(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            selected_testing = build_base / "voice_nav_mission" / "Testing"
            selected_testing.mkdir(parents=True)
            (selected_testing / "TAG").write_text(
                "../../voice_nav_sim/Testing/20260731-0000\n",
                encoding="utf-8",
            )
            write_ctest(
                build_base
                / "voice_nav_sim"
                / "Testing"
                / "20260731-0000"
                / "Test.xml"
            )

            completed = self.run_reporter(
                build_base,
                "voice_nav_mission",
            )

            self.assertEqual(completed.returncode, 2)
            self.assertIn("unsafe CTest TAG entry", completed.stderr)

    def test_report_rejects_xml_symlink_to_sibling_package(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            sibling_result = (
                build_base / "voice_nav_sim" / "external.xunit.xml"
            )
            write_xunit(sibling_result, tests=9)
            selected_package = build_base / "voice_nav_mission"
            selected_package.mkdir()
            (selected_package / "borrowed.xunit.xml").symlink_to(
                sibling_result,
            )

            completed = self.run_reporter(
                build_base,
                "voice_nav_mission",
            )

            self.assertEqual(completed.returncode, 2)
            self.assertIn("result path contains symbolic link", completed.stderr)
            self.assertNotIn("Summary: 9 tests", completed.stdout)

    def test_report_rejects_symlinked_test_results_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace = Path(temporary_directory)
            build_base = workspace / "build"
            selected_package = build_base / "voice_nav_mission"
            write_xunit(selected_package / "good.xunit.xml", tests=1)
            external_results = workspace / "external-results"
            write_xunit(
                external_results / "hidden.xunit.xml",
                tests=9,
            )
            (selected_package / "test_results").symlink_to(
                external_results,
                target_is_directory=True,
            )

            completed = self.run_reporter(
                build_base,
                "voice_nav_mission",
            )

            self.assertEqual(completed.returncode, 2)
            self.assertIn("result path contains symbolic link", completed.stderr)
            self.assertNotIn("Summary: 1 test", completed.stdout)

    def test_report_rejects_unapproved_symlinked_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace = Path(temporary_directory)
            build_base = workspace / "build"
            selected_package = build_base / "voice_nav_mission"
            write_xunit(selected_package / "good.xunit.xml", tests=1)
            external_results = workspace / "generated-code"
            write_xunit(
                external_results / "hidden.gtest.xml",
                tests=9,
            )
            (selected_package / "generated-code").symlink_to(
                external_results,
                target_is_directory=True,
            )

            completed = self.run_reporter(
                build_base,
                "voice_nav_mission",
            )

            self.assertEqual(completed.returncode, 2)
            self.assertIn("result path contains symbolic link", completed.stderr)
            self.assertNotIn("Summary: 1 test", completed.stdout)

    def test_report_rejects_nested_package_xml_symlink(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace = Path(temporary_directory)
            build_base = workspace / "build"
            selected_package = build_base / "voice_nav_mission"
            write_xunit(selected_package / "good.xunit.xml", tests=1)
            external_result = workspace / "hidden.xunit.xml"
            write_xunit(external_result, tests=9)
            nested_manifest = (
                selected_package
                / "test_results"
                / "voice_nav_mission"
                / "package.xml"
            )
            nested_manifest.parent.mkdir(parents=True)
            nested_manifest.symlink_to(external_result)

            completed = self.run_reporter(
                build_base,
                "voice_nav_mission",
            )

            self.assertEqual(completed.returncode, 2)
            self.assertIn("result path contains symbolic link", completed.stderr)
            self.assertNotIn("Summary: 1 test", completed.stdout)

    def test_report_rejects_spoofed_ament_python_module_target(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace = Path(temporary_directory)
            build_base = workspace / "build"
            package_name = "voice_nav_agent"
            selected_package = build_base / package_name
            write_xunit(selected_package / "good.xunit.xml", tests=1)
            source_package = workspace / "src" / package_name
            write_package_manifest(
                source_package / "package.xml",
                package_name=package_name,
            )
            (selected_package / "package.xml").symlink_to(
                source_package / "package.xml"
            )
            external_module = workspace / "external-module"
            write_xunit(external_module / "hidden.xunit.xml", tests=9)
            (selected_package / package_name).symlink_to(
                external_module,
                target_is_directory=True,
            )

            completed = self.run_reporter(build_base, package_name)

            self.assertEqual(completed.returncode, 2)
            self.assertIn("unexpected symbolic-link target", completed.stderr)
            self.assertNotIn("Summary: 1 test", completed.stdout)

    def test_report_rejects_spoofed_ament_cmake_python_target(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace = Path(temporary_directory)
            build_base = workspace / "build"
            package_name = "voice_nav_mission"
            selected_package = build_base / package_name
            write_xunit(selected_package / "good.xunit.xml", tests=1)
            external_module = workspace / "external-module"
            testing = external_module / "Testing"
            testing.mkdir(parents=True)
            (testing / "TAG").write_text(
                "20260801-0000\nExperimental\n",
                encoding="utf-8",
            )
            write_ctest(
                testing / "20260801-0000" / "Test.xml",
                status="failed",
            )
            generated_link = (
                selected_package
                / "ament_cmake_python"
                / package_name
                / package_name
            )
            generated_link.parent.mkdir(parents=True)
            generated_link.symlink_to(
                external_module,
                target_is_directory=True,
            )

            completed = self.run_reporter(build_base, package_name)

            self.assertEqual(completed.returncode, 2)
            self.assertIn("unexpected symbolic-link target", completed.stderr)
            self.assertNotIn("Summary: 1 test", completed.stdout)

    def test_report_rejects_pivoted_ament_python_targets(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace = Path(temporary_directory)
            build_base = workspace / "build"
            package_name = "voice_nav_agent"
            selected_package = build_base / package_name
            write_xunit(selected_package / "good.xunit.xml", tests=1)

            expected_source = workspace / "src" / package_name
            write_package_manifest(
                expected_source / "package.xml",
                package_name=package_name,
            )
            (expected_source / package_name).mkdir()

            external_source = (
                workspace / "external" / "src" / package_name
            )
            write_package_manifest(
                external_source / "package.xml",
                package_name=package_name,
            )
            write_xunit(
                external_source
                / package_name
                / "test_results"
                / "hidden.xunit.xml",
                tests=9,
            )
            pivot_target = workspace / "external" / "level"
            pivot_target.mkdir(parents=True)
            pivot = workspace / "pivot"
            pivot.symlink_to(pivot_target, target_is_directory=True)
            raw_source = pivot / ".." / "src" / package_name
            (selected_package / "package.xml").symlink_to(
                raw_source / "package.xml"
            )
            (selected_package / package_name).symlink_to(
                raw_source / package_name,
                target_is_directory=True,
            )

            completed = self.run_reporter(build_base, package_name)

            self.assertEqual(completed.returncode, 2)
            self.assertIn("unexpected symbolic-link target", completed.stderr)
            self.assertNotIn("Summary: 1 test", completed.stdout)

    def test_report_rejects_pivoted_ament_cmake_python_target(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace = Path(temporary_directory)
            build_base = workspace / "build"
            package_name = "voice_nav_mission"
            selected_package = build_base / package_name
            write_xunit(selected_package / "good.xunit.xml", tests=1)
            expected_target = (
                selected_package / "rosidl_generator_py" / package_name
            )
            expected_target.mkdir(parents=True)
            external_target = (
                workspace
                / "external"
                / "build"
                / package_name
                / "rosidl_generator_py"
                / package_name
            )
            write_xunit(
                external_target / "hidden.gtest.xml",
                tests=9,
            )
            pivot_target = workspace / "external" / "level"
            pivot_target.mkdir(parents=True)
            pivot = workspace / "pivot"
            pivot.symlink_to(pivot_target, target_is_directory=True)
            raw_target = (
                pivot
                / ".."
                / "build"
                / package_name
                / "rosidl_generator_py"
                / package_name
            )
            generated_link = (
                selected_package
                / "ament_cmake_python"
                / package_name
                / package_name
            )
            generated_link.parent.mkdir(parents=True)
            generated_link.symlink_to(
                raw_target,
                target_is_directory=True,
            )

            completed = self.run_reporter(build_base, package_name)

            self.assertEqual(completed.returncode, 2)
            self.assertIn("unexpected symbolic-link target", completed.stderr)
            self.assertNotIn("Summary: 1 test", completed.stdout)

    def test_report_rejects_spoofed_root_package_manifest_target(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace = Path(temporary_directory)
            build_base = workspace / "build"
            package_name = "voice_nav_agent"
            selected_package = build_base / package_name
            write_xunit(selected_package / "good.xunit.xml", tests=1)
            external_manifest = workspace / "external" / "package.xml"
            write_xunit(external_manifest, tests=9)
            (selected_package / "package.xml").symlink_to(external_manifest)

            completed = self.run_reporter(build_base, package_name)

            self.assertEqual(completed.returncode, 2)
            self.assertIn("unexpected symbolic-link target", completed.stderr)
            self.assertNotIn("Summary: 1 test", completed.stdout)

    def test_report_rejects_non_package_source_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace = Path(temporary_directory)
            build_base = workspace / "build"
            package_name = "voice_nav_agent"
            selected_package = build_base / package_name
            write_xunit(selected_package / "good.xunit.xml", tests=1)
            source_manifest = (
                workspace / "src" / package_name / "package.xml"
            )
            write_xunit(source_manifest, tests=9)
            (selected_package / "package.xml").symlink_to(source_manifest)

            completed = self.run_reporter(build_base, package_name)

            self.assertEqual(completed.returncode, 2)
            self.assertIn("invalid source package manifest", completed.stderr)
            self.assertNotIn("Summary: 1 test", completed.stdout)

    def test_report_rejects_unknown_source_manifest_encoding(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace = Path(temporary_directory)
            build_base = workspace / "build"
            package_name = "voice_nav_agent"
            selected_package = build_base / package_name
            write_xunit(selected_package / "good.xunit.xml", tests=1)
            source_manifest = (
                workspace / "src" / package_name / "package.xml"
            )
            source_manifest.parent.mkdir(parents=True)
            source_manifest.write_bytes(
                b'<?xml version="1.0" encoding="no-such-codec"?>\n'
                b"<package><name>voice_nav_agent</name></package>\n"
            )
            (selected_package / "package.xml").symlink_to(source_manifest)

            completed = self.run_reporter(build_base, package_name)

            self.assertEqual(completed.returncode, 2)
            self.assertIn("invalid source package manifest", completed.stderr)
            self.assertNotIn("Traceback", completed.stderr)
            self.assertNotIn("Summary: 1 test", completed.stdout)

    def test_report_rejects_symlinked_source_layout_components(self) -> None:
        for symlink_component in ("src", "package"):
            with self.subTest(symlink_component=symlink_component):
                with tempfile.TemporaryDirectory() as temporary_directory:
                    workspace = Path(temporary_directory)
                    build_base = workspace / "build"
                    package_name = "voice_nav_agent"
                    selected_package = build_base / package_name
                    write_xunit(
                        selected_package / "good.xunit.xml",
                        tests=1,
                    )
                    external_root = workspace / "external-source"
                    external_package = external_root / package_name
                    write_package_manifest(
                        external_package / "package.xml",
                        package_name=package_name,
                    )
                    external_module = external_package / package_name
                    write_xunit(
                        external_module
                        / "test_results"
                        / "hidden.xunit.xml",
                        tests=9,
                    )
                    if symlink_component == "src":
                        (workspace / "src").symlink_to(
                            external_root,
                            target_is_directory=True,
                        )
                    else:
                        source_root = workspace / "src"
                        source_root.mkdir()
                        (source_root / package_name).symlink_to(
                            external_package,
                            target_is_directory=True,
                        )
                    expected_source = workspace / "src" / package_name
                    (selected_package / "package.xml").symlink_to(
                        expected_source / "package.xml"
                    )
                    (selected_package / package_name).symlink_to(
                        expected_source / package_name,
                        target_is_directory=True,
                    )

                    completed = self.run_reporter(build_base, package_name)

                    self.assertEqual(completed.returncode, 2)
                    self.assertIn(
                        "invalid source package layout",
                        completed.stderr,
                    )
                    self.assertNotIn("Summary: 1 test", completed.stdout)

    def test_report_allows_known_non_evidence_build_symlinks(self) -> None:
        cases = (
            (
                "voice_nav_agent",
                Path("voice_nav_agent"),
            ),
            (
                "voice_nav_mission",
                Path(
                    "ament_cmake_python/voice_nav_mission/voice_nav_mission"
                ),
            ),
        )
        for package_name, relative_link in cases:
            with self.subTest(relative_link=relative_link):
                with tempfile.TemporaryDirectory() as temporary_directory:
                    workspace = Path(temporary_directory)
                    build_base = workspace / "build"
                    selected_package = build_base / package_name
                    write_xunit(
                        selected_package / "good.xunit.xml",
                        tests=1,
                    )
                    if package_name == "voice_nav_mission":
                        write_mission_launch_evidence(
                            selected_package
                            / "test_results"
                            / "voice_nav_mission",
                        )
                    source_package = workspace / "src" / package_name
                    write_package_manifest(
                        source_package / "package.xml",
                        package_name=package_name,
                    )
                    if relative_link == Path(package_name):
                        target = source_package / package_name
                        (selected_package / "package.xml").symlink_to(
                            source_package / "package.xml"
                        )
                    else:
                        target = (
                            selected_package
                            / "rosidl_generator_py"
                            / package_name
                        )
                    target.mkdir(parents=True)
                    link = selected_package / relative_link
                    link.parent.mkdir(parents=True, exist_ok=True)
                    link.symlink_to(target, target_is_directory=True)

                    completed = self.run_reporter(
                        build_base,
                        package_name,
                    )

                    self.assertEqual(
                        completed.returncode,
                        0,
                        completed.stderr,
                    )
                    expected_tests = (
                        7 if package_name == "voice_nav_mission" else 1
                    )
                    self.assertIn(
                        f"Summary: {expected_tests} test",
                        completed.stdout,
                    )

    def test_anchored_snapshot_rejects_source_replaced_by_symlink(self) -> None:
        from scripts.colcon_evidence import open_result_snapshot

        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace = Path(temporary_directory) / "workspace"
            package_directory = workspace / "build" / "voice_nav_mission"
            source_result = package_directory / "current.xunit.xml"
            write_xunit(source_result, tests=1)
            moved_result = package_directory / "original.xunit.xml"
            external_result = workspace / "external.xunit.xml"
            write_xunit(external_result, tests=9)
            sandbox_package = workspace / "sandbox" / "voice_nav_mission"

            with open_result_snapshot(package_directory) as snapshot:
                source_result.rename(moved_result)
                source_result.symlink_to(external_result)

                with self.assertRaisesRegex(
                    ValueError,
                    "changed after evidence discovery",
                ):
                    snapshot.stage(sandbox_package)

            staged_result = sandbox_package / "current.xunit.xml"
            self.assertFalse(staged_result.is_symlink())
            self.assertTrue(external_result.is_file())
            self.assertTrue(moved_result.is_file())

    def test_snapshot_stage_failure_does_not_leak_file_descriptors(self) -> None:
        from scripts.colcon_evidence import open_result_snapshot

        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace = Path(temporary_directory) / "workspace"
            package_directory = workspace / "build" / "voice_nav_mission"
            source_result = package_directory / "current.xunit.xml"
            write_xunit(source_result, tests=1)
            sandbox_package = workspace / "sandbox" / "voice_nav_mission"
            staged_result = sandbox_package / source_result.name
            write_xunit(staged_result, tests=9)

            with open_result_snapshot(package_directory) as snapshot:
                descriptors_before = len(list(Path("/proc/self/fd").iterdir()))

                with self.assertRaises(ValueError):
                    snapshot.stage(sandbox_package)

                descriptors_after = len(list(Path("/proc/self/fd").iterdir()))

            self.assertEqual(descriptors_after, descriptors_before)
            self.assertTrue(staged_result.is_file())

    def test_snapshot_rejects_result_added_after_discovery(self) -> None:
        from scripts.colcon_evidence import open_result_snapshot

        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace = Path(temporary_directory) / "workspace"
            package_directory = workspace / "build" / "voice_nav_mission"
            write_xunit(package_directory / "current.xunit.xml", tests=1)
            sandbox_package = workspace / "sandbox" / "voice_nav_mission"

            with open_result_snapshot(package_directory) as snapshot:
                write_xunit(package_directory / "late.xunit.xml", tests=9)

                with self.assertRaisesRegex(
                    ValueError,
                    "changed after evidence discovery",
                ):
                    snapshot.stage(sandbox_package)

            self.assertFalse((sandbox_package / "late.xunit.xml").exists())

    def test_clear_prevalidates_every_result_before_unlinking(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            selected_package = build_base / "voice_nav_mission"
            safe_result = selected_package / "a_safe.xunit.xml"
            target_result = selected_package / "b_target.xunit.xml"
            unsafe_link = selected_package / "z_unsafe.xunit.xml"
            write_xunit(safe_result, tests=1)
            write_xunit(target_result, tests=1)
            unsafe_link.symlink_to(target_result)

            completed = self.run_reporter(
                build_base,
                "voice_nav_mission",
                clear=True,
            )

            self.assertEqual(completed.returncode, 2)
            self.assertIn("result path contains symbolic link", completed.stderr)
            self.assertTrue(safe_result.is_file())
            self.assertTrue(target_result.is_file())
            self.assertTrue(unsafe_link.is_symlink())

    def test_clear_preserves_per_package_ownership(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            selected_package = build_base / "voice_nav_mission"
            selected_package.mkdir(parents=True)
            sibling_result = (
                build_base / "voice_nav_sim" / "owned.xunit.xml"
            )
            write_xunit(sibling_result, tests=1)
            borrowed_link = selected_package / "borrowed.xunit.xml"
            borrowed_link.symlink_to(sibling_result)

            completed = self.run_reporter(
                build_base,
                "voice_nav_mission",
                "voice_nav_sim",
                clear=True,
            )

            self.assertEqual(completed.returncode, 2)
            self.assertIn("result path contains symbolic link", completed.stderr)
            self.assertTrue(sibling_result.is_file())
            self.assertTrue(borrowed_link.is_symlink())

    def test_anchored_clear_rejects_replaced_parent_directory(self) -> None:
        from scripts.colcon_evidence import open_result_deletion_plan

        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace = Path(temporary_directory) / "workspace"
            package_directory = workspace / "build" / "voice_nav_mission"
            result_file = package_directory / "results" / "current.xml"
            write_xunit(result_file, tests=1)
            external_directory = workspace / "external"
            external_result = external_directory / "current.xml"
            write_xunit(external_result, tests=9)
            moved_directory = workspace / "moved-results"

            with open_result_deletion_plan(
                package_directory,
                (result_file,),
            ) as deletion_plan:
                result_file.parent.rename(moved_directory)
                result_file.parent.symlink_to(
                    external_directory,
                    target_is_directory=True,
                )

                with self.assertRaisesRegex(
                    ValueError,
                    "changed after evidence collection",
                ):
                    deletion_plan.unlink_all()

            self.assertTrue(external_result.is_file())
            self.assertTrue((moved_directory / "current.xml").is_file())

    def test_clear_rejects_file_replaced_after_snapshot(self) -> None:
        from scripts.colcon_evidence import open_result_deletion_plan
        from scripts.colcon_evidence import open_result_snapshot

        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace = Path(temporary_directory) / "workspace"
            package_directory = workspace / "build" / "voice_nav_mission"
            result_file = package_directory / "current.xunit.xml"
            write_xunit(result_file, tests=1)
            sandbox_package = workspace / "sandbox" / "voice_nav_mission"

            with open_result_snapshot(package_directory) as snapshot:
                snapshot.stage(sandbox_package)
                result_identity = snapshot.identity_for(
                    Path("current.xunit.xml")
                )

            result_file.unlink()
            write_xunit(result_file, tests=9)

            with self.assertRaisesRegex(
                ValueError,
                "changed after evidence collection",
            ):
                with open_result_deletion_plan(
                    package_directory,
                    (result_file,),
                    expected_identities=(result_identity,),
                ):
                    pass

            self.assertTrue(result_file.is_file())

    def test_safe_ctest_tag_is_scoped_and_reported(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            testing_directory = (
                build_base / "voice_nav_audio" / "Testing"
            )
            testing_directory.mkdir(parents=True)
            (testing_directory / "TAG").write_text(
                "20260731-0000\nExperimental\n",
                encoding="utf-8",
            )
            write_ctest(
                testing_directory / "20260731-0000" / "Test.xml"
            )

            completed = self.run_reporter(
                build_base,
                "voice_nav_audio",
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn(
                "Summary: 1 test, 0 errors, 0 failures, 0 skipped",
                completed.stdout,
            )

    def test_report_rejects_ctest_tag_with_missing_result(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            selected_package = build_base / "voice_nav_mission"
            write_xunit(selected_package / "valid.xunit.xml", tests=1)
            testing_directory = selected_package / "Testing"
            testing_directory.mkdir()
            (testing_directory / "TAG").write_text(
                "20260731-0000\nExperimental\n",
                encoding="utf-8",
            )

            completed = self.run_reporter(
                build_base,
                "voice_nav_mission",
            )

            self.assertEqual(completed.returncode, 2)
            self.assertIn(
                "CTest TAG result does not exist",
                completed.stderr,
            )
            self.assertNotIn("Summary: 1 test", completed.stdout)

    def test_report_rejects_ctest_result_with_wrong_schema(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            selected_package = build_base / "voice_nav_mission"
            write_xunit(selected_package / "valid.xunit.xml", tests=1)
            testing_directory = selected_package / "Testing"
            testing_directory.mkdir()
            (testing_directory / "TAG").write_text(
                "20260731-0000\nExperimental\n",
                encoding="utf-8",
            )
            write_xunit(
                testing_directory / "20260731-0000" / "Test.xml",
                tests=9,
            )

            completed = self.run_reporter(
                build_base,
                "voice_nav_mission",
            )

            self.assertEqual(completed.returncode, 2)
            self.assertIn(
                "CTest TAG did not produce a result",
                completed.stderr,
            )
            self.assertNotIn("Summary: 10 tests", completed.stdout)

    def test_report_rejects_malformed_mandatory_xunit_result(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            selected_package = build_base / "voice_nav_mission"
            write_xunit(selected_package / "valid.xunit.xml", tests=1)
            malformed_result = (
                selected_package
                / "test_results"
                / "voice_nav_mission"
                / "broken.xunit.xml"
            )
            malformed_result.parent.mkdir(parents=True)
            malformed_result.write_text(
                "<testsuite tests='9' failures='0'",
                encoding="utf-8",
            )

            completed = self.run_reporter(
                build_base,
                "voice_nav_mission",
            )

            self.assertEqual(completed.returncode, 2)
            self.assertIn(
                "mandatory xUnit result was not consumed",
                completed.stderr,
            )
            self.assertNotIn("Summary: 1 test", completed.stdout)

    def test_boundary_allows_missing_build_base(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "missing"

            completed = self.run_boundary_check(
                build_base,
                "voice_nav_mission",
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_boundary_allows_only_exact_packages_and_nested_results(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            write_xunit(
                build_base
                / "voice_nav_mission"
                / "nested"
                / "test_results"
                / "current.xml",
                tests=1,
            )
            (build_base / "voice_nav_sim").mkdir()

            completed = self.run_boundary_check(
                build_base,
                "voice_nav_mission",
                "voice_nav_sim",
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_boundary_rejects_stale_directory_without_deleting_it(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            (build_base / "voice_nav_mission").mkdir(parents=True)
            stale_directory = build_base / "voice_nav_mission.stale-l0009"
            stale_directory.mkdir()

            completed = self.run_boundary_check(
                build_base,
                "voice_nav_mission",
            )

            self.assertEqual(completed.returncode, 2)
            self.assertIn(
                "voice_nav_mission.stale-l0009",
                completed.stderr,
            )
            self.assertTrue(stale_directory.is_dir())

    def test_boundary_rejects_removed_package_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            (build_base / "voice_nav_old_name").mkdir(parents=True)

            completed = self.run_boundary_check(
                build_base,
                "voice_nav_mission",
            )

            self.assertEqual(completed.returncode, 2)
            self.assertIn("voice_nav_old_name", completed.stderr)

    def test_boundary_reports_multiple_offenders_in_sorted_order(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            (build_base / "z_old").mkdir(parents=True)
            (build_base / "a_stale").mkdir()

            completed = self.run_boundary_check(
                build_base,
                "voice_nav_mission",
            )

            self.assertEqual(completed.returncode, 2)
            self.assertLess(
                completed.stderr.index("a_stale"),
                completed.stderr.index("z_old"),
            )

    def test_boundary_rejects_empty_package_allowlist(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            completed = self.run_boundary_check(
                Path(temporary_directory) / "build",
            )

            self.assertEqual(completed.returncode, 2)
            self.assertIn("at least one --package", completed.stderr)

    def test_package_names_match_colcon_compatible_syntax(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            (build_base / "voice-nav-tools").mkdir(parents=True)

            compatible = self.run_boundary_check(
                build_base,
                "voice-nav-tools",
            )
            (build_base / "_private").mkdir()
            leading_underscore = self.run_boundary_check(
                build_base,
                "_private",
            )

            self.assertEqual(compatible.returncode, 0, compatible.stderr)
            self.assertEqual(leading_underscore.returncode, 2)
            self.assertIn("invalid package name", leading_underscore.stderr)

    def test_boundary_rejects_direct_symlink(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            build_base.mkdir()
            target = Path(temporary_directory) / "target"
            target.mkdir()
            (build_base / "voice_nav_mission").symlink_to(
                target,
                target_is_directory=True,
            )

            completed = self.run_boundary_check(
                build_base,
                "voice_nav_mission",
            )

            self.assertEqual(completed.returncode, 2)
            self.assertIn("voice_nav_mission", completed.stderr)
            self.assertIn("symbolic link", completed.stderr)

    def test_verify_clears_and_reports_only_selected_packages(self) -> None:
        verify = VERIFY_SCRIPT.read_text(encoding="utf-8")
        verify_lines = verify.splitlines()
        boundary_command = (
            'python3 scripts/check_colcon_build_boundary.py '
            '"${build_boundary_args[@]}"'
        )

        self.assertIn(
            "colcon list --base-paths src --names-only",
            verify,
        )
        boundary_checks = [
            index
            for index, line in enumerate(verify_lines)
            if line.strip() == boundary_command
        ]
        colcon_tests = [
            index
            for index, line in enumerate(verify_lines)
            if line.strip() == "colcon test \\"
        ]
        final_report = next(
            index
            for index, line in enumerate(verify_lines)
            if line.strip()
            == 'python3 scripts/report_test_results.py '
            '"${test_result_args[@]}"'
        )
        self.assertTrue(boundary_checks)
        self.assertTrue(colcon_tests)
        self.assertLess(max(colcon_tests), boundary_checks[-1])
        self.assertLess(boundary_checks[-1], final_report)
        self.assertIn('test_result_args=("--build-base" "build")', verify)
        self.assertIn(
            'python3 scripts/report_test_results.py "${test_result_args[@]}" '
            "--clear",
            verify,
        )
        self.assertIn(
            'python3 scripts/report_test_results.py "${test_result_args[@]}"',
            verify,
        )
        self.assertNotIn("colcon test-result --verbose", verify)

    def test_ci_runs_repository_contract_tests_after_ros_install(self) -> None:
        workflow = CI_WORKFLOW.read_text(encoding="utf-8")

        ros_install = workflow.index(
            "- name: Install ROS 2 Jazzy development environment"
        )
        contract_tests = workflow.index(
            "- name: Run repository contract tests"
        )
        canonical_verify = workflow.index(
            "- name: Run canonical workspace verification"
        )

        self.assertLess(ros_install, contract_tests)
        self.assertLess(contract_tests, canonical_verify)

        contract_step = workflow[contract_tests:canonical_verify]
        self.assertIn("set +u", contract_step)
        self.assertIn(
            'source "/opt/ros/${ROS_DISTRO}/setup.bash"',
            contract_step,
        )
        self.assertIn("set -u", contract_step)
        self.assertLess(
            contract_step.index("set +u"),
            contract_step.index(
                'source "/opt/ros/${ROS_DISTRO}/setup.bash"'
            ),
        )
        self.assertLess(
            contract_step.index(
                'source "/opt/ros/${ROS_DISTRO}/setup.bash"'
            ),
            contract_step.index("set -u"),
        )

    def test_verify_fails_closed_when_package_discovery_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace = Path(temporary_directory) / "workspace"
            scripts_directory = workspace / "scripts"
            scripts_directory.mkdir(parents=True)
            shutil.copyfile(VERIFY_SCRIPT, scripts_directory / "verify.sh")
            (scripts_directory / "check_clean_motion_gate_install.sh").write_text(
                "#!/usr/bin/env bash\nexit 0\n",
                encoding="utf-8",
            )
            (workspace / "install").mkdir()
            (workspace / "install" / "setup.bash").write_text(
                "",
                encoding="utf-8",
            )

            trace_file = workspace / "trace.log"
            bash_environment = workspace / "test-environment.bash"
            bash_environment.write_text(
                """python3() { return 0; }
git() { return 0; }
rosdep() { return 0; }
xacro() { return 0; }
check_urdf() { return 0; }
gz() { return 0; }
realpath() { printf '%s\\n' "${1:-}"; }
colcon() {
  case "${1:-}" in
    list)
      printf 'voice_nav_interfaces\\n'
      return 1
      ;;
    build)
      printf 'colcon:build\\n' >> "${VOICE_NAV_TEST_TRACE}"
      return 0
      ;;
    test)
      printf 'colcon:test\\n' >> "${VOICE_NAV_TEST_TRACE}"
      return 0
      ;;
  esac
  return 0
}
export -f python3 git rosdep xacro check_urdf gz realpath colcon
""",
                encoding="utf-8",
            )
            environment = os.environ.copy()
            environment["BASH_ENV"] = str(bash_environment)
            environment["VOICE_NAV_TEST_TRACE"] = str(trace_file)

            completed = subprocess.run(
                ["bash", str(scripts_directory / "verify.sh")],
                cwd=workspace,
                env=environment,
                check=False,
                capture_output=True,
                text=True,
            )

            trace = (
                trace_file.read_text(encoding="utf-8")
                if trace_file.exists()
                else ""
            )
            self.assertEqual(completed.returncode, 5, completed.stderr)
            self.assertIn(
                "Failed to discover ROS packages under src",
                completed.stderr,
            )
            self.assertNotIn("colcon:build", trace)


if __name__ == "__main__":
    unittest.main()
