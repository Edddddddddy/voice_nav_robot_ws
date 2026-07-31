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


def write_xunit(path: Path, *, tests: int, skipped: int = 0) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        (
            f'<testsuite name="fixture" tests="{tests}" errors="0" '
            f'failures="0" skipped="{skipped}"></testsuite>\n'
        ),
        encoding="utf-8",
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
                build_base / "voice_nav_mission" / "current.xml",
                tests=3,
                skipped=1,
            )
            write_xunit(
                build_base
                / "voice_nav_mission.stale-l0009"
                / "stale.xml",
                tests=74,
                skipped=4,
            )

            completed = self.run_reporter(
                build_base,
                "voice_nav_mission",
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn(
                "Summary: 3 tests, 0 errors, 0 failures, 1 skipped",
                completed.stdout,
            )
            self.assertNotIn("74 tests", completed.stdout)

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

    def test_safe_ctest_tag_is_scoped_and_reported(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_base = Path(temporary_directory) / "build"
            testing_directory = (
                build_base / "voice_nav_mission" / "Testing"
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
                "voice_nav_mission",
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn(
                "Summary: 1 test, 0 errors, 0 failures, 0 skipped",
                completed.stdout,
            )

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

        self.assertIn(
            "colcon list --base-paths src --names-only",
            verify,
        )
        self.assertEqual(
            verify.count(
                'python3 scripts/check_colcon_build_boundary.py '
                '"${build_boundary_args[@]}"'
            ),
            2,
        )
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


if __name__ == "__main__":
    unittest.main()
