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
