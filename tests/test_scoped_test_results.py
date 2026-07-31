import subprocess
import tempfile
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
REPORTER = REPOSITORY_ROOT / "scripts" / "report_test_results.py"
VERIFY_SCRIPT = REPOSITORY_ROOT / "scripts" / "verify.sh"


def write_xunit(path: Path, *, tests: int, skipped: int = 0) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        (
            f'<testsuite name="fixture" tests="{tests}" errors="0" '
            f'failures="0" skipped="{skipped}"></testsuite>\n'
        ),
        encoding="utf-8",
    )


class ScopedTestResultsTest(unittest.TestCase):
    def run_reporter(
        self,
        build_base: Path,
        *packages: str,
        clear: bool = False,
    ) -> subprocess.CompletedProcess[str]:
        command = [
            "python3",
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

    def test_verify_clears_and_reports_only_selected_packages(self) -> None:
        verify = VERIFY_SCRIPT.read_text(encoding="utf-8")

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


if __name__ == "__main__":
    unittest.main()
