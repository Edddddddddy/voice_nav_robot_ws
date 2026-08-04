import shlex
import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PREPARE_SCRIPT = REPOSITORY_ROOT / "scripts" / "prepare_git_context.sh"


class GitContextContractTest(unittest.TestCase):
    def bash_path(self, path: Path) -> str:
        normalized = str(path.resolve()).replace("\\", "/")
        if ":" not in normalized:
            return normalized
        drive, remainder = normalized.split(":", 1)
        return f"/mnt/{drive.lower()}{remainder}"

    def expected_git_dir(self, path: Path) -> str:
        return self.bash_path(path)

    def write_lf(self, path: Path, content: str) -> None:
        with path.open("w", encoding="utf-8", newline="\n") as stream:
            stream.write(content)

    def run_context(self, workspace: Path) -> subprocess.CompletedProcess[str]:
        child = textwrap.dedent(
            f"""
            set -euo pipefail
            source {shlex.quote(self.bash_path(PREPARE_SCRIPT))}
            voice_nav_prepare_git_context {shlex.quote(self.bash_path(workspace))}
            printf 'GIT_DIR=%s\\n' "$GIT_DIR"
            printf 'GIT_WORK_TREE=%s\\n' "$GIT_WORK_TREE"
            bash -c 'printf "CHILD_GIT_DIR=%s\\n" "$GIT_DIR"; printf "CHILD_GIT_WORK_TREE=%s\\n" "$GIT_WORK_TREE"'
            """
        )
        command_path = workspace.parent / "run-git-context-test.sh"
        self.write_lf(command_path, child)
        command_path.chmod(0o755)
        try:
            return subprocess.run(
                ["bash", self.bash_path(command_path)],
                check=False,
                capture_output=True,
                text=True,
            )
        finally:
            command_path.unlink()

    def write_git_dir(self, path: Path) -> None:
        path.mkdir(parents=True)
        self.write_lf(path / "HEAD", "ref: refs/heads/main\n")

    def write_pointer(self, workspace: Path, target: str) -> None:
        workspace.mkdir(parents=True)
        self.write_lf(workspace / ".git", f"gitdir: {target}\n")

    def test_regular_git_directory_exports_context_to_child(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace = Path(temporary_directory) / "workspace"
            git_directory = workspace / ".git"
            workspace.mkdir()
            self.write_git_dir(git_directory)

            completed = self.run_context(workspace)

        self.assertEqual(completed.returncode, 0, completed.stderr)
        expected = f"GIT_DIR={self.expected_git_dir(git_directory)}"
        self.assertIn(expected, completed.stdout)
        self.assertIn(
            f"GIT_WORK_TREE={self.bash_path(workspace)}",
            completed.stdout,
        )
        self.assertIn(f"CHILD_{expected}", completed.stdout)
        self.assertIn(
            f"CHILD_GIT_WORK_TREE={self.bash_path(workspace)}",
            completed.stdout,
        )

    def test_relative_gitdir_pointer_is_resolved_from_pointer_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            workspace = root / "nested" / "workspace"
            git_directory = root / "metadata" / "worktree"
            self.write_git_dir(git_directory)
            self.write_pointer(workspace, "../../metadata/worktree")

            completed = self.run_context(workspace)

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn(
            f"GIT_DIR={self.expected_git_dir(git_directory)}",
            completed.stdout,
        )
        self.assertIn(
            f"GIT_WORK_TREE={self.bash_path(workspace)}",
            completed.stdout,
        )

    def test_windows_absolute_gitdir_pointer_uses_wslpath_when_needed(self) -> None:
        temporary_directory_options = {}
        if sys.platform != "win32" and Path("/mnt/c").is_dir():
            temporary_directory_options["dir"] = "/mnt/c"

        with tempfile.TemporaryDirectory(**temporary_directory_options) as temporary_directory:
            root = Path(temporary_directory)
            workspace = root / "workspace"
            git_directory = root / "metadata" / "worktree"
            self.write_git_dir(git_directory)
            resolved_git_directory = str(git_directory.resolve()).replace(
                "\\", "/"
            )
            if resolved_git_directory.startswith("/mnt/c/"):
                windows_target = "C:" + chr(92) + resolved_git_directory[7:].replace(
                    "/", chr(92)
                )
            elif ":" in resolved_git_directory:
                windows_target = resolved_git_directory.replace(
                    "/", chr(92)
                )
            else:
                windows_target = (
                    "C:"
                    + chr(92)
                    + "Users"
                    + chr(92)
                    + "fixture"
                    + chr(92)
                    + "metadata"
                    + chr(92)
                    + "worktree"
                )
            self.write_pointer(workspace, windows_target)

            completed = self.run_context(workspace)

            if (
                ":" not in resolved_git_directory
                and not resolved_git_directory.startswith("/mnt/c/")
            ):
                self.assertNotEqual(completed.returncode, 0)
                self.assertIn("wslpath", completed.stderr)
                return

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn(
            f"GIT_DIR={self.expected_git_dir(git_directory)}",
            completed.stdout,
        )

    def test_empty_malformed_and_missing_pointers_fail_closed(self) -> None:
        fixtures = ("", "not-a-gitdir", "gitdir: ", "gitdir: missing")
        for pointer in fixtures:
            with self.subTest(pointer=pointer):
                with tempfile.TemporaryDirectory() as temporary_directory:
                    workspace = Path(temporary_directory) / "workspace"
                    workspace.mkdir()
                    self.write_lf(workspace / ".git", f"{pointer}\n")

                    completed = self.run_context(workspace)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn("Git context", completed.stderr)
                self.assertNotIn("GIT_DIR=", completed.stdout)

    def test_directory_without_head_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace = Path(temporary_directory) / "workspace"
            workspace.mkdir()
            (workspace / ".git").mkdir()

            completed = self.run_context(workspace)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("HEAD", completed.stderr)
        self.assertNotIn("GIT_DIR=", completed.stdout)


if __name__ == "__main__":
    unittest.main()
