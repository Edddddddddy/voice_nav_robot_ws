import os
import shlex
import subprocess
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

    def run_git(self, workspace: Path, *arguments: str) -> None:
        completed = subprocess.run(
            ["git", "-C", str(workspace), *arguments],
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode != 0:
            self.fail(
                f"git {' '.join(arguments)} failed: "
                f"stdout={completed.stdout!r} stderr={completed.stderr!r}"
            )

    def create_repository(
        self,
        root: Path,
        *,
        move_git_directory: bool = False,
        target_name: str = "worktree",
    ) -> tuple[Path, Path]:
        workspace = root / "workspace"
        workspace.mkdir()
        self.run_git(workspace, "init", "--quiet")
        self.write_lf(workspace / "tracked-marker.txt", "fixture\n")
        self.run_git(workspace, "add", "tracked-marker.txt")
        commit = subprocess.run(
            [
                "git",
                "-C",
                str(workspace),
                "-c",
                "user.name=VoiceNav fixture",
                "-c",
                "user.email=voice-nav-fixture@example.invalid",
                "commit",
                "--quiet",
                "-m",
                "fixture",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        if commit.returncode != 0:
            self.fail(
                "git commit failed: "
                f"stdout={commit.stdout!r} stderr={commit.stderr!r}"
            )

        git_directory = workspace / ".git"
        if move_git_directory:
            git_directory = root / "metadata" / target_name
            git_directory.parent.mkdir(parents=True)
            (workspace / ".git").rename(git_directory)
        return workspace, git_directory

    def write_pointer(self, workspace: Path, payload: bytes) -> None:
        workspace.mkdir(parents=True, exist_ok=True)
        (workspace / ".git").write_bytes(payload)

    def wslpath_environment(
        self,
        root: Path,
        *,
        output: str = "",
        exit_code: int = 0,
        expected_argument: str = "",
    ) -> dict[str, str]:
        stub_directory = root / "wslpath-bin"
        stub_directory.mkdir()
        stub = stub_directory / "wslpath"
        self.write_lf(
            stub,
            """#!/usr/bin/env bash
if [[ "${1:-}" != "-u" || "${2:-}" != "--" ]]; then
  exit 91
fi
if [[ -n "${VOICE_NAV_EXPECTED_WSLPATH_ARG:-}" && "${@: -1}" != "${VOICE_NAV_EXPECTED_WSLPATH_ARG}" ]]; then
  exit 92
fi
printf '%b' "${VOICE_NAV_WSLPATH_OUTPUT:-}"
exit "${VOICE_NAV_WSLPATH_EXIT_CODE:-0}"
""",
        )
        stub.chmod(0o755)
        environment = os.environ.copy()
        environment["PATH"] = ":".join(
            (self.bash_path(stub_directory), environment.get("PATH", ""))
        )
        environment["VOICE_NAV_WSLPATH_OUTPUT"] = (
            output.replace("\\", "\\\\")
            .replace("\r", r"\r")
            .replace("\n", r"\n")
        )
        environment["VOICE_NAV_WSLPATH_EXIT_CODE"] = str(exit_code)
        environment["VOICE_NAV_EXPECTED_WSLPATH_ARG"] = expected_argument
        environment["VOICE_NAV_WSLPATH_BIN"] = self.bash_path(stub_directory)
        return environment

    def run_context(
        self,
        workspace: Path,
        *,
        environment: dict[str, str] | None = None,
    ) -> subprocess.CompletedProcess[str]:
        path_setup = ""
        if environment is not None:
            stub_directory = environment.get("VOICE_NAV_WSLPATH_BIN")
            if stub_directory:
                path_setup = "\n".join(
                    (
                        f"export PATH={shlex.quote(stub_directory)}:\"$PATH\"",
                        "export VOICE_NAV_WSLPATH_OUTPUT="
                        f"{shlex.quote(environment.get('VOICE_NAV_WSLPATH_OUTPUT', ''))}",
                        "export VOICE_NAV_WSLPATH_EXIT_CODE="
                        f"{shlex.quote(environment.get('VOICE_NAV_WSLPATH_EXIT_CODE', '0'))}",
                        "export VOICE_NAV_EXPECTED_WSLPATH_ARG="
                        f"{shlex.quote(environment.get('VOICE_NAV_EXPECTED_WSLPATH_ARG', ''))}",
                    )
                ) + "\n"
        child = textwrap.dedent(
            f"""
            set -euo pipefail
            {path_setup}
            source {shlex.quote(self.bash_path(PREPARE_SCRIPT))}
            voice_nav_prepare_git_context {shlex.quote(self.bash_path(workspace))}
            printf 'GIT_DIR=%s\\n' "$GIT_DIR"
            printf 'GIT_WORK_TREE=%s\\n' "$GIT_WORK_TREE"
            cd -- "$GIT_WORK_TREE"
            bash -c '
              set -euo pipefail
              printf "CHILD_GIT_DIR=%s\\n" "$GIT_DIR"
              printf "CHILD_GIT_WORK_TREE=%s\\n" "$GIT_WORK_TREE"
              printf "CHILD_HEAD=%s\\n" "$(git rev-parse --verify HEAD^{{commit}})"
              printf "CHILD_FILES=%s\\n" "$(git ls-files --cached)"
              printf "CHILD_STATUS=%s\\n" "$(git status --porcelain --untracked-files=no)"
            '
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
                env=environment,
            )
        finally:
            command_path.unlink()

    def assert_valid_context(
        self,
        completed: subprocess.CompletedProcess[str],
        workspace: Path,
        git_directory: Path,
    ) -> None:
        self.assertEqual(
            completed.returncode,
            0,
            f"stderr={completed.stderr!r} stdout={completed.stdout!r}",
        )
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
        self.assertIn("CHILD_HEAD=", completed.stdout)
        self.assertIn("CHILD_FILES=tracked-marker.txt", completed.stdout)
        self.assertIn("CHILD_STATUS=\n", completed.stdout)

    def test_regular_git_directory_runs_real_git_in_inherited_child(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace, git_directory = self.create_repository(
                Path(temporary_directory)
            )
            completed = self.run_context(workspace)

        self.assert_valid_context(completed, workspace, git_directory)

    def test_relative_gitdir_pointer_runs_real_git_in_inherited_child(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            workspace, git_directory = self.create_repository(
                root,
                move_git_directory=True,
            )
            relative_target = os.path.relpath(git_directory, workspace).replace(
                "\\", "/"
            )
            self.write_pointer(
                workspace,
                f"gitdir: {relative_target}\n".encode(),
            )
            completed = self.run_context(workspace)

        self.assert_valid_context(completed, workspace, git_directory)

    def test_windows_absolute_gitdir_pointer_accepts_spaces_and_runs_git(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            workspace, git_directory = self.create_repository(
                root,
                move_git_directory=True,
                target_name="with spaces/worktree",
            )
            windows_target = r"C:\fixture\metadata\with spaces\worktree"
            self.write_pointer(
                workspace,
                f"gitdir: {windows_target}\n".encode(),
            )
            expected_argument = windows_target.replace("\\", "/")
            environment = self.wslpath_environment(
                root,
                output=f"{self.bash_path(git_directory)}\n",
                expected_argument=expected_argument,
            )
            completed = self.run_context(
                workspace,
                environment=environment,
            )

        self.assert_valid_context(completed, workspace, git_directory)

    def test_gitfile_line_boundaries_and_quotes_fail_closed(self) -> None:
        payloads = (
            b"gitdir: C:\\fixture\\metadata\\worktree\r\n",
            b"gitdir: C:\\fixture\\metadata\\worktree\n\n",
            b"gitdir: C:\\fixture\\metadata\\worktree\nsecond\n",
            b'gitdir: "C:\\fixture\\metadata\\worktree"\n',
        )
        for payload in payloads:
            with self.subTest(payload=payload):
                with tempfile.TemporaryDirectory() as temporary_directory:
                    workspace = Path(temporary_directory) / "workspace"
                    self.write_pointer(workspace, payload)
                    completed = self.run_context(workspace)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn("Git context", completed.stderr)
                self.assertNotIn("GIT_DIR=", completed.stdout)

    def test_windows_path_conversion_failures_fail_closed(self) -> None:
        scenarios = (
            ("nonzero", 7, "", "conversion failed"),
            (
                "multiline",
                0,
                "/tmp/fixture-git-dir\n\n",
                "invalid path",
            ),
            (
                "crlf",
                0,
                "/tmp/fixture-git-dir\r\n",
                "invalid path",
            ),
            (
                "missing",
                0,
                "/tmp/fixture-git-dir-does-not-exist\n",
                "does not exist",
            ),
        )
        for name, exit_code, output, expected_error in scenarios:
            with self.subTest(name=name):
                with tempfile.TemporaryDirectory() as temporary_directory:
                    root = Path(temporary_directory)
                    workspace = root / "workspace"
                    self.write_pointer(
                        workspace,
                        b"gitdir: C:\\fixture\\metadata\\worktree\n",
                    )
                    environment = self.wslpath_environment(
                        root,
                        output=output,
                        exit_code=exit_code,
                        expected_argument="C:/fixture/metadata/worktree",
                    )
                    completed = self.run_context(
                        workspace,
                        environment=environment,
                    )

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(expected_error, completed.stderr)
                self.assertNotIn("GIT_DIR=", completed.stdout)

    def test_empty_malformed_and_missing_pointers_fail_closed(self) -> None:
        fixtures = (b"", b"not-a-gitdir\n", b"gitdir: \n", b"gitdir: missing\n")
        for pointer in fixtures:
            with self.subTest(pointer=pointer):
                with tempfile.TemporaryDirectory() as temporary_directory:
                    workspace = Path(temporary_directory) / "workspace"
                    self.write_pointer(workspace, pointer)
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

    def test_directory_with_fake_head_fails_closed_before_child_git(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            workspace = Path(temporary_directory) / "workspace"
            workspace.mkdir()
            (workspace / ".git").mkdir()
            self.write_lf(workspace / ".git" / "HEAD", "ref: refs/heads/main\n")
            completed = self.run_context(workspace)

        self.assertNotEqual(completed.returncode, 0)
        self.assertRegex(
            completed.stderr,
            r"Git (directory|HEAD) probe failed",
        )
        self.assertNotIn("GIT_DIR=", completed.stdout)


if __name__ == "__main__":
    unittest.main()
