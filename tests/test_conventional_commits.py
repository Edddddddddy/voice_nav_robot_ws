import subprocess
import sys
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CHECKER = REPOSITORY_ROOT / "scripts" / "check_conventional_commits.py"


class ConventionalCommitContractTest(unittest.TestCase):
    def check_subject(self, subject: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [sys.executable, str(CHECKER), "--subject", subject],
            text=True,
            capture_output=True,
            check=False,
        )

    def test_accepts_allowed_type_and_chinese_summary(self) -> None:
        result = self.check_subject("ci(workflow): 增加可审计跳过授权")

        self.assertEqual(result.returncode, 0, result.stderr)

    def test_rejects_a_subject_without_conventional_prefix(self) -> None:
        result = self.check_subject("增加可审计跳过授权")

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Conventional Commit", result.stderr)

    def test_rejects_an_english_only_summary(self) -> None:
        result = self.check_subject("ci(workflow): add auditable skip")

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("简体中文", result.stderr)


if __name__ == "__main__":
    unittest.main()
