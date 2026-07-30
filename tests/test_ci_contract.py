import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CI_WORKFLOW = REPOSITORY_ROOT / ".github" / "workflows" / "ci.yml"


class CiWorkflowContractTest(unittest.TestCase):
    def test_rosdep_install_uses_supported_noninteractive_flag(self) -> None:
        lines = CI_WORKFLOW.read_text(encoding="utf-8").splitlines()
        start = next(
            index
            for index, line in enumerate(lines)
            if line.strip() == "rosdep install \\"
        )
        command_lines: list[str] = []

        for line in lines[start:]:
            command_lines.append(line.strip())
            if not line.rstrip().endswith("\\"):
                break

        command = "\n".join(command_lines)
        self.assertNotIn("--yes", command)
        self.assertTrue(
            any(line in {"-y", "--default-yes"} for line in command_lines),
            "rosdep install must be explicitly non-interactive",
        )


if __name__ == "__main__":
    unittest.main()
