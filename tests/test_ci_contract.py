import unittest
from pathlib import Path

import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CI_WORKFLOW = REPOSITORY_ROOT / ".github" / "workflows" / "ci.yml"
REQUIRED_CONTEXT = "required / ubuntu-24.04 / ros-jazzy"


class CiWorkflowContractTest(unittest.TestCase):
    @staticmethod
    def workflow() -> dict:
        return yaml.safe_load(CI_WORKFLOW.read_text(encoding="utf-8"))

    def test_remote_ci_is_one_fast_governance_job(self) -> None:
        workflow = CI_WORKFLOW.read_text(encoding="utf-8")
        parsed = self.workflow()

        self.assertEqual(list(parsed["jobs"]), ["quality-gate"])
        quality_gate = parsed["jobs"]["quality-gate"]
        self.assertEqual(quality_gate["name"], REQUIRED_CONTEXT)
        self.assertEqual(parsed["permissions"], {"contents": "read"})
        self.assertEqual(quality_gate["permissions"], {"contents": "read"})
        self.assertIn("pull_request:", workflow)
        self.assertIn("push:", workflow)
        self.assertIn("shellcheck scripts/*.sh", workflow)
        self.assertIn("actionlint", workflow)
        self.assertIn("Run governance contract tests", workflow)
        self.assertNotIn("tests.test_scoped_test_results", workflow)
        self.assertIn("Check Conventional Commit subjects for pull request", workflow)
        self.assertNotIn("workflow_dispatch", workflow)
        self.assertNotIn("ci_skip", workflow)
        self.assertNotIn("statuses:", workflow)
        self.assertNotIn("checks:", workflow)
        self.assertNotIn("github.rest.", workflow)
        self.assertNotIn("rosdep", workflow)
        self.assertNotIn("colcon", workflow)
        self.assertNotIn("scripts/verify.sh", workflow)
        self.assertNotIn("Gazebo", workflow)

    def test_remote_governance_keeps_safe_commit_ranges(self) -> None:
        workflow = CI_WORKFLOW.read_text(encoding="utf-8")

        self.assertIn("github.event.pull_request.base.sha", workflow)
        self.assertIn("github.event.pull_request.head.sha", workflow)
        self.assertIn("github.event.before", workflow)
        self.assertIn("0000000000000000000000000000000000000000", workflow)


if __name__ == "__main__":
    unittest.main()
