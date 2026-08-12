import unittest
from pathlib import Path

import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CI_WORKFLOW = REPOSITORY_ROOT / ".github" / "workflows" / "ci.yml"
REQUIRED_CONTEXT = "required / ubuntu-24.04 / ros-jazzy"
QUALITY_POLICY = REPOSITORY_ROOT / "docs" / "process" / "quality-policy.md"
TESTING_STRATEGY = REPOSITORY_ROOT / "docs" / "process" / "testing-strategy.md"
RETIRED_REMOTE_PRODUCT_CI_STATEMENTS = (
    "Hosted CI reproduces Ubuntu 24.04 and ROS 2 Jazzy for deterministic and "
    "headless checks.",
    "PR CI uses deterministic in-memory fakes as soon as their Module exists and "
    "adds bounded headless Gazebo tests with the v0.2 simulation milestones.",
    "At v0.1 the hosted gate covers repository metadata, static robot-model "
    "validation, package build, and package tests; it does not claim to launch "
    "Gazebo.",
    "Voice and LLM CI uses deterministic fakes; large local models are milestone "
    "tests, not ordinary CI dependencies.",
)


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
        self.assertIn("tests.test_skill_contract", workflow)
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

    def test_governance_documents_keep_product_checks_local(self) -> None:
        quality_policy = " ".join(
            QUALITY_POLICY.read_text(encoding="utf-8").split()
        )
        testing_strategy = " ".join(
            TESTING_STRATEGY.read_text(encoding="utf-8").split()
        )

        self.assertRegex(quality_policy, r"远端.*CI\s*只运行快速治理检查")
        self.assertRegex(
            quality_policy,
            r"不安装 ROS，也不运行 package build、package test、\s*headless Gazebo",
        )
        self.assertRegex(quality_policy, r"本地 WSL.*exact HEAD")
        self.assertRegex(
            quality_policy,
            r"Voice\s*与\s*LLM.*本地产品测试.*确定性 in-memory fake",
        )
        self.assertRegex(
            testing_strategy,
            r"本地 WSL.*exact HEAD.*确定性 in-memory fake.*headless Gazebo test",
        )
        self.assertRegex(
            testing_strategy,
            r"远端 PR CI\s*只运行.*shellcheck.*actionlint.*治理契约.*Conventional Commit",
        )
        governance_documents = "\n".join((quality_policy, testing_strategy))
        for retired_statement in RETIRED_REMOTE_PRODUCT_CI_STATEMENTS:
            with self.subTest(retired_statement=retired_statement):
                self.assertNotIn(retired_statement, governance_documents)
                with self.assertRaises(AssertionError):
                    self.assertNotIn(
                        retired_statement,
                        f"{governance_documents}\n{retired_statement}",
                    )


if __name__ == "__main__":
    unittest.main()
