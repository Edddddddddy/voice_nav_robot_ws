import re
import unittest
from pathlib import Path

import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SKILLS_ROOT = REPOSITORY_ROOT / ".agents" / "skills"
ROLE_SKILLS = {
    "voice-nav-requirements": "需求澄清",
    "voice-nav-worker": "实施",
    "voice-nav-review": "审查",
}


def load_frontmatter(skill_path: Path) -> dict[str, object]:
    text = skill_path.read_text(encoding="utf-8")
    match = re.match(r"\A---\n(.*?)\n---\n", text, re.DOTALL)
    if match is None:
        raise AssertionError(f"{skill_path} 缺少有效 YAML frontmatter")
    frontmatter = yaml.safe_load(match.group(1))
    if not isinstance(frontmatter, dict):
        raise AssertionError(f"{skill_path} frontmatter 必须是 mapping")
    return frontmatter


class SkillContractTest(unittest.TestCase):
    def test_role_skills_are_repository_scoped_and_explicit(self) -> None:
        for skill_name in ROLE_SKILLS:
            with self.subTest(skill=skill_name):
                skill_directory = SKILLS_ROOT / skill_name
                skill_path = skill_directory / "SKILL.md"
                metadata_path = skill_directory / "agents" / "openai.yaml"
                self.assertTrue(skill_path.is_file())
                self.assertTrue(metadata_path.is_file())

                frontmatter = load_frontmatter(skill_path)
                self.assertEqual(frontmatter.get("name"), skill_name)
                description = frontmatter.get("description")
                self.assertIsInstance(description, str)
                self.assertIn("VoiceNav", description)

                metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
                self.assertIsInstance(metadata, dict)
                self.assertIs(
                    metadata["policy"]["allow_implicit_invocation"], False
                )
                default_prompt = metadata["interface"]["default_prompt"]
                self.assertIn(f"${skill_name}", default_prompt)

    def test_agents_defines_explicit_skill_routing_and_precedence(self) -> None:
        agents = (REPOSITORY_ROOT / "AGENTS.md").read_text(encoding="utf-8")
        normalized_agents = " ".join(agents.split())
        for skill_name, role in ROLE_SKILLS.items():
            with self.subTest(skill=skill_name):
                self.assertIn(
                    f"{role}使用 `${skill_name}`",
                    normalized_agents,
                )
                self.assertIn(f"`${skill_name}`", agents)

        for marker in (
            "显式写出对应 `$skill-name`",
            "同一子任务最多选择一份 VoiceNav 角色 Skill",
            "不得扩大 Issue 范围或角色权限",
            "系统/用户明确指令",
            "仓库 `AGENTS.md`",
            "已批准 Issue/ADR",
            "不得声称已使用",
            "重启或新建 Task",
            "个人目录中的同名 Skill 不是项目",
        ):
            with self.subTest(marker=marker):
                self.assertIn(marker, agents)

    def test_worker_skill_uses_current_local_product_validation_policy(self) -> None:
        worker = (
            SKILLS_ROOT / "voice-nav-worker" / "SKILL.md"
        ).read_text(encoding="utf-8")
        self.assertIn("本地 WSL", worker)
        self.assertIn("exact HEAD", worker)
        self.assertIn("适用", worker)
        self.assertIn("远端 required CI 只验证治理", worker)
        self.assertNotIn("complete repository gate", worker)
        self.assertNotIn("完整仓库门禁", worker)
        self.assertIn("AGENTS.md` 的“交付与证据”协议", worker)
        references = SKILLS_ROOT / "voice-nav-worker" / "references"
        self.assertFalse(references.exists() and any(references.iterdir()))


if __name__ == "__main__":
    unittest.main()
