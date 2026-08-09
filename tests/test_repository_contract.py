import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path

import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CHECKER = REPOSITORY_ROOT / "scripts" / "check_repository.py"


class RepositoryContractTest(unittest.TestCase):
    def run_checker(self, root: Path) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [sys.executable, str(CHECKER), "--root", str(root)],
            check=False,
            capture_output=True,
            text=True,
        )

    def write_readme(self, root: Path, content: str = "# Example\n") -> Path:
        path = root / "README.md"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
        return path

    def test_repository_contract_passes(self) -> None:
        completed = self.run_checker(REPOSITORY_ROOT)

        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_root_context_is_allowed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.write_readme(root)
            (root / "CONTEXT.md").write_text(
                "# VoiceNav glossary\n\nA term has one stable meaning.\n",
                encoding="utf-8",
            )

            completed = self.run_checker(root)

        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_retired_documentation_paths_fail(self) -> None:
        for relative_path in ("course", "docs/work-items"):
            with self.subTest(relative_path=relative_path):
                with tempfile.TemporaryDirectory() as temporary_directory:
                    root = Path(temporary_directory)
                    retired_path = root / relative_path
                    retired_path.mkdir(parents=True)
                    (retired_path / "README.md").write_text(
                        "# Retired\n", encoding="utf-8"
                    )

                    completed = self.run_checker(root)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(
                    f"retired documentation path remains: {relative_path}",
                    completed.stderr,
                )

    def test_issue_and_pull_request_templates_are_decision_complete(self) -> None:
        issue_template_paths = (
            REPOSITORY_ROOT / ".github" / "ISSUE_TEMPLATE" / "prd.yml",
            REPOSITORY_ROOT / ".github" / "ISSUE_TEMPLATE" / "task.yml",
        )
        required_issue_fields = (
            "issue_linkage",
            "acceptance",
            "rollback",
            "interface_impact",
            "risks",
            "dependencies",
            "verification",
        )
        for template_path in issue_template_paths:
            try:
                form = yaml.safe_load(template_path.read_text(encoding="utf-8"))
            except yaml.YAMLError as error:
                self.fail(f"invalid Issue form YAML in {template_path}: {error}")
            self.assertIsInstance(form, dict)
            self.assertIsInstance(form.get("name"), str)
            self.assertIsInstance(form.get("description"), str)
            self.assertIsInstance(form.get("title"), str)
            body = form.get("body")
            self.assertIsInstance(body, list)
            if not isinstance(body, list):
                continue

            fields = {}
            for item in body:
                self.assertIsInstance(item, dict)
                if not isinstance(item, dict):
                    continue
                item_type = item.get("type")
                self.assertIn(item_type, {"markdown", "textarea", "dropdown", "checkboxes"})
                if item_type == "markdown":
                    continue
                field_id = item.get("id")
                self.assertIsInstance(field_id, str)
                self.assertNotIn(field_id, fields)
                fields[field_id] = item
                attributes = item.get("attributes")
                self.assertIsInstance(attributes, dict)
                if isinstance(attributes, dict):
                    self.assertIsInstance(attributes.get("label"), str)
                    if item_type == "dropdown":
                        self.assertIsInstance(attributes.get("options"), list)

            for field in required_issue_fields:
                with self.subTest(template=template_path.name, field=field):
                    self.assertIn(field, fields)
                    validations = fields[field].get("validations")
                    self.assertIsInstance(validations, dict)
                    if isinstance(validations, dict):
                        self.assertIs(validations.get("required"), True)

            interface_options = fields["interface_impact"]["attributes"]["options"]
            if template_path.name == "task.yml":
                self.assertEqual(
                    interface_options,
                    [
                        "No Stable Interface change",
                        "Backward-compatible Stable Interface change",
                        "Breaking Stable Interface change",
                    ],
                )
                self.assertNotIn("Not yet known", interface_options)
            else:
                self.assertIn("Not yet known", interface_options)

        pull_request_template = (
            REPOSITORY_ROOT / ".github" / "PULL_REQUEST_TEMPLATE.md"
        ).read_text(encoding="utf-8")
        for heading in (
            "## Issue 链接",
            "## 结果",
            "## 验收映射",
            "## 最终测试摘要",
            "## 接口影响",
            "## 回滚",
            "## 剩余风险",
        ):
            with self.subTest(heading=heading):
                self.assertIn(heading, pull_request_template)
        self.assertIn("Closes #", pull_request_template)
        self.assertNotRegex(
            pull_request_template.lower(), r"lesson|learning[- ]record|work[- ]item"
        )

    def test_agent_workflow_docs_define_recoverable_protocol(self) -> None:
        agents = (REPOSITORY_ROOT / "AGENTS.md").read_text(encoding="utf-8")
        for marker in (
            "## Roles",
            "### Manager",
            "### Worker",
            "### Reviewer",
            "## Skill routing",
            "voice-nav-requirements",
            "voice-nav-worker",
            "voice-nav-review",
            "## Context recovery",
            "## Forbidden",
            "polling",
            "subagents",
        ):
            with self.subTest(document="AGENTS.md", marker=marker):
                self.assertIn(marker, agents)

        protocol_markers = (
            "VOICE_NAV_HANDOFF: ready|blocked|reviewed",
            "VOICE_NAV_PERSISTED",
            "VOICE_NAV_EVENT: blocked|completed|reviewed",
            "Is the sole transport owner for GitHub writes",
            "The Manager writes the evidence to GitHub and returns",
            "the canonical URL",
            "Neither role fabricates an evidence URL or sends a final event before",
            "No role polls GitHub, CI, or another",
        )
        for marker in protocol_markers:
            with self.subTest(document="AGENTS.md", marker=marker):
                self.assertIn(marker, agents)

        handoff = "VOICE_NAV_HANDOFF: ready|blocked|reviewed"
        persisted = "VOICE_NAV_PERSISTED"
        final_event = "VOICE_NAV_EVENT: blocked|completed|reviewed"
        self.assertLess(agents.index(handoff), agents.index(persisted))
        self.assertLess(agents.index(persisted), agents.index(final_event))

        self.assertIn(
            "A `reviewed` event with a P0/P1 finding that blocks merge must fill "
            "`decision_needed`",
            agents,
        )
        self.assertIn(
            "use `none` only when no decision/action is needed",
            agents,
        )

        agent_docs = (
            REPOSITORY_ROOT / "docs" / "agents" / "README.md"
        ).read_text(encoding="utf-8")
        for marker in (
            "## GitHub Issue tracking",
            "## Standard labels",
            "type:prd",
            "type:task",
            "ready-for-agent",
            "in-progress",
            "blocked",
            "review-needed",
            "## External PR intake",
            "## Single-context layout",
            "CONTEXT.md",
        ):
            with self.subTest(document="docs/agents/README.md", marker=marker):
                self.assertIn(marker, agent_docs)
        self.assertNotIn("needs-review", agent_docs)

        normalized_agent_docs = " ".join(agent_docs.split())
        for marker in (
            "The first direct `VOICE_NAV_HANDOFF` may carry the complete Chinese "
            "evidence needed for that transport.",
            "After the Manager returns `VOICE_NAV_PERSISTED` with the canonical "
            "URL, only the final `VOICE_NAV_EVENT` uses the compact envelope.",
        ):
            with self.subTest(document="docs/agents/README.md", marker=marker):
                self.assertIn(marker, normalized_agent_docs)

    def test_missing_relative_markdown_link_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.write_readme(root, "# Example\n\n[Missing](missing.md)\n")

            completed = self.run_checker(root)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("broken local Markdown link", completed.stderr)

    def test_unsupported_markdown_link_scheme_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.write_readme(root, "# Example\n\n[Local secret](file:///tmp/secret)\n")

            completed = self.run_checker(root)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("unsupported Markdown link scheme: file", completed.stderr)

    def test_unclosed_markdown_fence_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.write_readme(root, "# Example\n\n```text\nnever closed\n")

            completed = self.run_checker(root)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("unclosed Markdown fence", completed.stderr)

    def test_markdown_under_source_packages_is_checked(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.write_readme(root)
            source_readme = root / "src" / "example" / "README.md"
            source_readme.parent.mkdir(parents=True)
            source_readme.write_text(
                "# Example\n\n[Missing](missing.md)\n",
                encoding="utf-8",
            )

            completed = self.run_checker(root)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("broken local Markdown link: src", completed.stderr)

    def test_legacy_documentation_layout_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.write_readme(root)
            legacy_directory = root / "lessons"
            legacy_directory.mkdir()
            (legacy_directory / "0001.html").write_text(
                "<h1>Legacy</h1>\n", encoding="utf-8"
            )

            completed = self.run_checker(root)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("retired documentation path remains: lessons", completed.stderr)

    def test_untracked_text_with_trailing_whitespace_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.write_readme(root)
            bad_source = root / "scripts" / "bad.py"
            bad_source.parent.mkdir()
            bad_source.write_text("value = 1  \n", encoding="utf-8")

            completed = self.run_checker(root)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("trailing whitespace: scripts", completed.stderr)

    def test_ros_package_versions_must_match_repository_version(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.write_readme(root)
            (root / "VERSION").write_text("0.1.0\n", encoding="utf-8")
            package_directory = root / "src" / "example_package"
            package_directory.mkdir(parents=True)
            (package_directory / "package.xml").write_text(
                textwrap.dedent(
                    """\
                    <?xml version="1.0"?>
                    <package format="3">
                      <name>example_package</name>
                      <version>0.0.0</version>
                    </package>
                    """
                ),
                encoding="utf-8",
            )

            completed = self.run_checker(root)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "example_package version 0.0.0 does not match VERSION 0.1.0",
            completed.stderr,
        )

    def test_python_package_setup_version_must_match_repository_version(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.write_readme(root)
            (root / "VERSION").write_text("0.1.0\n", encoding="utf-8")
            package_directory = root / "src" / "example_package"
            package_directory.mkdir(parents=True)
            (package_directory / "package.xml").write_text(
                textwrap.dedent(
                    """\
                    <?xml version="1.0"?>
                    <package format="3">
                      <name>example_package</name>
                      <version>0.1.0</version>
                    </package>
                    """
                ),
                encoding="utf-8",
            )
            (package_directory / "setup.py").write_text(
                "setup(name='example_package', version='0.0.0')\n",
                encoding="utf-8",
            )

            completed = self.run_checker(root)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "example_package setup.py version 0.0.0 does not match VERSION 0.1.0",
            completed.stderr,
        )

    def test_setup_version_in_comment_is_ignored(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.write_readme(root)
            (root / "VERSION").write_text("0.1.0\n", encoding="utf-8")
            package_directory = root / "src" / "example_package"
            package_directory.mkdir(parents=True)
            (package_directory / "package.xml").write_text(
                textwrap.dedent(
                    """\
                    <?xml version="1.0"?>
                    <package format="3">
                      <name>example_package</name>
                      <version>0.1.0</version>
                    </package>
                    """
                ),
                encoding="utf-8",
            )
            (package_directory / "setup.py").write_text(
                textwrap.dedent(
                    """\
                    # Old example: setup(version='0.1.0')
                    setup(name='example_package', version='0.0.0')
                    """
                ),
                encoding="utf-8",
            )

            completed = self.run_checker(root)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "example_package setup.py version 0.0.0 does not match VERSION 0.1.0",
            completed.stderr,
        )


if __name__ == "__main__":
    unittest.main()
