import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path


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

    def write_catalog(self, root: Path, lessons: str) -> None:
        catalog_path = root / "course" / "catalog.toml"
        catalog_path.parent.mkdir(parents=True, exist_ok=True)
        catalog_path.write_text(
            "schema_version = 1\n\n" + textwrap.dedent(lessons),
            encoding="utf-8",
        )

    def create_minimal_course(self, root: Path) -> Path:
        lesson_path = root / "course" / "lessons" / "0001-bootstrap.md"
        record_path = root / "course" / "records" / "0001-bootstrap.md"
        lesson_path.parent.mkdir(parents=True, exist_ok=True)
        record_path.parent.mkdir(parents=True, exist_ok=True)
        lesson_path.write_text("# Bootstrap\n", encoding="utf-8")
        record_path.write_text("# Record\n", encoding="utf-8")
        self.write_catalog(
            root,
            """\
                [[lessons]]
                id = "0001"
                slug = "bootstrap"
                status = "completed"
                lesson = "course/lessons/0001-bootstrap.md"
                record = "course/records/0001-bootstrap.md"
            """,
        )
        return lesson_path

    def test_repository_course_catalog_passes(self) -> None:
        completed = self.run_checker(REPOSITORY_ROOT)

        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_valid_course_catalog_passes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            lesson_directory = root / "course" / "lessons"
            record_directory = root / "course" / "records"
            lesson_directory.mkdir(parents=True)
            record_directory.mkdir(parents=True)
            (lesson_directory / "0001-bootstrap.md").write_text(
                "# Bootstrap\n", encoding="utf-8"
            )
            (record_directory / "0001-bootstrap.md").write_text(
                "# Record\n", encoding="utf-8"
            )
            self.write_catalog(
                root,
                """\
                    [[lessons]]
                    id = "0001"
                    slug = "bootstrap"
                    status = "completed"
                    lesson = "course/lessons/0001-bootstrap.md"
                    record = "course/records/0001-bootstrap.md"
                """,
            )

            completed = self.run_checker(root)

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("Repository contract passed", completed.stdout)

    def test_duplicate_lesson_ids_fail(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            lesson_directory = root / "course" / "lessons"
            record_directory = root / "course" / "records"
            lesson_directory.mkdir(parents=True)
            record_directory.mkdir(parents=True)
            for slug in ("bootstrap", "interfaces"):
                (lesson_directory / f"0001-{slug}.md").write_text(
                    f"# {slug}\n", encoding="utf-8"
                )
                (record_directory / f"0001-{slug}.md").write_text(
                    f"# {slug}\n", encoding="utf-8"
                )
            self.write_catalog(
                root,
                """\
                    [[lessons]]
                    id = "0001"
                    slug = "bootstrap"
                    status = "completed"
                    lesson = "course/lessons/0001-bootstrap.md"
                    record = "course/records/0001-bootstrap.md"

                    [[lessons]]
                    id = "0001"
                    slug = "interfaces"
                    status = "completed"
                    lesson = "course/lessons/0001-interfaces.md"
                    record = "course/records/0001-interfaces.md"
                """,
            )

            completed = self.run_checker(root)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("duplicate course lesson id: 0001", completed.stderr)

    def test_lesson_ids_must_be_sorted_and_contiguous(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            for lesson_id in ("0001", "0003"):
                lesson_path = root / "course" / "lessons" / f"{lesson_id}-lesson.md"
                record_path = root / "course" / "records" / f"{lesson_id}-lesson.md"
                lesson_path.parent.mkdir(parents=True, exist_ok=True)
                record_path.parent.mkdir(parents=True, exist_ok=True)
                lesson_path.write_text("# Lesson\n", encoding="utf-8")
                record_path.write_text("# Record\n", encoding="utf-8")
            self.write_catalog(
                root,
                """\
                    [[lessons]]
                    id = "0001"
                    slug = "first"
                    status = "completed"
                    lesson = "course/lessons/0001-lesson.md"
                    record = "course/records/0001-lesson.md"

                    [[lessons]]
                    id = "0003"
                    slug = "third"
                    status = "planned"
                    lesson = "course/lessons/0003-lesson.md"
                    record = "course/records/0003-lesson.md"
                """,
            )

            completed = self.run_checker(root)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("sorted and contiguous", completed.stderr)

    def test_course_paths_cannot_escape_their_directories(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            outside_lesson = root / "outside.md"
            record_path = root / "course" / "records" / "0001-bootstrap.md"
            outside_lesson.write_text("# Outside\n", encoding="utf-8")
            record_path.parent.mkdir(parents=True)
            record_path.write_text("# Record\n", encoding="utf-8")
            self.write_catalog(
                root,
                """\
                    [[lessons]]
                    id = "0001"
                    slug = "bootstrap"
                    status = "completed"
                    lesson = "course/lessons/../../outside.md"
                    record = "course/records/0001-bootstrap.md"
                """,
            )

            completed = self.run_checker(root)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must stay under course/lessons", completed.stderr)

    def test_unknown_lesson_status_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            lesson_path = root / "course" / "lessons" / "0001-bootstrap.md"
            record_path = root / "course" / "records" / "0001-bootstrap.md"
            lesson_path.parent.mkdir(parents=True)
            record_path.parent.mkdir(parents=True)
            lesson_path.write_text("# Lesson\n", encoding="utf-8")
            record_path.write_text("# Record\n", encoding="utf-8")
            self.write_catalog(
                root,
                """\
                    [[lessons]]
                    id = "0001"
                    slug = "bootstrap"
                    status = "done-ish"
                    lesson = "course/lessons/0001-bootstrap.md"
                    record = "course/records/0001-bootstrap.md"
                """,
            )

            completed = self.run_checker(root)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("invalid status", completed.stderr)

    def test_missing_relative_markdown_link_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            lesson_path = self.create_minimal_course(root)
            lesson_path.write_text(
                "# Bootstrap\n\n[Missing](../reference/missing.md)\n",
                encoding="utf-8",
            )

            completed = self.run_checker(root)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("broken local Markdown link", completed.stderr)

    def test_unsupported_markdown_link_scheme_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            lesson_path = self.create_minimal_course(root)
            lesson_path.write_text(
                "# Bootstrap\n\n[Local secret](file:///tmp/secret)\n",
                encoding="utf-8",
            )

            completed = self.run_checker(root)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("unsupported Markdown link scheme: file", completed.stderr)

    def test_unclosed_markdown_fence_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            lesson_path = self.create_minimal_course(root)
            lesson_path.write_text(
                "# Bootstrap\n\n```text\nnever closed\n",
                encoding="utf-8",
            )

            completed = self.run_checker(root)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("unclosed Markdown fence", completed.stderr)

    def test_markdown_under_source_packages_is_checked(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.create_minimal_course(root)
            source_readme = root / "src" / "example" / "README.md"
            source_readme.parent.mkdir(parents=True)
            source_readme.write_text(
                "# Example\n\n[Missing](missing.md)\n",
                encoding="utf-8",
            )

            completed = self.run_checker(root)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "broken local Markdown link: src", completed.stderr
        )

    def test_legacy_documentation_layout_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.create_minimal_course(root)
            legacy_directory = root / "lessons"
            legacy_directory.mkdir()
            (legacy_directory / "0001.html").write_text(
                "<h1>Legacy</h1>\n", encoding="utf-8"
            )

            completed = self.run_checker(root)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("legacy documentation path remains: lessons", completed.stderr)

    def test_untracked_text_with_trailing_whitespace_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.create_minimal_course(root)
            bad_source = root / "scripts" / "bad.py"
            bad_source.parent.mkdir()
            bad_source.write_text("value = 1  \n", encoding="utf-8")

            completed = self.run_checker(root)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("trailing whitespace: scripts", completed.stderr)

    def test_ros_package_versions_must_match_repository_version(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.create_minimal_course(root)
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
            self.create_minimal_course(root)
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
            self.create_minimal_course(root)
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
