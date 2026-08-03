import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CHECKER = REPOSITORY_ROOT / "scripts" / "check_fault_producer_contract.py"
ARTIFACTS = (
    "src/voice_nav_sim/test_support/fault_producer_actions.py",
    "src/voice_nav_sim/test/fault_producer.py",
    "src/voice_nav_sim/test/test_fault_producer_pair.py",
    "src/voice_nav_sim/test/test_fault_producer_protocol.py",
    "src/voice_nav_sim/CMakeLists.txt",
    "src/voice_nav_sim/package.xml",
)


class FaultProducerContractTest(unittest.TestCase):
    def copy_fixture(self, root: Path) -> None:
        for relative in ARTIFACTS:
            destination = root / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(REPOSITORY_ROOT / relative, destination)

    def run_checker(self, mutation=None):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.copy_fixture(root)
            if mutation is not None:
                mutation(root)
            return subprocess.run(
                [sys.executable, str(CHECKER), "--root", str(root)],
                check=False,
                capture_output=True,
                text=True,
            )

    @staticmethod
    def replace(root: Path, relative: str, old: str, new: str) -> None:
        path = root / relative
        source = path.read_text(encoding="utf-8")
        if old not in source:
            raise AssertionError(f"mutation source missing in {relative}: {old}")
        path.write_text(source.replace(old, new, 1), encoding="utf-8")

    def assert_rejected(self, mutation, message: str) -> None:
        completed = self.run_checker(mutation)
        self.assertNotEqual(completed.returncode, 0, completed.stdout)
        self.assertIn(message, completed.stderr)

    def test_repository_contract_passes(self) -> None:
        completed = subprocess.run(
            [
                sys.executable,
                str(CHECKER),
                "--root",
                str(REPOSITORY_ROOT),
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_candidate_fqn_is_not_configurable(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[0],
                "name='collision_monitor'",
                "name='renamed_monitor'",
            ),
            "candidate action FQN",
        )

    def test_actions_cannot_respawn(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[0],
                "output='screen',\n        parameters=",
                "output='screen',\n        respawn=True,\n        parameters=",
            ),
            "must never respawn",
        )

    def test_actions_cannot_hide_identity_in_a_namespace(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[0],
                "name='collision_monitor',\n        output='screen',",
                (
                    "name='collision_monitor',\n"
                    "        namespace='hidden',\n"
                    "        output='screen',"
                ),
            ),
            "must not use namespace or remapping",
        )

    def test_candidate_cannot_own_control_client(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[1],
                "if self.role == 'authority':",
                "if self.role == 'candidate':",
            ),
            "only the authority role",
        )

    def test_control_client_cannot_move_into_candidate_else(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                ARTIFACTS[1],
                (
                    "            self.control_client = self.create_client(\n"
                    "                InternalMotionGateControl,\n"
                    "                CONTROL_SERVICE,\n"
                    "            )\n"
                ),
                "            self.control_client = None\n",
            )
            self.replace(
                root,
                ARTIFACTS[1],
                "        else:\n            self.ready_publisher =",
                (
                    "        else:\n"
                    "            self.control_client = self.create_client(\n"
                    "                InternalMotionGateControl,\n"
                    "                CONTROL_SERVICE,\n"
                    "            )\n"
                    "            self.ready_publisher ="
                ),
            )

        self.assert_rejected(
            mutation,
            "only the authority role",
        )

    def test_authority_must_renew(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[1],
                "InternalMotionGateControl.Request.RENEW",
                "InternalMotionGateControl.Request.OPEN",
            ),
            "PREPARE/OPEN/RENEW",
        )

    def test_candidate_must_acknowledge_state_before_prepare(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[1],
                "self.wait_for_candidate_ready(executor)",
                "self.wait_for_candidate_state_reader(executor)",
            ),
            "readiness must precede PREPARE",
        )

    def test_final_controller_must_exist_before_prepare(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[1],
                "self.wait_for_final_controller_reader(executor)",
                "self.wait_for_candidate_state_reader(executor)",
            ),
            "final controller readiness must precede PREPARE",
        )

    def test_parent_cannot_import_control_protocol(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[2],
                "from voice_nav_mission.msg import InternalMotionGateState",
                (
                    "from voice_nav_mission.msg import InternalMotionGateState\n"
                    "from voice_nav_mission.srv import InternalMotionGateControl"
                ),
            ),
            "observe only",
        )

    def test_launch_test_must_outlive_both_leases(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[2],
                "SUSTAINED_LIVE_SECONDS = 0.55",
                "SUSTAINED_LIVE_SECONDS = 0.10",
            ),
            "outlive authority and candidate leases",
        )

    def test_launch_test_cannot_return_before_sustain_window(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[2],
                (
                    "        self.require_sustained_renewal_and_"
                    "candidate_traffic(state)"
                ),
                (
                    "        return\n"
                    "        self.require_sustained_renewal_and_"
                    "candidate_traffic(state)"
                ),
            ),
            "unconditionally finish with sustained evidence",
        )

    def test_protocol_evidence_cannot_be_decorated_as_skipped(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[3],
                (
                    "    def test_pending_then_applied_retries_with_"
                    "fresh_request_ids(self):"
                ),
                (
                    "    @unittest.skip('disabled')\n"
                    "    def test_pending_then_applied_retries_with_"
                    "fresh_request_ids(self):"
                ),
            ),
            "must execute without skip/return",
        )

    def test_cmake_timeout_is_pinned(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[4],
                "test/test_fault_producer_pair.py\n    TIMEOUT 30",
                "test/test_fault_producer_pair.py\n    TIMEOUT 29",
            ),
            "register its isolated launch test",
        )

    def test_readiness_dependency_is_direct(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[5],
                "  <test_depend>std_msgs</test_depend>\n",
                "",
            ),
            "missing helper test dependencies",
        )


if __name__ == "__main__":
    unittest.main()
