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
    "src/voice_nav_sim/test/test_authority_process_death.py",
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

    def test_actions_property_cannot_inject_an_unregistered_action(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[0],
                "return (self.authority, self.candidate)",
                "return (self.authority, self.candidate, self.authority)",
            ),
            "actions property",
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

    def test_authority_kill_must_target_exact_action(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "            authority,\n        )",
                "            candidate,\n        )",
            ),
            "exact authority SIGKILL",
        )

    def test_authority_latency_must_start_at_process_exit(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "terminal_receipt - exit_ns",
                "terminal_receipt - signal_intent_ns",
            ),
            "exact ProcessExited latency",
        )

    def test_authority_exit_observation_cannot_be_fabricated(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "return crash_adapter.exit_observation(authority)",
                (
                    "return ('authority', -signal.SIGKILL, "
                    "time.monotonic_ns())"
                ),
            ),
            "exact ProcessExited observation source",
        )

    def test_authority_adapter_exit_method_cannot_be_rebound(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                (
                    "        label, returncode, exit_ns = "
                    "self.wait_for_exact_exit("
                ),
                (
                    "        crash_adapter.exit_observation = lambda "
                    "_action: ('authority', -signal.SIGKILL, "
                    "time.monotonic_ns())\n"
                    "        label, returncode, exit_ns = "
                    "self.wait_for_exact_exit("
                ),
            ),
            "adapter rebinding",
        )

    def test_authority_adapter_cannot_be_replaced_by_a_subclass(self) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                ARTIFACTS[6],
                "@pytest.mark.launch_test",
                (
                    "class ForgedAdapter(\n"
                    "    launch_crash_adapter.LaunchCrashAdapter,\n"
                    "):\n"
                    "    def exit_observation(self, _action):\n"
                    "        return ('authority', -signal.SIGKILL, "
                    "time.monotonic_ns())\n\n\n"
                    "@pytest.mark.launch_test"
                ),
            )
            self.replace(
                root,
                ARTIFACTS[6],
                (
                    "crash_adapter = "
                    "launch_crash_adapter.LaunchCrashAdapter(ledger)"
                ),
                "crash_adapter = ForgedAdapter(ledger)",
            )

        self.assert_rejected(
            mutation,
            "exact LaunchCrashAdapter construction",
        )

    def test_authority_kill_requires_fresh_arming_barrier(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "self.wait_for_fresh_arming_barrier(initial)",
                "self.wait_for_initial_armed()",
            ),
            "fresh arming barrier",
        )

    def test_authority_arming_age_assertion_keeps_its_polarity(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "signal_intent_ns - state_receipt",
                "state_receipt - signal_intent_ns",
            ),
            "fresh arming barrier assertions",
        )

    def test_authority_observer_cannot_inject_final_output(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "        self.output_subscription = ",
                (
                    "        self.zero_injector = self.node.create_publisher(\n"
                    "            TwistStamped, FINAL_TOPIC, 10,\n"
                    "        )\n"
                    "        self.output_subscription = "
                ),
            ),
            "observer-only authority tracer",
        )

    def test_authority_observer_cannot_alias_a_publisher_factory(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "        self.output_subscription = ",
                (
                    "        publisher_factory = self.node.create_publisher\n"
                    "        self.zero_injector = publisher_factory(\n"
                    "            TwistStamped, FINAL_TOPIC, 10,\n"
                    "        )\n"
                    "        self.zero_injector.publish(TwistStamped())\n"
                    "        self.output_subscription = "
                ),
            ),
            "observer-only authority tracer",
        )

    def test_authority_observer_cannot_reflect_a_publisher_factory(
        self,
    ) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "        self.output_subscription = ",
                (
                    "        factory = getattr(\n"
                    "            self.node, 'create_publisher',\n"
                    "        )\n"
                    "        injector = factory(\n"
                    "            TwistStamped, FINAL_TOPIC, 10,\n"
                    "        )\n"
                    "        getattr(injector, 'publish')(TwistStamped())\n"
                    "        self.output_subscription = "
                ),
            ),
            "dynamic reflection",
        )

    def test_authority_observer_cannot_use_operator_methodcaller(
        self,
    ) -> None:
        def mutation(root: Path) -> None:
            self.replace(
                root,
                ARTIFACTS[6],
                "import math\n",
                "import math\nfrom operator import methodcaller\n",
            )
            self.replace(
                root,
                ARTIFACTS[6],
                "        self.output_subscription = ",
                (
                    "        make = methodcaller(\n"
                    "            'create_publisher', TwistStamped, "
                    "FINAL_TOPIC, 10,\n"
                    "        )\n"
                    "        injector = make(self.node)\n"
                    "        methodcaller(\n"
                    "            'publish', TwistStamped(),\n"
                    "        )(injector)\n"
                    "        self.output_subscription = "
                ),
            )

        self.assert_rejected(
            mutation,
            "dynamic reflection",
        )

    def test_authority_receipt_fence_precedes_observer_lock(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                (
                    "        receipt_ns = time.monotonic_ns()\n"
                    "        with self.lock:\n"
                    "            self.states.append((receipt_ns, message))"
                ),
                (
                    "        with self.lock:\n"
                    "            receipt_ns = time.monotonic_ns()\n"
                    "            self.states.append((receipt_ns, message))"
                ),
            ),
            "callback-entry receipt fences",
        )

    def test_authority_test_cannot_return_before_terminal_evidence(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "        self.assert_no_preexit_retirement(\n",
                (
                    "        return\n"
                    "        self.assert_no_preexit_retirement(\n"
                ),
            ),
            "execute without skip or early return",
        )

    def test_authority_exit_code_cannot_use_broad_allowlist(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "allowable_exit_codes=[-signal.SIGKILL]",
                "allowable_exit_codes=[0, -signal.SIGKILL]",
            ),
            "exact exhaustive exit ledger",
        )

    def test_authority_terminal_reason_is_exact(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "InternalMotionGateState.AUTHORITY_EXPIRED",
                "InternalMotionGateState.CANDIDATE_EXPIRED",
            ),
            "AUTHORITY_EXPIRED terminal evidence",
        )

    def test_authority_terminal_zero_is_newer_than_armed_baseline(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                (
                    "            terminal.zero_publish_seq,\n"
                    "            armed.zero_publish_seq,"
                ),
                "            terminal.zero_publish_seq,\n            0,",
            ),
            "new zero publish sequence",
        )

    def test_authority_latency_threshold_is_pinned(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "AUTHORITY_STOP_DEADLINE_NS = 300_000_000",
                "AUTHORITY_STOP_DEADLINE_NS = 301_000_000",
            ),
            "authority timing constants",
        )

    def test_authority_exit_registration_is_exact(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "expect_sigkill(producers.authority, 'authority')",
                "expect_sigkill(producers.candidate, 'authority')",
            ),
            "exact launch exit registrations",
        )

    def test_authority_launch_cannot_append_unregistered_process(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                (
                    "                launch_testing.actions.ReadyToTest(),\n"
                    "            ]"
                ),
                (
                    "                launch_testing.actions.ReadyToTest(),\n"
                    "                motion_gate,\n"
                    "            ]"
                ),
            ),
            "exact launch exit registrations",
        )

    def test_authority_launch_context_keeps_exact_action_identity(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "            'authority': producers.authority,",
                "            'authority': producers.candidate,",
            ),
            "exact launch context",
        )

    def test_authority_candidate_counter_evidence_is_exact(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "endpoint.node_name == 'collision_monitor'",
                "endpoint.node_name == 'authority_death_authority'",
            ),
            "surviving candidate counter-evidence",
        )

    def test_authority_candidate_counter_evidence_requires_exact_gid(
        self,
    ) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "bytes(compatible[0].endpoint_gid) == expected_gid",
                "bytes(compatible[0].endpoint_gid) != expected_gid",
            ),
            "surviving candidate counter-evidence",
        )

    def test_authority_candidate_binding_rejects_a_second_writer(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "len(compatible) == 1",
                "len(compatible) >= 1",
            ),
            "exact writer binding",
        )

    def test_authority_candidate_hold_rejects_a_second_writer(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "            len(compatible_writers),\n            1,",
                "            len(compatible_writers),\n            0,",
            ),
            "one compatible writer",
        )

    def test_authority_candidate_writer_set_cannot_be_truncated(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "        self.assertEqual(\n            len(compatible_writers),",
                (
                    "        compatible_writers = compatible_writers[:1]\n"
                    "        self.assertEqual(\n"
                    "            len(compatible_writers),"
                ),
            ),
            "immutable endpoint evidence collections",
        )

    def test_authority_candidate_writer_set_cannot_be_slice_deleted(
        self,
    ) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "        self.assertEqual(\n            len(compatible_writers),",
                (
                    "        del compatible_writers[1:]\n"
                    "        self.assertEqual(\n"
                    "            len(compatible_writers),"
                ),
            ),
            "immutable endpoint evidence collections",
        )

    def test_authority_assertion_callable_cannot_be_rebound(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "        self.assertEqual(\n            len(compatible_writers),",
                (
                    "        self.assertEqual = lambda *_args, "
                    "**_kwargs: None\n"
                    "        self.assertEqual(\n"
                    "            len(compatible_writers),"
                ),
            ),
            "assertion and predicate callable identity",
        )

    def test_authority_tracer_requires_reviewed_ast_fingerprint(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "        for action in (motion_gate, authority, candidate):",
                (
                    "        unreviewed_marker = 1\n"
                    "        for action in (motion_gate, authority, candidate):"
                ),
            ),
            "review-locked authority tracer",
        )

    def test_authority_zero_hold_requires_a_receipt_span(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "held[-1][0] - held[0][0]",
                "held[-1][0] - held[-1][0]",
            ),
            "receipt-span assertions",
        )

    def test_authority_zero_hold_span_keeps_greater_equal_polarity(
        self,
    ) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                (
                    "        self.assertGreaterEqual(\n"
                    "            held[-1][0] - held[0][0],\n"
                    "            ZERO_HOLD_MIN_SPAN_NS,\n"
                    "        )"
                ),
                (
                    "        self.assertLessEqual(\n"
                    "            held[-1][0] - held[0][0],\n"
                    "            ZERO_HOLD_MIN_SPAN_NS,\n"
                    "        )"
                ),
            ),
            "receipt-span assertions",
        )

    def test_authority_cmake_timeout_is_pinned(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[4],
                "test/test_authority_process_death.py\n    TIMEOUT 30",
                "test/test_authority_process_death.py\n    TIMEOUT 29",
            ),
            "authority process-death launch test",
        )

    def test_authority_barrier_requires_a_new_renew(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "> baseline_state.control_seq",
                ">= baseline_state.control_seq",
            ),
            "fresh RENEW arming evidence",
        )

    def test_authority_preexit_fence_uses_process_exit(self) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "barrier_started_ns <= receipt < exit_ns",
                "barrier_started_ns <= receipt < barrier_started_ns",
            ),
            "pre-exit live counter-evidence",
        )

    def test_authority_terminal_samples_are_bounded_by_receipt_deadline(
        self,
    ) -> None:
        self.assert_rejected(
            lambda root: self.replace(
                root,
                ARTIFACTS[6],
                "and sample[0] <= deadline_ns",
                "and sample[0] <= time.monotonic_ns()",
            ),
            "receipt deadline",
        )


if __name__ == "__main__":
    unittest.main()
