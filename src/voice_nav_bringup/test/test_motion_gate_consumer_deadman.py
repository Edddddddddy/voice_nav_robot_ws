# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""One real MotionGate SIGKILL and controller consumer-timeout acceptance."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import time
import unittest

import launch_testing
from launch_testing.asserts import assertExitCodes
import launch_testing.markers
import pytest
import rclpy


def _load_support():
    support_path = Path(__file__).with_name('crash_stop_support.py')
    specification = importlib.util.spec_from_file_location(
        'voice_nav_crash_stop_support',
        support_path,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load crash-stop test support')
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


support = _load_support()


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return support.generate_product_test_description(
        'l0011a_motion_gate_deadman'
    )


class MotionGateConsumerDeadmanTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(
        self,
        proc_info,
        runtime_capture,
        gate_capture,
        partition,
        restarts,
    ):
        self.probe = support.CrashStopProbe()
        self.partition = partition
        self.restarts = restarts
        self.runtime_process = self.probe.wait_process_capture(
            runtime_capture
        )
        self.gate_process = self.probe.wait_process_capture(gate_capture)
        self.addCleanup(self.probe.destroy)
        self.addCleanup(self.restarts.close)
        self.addCleanup(runtime_capture.close)
        self.addCleanup(gate_capture.close)
        self.addCleanup(
            support.gazebo_shutdown.structured_stop_gazebo,
            proc_info,
            expected_partition=partition,
        )
        self.addCleanup(self.probe.inhibit_for_cleanup)

    def test_motion_gate_sigkill_consumer_zero_and_recovery(
        self,
        launch_service,
        proc_info,
        gate,
        restarts,
    ):
        try:
            old_runtime = self.probe.wait_runtime_ready()
            self.probe.wait_gate_instance()
            self.probe.send_goal(
                old_runtime,
                source_instance_id='gate-crash-seed',
                source_seq=1,
                distance_m=1.0,
            )
            moving = self.probe.wait_for_armed_motion()
            old_gate = moving['gate']
            final_gid = self.probe.assert_unique_final_owner()
            last_nonzero_sim_ns = support._stamp_ns(moving['final'][1])

            kill_ack_ns = self.gate_process.kill(
                lambda: self.probe.count_fqn('/motion_gate_node')
            )
            proc_info.assertWaitForShutdown(gate, timeout=10.0)
            consumer_zero = self.probe.wait_consumer_zero(
                kill_ack_ns,
                last_nonzero_sim_ns,
            )
            stationarity = self.probe.wait_stationary(
                consumer_zero['zero_sim_ns'],
                consumer_zero['zero_receipt_ns'],
            )
            runtime_fault = self.probe.wait_runtime_fault()

            replacement = support.restart_product_node(
                launch_service,
                restarts,
                executable=support.GATE_NODE,
                name=support.GATE_NODE,
                config_filename='motion_gate.yaml',
            )
            replacement_process = self.probe.wait_process_capture(
                replacement.capture
            )
            proc_info.assertWaitForStartup(
                replacement.action,
                timeout=15.0,
            )
            new_gate_sample = self.probe.wait_gate_instance(
                previous_instance=old_gate.gate_instance_id
            )
            if new_gate_sample.gate_instance_id == old_gate.gate_instance_id:
                raise AssertionError('Gate restart reused its instance ID')
            new_gate_zero = self.probe.wait_new_gate_zero(
                old_gate.gate_instance_id
            )
            no_goal = self.probe.assert_no_goal_zero_window(
                new_gate_zero[0]
            )
            stale_gate = self.probe.assert_stale_gate_tuple(old_gate)
            replay_start_ns = time.monotonic_ns()
            self.probe.publish_old_candidate(old_gate.candidate_topic)
            replay_window = self.probe.assert_no_goal_zero_window(
                replay_start_ns
            )

            recovered_runtime = self.probe.wait_runtime_ready()
            if (
                recovered_runtime.runtime_instance_id
                != old_runtime.runtime_instance_id
            ):
                raise AssertionError(
                    'Gate restart unexpectedly changed the Runtime instance ID'
                )
            if recovered_runtime.admission_epoch <= old_runtime.admission_epoch:
                raise AssertionError(
                    'Gate restart did not rotate the Runtime admission epoch'
                )
            recovery_goal = self.probe.send_goal(
                recovered_runtime,
                source_instance_id='gate-crash-recovery',
                source_seq=1,
                distance_m=0.25,
            )
            recovery_status, recovery_result = self.probe.wait_goal_result(
                recovery_goal,
                timeout=45.0,
            )
            support.assert_action_result(
                recovery_status,
                recovery_result,
                support.ExecuteMission.Result.SUCCEEDED,
            )
            recovery_zero = self.probe.wait_zero_after(
                time.monotonic_ns()
            )
            recovery_stationarity = self.probe.wait_stationary(
                recovery_zero['zero_sim_ns'],
                recovery_zero['zero_ns'],
            )
            self.probe.evidence(
                'motion_gate_consumer_deadman',
                status='passed',
                old_runtime_id=old_runtime.runtime_instance_id,
                recovered_runtime_id=recovered_runtime.runtime_instance_id,
                old_gate_instance=old_gate.gate_instance_id,
                new_gate_instance=new_gate_sample.gate_instance_id,
                runtime_fault_epoch=int(runtime_fault.admission_epoch),
                old_lease=old_gate.lease_id,
                final_owner_gid=final_gid,
                consumer_timeout_s=consumer_zero['delta_ns'] / 1e9,
                stationarity_settle_ms=stationarity['settle_ns'] / 1e6,
                stationarity_hold_ms=stationarity['hold_ns'] / 1e6,
                stale_gate=stale_gate,
                no_goal_start_sim_ns=no_goal['start_sim_ns'],
                no_goal_end_sim_ns=no_goal['end_sim_ns'],
                replay_start_sim_ns=replay_window['start_sim_ns'],
                replay_end_sim_ns=replay_window['end_sim_ns'],
                recovery_stationary_settle_ms=(
                    recovery_stationarity['settle_ns'] / 1e6
                ),
                replacement_pid=replacement_process.snapshot.pid,
                replacement_starttime_ticks=(
                    replacement_process.snapshot.starttime_ticks
                ),
                cleanup='launch handles, structured Gazebo stop, and zero cleanup registered',
            )
        except Exception as error:
            self.probe.evidence(
                'motion_gate_consumer_deadman',
                status='failed',
                error=str(error),
                diagnostics=self.probe.diagnostic(),
            )
            raise


@launch_testing.post_shutdown_test()
class MotionGateConsumerDeadmanShutdownTest(unittest.TestCase):

    def test_only_target_accepts_sigkill(
        self,
        proc_info,
        runtime,
        gate,
        restarts,
    ):
        assertExitCodes(proc_info, process=gate, allowable_exit_codes=[-9])
        assertExitCodes(proc_info, process=runtime, allowable_exit_codes=[0, -2])
        for record in restarts.records:
            assertExitCodes(
                proc_info,
                process=record.action,
                allowable_exit_codes=[0, -2],
            )
