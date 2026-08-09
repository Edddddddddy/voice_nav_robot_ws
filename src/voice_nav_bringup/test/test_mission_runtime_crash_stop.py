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

"""One real Runtime SIGKILL, zero proof, and restart-isolation acceptance."""

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
        'l0011a_runtime_crash_stop'
    )


class MissionRuntimeCrashStopTest(unittest.TestCase):

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

    def test_runtime_sigkill_zero_and_new_goal_recovery(
        self,
        launch_service,
        proc_info,
        runtime,
        restarts,
    ):
        evidence = {}
        try:
            old_runtime = self.probe.wait_runtime_ready()
            self.probe.wait_gate_instance()
            self.probe.send_goal(
                old_runtime,
                source_instance_id='runtime-crash-seed',
                source_seq=1,
                distance_m=1.0,
            )
            moving = self.probe.wait_for_armed_motion()
            old_gate = moving['gate']
            old_candidate_topic = old_gate.candidate_topic

            kill_ack_ns = self.runtime_process.kill(
                lambda: self.probe.count_fqn('/mission_runtime_node')
            )
            proc_info.assertWaitForShutdown(runtime, timeout=10.0)
            crash_stop = self.probe.wait_runtime_zero(
                kill_ack_ns,
                old_gate.zero_publish_seq,
            )
            stationarity = self.probe.wait_stationary(
                crash_stop['zero_sim_ns'],
                crash_stop['zero_ns'],
            )

            replacement = support.restart_product_node(
                launch_service,
                restarts,
                executable=support.RUNTIME_NODE,
                name=support.RUNTIME_NODE,
                config_filename='mission_runtime.yaml',
            )
            replacement_process = self.probe.wait_process_capture(
                replacement.capture
            )
            proc_info.assertWaitForStartup(
                replacement.action,
                timeout=15.0,
            )
            new_runtime = self.probe.wait_runtime_ready(
                previous_runtime_id=old_runtime.runtime_instance_id
            )
            if (
                new_runtime.runtime_instance_id
                == old_runtime.runtime_instance_id
            ):
                raise AssertionError('Runtime restart reused its instance ID')
            self.probe.wait_no_publishers(old_candidate_topic)

            stale_goal = self.probe.send_goal(
                old_runtime,
                source_instance_id='runtime-crash-old-replay',
                source_seq=1,
                distance_m=0.25,
            )
            stale_status, stale_result = self.probe.wait_goal_result(
                stale_goal,
                timeout=15.0,
            )
            support.assert_action_result(
                stale_status,
                stale_result,
                support.ExecuteMission.Result.STALE_REQUEST,
            )
            no_goal = self.probe.assert_no_goal_zero_window(
                time.monotonic_ns()
            )

            recovery_goal = self.probe.send_goal(
                new_runtime,
                source_instance_id='runtime-crash-recovery',
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

            evidence = {
                'status': 'passed',
                'old_runtime_id': old_runtime.runtime_instance_id,
                'new_runtime_id': new_runtime.runtime_instance_id,
                'old_epoch': int(old_runtime.admission_epoch),
                'new_epoch': int(new_runtime.admission_epoch),
                'old_gate_instance': old_gate.gate_instance_id,
                'old_lease': old_gate.lease_id,
                'gate_zero_latency_ms': crash_stop['latency_ns'] / 1e6,
                'stationary_settle_ms': stationarity['settle_ns'] / 1e6,
                'stationary_hold_ms': stationarity['hold_ns'] / 1e6,
                'stale_result': int(stale_result.code),
                'no_goal_start_sim_ns': no_goal['start_sim_ns'],
                'no_goal_end_sim_ns': no_goal['end_sim_ns'],
                'recovery_stationary_settle_ms': (
                    recovery_stationarity['settle_ns'] / 1e6
                ),
                'replacement_pid': replacement_process.snapshot.pid,
                'replacement_starttime_ticks': (
                    replacement_process.snapshot.starttime_ticks
                ),
                'cleanup': 'launch handles, structured Gazebo stop, and zero cleanup registered',
            }
            self.probe.evidence('mission_runtime_crash_stop', **evidence)
        except Exception as error:
            self.probe.evidence(
                'mission_runtime_crash_stop',
                status='failed',
                error=str(error),
                diagnostics=self.probe.diagnostic(),
            )
            raise


@launch_testing.post_shutdown_test()
class MissionRuntimeCrashStopShutdownTest(unittest.TestCase):

    def test_only_target_accepts_sigkill(
        self,
        proc_info,
        runtime,
        gate,
        restarts,
    ):
        assertExitCodes(proc_info, process=runtime, allowable_exit_codes=[-9])
        assertExitCodes(proc_info, process=gate, allowable_exit_codes=[0, -2])
        for record in restarts.records:
            assertExitCodes(
                proc_info,
                process=record.action,
                allowable_exit_codes=[0, -2],
            )
