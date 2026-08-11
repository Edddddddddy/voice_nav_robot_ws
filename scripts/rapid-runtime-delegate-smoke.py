#!/usr/bin/env python3
"""Small public Runtime -> private rapid bridge action smoke."""

import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from voice_nav_interfaces.action import ExecuteMission
from voice_nav_interfaces.msg import MissionState, MissionStep


class RuntimeDelegateSmoke(Node):
    """Observe the public snapshot and execute one short typed Mission."""

    def __init__(self):
        super().__init__('rapid_runtime_delegate_smoke')
        self.state = None
        self.feedback = []
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            MissionState, '/mission/state', self._on_state, qos
        )
        self.client = ActionClient(self, ExecuteMission, '/mission/execute')

    def _on_state(self, message):
        self.state = message

    def run(self):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.state is not None
                and self.state.availability == MissionState.AVAILABLE
                and self.state.supported_step_mask == 0b1011
            ):
                break
        else:
            raise RuntimeError('public rapid Runtime did not become available')
        if not self.client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError('public ExecuteMission action is unavailable')
        goal = ExecuteMission.Goal()
        goal.source_instance_id = 'rapid-smoke'
        goal.source_seq = 1
        goal.runtime_instance_id = self.state.runtime_instance_id
        goal.admission_epoch = self.state.admission_epoch
        step = MissionStep()
        step.kind = MissionStep.MOVE_DISTANCE
        step.distance_m = 0.05
        goal.steps = [step]
        sent = self.client.send_goal_async(
            goal,
            feedback_callback=lambda item: self.feedback.append(
                float(item.feedback.progress)
            ),
        )
        rclpy.spin_until_future_complete(self, sent, timeout_sec=3.0)
        handle = sent.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('public Runtime rejected the smoke Goal')
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=8.0)
        wrapped = result_future.result()
        if wrapped is None or wrapped.result.code != ExecuteMission.Result.SUCCEEDED:
            detail = None if wrapped is None else wrapped.result.detail
            raise RuntimeError(f'delegated Mission failed: {detail}')
        if not self.feedback or self.feedback[-1] < 1.0:
            raise RuntimeError('delegated feedback did not reach completion')
        print(
            'RUNTIME_DELEGATE=PASS '
            f'feedback={len(self.feedback)} progress={self.feedback[-1]:.2f}'
        )


def main():
    """Run the bounded smoke and preserve a non-zero failure code."""
    rclpy.init()
    node = RuntimeDelegateSmoke()
    try:
        node.run()
    except Exception as error:
        print(f'RUNTIME_DELEGATE=FAIL {error}', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
