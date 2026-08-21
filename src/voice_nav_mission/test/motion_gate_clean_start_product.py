#!/usr/bin/env python3
"""Verify that MotionGate starts and remains safely inhibited."""

from __future__ import annotations

import argparse
from collections import deque
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from voice_nav_mission.msg import InternalMotionGateState


STATE_TOPIC = "/motion_gate/internal/state"
FINAL_TOPIC = "/diff_drive_controller/cmd_vel"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--stable-seconds", type=float, default=3.0)
    return parser.parse_args()


def state_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def stop_group(process: subprocess.Popen[str]) -> dict[str, object]:
    sent: list[str] = []
    for name, signum, timeout in (
        ("SIGINT", signal.SIGINT, 10.0),
        ("SIGTERM", signal.SIGTERM, 5.0),
        ("SIGKILL", signal.SIGKILL, 2.0),
    ):
        if process.poll() is not None:
            break
        try:
            os.killpg(process.pid, signum)
            sent.append(name)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
    return {
        "pid": process.pid,
        "returncode": process.poll(),
        "signals": sent,
        "alive": process.poll() is None,
    }


def state_value(message: InternalMotionGateState) -> dict[str, object]:
    return {
        "gate_instance_id": message.gate_instance_id,
        "state_seq": message.state_seq,
        "control_seq": message.control_seq,
        "state": message.state,
        "motion_inhibited": message.motion_inhibited,
        "zero_selected": message.zero_selected,
        "reason": message.reason,
        "detail": message.detail,
    }


def is_zero(message: TwistStamped) -> bool:
    values = (
        message.twist.linear.x,
        message.twist.linear.y,
        message.twist.linear.z,
        message.twist.angular.x,
        message.twist.angular.y,
        message.twist.angular.z,
    )
    return all(value == 0.0 for value in values)


def main() -> int:
    args = parse_args()
    if args.run_dir.exists():
        raise RuntimeError(f"refusing to replace {args.run_dir}")
    if not args.config.is_file():
        raise RuntimeError(f"missing MotionGate config: {args.config}")
    args.run_dir.mkdir(parents=True)
    stdout_path = args.run_dir / "motion_gate.stdout"
    stderr_path = args.run_dir / "motion_gate.stderr"
    stdout = stdout_path.open("w", encoding="utf-8")
    stderr = stderr_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        [
            "ros2",
            "run",
            "voice_nav_mission",
            "motion_gate_node",
            "--ros-args",
            "--params-file",
            str(args.config),
        ],
        stdin=subprocess.DEVNULL,
        stdout=stdout,
        stderr=stderr,
        text=True,
        start_new_session=True,
    )

    states: deque[InternalMotionGateState] = deque(maxlen=256)
    final_commands: deque[TwistStamped] = deque(maxlen=1024)
    failure = ""
    started_at = time.monotonic()
    rclpy.init()
    node = Node("motion_gate_clean_start_observer")

    def observe_state(message: InternalMotionGateState) -> None:
        states.append(message)

    def observe_final(message: TwistStamped) -> None:
        final_commands.append(message)

    try:
        node.create_subscription(
            InternalMotionGateState,
            STATE_TOPIC,
            observe_state,
            state_qos(),
        )
        node.create_subscription(
            TwistStamped,
            FINAL_TOPIC,
            observe_final,
            10,
        )
        initial_deadline = time.monotonic() + 10.0
        while not states and time.monotonic() < initial_deadline:
            if process.poll() is not None:
                raise RuntimeError(f"MotionGate exited early: rc={process.returncode}")
            rclpy.spin_once(node, timeout_sec=0.1)
        if not states:
            raise RuntimeError("MotionGate did not publish its initial state")

        stable_deadline = time.monotonic() + args.stable_seconds
        while time.monotonic() < stable_deadline:
            if process.poll() is not None:
                raise RuntimeError(f"MotionGate exited during observation: rc={process.returncode}")
            rclpy.spin_once(node, timeout_sec=0.05)

        bad_states = [
            state_value(message)
            for message in states
            if message.state != InternalMotionGateState.INHIBITED
            or not message.motion_inhibited
            or not message.zero_selected
        ]
        if bad_states:
            raise RuntimeError(f"MotionGate left safe inhibited state: {bad_states[-1]}")
        if len(final_commands) < 10:
            raise RuntimeError(
                f"MotionGate published too few final commands: {len(final_commands)}"
            )
        nonzero_count = sum(not is_zero(message) for message in final_commands)
        if nonzero_count:
            raise RuntimeError(f"MotionGate published {nonzero_count} nonzero commands")
    except Exception as error:
        failure = str(error)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cleanup = stop_group(process)
        stdout.close()
        stderr.close()

    evidence = {
        "ok": not failure,
        "failure": failure,
        "elapsed_seconds": round(time.monotonic() - started_at, 3),
        "state_count": len(states),
        "first_state": state_value(states[0]) if states else None,
        "last_state": state_value(states[-1]) if states else None,
        "final_command_count": len(final_commands),
        "nonzero_command_count": sum(
            not is_zero(message) for message in final_commands
        ),
        "cleanup": cleanup,
    }
    (args.run_dir / "evidence.json").write_text(
        json.dumps(evidence, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(evidence, ensure_ascii=False, sort_keys=True))
    return 0 if evidence["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
