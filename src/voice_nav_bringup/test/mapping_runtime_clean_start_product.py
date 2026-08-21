#!/usr/bin/env python3
"""Observe Mapping Runtime readiness through the public MissionState topic."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from voice_nav_interfaces.msg import MissionState


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--stable-seconds", type=float, default=2.0)
    return parser.parse_args()


def stop_group(process: subprocess.Popen[str]) -> dict[str, object]:
    signals: list[str] = []
    for name, signum, timeout in (
        ("SIGINT", signal.SIGINT, 20.0),
        ("SIGTERM", signal.SIGTERM, 8.0),
        ("SIGKILL", signal.SIGKILL, 3.0),
    ):
        if process.poll() is not None:
            break
        try:
            os.killpg(process.pid, signum)
            signals.append(name)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
    return {
        "pid": process.pid,
        "returncode": process.poll(),
        "signals": signals,
        "alive": process.poll() is None,
    }


def as_evidence(message: MissionState) -> dict[str, object]:
    return {
        "runtime_instance_id": message.runtime_instance_id,
        "admission_epoch": message.admission_epoch,
        "operating_mode": message.operating_mode,
        "availability": message.availability,
        "gate_state": message.gate_state,
        "active_step": message.active_step,
    }


def main() -> int:
    args = parse_args()
    if args.run_dir.exists():
        raise RuntimeError(f"refusing to replace {args.run_dir}")
    args.run_dir.mkdir(parents=True)
    stdout_path = args.run_dir / "launch.stdout"
    stderr_path = args.run_dir / "launch.stderr"
    evidence_path = args.run_dir / "evidence.json"

    stdout = stdout_path.open("w", encoding="utf-8")
    stderr = stderr_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "voice_nav_bringup",
            "mapping_mvp.launch.py",
            "headless:=true",
            "shutdown_on_gazebo_exit:=true",
        ],
        stdin=subprocess.DEVNULL,
        stdout=stdout,
        stderr=stderr,
        text=True,
        start_new_session=True,
    )

    states: list[dict[str, object]] = []
    failure = ""
    ready_at: float | None = None
    started_at = time.monotonic()
    rclpy.init()
    node = Node("mapping_runtime_clean_start_observer")
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    def observe(message: MissionState) -> None:
        nonlocal ready_at
        states.append(as_evidence(message))
        if (
            message.operating_mode == MissionState.MAPPING
            and message.availability == MissionState.AVAILABLE
            and message.gate_state == MissionState.GATE_INHIBITED
        ):
            ready_at = time.monotonic()

    node.create_subscription(MissionState, "/mission/state", observe, qos)
    try:
        deadline = started_at + args.startup_timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError(
                    f"Mapping launch exited before ready: rc={process.returncode}"
                )
            rclpy.spin_once(node, timeout_sec=0.1)
            if states and (
                states[-1]["availability"] == MissionState.FAULTED
                or states[-1]["gate_state"] == MissionState.GATE_FAULTED
            ):
                raise RuntimeError(f"Mapping Runtime faulted before ready: {states[-1]}")
            if ready_at is not None:
                break
        if ready_at is None:
            raise RuntimeError("Mapping Runtime did not become ready")

        stable_deadline = time.monotonic() + args.stable_seconds
        while time.monotonic() < stable_deadline:
            if process.poll() is not None:
                raise RuntimeError("Mapping launch exited during ready stability window")
            rclpy.spin_once(node, timeout_sec=0.1)
            if states[-1]["availability"] != MissionState.AVAILABLE:
                raise RuntimeError(f"Mapping Runtime left AVAILABLE: {states[-1]}")
            if states[-1]["gate_state"] != MissionState.GATE_INHIBITED:
                raise RuntimeError(f"Mapping gate left INHIBITED: {states[-1]}")
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
        "states": states,
        "cleanup": cleanup,
    }
    evidence_path.write_text(
        json.dumps(evidence, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(evidence, ensure_ascii=False, sort_keys=True))
    return 0 if evidence["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
