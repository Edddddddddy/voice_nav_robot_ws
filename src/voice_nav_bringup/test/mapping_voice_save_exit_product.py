#!/usr/bin/env python3
"""Black-box acceptance for voice-controlled Mapping save and exit.

This test uses the installed product entry and two prerecorded WAV files.  It
does not call Agent, Mission, SLAM, or process internals directly.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Callable


REQUIRED_MAP_FILES = {
    "manifest.yaml",
    "map.data",
    "map.pgm",
    "map.posegraph",
    "map.yaml",
    "named_places.yaml",
}


def run(command: list[str], timeout: float = 20.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        capture_output=True,
        check=False,
        text=True,
        timeout=timeout,
    )


def read(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except FileNotFoundError:
        return ""


def events(path: Path) -> list[dict[str, object]]:
    parsed: list[dict[str, object]] = []
    for line in read(path).splitlines():
        marker = "VOICE_NAV "
        offset = line.find(marker)
        if offset < 0:
            continue
        try:
            value = json.loads(line[offset + len(marker) :])
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            parsed.append(value)
    return parsed


def wait_until(predicate: Callable[[], bool], timeout: float, label: str) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.25)
    raise RuntimeError(f"timeout: {label}")


def matching_event(
    path: Path,
    event: str,
    **fields: object,
) -> dict[str, object] | None:
    for item in reversed(events(path)):
        if item.get("event") != event:
            continue
        if all(item.get(key) == value for key, value in fields.items()):
            return item
    return None


def play(wav: Path, sink: str) -> dict[str, object]:
    result = run(["paplay", f"--device={sink}", str(wav)], timeout=30.0)
    if result.returncode != 0:
        raise RuntimeError(f"WAV playback failed: {result.stderr.strip()}")
    return {"wav": str(wav), "returncode": result.returncode}


def group_alive(process: subprocess.Popen[str]) -> bool:
    try:
        os.killpg(process.pid, 0)
    except ProcessLookupError:
        return False
    return True


def stop_group(process: subprocess.Popen[str] | None) -> dict[str, object]:
    if process is None:
        return {"started": False}
    sent: list[str] = []
    for name, signum, budget in (
        ("SIGINT", signal.SIGINT, 20.0),
        ("SIGTERM", signal.SIGTERM, 8.0),
        ("SIGKILL", signal.SIGKILL, 3.0),
    ):
        if not group_alive(process):
            break
        try:
            os.killpg(process.pid, signum)
            sent.append(name)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=budget)
        except subprocess.TimeoutExpired:
            continue
    return {
        "started": True,
        "pid": process.pid,
        "signals": sent,
        "returncode": process.poll(),
        "group_alive": group_alive(process),
    }


def validate_map_package(package: Path) -> list[str]:
    files = {path.name for path in package.iterdir() if path.is_file()}
    missing = REQUIRED_MAP_FILES - files
    if missing:
        raise RuntimeError(f"saved map package is incomplete: {sorted(missing)}")
    empty = sorted(
        name for name in REQUIRED_MAP_FILES if (package / name).stat().st_size == 0
    )
    if empty:
        raise RuntimeError(f"saved map files are empty: {empty}")
    if "image: map.pgm" not in read(package / "map.yaml"):
        raise RuntimeError("map.yaml does not reference map.pgm")
    return sorted(files)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--start-wav", type=Path, required=True)
    parser.add_argument("--stop-save-wav", type=Path, required=True)
    parser.add_argument("--pulse-sink", default="VoiceNavMappingExitTest")
    parser.add_argument("--display", choices=("headless", "gui"), default="headless")
    parser.add_argument("--exit-timeout", type=float, default=20.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.run_dir.exists():
        raise RuntimeError(f"refusing to replace {args.run_dir}")
    for wav in (args.start_wav, args.stop_save_wav):
        if not wav.is_file():
            raise RuntimeError(f"missing WAV: {wav}")
    args.run_dir.mkdir(parents=True)

    stdout_path = args.run_dir / "mapping.stdout"
    stderr_path = args.run_dir / "mapping.stderr"
    evidence_path = args.run_dir / "evidence.json"
    pulse_source = f"{args.pulse_sink}.monitor"
    original_source = run(["pactl", "get-default-source"]).stdout.strip()
    evidence: dict[str, object] = {
        "original_source": original_source,
        "playback": [],
        "cleanup": {},
    }
    process: subprocess.Popen[str] | None = None
    module_id = ""
    stdout = None
    stderr = None
    failure = ""

    try:
        loaded = run(
            [
                "pactl",
                "load-module",
                "module-null-sink",
                f"sink_name={args.pulse_sink}",
                "rate=48000",
                "channels=1",
            ]
        )
        if loaded.returncode != 0 or not loaded.stdout.strip().isdigit():
            raise RuntimeError("could not create isolated WAV input")
        module_id = loaded.stdout.strip()
        if run(["pactl", "set-default-source", pulse_source]).returncode != 0:
            raise RuntimeError("could not select isolated WAV input")

        stdout = stdout_path.open("w", encoding="utf-8")
        stderr = stderr_path.open("w", encoding="utf-8")
        process = subprocess.Popen(
            [
                "ros2",
                "run",
                "voice_nav_bringup",
                "voice_nav_app",
                "--mode",
                "mapping",
                "--display",
                args.display,
                "--input",
                "vad-auto",
            ],
            stdin=subprocess.DEVNULL,
            stdout=stdout,
            stderr=stderr,
            text=True,
            start_new_session=True,
        )
        wait_until(
            lambda: process.poll() is None and '"status":"ready"' in read(stdout_path),
            180.0,
            "Mapping app ready",
        )

        evidence["playback"].append(play(args.start_wav, args.pulse_sink))
        wait_until(
            lambda: matching_event(
                stderr_path, "agent_decision", reason="mapping_patrol"
            )
            is not None,
            45.0,
            "start-mapping decision",
        )
        wait_until(
            lambda: matching_event(stderr_path, "mission_result", code=0) is not None,
            120.0,
            "first successful patrol Mission",
        )

        evidence["playback"].append(play(args.stop_save_wav, args.pulse_sink))
        wait_until(
            lambda: matching_event(
                stderr_path, "agent_decision", reason="voice_stop_and_save"
            )
            is not None,
            45.0,
            "stop-and-save decision",
        )
        decision = matching_event(
            stderr_path, "agent_decision", reason="voice_stop_and_save"
        )
        assert decision is not None
        generation = decision.get("generation")
        wait_until(
            lambda: matching_event(
                stderr_path,
                "mission_result",
                generation=generation,
            )
            is not None,
            90.0,
            "save-map Mission result",
        )
        save_result = matching_event(
            stderr_path, "mission_result", generation=generation
        )
        if save_result is None or save_result.get("code") != 0:
            raise RuntimeError(f"save-map Mission failed: {save_result}")
        evidence["save_result"] = save_result

        package = Path(os.environ["XDG_DATA_HOME"]) / "voice_nav/maps/voice_mvp"
        wait_until(package.is_dir, 20.0, "saved map package")
        evidence["map_package"] = str(package)
        evidence["map_package_files"] = validate_map_package(package)

        try:
            returncode = process.wait(timeout=args.exit_timeout)
        except subprocess.TimeoutExpired as error:
            raise RuntimeError(
                "Mapping app did not exit after successful stop-and-save"
            ) from error
        evidence["app_returncode"] = returncode
        wait_until(lambda: not group_alive(process), 10.0, "owned process group exit")
        if returncode != 0:
            raise RuntimeError(f"Mapping app exited with rc={returncode}")
        evidence["owned_process_group_alive"] = False
    except Exception as error:
        failure = str(error)
    finally:
        evidence["cleanup"] = stop_group(process)
        restored = run(["pactl", "set-default-source", original_source])
        evidence["restored_source"] = run(
            ["pactl", "get-default-source"]
        ).stdout.strip()
        evidence["restore_returncode"] = restored.returncode
        if module_id:
            evidence["module_unload_returncode"] = run(
                ["pactl", "unload-module", module_id]
            ).returncode
        if stdout is not None:
            stdout.close()
        if stderr is not None:
            stderr.close()

    evidence["failure"] = failure
    evidence["ok"] = not failure and evidence["restored_source"] == original_source
    evidence_path.write_text(
        json.dumps(evidence, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(evidence, ensure_ascii=False, sort_keys=True))
    return 0 if evidence["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
