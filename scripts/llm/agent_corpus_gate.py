#!/usr/bin/env python3
"""Run the versioned Mandarin Agent corpus through the locked local model."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
from typing import Any

try:
    from scripts.llm import artifact_manager as artifacts
except ImportError:  # pragma: no cover - direct script execution
    import artifact_manager as artifacts


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
CORPUS_PATH = (
    REPOSITORY_ROOT / 'models' / 'corpora' / 'voice_nav_agent_v1.json'
)
ALLOWED_OUTCOMES = frozenset(
    {'reply', 'clarify', 'mission', 'cancel', 'stop', 'rejected'}
)
CORPUS_TIMEOUT_SECONDS = 20.0
MAX_CASES = 32
MAX_OUTCOME_REASON_CHARS = 64


class CorpusGateError(artifacts.ArtifactError):
    """The real Agent corpus did not satisfy its closed contract."""


def _bounded_outcome_reason(outcome: Any) -> str:
    reason = getattr(outcome, 'reason', '')
    if not isinstance(reason, str):
        return 'unknown'
    bounded = reason[:MAX_OUTCOME_REASON_CHARS]
    safe = ''.join(
        char
        if (
            'a' <= char <= 'z'
            or 'A' <= char <= 'Z'
            or '0' <= char <= '9'
            or char in '._-'
        )
        else '_'
        for char in bounded
    )
    return safe or 'unknown'


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _load_corpus(path: Path) -> tuple[dict[str, Any], str]:
    try:
        raw = path.read_bytes()
        value = json.loads(raw.decode('utf-8'))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise CorpusGateError(f'cannot load corpus: {path}') from error
    if not isinstance(value, dict) or set(value) != {
        'schema_version', 'corpus_id', 'language', 'cases'
    }:
        raise CorpusGateError('corpus top-level schema is not closed')
    if value['schema_version'] != 1 or value['corpus_id'] != (
        'voice_nav.agent.corpus.v1'
    ) or value['language'] != 'zh-CN':
        raise CorpusGateError(
            'corpus identity is not voice_nav.agent.corpus.v1'
        )
    cases = value['cases']
    if not isinstance(cases, list) or not 1 <= len(cases) <= MAX_CASES:
        raise CorpusGateError('corpus case count is outside the bounded range')
    seen: set[str] = set()
    for case in cases:
        if not isinstance(case, dict) or set(case) != {
            'id', 'text', 'kind', 'expected_kinds'
        }:
            raise CorpusGateError('corpus case schema is not closed')
        case_id = case['id']
        if (
            not isinstance(case_id, str)
            or not re.fullmatch(r'[a-z][a-z0-9_]{2,63}', case_id)
            or case_id in seen
        ):
            raise CorpusGateError('corpus case identity is invalid')
        seen.add(case_id)
        if (
            not isinstance(case['text'], str)
            or not 1 <= len(case['text']) <= 512
        ):
            raise CorpusGateError(f'corpus text is invalid: {case_id}')
        if type(case['kind']) is not int or case['kind'] not in {1, 2}:
            raise CorpusGateError(
                f'corpus VoiceTurn kind is invalid: {case_id}'
            )
        expected = case['expected_kinds']
        if (
            not isinstance(expected, list)
            or not expected
            or any(kind not in ALLOWED_OUTCOMES for kind in expected)
        ):
            raise CorpusGateError(
                f'corpus expected outcomes are invalid: {case_id}'
            )
        if 'mission' in expected and case['kind'] == 2:
            raise CorpusGateError('STOP case may not expect a Mission')
    if not any(
        case['expected_kinds'] == ['mission'] for case in cases
    ):
        raise CorpusGateError(
            'corpus requires a strict positive Mission case'
        )
    return value, hashlib.sha256(raw).hexdigest()


def _runtime_snapshot():
    from voice_nav_agent.core import MissionState

    return MissionState(
        runtime_instance_id='runtime-corpus-a',
        admission_epoch=1,
        operating_mode=MissionState.NAVIGATION,
        availability=MissionState.AVAILABLE,
        gate_state=MissionState.GATE_INHIBITED,
        active_step=2**32 - 1,
        supported_step_mask=0b1111,
        max_steps=3,
        named_place_ids=('lobby',),
    )


def _validate_mission_outcome(
    result: Any,
    runtime_snapshot: Any,
    case_id: str,
) -> int:
    from voice_nav_agent.core import (
        Mission,
        MissionProposal,
        MissionStep,
        SemanticValidator,
    )

    mission = getattr(result, 'mission', None)
    if not isinstance(mission, Mission):
        raise CorpusGateError(
            f'corpus Mission is missing or untyped: {case_id}'
        )
    if not isinstance(mission.steps, tuple) or not mission.steps:
        raise CorpusGateError(
            f'corpus Mission steps are not a non-empty tuple: {case_id}'
        )
    if not all(isinstance(step, MissionStep) for step in mission.steps):
        raise CorpusGateError(
            f'corpus Mission contains an untyped step: {case_id}'
        )

    token = mission.token
    snapshot_fields = (
        ('runtime_instance_id', runtime_snapshot.runtime_instance_id),
        ('admission_epoch', runtime_snapshot.admission_epoch),
        ('operating_mode', runtime_snapshot.operating_mode),
        ('supported_step_mask', runtime_snapshot.supported_step_mask),
        ('max_steps', runtime_snapshot.max_steps),
        ('availability', runtime_snapshot.availability),
        ('gate_state', runtime_snapshot.gate_state),
        ('named_place_ids', runtime_snapshot.named_place_ids),
    )
    if any(getattr(token, name, object()) != value for name, value in snapshot_fields):
        raise CorpusGateError(
            f'corpus Mission token differs from immutable snapshot: {case_id}'
        )

    validation = SemanticValidator().validate(
        MissionProposal(mission.steps, token), token
    )
    if not validation.accepted or validation.mission != mission:
        raise CorpusGateError(
            f'corpus Mission failed SemanticValidator: {case_id}'
        )

    navigate_steps = [
        step for step in mission.steps
        if step.kind == MissionStep.NAVIGATE_TO
    ]
    if not navigate_steps:
        raise CorpusGateError(
            f'positive corpus Mission has no navigate_to target: {case_id}'
        )
    if any(
        step.target_id not in runtime_snapshot.named_place_ids
        for step in navigate_steps
    ):
        raise CorpusGateError(
            f'corpus Mission target is outside immutable snapshot: {case_id}'
        )
    return 0


def _run_case(
    endpoint: str,
    case: dict[str, Any],
    sequence: int,
    runtime_snapshot: Any,
) -> tuple[str, int]:
    from voice_nav_agent._agent_engine import AgentEngine
    from voice_nav_agent._planner import LoopbackPlanner
    from voice_nav_agent.core import VoiceTurn

    planner = LoopbackPlanner(endpoint)
    outcomes: list[Any] = []
    completed = threading.Event()

    def on_outcome(outcome: Any, _turn: VoiceTurn) -> None:
        outcomes.append(outcome)
        completed.set()

    engine = AgentEngine(
        f'corpus-agent-{sequence}',
        planner=planner,
        on_outcome=on_outcome,
    )
    turn = VoiceTurn(
        voice_instance_id='voice-corpus-a',
        voice_seq=sequence,
        session_id='corpus-session-a',
        turn_id=f'corpus-turn-{sequence}',
        kind=case['kind'],
        text=case['text'],
        confidence=1.0,
    )
    try:
        outcome = engine.handle_turn(turn, runtime_snapshot)
        if outcome is not None and not outcomes:
            outcomes.append(outcome)
            completed.set()
        if not completed.wait(CORPUS_TIMEOUT_SECONDS):
            raise CorpusGateError(f'corpus case timed out: {case["id"]}')
        if len(outcomes) != 1:
            raise CorpusGateError(
                f'corpus case emitted multiple outcomes: {case["id"]}'
            )
        result = outcomes[0]
        if result.kind not in case['expected_kinds']:
            raise CorpusGateError(
                'corpus case outcome mismatch: '
                f'{case["id"]}:{result.kind} '
                f'reason={_bounded_outcome_reason(result)}'
            )
        if (
            result.kind == 'mission'
            and 'mission' not in case['expected_kinds']
        ):
            raise CorpusGateError(
                f'corpus case produced an unexpected Mission: {case["id"]}'
            )
        unauthorized_missions = (
            _validate_mission_outcome(result, runtime_snapshot, case['id'])
            if result.kind == 'mission'
            else 0
        )
        return result.kind, unauthorized_missions
    finally:
        engine.shutdown()


def _peak_vm_hwm_kb(pid: int) -> int:
    try:
        status = Path(f'/proc/{pid}/status').read_text(encoding='utf-8')
        for line in status.splitlines():
            if line.startswith('VmHWM:'):
                match = re.search(r'([0-9]+)', line)
                if match:
                    return int(match.group(1))
    except (OSError, ValueError):
        pass
    return 0


def run_corpus(root: Path, repo_root: Path, corpus_path: Path) -> None:
    """Start the locked server and run every bounded corpus case."""
    manifest = artifacts.load_lock_manifest()
    digest = artifacts.lock_sha256()
    corpus, corpus_digest = _load_corpus(corpus_path)
    artifacts.validate_notice_consistency(manifest)
    artifacts.verify_repository_artifact_boundary(repo_root)
    root = artifacts.validate_artifact_root(root, create=False)
    bundle = artifacts.verify_bundle(root, manifest, digest, check_server=True)
    server = bundle / 'bin' / 'llama-server'
    model = bundle / 'models' / manifest.model_file
    runtime = manifest.runtime
    artifacts._check_port_available(runtime['host'], runtime['port'])
    evidence_dir = Path(
        tempfile.mkdtemp(prefix='voice-nav-llm-corpus-', dir='/tmp')
    )
    log_path = evidence_dir / 'server.log'
    command = [
        str(server),
        '--model', str(model),
        '--host', runtime['host'],
        '--port', str(runtime['port']),
        '--ctx-size', str(runtime['context']),
        '--n-predict', str(runtime['max_output']),
        '--parallel', str(runtime['parallel']),
    ]
    try:
        process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except OSError as error:
        raise CorpusGateError('cannot start locked llama-server') from error
    if process.stdout is None:
        raise CorpusGateError('llama-server log pipe was not created')
    log_state: dict[str, Any] = {}
    log_errors: list[BaseException] = []
    log_reader = threading.Thread(
        target=artifacts._capture_process_log,
        args=(process.stdout, log_path, log_state, log_errors),
        name='voice-nav-llm-corpus-log-reader',
        daemon=True,
    )
    log_reader.start()
    started = time.monotonic()
    kinds: list[str] = []
    mission_count = 0
    unauthorized_missions = 0
    metadata: dict[str, str] = {}
    peak_memory_kb = 0
    termination_escalated = False
    try:
        artifacts._wait_for_owned_listener(
            runtime['host'], runtime['port'], process,
            artifacts.REAL_READINESS_SECONDS,
        )
        artifacts._wait_for_server(
            runtime['host'], runtime['port'], process,
            artifacts.REAL_READINESS_SECONDS,
        )
        artifacts._check_loopback_listener(
            runtime['host'], runtime['port'], process
        )
        from voice_nav_agent._planner import LoopbackPlanner

        metadata = LoopbackPlanner(
            f"http://{runtime['host']}:{runtime['port']}"
        ).metadata
        runtime_snapshot = _runtime_snapshot()
        for sequence, case in enumerate(corpus['cases'], start=1):
            kind, unauthorized = _run_case(
                f"http://{runtime['host']}:{runtime['port']}",
                case, sequence, runtime_snapshot,
            )
            kinds.append(kind)
            mission_count += int(kind == 'mission')
            unauthorized_missions += unauthorized
            peak_memory_kb = max(peak_memory_kb, _peak_vm_hwm_kb(process.pid))
    except (
        artifacts.ArtifactError, CorpusGateError, OSError, RuntimeError
    ) as error:
        raise CorpusGateError(str(error)) from error
    finally:
        try:
            termination_escalated = artifacts._terminate_process_group(process)
        finally:
            log_reader.join(artifacts.PROCESS_TERM_SECONDS + 2.0)
            if log_reader.is_alive():
                process.stdout.close()
                log_reader.join(artifacts.PROCESS_TERM_SECONDS)
    if log_errors:
        raise CorpusGateError('server log capture failed') from log_errors[0]
    if termination_escalated:
        raise CorpusGateError('llama-server required SIGKILL during shutdown')
    if process.returncode not in (0, -15):
        raise CorpusGateError(
            f'llama-server exited with code {process.returncode}'
        )
    if mission_count < 1:
        raise CorpusGateError('corpus produced no positive Mission')
    head = artifacts._repository_head(repo_root)
    elapsed_ms = int((time.monotonic() - started) * 1000)
    model_size, model_sha256 = artifacts.file_sha256(model)
    server_size, server_sha256 = artifacts.file_sha256(server)
    print(
        'REAL_LLM_CORPUS=PASS '
        f'HEAD={head} lock_sha256={digest} '
        f'corpus_sha256={corpus_digest} '
        f'prompt_sha256={metadata["prompt_sha256"]} '
        f'mission_schema_sha256={metadata["mission_schema_sha256"]} '
        f'model_sha256={model_sha256} model_bytes={model_size} '
        f'server_sha256={server_sha256} server_bytes={server_size} '
        f'cases={len(kinds)} outcomes={",".join(kinds)} '
        f'missions={mission_count} unauthorized_missions={unauthorized_missions} '
        f'elapsed_ms={elapsed_ms} peak_vm_hwm_kb={peak_memory_kb} '
        f'listen={runtime["host"]}:{runtime["port"]} log={log_path}'
    )


def main(argv: list[str] | None = None) -> int:
    """Parse the real-gate arguments and report one machine-readable result."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--repo-root', type=Path, default=REPOSITORY_ROOT)
    parser.add_argument('--corpus', type=Path, default=CORPUS_PATH)
    arguments = parser.parse_args(argv)
    try:
        run_corpus(arguments.root, arguments.repo_root, arguments.corpus)
    except (artifacts.ArtifactError, CorpusGateError) as error:
        print(
            f'REAL_LLM_CORPUS=FAIL reason={artifacts._safe_error(error)}',
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
