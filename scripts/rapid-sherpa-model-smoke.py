#!/usr/bin/env python3
"""Load every locked rapid speech model once without opening audio devices."""

import argparse
import audioop
import io
import json
import tempfile
import wave
from contextlib import redirect_stdout
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from voice_nav_agent.sherpa_worker import (
    _keyword_spotter,
    _recognizer,
    _vad,
    run,
)


def _pcm16(path):
    with wave.open(str(path), 'rb') as source:
        pcm = source.readframes(source.getnframes())
        if source.getnchannels() != 1:
            raise SystemExit('speech fixture must be mono')
        pcm, _ = audioop.ratecv(
            pcm, source.getsampwidth(), 1, source.getframerate(), 16000,
            None,
        )
    return pcm


def main():
    """Load the models and optionally detect a synthesized wake fixture."""
    parser = argparse.ArgumentParser()
    parser.add_argument('voice_root', type=Path)
    parser.add_argument('--kws-wav', type=Path)
    parser.add_argument('--pipeline-wav', type=Path)
    arguments = parser.parse_args()
    models = arguments.voice_root / 'models'
    keyword = _keyword_spotter(
        models / 'sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20',
        models / 'rapid_keywords.txt',
    )
    detector = _vad(models / 'silero_vad.int8.onnx')
    recognizer = _recognizer(
        models / 'sherpa-onnx-streaming-zipformer-zh-int8-2025-06-30'
    )
    if not all((keyword, detector, recognizer)):
        raise SystemExit('one or more speech models failed to load')
    detected = 'not_checked'
    if arguments.kws_wav:
        pcm = _pcm16(arguments.kws_wav)
        samples = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768
        stream = keyword.create_stream()
        stream.accept_waveform(16000, samples)
        stream.accept_waveform(16000, np.zeros(16000, dtype=np.float32))
        stream.input_finished()
        result = ''
        while keyword.is_ready(stream):
            keyword.decode_stream(stream)
            result = keyword.get_result(stream) or result
        if not result:
            raise SystemExit('custom 小智 keyword was not detected')
        detected = result
    pipeline_text = 'not_checked'
    if arguments.pipeline_wav:
        with tempfile.NamedTemporaryFile() as raw:
            raw.write(_pcm16(arguments.pipeline_wav) + bytes(32000))
            raw.flush()
            output = io.StringIO()
            with redirect_stdout(output):
                run(SimpleNamespace(
                    pcm_fifo=Path(raw.name),
                    kws_model=(
                        models /
                        'sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20'
                    ),
                    vad_model=models / 'silero_vad.int8.onnx',
                    asr_model=(
                        models /
                        'sherpa-onnx-streaming-zipformer-zh-int8-2025-06-30'
                    ),
                    keywords_file=models / 'rapid_keywords.txt',
                    wake_timeout=8.0,
                ))
        events = [json.loads(line) for line in output.getvalue().splitlines()]
        commands = [
            event.get('text', '') for event in events
            if event.get('wake_authorized') is True
        ]
        if not commands or not commands[-1]:
            raise SystemExit('KWS/VAD/ASR pipeline emitted no final command')
        pipeline_text = commands[-1]
    print(
        'SHERPA_MODELS=PASS kws=zipformer vad=silero asr=zipformer '
        f'detected={detected} pipeline_text={pipeline_text}'
    )


if __name__ == '__main__':
    main()
