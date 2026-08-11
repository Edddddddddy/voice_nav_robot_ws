"""Synthesize one bounded utterance with the locked Chaowen int8 model."""

import argparse
import sys
import wave
from pathlib import Path

import numpy as np

import sherpa_onnx


def _engine(model_directory):
    vits = sherpa_onnx.OfflineTtsVitsModelConfig(
        model=str(model_directory / 'zh_CN-chaowen-medium.onnx'),
        lexicon=str(model_directory / 'lexicon.txt'),
        tokens=str(model_directory / 'tokens.txt'),
    )
    model = sherpa_onnx.OfflineTtsModelConfig(
        vits=vits, num_threads=2, provider='cpu'
    )
    rule_fsts = ','.join(str(model_directory / name) for name in (
        'phone.fst', 'date.fst', 'number.fst'
    ))
    return sherpa_onnx.OfflineTts(sherpa_onnx.OfflineTtsConfig(
        model=model,
        rule_fsts=rule_fsts,
        max_num_sentences=1,
    ))


def synthesize(model_directory, text, output):
    """Generate and write one mono 16-bit WAV for the playback worker."""
    audio = _engine(model_directory).generate(text, sid=0, speed=1.0)
    samples = np.asarray(audio.samples, dtype=np.float32)
    if samples.size == 0 or not np.isfinite(samples).all():
        raise RuntimeError('Chaowen produced invalid audio')
    pcm = np.rint(np.clip(samples, -1.0, 1.0) * 32767.0).astype('<i2')
    with wave.open(str(output), 'wb') as target:
        target.setnchannels(1)
        target.setsampwidth(2)
        target.setframerate(int(audio.sample_rate))
        target.writeframes(pcm.tobytes())


def main():
    """Validate inputs and synthesize text received through private stdin."""
    parser = argparse.ArgumentParser()
    parser.add_argument('model_directory', type=Path)
    parser.add_argument('output', type=Path)
    arguments = parser.parse_args()
    text = sys.stdin.read().strip()
    required = (
        'zh_CN-chaowen-medium.onnx',
        'lexicon.txt',
        'tokens.txt',
        'phone.fst',
        'date.fst',
        'number.fst',
    )
    if not text or len(text) > 512:
        raise SystemExit('TTS text must contain 1..512 characters')
    if not all((arguments.model_directory / name).is_file()
               for name in required):
        raise SystemExit('locked Chaowen model is incomplete')
    synthesize(arguments.model_directory, text, arguments.output)


if __name__ == '__main__':
    main()
