"""Write final Vosk microphone transcripts, one per stdout line."""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

from vosk import KaldiRecognizer, Model


def _fifo_chunks(path):
    """Read 16 kHz signed PCM produced by the C++ AudioEngine worker."""
    with Path(path).open('rb', buffering=0) as stream:
        while True:
            chunk = stream.read(8000)
            if not chunk:
                return
            yield chunk


def _pulse_chunks():
    """Capture WSLg/Pulse PCM without requiring an ALSA default device."""
    process = subprocess.Popen(
        [
            'parec',
            '--raw',
            '--format=s16le',
            '--rate=16000',
            '--channels=1',
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )
    try:
        while True:
            chunk = process.stdout.read(8000)
            if not chunk:
                return
            yield chunk
    finally:
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2)


def _portaudio_chunks():
    """Fall back to PyAudio when Pulse capture is unavailable."""
    import pyaudio

    audio = pyaudio.PyAudio()
    stream = audio.open(
        format=pyaudio.paInt16,
        channels=1,
        rate=16000,
        input=True,
        frames_per_buffer=4000,
    )
    try:
        while True:
            yield stream.read(4000, exception_on_overflow=False)
    finally:
        stream.close()
        audio.terminate()


def main():
    """Run Vosk over the best local capture source."""
    parser = argparse.ArgumentParser()
    parser.add_argument('model_directory', type=Path)
    parser.add_argument('--pcm-fifo', type=Path)
    arguments = parser.parse_args()
    if not arguments.model_directory.is_dir():
        raise SystemExit('Vosk model directory does not exist')
    recognizer = KaldiRecognizer(Model(str(arguments.model_directory)), 16000)
    use_pulse = bool(os.environ.get('PULSE_SERVER')) and shutil.which('parec')
    if arguments.pcm_fifo is not None:
        chunks, source = _fifo_chunks(arguments.pcm_fifo), 'audio_engine_fifo'
    elif use_pulse:
        chunks, source = _pulse_chunks(), 'pulse'
    else:
        chunks, source = _portaudio_chunks(), 'portaudio'
    print(json.dumps({'audio_source': source}), file=sys.stderr, flush=True)
    for chunk in chunks:
        if recognizer.AcceptWaveform(chunk):
            text = json.loads(recognizer.Result()).get('text', '').strip()
            if text:
                print(text, flush=True)


if __name__ == '__main__':
    main()
