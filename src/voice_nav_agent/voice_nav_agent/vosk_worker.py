"""Write final Vosk microphone transcripts, one per stdout line."""

import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

from vosk import KaldiRecognizer, Model


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
    if len(sys.argv) != 2 or not Path(sys.argv[1]).is_dir():
        raise SystemExit('usage: vosk_worker.py MODEL_DIRECTORY')
    recognizer = KaldiRecognizer(Model(sys.argv[1]), 16000)
    use_pulse = bool(os.environ.get('PULSE_SERVER')) and shutil.which('parec')
    chunks = _pulse_chunks() if use_pulse else _portaudio_chunks()
    source = 'pulse' if use_pulse else 'portaudio'
    print(json.dumps({'audio_source': source}), file=sys.stderr, flush=True)
    for chunk in chunks:
        if recognizer.AcceptWaveform(chunk):
            text = json.loads(recognizer.Result()).get('text', '').strip()
            if text:
                print(text, flush=True)


if __name__ == '__main__':
    main()
