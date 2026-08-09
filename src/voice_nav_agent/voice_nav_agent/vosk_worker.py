"""Write final Vosk microphone transcripts, one per stdout line."""
import json
import sys
import pyaudio
from vosk import KaldiRecognizer, Model

model = Model(sys.argv[1])
recognizer = KaldiRecognizer(model, 16000)
audio = pyaudio.PyAudio()
stream = audio.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True,
                    frames_per_buffer=4000)
try:
    while True:
        if recognizer.AcceptWaveform(stream.read(4000, exception_on_overflow=False)):
            text = json.loads(recognizer.Result()).get('text', '').strip()
            if text:
                print(text, flush=True)
finally:
    stream.close(); audio.terminate()
