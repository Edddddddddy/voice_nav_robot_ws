"""Run the rapid wake, endpoint, and streaming ASR pipeline on private PCM."""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

import sherpa_onnx

SAMPLE_RATE = 16000


def _chunks(path):
    """Yield small normalized frames from the AudioEngine private FIFO."""
    with Path(path).open('rb', buffering=0) as stream:
        while True:
            pcm = stream.read(2048)
            if not pcm:
                return
            yield np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768


def _keyword_spotter(model, keywords):
    return sherpa_onnx.KeywordSpotter(
        tokens=str(model / 'tokens.txt'),
        encoder=str(
            model / 'encoder-epoch-13-avg-2-chunk-16-left-64.int8.onnx'
        ),
        decoder=str(model / 'decoder-epoch-13-avg-2-chunk-16-left-64.onnx'),
        joiner=str(
            model / 'joiner-epoch-13-avg-2-chunk-16-left-64.int8.onnx'
        ),
        keywords_file=str(keywords),
        num_threads=2,
        keywords_score=2.0,
        keywords_threshold=0.1,
    )


def _vad(model):
    silero = sherpa_onnx.SileroVadModelConfig(
        model=str(model),
        threshold=0.5,
        min_silence_duration=0.7,
        min_speech_duration=0.2,
        window_size=512,
        max_speech_duration=20.0,
    )
    config = sherpa_onnx.VadModelConfig(
        silero_vad=silero, sample_rate=SAMPLE_RATE, num_threads=1
    )
    return sherpa_onnx.VoiceActivityDetector(config, 30.0)


def _recognizer(model):
    return sherpa_onnx.OnlineRecognizer.from_transducer(
        tokens=str(model / 'tokens.txt'),
        encoder=str(model / 'encoder.int8.onnx'),
        decoder=str(model / 'decoder.onnx'),
        joiner=str(model / 'joiner.int8.onnx'),
        num_threads=2,
        decoding_method='greedy_search',
    )


def _drain(recognizer, stream):
    while recognizer.is_ready(stream):
        recognizer.decode_stream(stream)


def _finish(recognizer, stream):
    stream.accept_waveform(SAMPLE_RATE, np.zeros(4800, dtype=np.float32))
    stream.input_finished()
    _drain(recognizer, stream)
    return recognizer.get_result(stream).strip()


def run(arguments):
    """Emit one JSON record for each wake-authorized final transcript."""
    spotter = _keyword_spotter(arguments.kws_model, arguments.keywords_file)
    detector = _vad(arguments.vad_model)
    recognizer = _recognizer(arguments.asr_model)
    keyword_stream = spotter.create_stream()
    asr_stream = None
    speech_started = False
    turn_samples = 0
    print(
        json.dumps({'audio_source': 'audio_engine_fifo', 'backend': 'sherpa'}),
        file=sys.stderr,
        flush=True,
    )
    for samples in _chunks(arguments.pcm_fifo):
        keyword_stream.accept_waveform(SAMPLE_RATE, samples)
        while spotter.is_ready(keyword_stream):
            spotter.decode_stream(keyword_stream)
        keyword = spotter.get_result(keyword_stream)
        if keyword:
            spotter.reset_stream(keyword_stream)
            if keyword == '紧急停止':
                print(json.dumps({
                    'text': keyword,
                    'wake_authorized': True,
                }), flush=True)
                asr_stream = None
                detector.reset()
                continue
            detector.reset()
            asr_stream = recognizer.create_stream()
            speech_started = False
            turn_samples = 0
            wake = json.dumps({'wake': keyword})
            print(wake, flush=True)
            print(wake, file=sys.stderr, flush=True)
            continue
        if asr_stream is None:
            continue
        turn_samples += len(samples)
        detector.accept_waveform(samples)
        asr_stream.accept_waveform(SAMPLE_RATE, samples)
        _drain(recognizer, asr_stream)
        speech_started = speech_started or detector.is_speech_detected()
        endpoint = speech_started and not detector.empty()
        timed_out = turn_samples >= int(
            SAMPLE_RATE * (20.0 if speech_started else arguments.wake_timeout)
        )
        if not endpoint and not timed_out:
            continue
        text = _finish(recognizer, asr_stream) if speech_started else ''
        if text:
            print(
                json.dumps({'text': text, 'wake_authorized': True}),
                flush=True,
            )
        if not detector.empty():
            detector.pop()
        detector.reset()
        asr_stream = None


def main():
    """Validate model paths and run the private speech pipeline."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--pcm-fifo', type=Path, required=True)
    parser.add_argument('--kws-model', type=Path, required=True)
    parser.add_argument('--vad-model', type=Path, required=True)
    parser.add_argument('--asr-model', type=Path, required=True)
    parser.add_argument('--keywords-file', type=Path, required=True)
    parser.add_argument('--wake-timeout', type=float, default=8.0)
    arguments = parser.parse_args()
    required = (
        arguments.pcm_fifo,
        arguments.kws_model,
        arguments.vad_model,
        arguments.asr_model,
        arguments.keywords_file,
    )
    if not all(path.exists() for path in required):
        raise SystemExit('sherpa speech asset does not exist')
    run(arguments)


if __name__ == '__main__':
    main()
