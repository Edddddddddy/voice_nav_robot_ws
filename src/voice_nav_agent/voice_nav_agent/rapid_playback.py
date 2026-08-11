"""Bounded Piper/paplay PlaybackScope for the rapid voice endpoint."""

import audioop
import errno
import os
from pathlib import Path
import secrets
import subprocess
import tempfile
import threading
import time
import wave

from rclpy.action import CancelResponse
from voice_nav_interfaces.action import Speak


class PiperPlayback:
    """Own synthesis, playback, feedback, cancellation, and barge-in."""

    def __init__(self, logger, piper, model, playback_fifo=''):
        self.logger = logger
        self.piper = piper
        self.model = model
        self.playback_fifo = playback_fifo
        self.lock = threading.Lock()
        self.generation = 0
        self.process = None
        self.active = False
        self.allows_barge_in = False

    def execute(self, handle):
        """Execute one Speak goal to its real terminal playback state."""
        self.logger.info('SPEAK: %s' % handle.request.text)
        generation = self._begin(handle.request.allow_barge_in)
        if not (
            self.piper
            and Path(self.piper).is_file()
            and Path(self.model).is_file()
        ):
            return self._finish(
                handle, generation, Speak.Result.COMPLETED,
                'printed because rapid speech assets are unavailable'
            )
        wav = Path(tempfile.gettempdir()) / (
            'voice-nav-' + secrets.token_hex(8) + '.wav'
        )
        try:
            synth = subprocess.Popen(
                [
                    self.piper,
                    '--model', self.model,
                    '--output_file', str(wav),
                ],
                stdin=subprocess.PIPE,
                text=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            if not self._adopt(generation, synth):
                return self._barged(handle, generation, 'before synthesis')
            synth.communicate(input=handle.request.text, timeout=30)
            if synth.returncode != 0:
                return self._interrupted_or_failed(handle, generation)
            if self.playback_fifo:
                return self._play_fifo(handle, generation, wav)
            player = subprocess.Popen(
                ['paplay', str(wav)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            if not self._adopt(generation, player):
                return self._barged(handle, generation, 'before playback')
            return self._play(handle, generation, player)
        except (OSError, subprocess.SubprocessError) as error:
            self.logger.warning('Piper playback failed: %s' % error)
            return self._interrupted_or_failed(handle, generation)
        finally:
            wav.unlink(missing_ok=True)

    def _play(self, handle, generation, player):
        started = time.monotonic()
        while player.poll() is None:
            if handle.is_cancel_requested:
                self._interrupt_generation(generation)
                return self._finish(
                    handle, generation, Speak.Result.CANCELED,
                    'playback canceled'
                )
            if not self._is_current(generation):
                return self._barged(handle, generation, 'during playback')
            elapsed = time.monotonic() - started
            self._feedback(handle, elapsed)
            time.sleep(0.05)
        code = (
            Speak.Result.COMPLETED
            if player.returncode == 0 else Speak.Result.FAILED
        )
        detail = (
            'playback completed'
            if code == Speak.Result.COMPLETED else 'paplay failed'
        )
        return self._finish(handle, generation, code, detail)

    def _play_fifo(self, handle, generation, wav):
        with wave.open(str(wav), 'rb') as source:
            channels = source.getnchannels()
            width = source.getsampwidth()
            sample_rate = source.getframerate()
            pcm = source.readframes(source.getnframes())
        if channels == 2:
            pcm = audioop.tomono(pcm, width, 0.5, 0.5)
        elif channels != 1:
            return self._finish(
                handle, generation, Speak.Result.FAILED,
                'unsupported Piper channel count'
            )
        if width != 2:
            pcm = audioop.lin2lin(pcm, width, 2)
        pcm, _ = audioop.ratecv(pcm, 2, 1, sample_rate, 48000, None)
        padding = (-len(pcm)) % 960
        if padding:
            pcm += bytes(padding)
        duration = len(pcm) / 2.0 / 48000.0
        descriptor, open_status = self._open_fifo(handle, generation)
        if descriptor is None:
            if open_status == 'canceled':
                self._interrupt_generation(generation)
                return self._finish(
                    handle, generation, Speak.Result.CANCELED,
                    'playback canceled'
                )
            if open_status == 'barged':
                return self._barged(handle, generation, 'before playback')
            return self._interrupted_or_failed(handle, generation)
        started = time.monotonic()
        try:
            offset = 0
            while offset < len(pcm):
                terminal = self._terminal_during_playback(handle, generation)
                if terminal is not None:
                    return terminal
                try:
                    offset += os.write(descriptor, pcm[offset:offset + 960])
                except BlockingIOError:
                    time.sleep(0.01)
                self._feedback(handle, min(time.monotonic() - started, duration))
            while time.monotonic() - started < duration:
                terminal = self._terminal_during_playback(handle, generation)
                if terminal is not None:
                    return terminal
                self._feedback(handle, time.monotonic() - started)
                time.sleep(0.05)
        finally:
            os.close(descriptor)
        return self._finish(
            handle, generation, Speak.Result.COMPLETED,
            'AudioEngine playback completed'
        )

    def _open_fifo(self, handle, generation):
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if handle.is_cancel_requested:
                return None, 'canceled'
            if not self._is_current(generation):
                return None, 'barged'
            try:
                return (
                    os.open(
                        self.playback_fifo, os.O_WRONLY | os.O_NONBLOCK
                    ),
                    'opened',
                )
            except OSError as error:
                if error.errno not in (errno.ENOENT, errno.ENXIO):
                    raise
                time.sleep(0.02)
        return None, 'timeout'

    def _terminal_during_playback(self, handle, generation):
        if handle.is_cancel_requested:
            self._interrupt_generation(generation)
            return self._finish(
                handle, generation, Speak.Result.CANCELED,
                'playback canceled'
            )
        if not self._is_current(generation):
            return self._barged(handle, generation, 'during playback')
        return None

    @staticmethod
    def _feedback(handle, elapsed):
        feedback = Speak.Feedback()
        feedback.played.sec = int(elapsed)
        feedback.played.nanosec = int(
            (elapsed % 1.0) * 1_000_000_000
        )
        handle.publish_feedback(feedback)

    def interrupt(self, force=False):
        """Interrupt an allowed scope and report whether playback was active."""
        with self.lock:
            active = self._active_locked()
            if active and (force or self.allows_barge_in):
                self._terminate_locked()
                self.generation += 1
            return active

    def cancel(self, _request):
        """Accept ROS Action cancellation and rotate the PlaybackScope."""
        with self.lock:
            self._terminate_locked()
            self.generation += 1
        return CancelResponse.ACCEPT

    def close(self):
        """Stop any owned child process during node shutdown."""
        with self.lock:
            self._terminate_locked()

    def _begin(self, allow_barge_in):
        with self.lock:
            self._terminate_locked()
            self.generation += 1
            self.allows_barge_in = allow_barge_in
            self.active = True
            return self.generation

    def _adopt(self, generation, process):
        with self.lock:
            if generation != self.generation:
                process.terminate()
                return False
            self.process = process
            return True

    def _interrupt_generation(self, generation):
        with self.lock:
            if generation == self.generation:
                self._terminate_locked()
                self.generation += 1

    def _is_current(self, generation):
        with self.lock:
            return generation == self.generation

    def _active_locked(self):
        return self.active

    def _terminate_locked(self):
        if self._active_locked():
            if self.process is not None and self.process.poll() is None:
                try:
                    self.process.terminate()
                except OSError:
                    pass
        self.process = None
        self.active = False

    def _interrupted_or_failed(self, handle, generation):
        with self.lock:
            current = generation == self.generation
            if current:
                self._terminate_locked()
        if not current:
            return self._barged(handle, generation, 'during synthesis')
        return self._finish(
            handle, generation, Speak.Result.FAILED, 'Piper synthesis failed'
        )

    def _barged(self, handle, generation, when):
        return self._finish(
            handle, generation, Speak.Result.BARGED_IN,
            'playback interrupted ' + when
        )

    def _finish(self, handle, generation, code, detail):
        with self.lock:
            if generation == self.generation:
                self.process = None
                self.allows_barge_in = False
                self.active = False
        result = Speak.Result()
        result.code, result.detail = code, detail
        if code == Speak.Result.COMPLETED:
            handle.succeed()
        elif code == Speak.Result.CANCELED:
            handle.canceled()
        else:
            handle.abort()
        return result
