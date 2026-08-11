"""Bounded Piper/paplay PlaybackScope for the rapid voice endpoint."""

from pathlib import Path
import secrets
import subprocess
import tempfile
import threading
import time

from rclpy.action import CancelResponse
from voice_nav_interfaces.action import Speak


class PiperPlayback:
    """Own synthesis, playback, feedback, cancellation, and barge-in."""

    def __init__(self, logger, piper, model):
        self.logger = logger
        self.piper = piper
        self.model = model
        self.lock = threading.Lock()
        self.generation = 0
        self.process = None
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
            feedback = Speak.Feedback()
            elapsed = time.monotonic() - started
            feedback.played.sec = int(elapsed)
            feedback.played.nanosec = int(
                (elapsed % 1.0) * 1_000_000_000
            )
            handle.publish_feedback(feedback)
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
        return self.process is not None and self.process.poll() is None

    def _terminate_locked(self):
        if self._active_locked():
            try:
                self.process.terminate()
            except OSError:
                pass
        self.process = None

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
        result = Speak.Result()
        result.code, result.detail = code, detail
        if code == Speak.Result.COMPLETED:
            handle.succeed()
        elif code == Speak.Result.CANCELED:
            handle.canceled()
        else:
            handle.abort()
        return result
