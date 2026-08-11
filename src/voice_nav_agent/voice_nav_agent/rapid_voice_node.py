"""Wake-gated local speech input and Piper output for the rapid demo."""

import json
import os
import secrets
import stat
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from voice_nav_interfaces.action import Speak
from voice_nav_interfaces.msg import VoiceTurn
from voice_nav_interfaces.srv import StopMission

from .rapid_commands import WakeGate, normalize_rapid_command
from .rapid_playback import PiperPlayback

STOP_PHRASES = ('\u505c\u6b62', '\u7d27\u6025\u505c\u6b62')


class RapidVoiceNode(Node):
    """Connect local wake-gated speech to the formal Voice interfaces."""

    def __init__(self):
        """Create the public Voice endpoint and private speech workers."""
        super().__init__('rapid_voice_node')
        self.instance_id, self.sequence = secrets.token_hex(16), 0
        self.piper = self.declare_parameter('piper_path', '').value
        self.model = self.declare_parameter('piper_model', '').value
        self.speech_python = self.declare_parameter('vosk_python', '').value
        self.vosk_model = self.declare_parameter('vosk_model', '').value
        self.kws_model = self.declare_parameter('kws_model', '').value
        self.vad_model = self.declare_parameter('vad_model', '').value
        self.asr_model = self.declare_parameter('asr_model', '').value
        self.keywords_file = self.declare_parameter(
            'keywords_file', ''
        ).value
        self.pcm_fifo = self.declare_parameter('pcm_fifo', '').value
        self.playback_fifo = self.declare_parameter('playback_fifo', '').value
        self.owns_pcm_fifo = False
        self.owns_playback_fifo = False
        self.speech_worker = None
        self.wake_interrupted_playback = False
        self.work_group = ReentrantCallbackGroup()
        playback_fifo = (
            self.playback_fifo
            if self._prepare_fifo(
                self.playback_fifo, 'owns_playback_fifo'
            ) else ''
        )
        self.playback = PiperPlayback(
            self.get_logger(), self.piper, self.model, playback_fifo
        )
        self.wake_gate = WakeGate(
            self.declare_parameter('wake_word', '\u5c0f\u667a').value,
            float(self.declare_parameter('wake_timeout_s', 8.0).value),
        )
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.turns = self.create_publisher(VoiceTurn, '/voice/turn', qos)
        self.stop = self.create_client(
            StopMission, '/mission/stop', callback_group=self.work_group
        )
        self.speak = ActionServer(
            self,
            Speak,
            '/voice/speak',
            self.playback.execute,
            cancel_callback=self.playback.cancel,
            callback_group=self.work_group,
        )
        threading.Thread(target=self._read_terminal, daemon=True).start()
        self._start_speech_worker()
        self.get_logger().info(
            'Rapid voice ready. Say the wake word before a command.'
        )

    def _start_speech_worker(self):
        if not Path(self.speech_python).is_file():
            return
        fifo_ready = self._prepare_fifo(self.pcm_fifo, 'owns_pcm_fifo')
        sherpa_ready = (
            Path(self.kws_model).is_dir()
            and Path(self.vad_model).is_file()
            and Path(self.asr_model).is_dir()
            and Path(self.keywords_file).is_file()
            and fifo_ready
        )
        if sherpa_ready:
            arguments = [
                self.speech_python, '-u',
                str(Path(__file__).with_name('sherpa_worker.py')),
                '--pcm-fifo', self.pcm_fifo,
                '--kws-model', self.kws_model,
                '--vad-model', self.vad_model,
                '--asr-model', self.asr_model,
                '--keywords-file', self.keywords_file,
            ]
            reader = self._read_sherpa
        elif Path(self.vosk_model).is_dir():
            arguments = [
                self.speech_python, '-u',
                str(Path(__file__).with_name('vosk_worker.py')),
                self.vosk_model,
            ]
            if fifo_ready:
                arguments.extend(['--pcm-fifo', self.pcm_fifo])
            reader = self._read_vosk
        else:
            return
        self.speech_worker = subprocess.Popen(
            arguments, stdout=subprocess.PIPE, text=True
        )
        threading.Thread(target=reader, daemon=True).start()

    def _prepare_fifo(self, fifo, ownership_attribute):
        if not fifo:
            return False
        path = Path(fifo)
        if path.exists():
            if not stat.S_ISFIFO(path.stat().st_mode):
                self.get_logger().error('pcm_fifo exists but is not a FIFO')
                return False
            return True
        os.mkfifo(path, mode=0o600)
        setattr(self, ownership_attribute, True)
        return True

    def _read_terminal(self):
        while rclpy.ok():
            try:
                text = input('voice> ').strip()
            except (EOFError, KeyboardInterrupt):
                return
            self._publish_text(text)

    def _read_vosk(self):
        for text in self.speech_worker.stdout:
            self._publish_text(text.strip())

    def _read_sherpa(self):
        for line in self.speech_worker.stdout:
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if event.get('wake'):
                self.wake_interrupted_playback = self.playback.interrupt()
                continue
            text = str(event.get('text', '')).strip()
            if text and event.get('wake_authorized') is True:
                self._publish_text(
                    text,
                    wake_authorized=True,
                    wake_during_playback=self.wake_interrupted_playback,
                )
                self.wake_interrupted_playback = False

    def _publish_text(
        self, text, wake_authorized=False, wake_during_playback=False
    ):
        command = (
            text if wake_authorized else
            self.wake_gate.accept(text, time.monotonic())
        )
        if command is None:
            return
        command = normalize_rapid_command(command)
        is_stop = command in STOP_PHRASES
        during_playback = (
            self.playback.interrupt(force=is_stop) or wake_during_playback
        )
        self.sequence += 1
        turn = VoiceTurn()
        turn.voice_instance_id = self.instance_id
        turn.voice_seq = self.sequence
        turn.session_id, turn.turn_id = self.instance_id, secrets.token_hex(16)
        turn.kind = VoiceTurn.STOP if is_stop else VoiceTurn.COMMAND
        turn.text = command
        turn.confidence = 1.0
        turn.during_playback = during_playback
        if is_stop:
            self._send_stop(turn)
        self.turns.publish(turn)
        self.get_logger().info('Accepted voice command: %s' % command)

    def _send_stop(self, turn):
        if not self.stop.service_is_ready() and not self.stop.wait_for_service(
            timeout_sec=0.25
        ):
            self.get_logger().warning(
                'Direct voice STOP unavailable; Agent retry remains active'
            )
            return
        request = StopMission.Request()
        request.request_id = turn.turn_id
        request.source_instance_id = turn.voice_instance_id
        request.source_seq = turn.voice_seq
        request.reason = 'voice_stop'
        future = self.stop.call_async(request)
        future.add_done_callback(self._stop_done)

    def _stop_done(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                'Direct voice STOP result code=%d inhibited=%s'
                % (response.code, response.motion_inhibited)
            )
        except Exception as error:
            self.get_logger().warning('Direct voice STOP failed: %s' % error)

    def destroy_node(self):
        """Stop owned child processes and private FIFO resources."""
        self.playback.close()
        if (
            self.speech_worker is not None
            and self.speech_worker.poll() is None
        ):
            self.speech_worker.terminate()
            try:
                self.speech_worker.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.speech_worker.kill()
                self.speech_worker.wait(timeout=2)
        if self.owns_pcm_fifo:
            Path(self.pcm_fifo).unlink(missing_ok=True)
        if self.owns_playback_fifo:
            Path(self.playback_fifo).unlink(missing_ok=True)
        return super().destroy_node()


def main():
    """Run Voice with concurrent action and service callbacks."""
    rclpy.init()
    node = RapidVoiceNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
