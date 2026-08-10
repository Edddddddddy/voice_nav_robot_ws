"""Wake-gated terminal/Vosk input and Piper output for the rapid demo."""

import secrets
import subprocess
import tempfile
import threading
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from voice_nav_interfaces.action import Speak
from voice_nav_interfaces.msg import VoiceTurn

from .rapid_commands import WakeGate, normalize_rapid_command

STOP_PHRASES = ('\u505c\u6b62', '\u7d27\u6025\u505c\u6b62')


class RapidVoiceNode(Node):
    def __init__(self):
        super().__init__('rapid_voice_node')
        self.instance_id, self.sequence = secrets.token_hex(16), 0
        self.piper = self.declare_parameter('piper_path', '').value
        self.model = self.declare_parameter('piper_model', '').value
        self.vosk_python = self.declare_parameter('vosk_python', '').value
        self.vosk_model = self.declare_parameter('vosk_model', '').value
        self.wake_gate = WakeGate(
            self.declare_parameter('wake_word', '\u5c0f\u667a').value,
            float(self.declare_parameter('wake_timeout_s', 8.0).value),
        )
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.turns = self.create_publisher(VoiceTurn, '/voice/turn', qos)
        self.speak = ActionServer(self, Speak, '/voice/speak', self._speak)
        threading.Thread(target=self._read_terminal, daemon=True).start()
        if Path(self.vosk_python).is_file() and Path(self.vosk_model).is_dir():
            worker = Path(__file__).with_name('vosk_worker.py')
            self.vosk = subprocess.Popen(
                [self.vosk_python, '-u', str(worker), self.vosk_model],
                stdout=subprocess.PIPE, text=True,
            )
            threading.Thread(target=self._read_vosk, daemon=True).start()
        self.get_logger().info('Rapid voice ready. Say the wake word before a command.')

    def _read_terminal(self):
        while rclpy.ok():
            try:
                text = input('voice> ').strip()
            except (EOFError, KeyboardInterrupt):
                return
            self._publish_text(text)

    def _read_vosk(self):
        for text in self.vosk.stdout:
            self._publish_text(text.strip())

    def _publish_text(self, text):
        command = self.wake_gate.accept(text, time.monotonic())
        if command is None:
            return
        command = normalize_rapid_command(command)
        self.sequence += 1
        turn = VoiceTurn()
        turn.voice_instance_id, turn.voice_seq = self.instance_id, self.sequence
        turn.session_id, turn.turn_id = self.instance_id, secrets.token_hex(16)
        turn.kind = VoiceTurn.STOP if command in STOP_PHRASES else VoiceTurn.COMMAND
        turn.text, turn.confidence, turn.during_playback = command, 1.0, False
        self.turns.publish(turn)
        self.get_logger().info('Accepted voice command: %s' % command)

    def _speak(self, handle):
        self.get_logger().info('SPEAK: %s' % handle.request.text)
        if self.piper and Path(self.piper).is_file() and Path(self.model).is_file():
            wav = Path(tempfile.gettempdir()) / ('voice-nav-' + secrets.token_hex(8) + '.wav')
            try:
                subprocess.run([self.piper, '--model', self.model, '--output_file', str(wav)], input=handle.request.text, text=True, check=True, timeout=30)
                subprocess.Popen(['paplay', str(wav)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except (OSError, subprocess.SubprocessError) as error:
                self.get_logger().warning('Piper playback failed: %s' % error)
        result = Speak.Result()
        result.code, result.detail = Speak.Result.COMPLETED, 'printed by rapid voice node'
        handle.succeed()
        return result


def main():
    rclpy.init()
    node = RapidVoiceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
