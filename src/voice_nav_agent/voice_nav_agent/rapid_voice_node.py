"""Wake-gated terminal/Vosk input and Piper output for the rapid demo."""

from pathlib import Path
import secrets
import subprocess
import threading
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from voice_nav_interfaces.action import Speak
from voice_nav_interfaces.msg import VoiceTurn
from voice_nav_interfaces.srv import StopMission

from .rapid_commands import normalize_rapid_command, WakeGate
from .rapid_playback import PiperPlayback

STOP_PHRASES = ('\u505c\u6b62', '\u7d27\u6025\u505c\u6b62')


class RapidVoiceNode(Node):
    """Connect local wake-gated speech to the formal Voice interfaces."""

    def __init__(self):
        super().__init__('rapid_voice_node')
        self.instance_id, self.sequence = secrets.token_hex(16), 0
        self.piper = self.declare_parameter('piper_path', '').value
        self.model = self.declare_parameter('piper_model', '').value
        self.vosk_python = self.declare_parameter('vosk_python', '').value
        self.vosk_model = self.declare_parameter('vosk_model', '').value
        self.vosk = None
        self.work_group = ReentrantCallbackGroup()
        self.playback = PiperPlayback(self.get_logger(), self.piper, self.model)
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
        if Path(self.vosk_python).is_file() and Path(self.vosk_model).is_dir():
            worker = Path(__file__).with_name('vosk_worker.py')
            self.vosk = subprocess.Popen(
                [self.vosk_python, '-u', str(worker), self.vosk_model],
                stdout=subprocess.PIPE,
                text=True,
            )
            threading.Thread(target=self._read_vosk, daemon=True).start()
        self.get_logger().info(
            'Rapid voice ready. Say the wake word before a command.'
        )

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
        is_stop = command in STOP_PHRASES
        during_playback = self.playback.interrupt(force=is_stop)
        self.sequence += 1
        turn = VoiceTurn()
        turn.voice_instance_id, turn.voice_seq = self.instance_id, self.sequence
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
        self.playback.close()
        if self.vosk is not None and self.vosk.poll() is None:
            self.vosk.terminate()
        return super().destroy_node()


def main():
    """Run the rapid voice endpoint with concurrent action/service callbacks."""
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
