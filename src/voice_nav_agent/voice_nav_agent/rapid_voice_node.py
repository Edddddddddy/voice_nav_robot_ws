"""Terminal transcript and speech sink for the rapid local VoiceNav demo."""
import secrets
import threading

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from voice_nav_interfaces.action import Speak
from voice_nav_interfaces.msg import VoiceTurn


class RapidVoiceNode(Node):
    def __init__(self):
        super().__init__('rapid_voice_node')
        self.instance_id, self.sequence = secrets.token_hex(16), 0
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.turns = self.create_publisher(VoiceTurn, '/voice/turn', qos)
        self.speak = ActionServer(self, Speak, '/voice/speak', self._speak)
        threading.Thread(target=self._read_terminal, daemon=True).start()
        self.get_logger().info('Rapid voice ready. Enter Chinese commands in this terminal.')

    def _read_terminal(self):
        while rclpy.ok():
            try: text = input('voice> ').strip()
            except (EOFError, KeyboardInterrupt): return
            if not text: continue
            self.sequence += 1
            turn = VoiceTurn()
            turn.voice_instance_id, turn.voice_seq = self.instance_id, self.sequence
            turn.session_id, turn.turn_id = self.instance_id, secrets.token_hex(16)
            turn.kind = VoiceTurn.STOP if text in ('停止', '紧急停止') else VoiceTurn.COMMAND
            turn.text, turn.confidence, turn.during_playback = text, 1.0, False
            self.turns.publish(turn)

    def _speak(self, handle):
        self.get_logger().info('SPEAK: %s' % handle.request.text)
        result = Speak.Result(); result.code, result.detail = Speak.Result.COMPLETED, 'printed by rapid voice node'
        handle.succeed(); return result


def main():
    rclpy.init(); node = RapidVoiceNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
