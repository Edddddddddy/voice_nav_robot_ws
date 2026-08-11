"""ROS adapter for the rapid same-origin web console."""

import secrets
import threading
import time
from http.server import ThreadingHTTPServer

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_msgs.msg import OccupancyGrid

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from voice_nav_interfaces.msg import MissionState, VoiceTurn
from voice_nav_interfaces.srv import StopMission

from .web_console import ConsoleApi, handler_for


_MODE = {
    MissionState.MAPPING: 'mapping',
    MissionState.NAVIGATION: 'navigation',
}
_AVAILABILITY = {
    MissionState.UNAVAILABLE: 'unavailable',
    MissionState.AVAILABLE: 'available',
    MissionState.BUSY: 'busy',
    MissionState.FAULTED: 'faulted',
}
_GATE = {
    MissionState.GATE_INHIBITED: 'inhibited',
    MissionState.GATE_ARMED: 'armed',
    MissionState.GATE_FAULTED: 'faulted',
}


class RapidWebConsole(Node):
    """Bridge a minimal HTTP control surface to formal Voice interfaces."""

    def __init__(self):
        """Create subscriptions, the Voice source, and private HTTP server."""
        super().__init__('rapid_web_console')
        self.instance_id = secrets.token_hex(16)
        self.sequence = 0
        self.lock = threading.Lock()
        self.state_data = {
            'connected': False,
            'mode': 'unknown',
            'availability': 'unavailable',
            'gate': 'inhibited',
            'runtime_id': '',
            'epoch': 0,
            'active_step': 0,
            'max_steps': 0,
            'named_places': [],
            'pose': None,
            'last_event': '等待 Runtime 状态',
            'map_revision': 0,
        }
        self.state_seen_at = 0.0
        self.map_data = {'revision': 0, 'available': False}
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            MissionState, '/mission/state', self._on_state, qos
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, qos)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10
        )
        self.turns = self.create_publisher(VoiceTurn, '/voice/turn', 10)
        self.stop = self.create_client(StopMission, '/mission/stop')
        host = self.declare_parameter('bind_host', '127.0.0.1').value
        port = int(self.declare_parameter('port', 8088).value)
        static_root = (
            get_package_share_directory('voice_nav_agent') + '/web'
        )
        self.server = ThreadingHTTPServer(
            (host, port), handler_for(ConsoleApi(self, static_root))
        )
        self.server.daemon_threads = True
        self.http_thread = threading.Thread(
            target=self.server.serve_forever, daemon=True
        )
        self.http_thread.start()
        self.get_logger().info(
            'Rapid web console listening on http://%s:%d' % (host, port)
        )

    def _on_state(self, message):
        with self.lock:
            self.state_seen_at = time.monotonic()
            self.state_data.update({
                'mode': _MODE.get(message.operating_mode, 'unknown'),
                'availability': _AVAILABILITY.get(
                    message.availability, 'unknown'
                ),
                'gate': _GATE.get(message.gate_state, 'unknown'),
                'runtime_id': message.runtime_instance_id,
                'epoch': int(message.admission_epoch),
                'active_step': int(message.active_step),
                'max_steps': int(message.max_steps),
                'named_places': list(message.named_place_ids),
            })

    def _on_map(self, message):
        with self.lock:
            revision = self.map_data.get('revision', 0) + 1
            self.map_data = {
                'revision': revision,
                'available': True,
                'frame': message.header.frame_id,
                'width': int(message.info.width),
                'height': int(message.info.height),
                'resolution': float(message.info.resolution),
                'origin': {
                    'x': float(message.info.origin.position.x),
                    'y': float(message.info.origin.position.y),
                },
                'cells': list(message.data),
            }
            self.state_data['map_revision'] = revision

    def _on_pose(self, message):
        pose = message.pose.pose
        with self.lock:
            self.state_data['pose'] = {
                'x': round(float(pose.position.x), 3),
                'y': round(float(pose.position.y), 3),
                'z': round(float(pose.orientation.z), 5),
                'w': round(float(pose.orientation.w), 5),
            }

    def state_snapshot(self):
        """Return a bounded copy for one HTTP state response."""
        with self.lock:
            result = dict(self.state_data)
            result['named_places'] = list(result['named_places'])
            result['pose'] = (
                None if result['pose'] is None else dict(result['pose'])
            )
            result['connected'] = (
                time.monotonic() - self.state_seen_at <= 3.0
            )
            return result

    def map_snapshot(self):
        """Return the latest occupancy grid for canvas rendering."""
        with self.lock:
            result = dict(self.map_data)
            if 'cells' in result:
                result['cells'] = list(result['cells'])
            if 'origin' in result:
                result['origin'] = dict(result['origin'])
            return result

    def submit_command(self, text):
        """Publish one web command through the formal VoiceTurn seam."""
        turn = self._turn(VoiceTurn.COMMAND, text)
        self.turns.publish(turn)
        self._event('已发送：' + text)
        return {'accepted': True, 'turn_id': turn.turn_id}

    def request_stop(self):
        """Send the direct STOP and publish the identical retry identity."""
        turn = self._turn(VoiceTurn.STOP, '停止')
        direct = self.stop.service_is_ready()
        if direct:
            request = StopMission.Request()
            request.request_id = turn.turn_id
            request.source_instance_id = turn.voice_instance_id
            request.source_seq = turn.voice_seq
            request.reason = 'web_stop'
            future = self.stop.call_async(request)
            future.add_done_callback(self._stop_done)
        self.turns.publish(turn)
        self._event('STOP 已提交')
        return {
            'accepted': True,
            'direct_stop': direct,
            'turn_id': turn.turn_id,
        }

    def _turn(self, kind, text):
        with self.lock:
            self.sequence += 1
            sequence = self.sequence
        turn = VoiceTurn()
        turn.voice_instance_id = self.instance_id
        turn.voice_seq = sequence
        turn.session_id = self.instance_id
        turn.turn_id = secrets.token_hex(16)
        turn.kind = kind
        turn.text = text
        turn.confidence = 1.0
        turn.during_playback = False
        return turn

    def _stop_done(self, future):
        try:
            response = future.result()
            self._event(
                'STOP 结果 code=%d inhibited=%s'
                % (response.code, response.motion_inhibited)
            )
        except Exception as error:
            self._event('STOP 调用失败：' + str(error))

    def _event(self, text):
        with self.lock:
            self.state_data['last_event'] = text[:160]

    def destroy_node(self):
        """Stop only the HTTP server owned by this node."""
        self.server.shutdown()
        self.server.server_close()
        self.http_thread.join(timeout=2.0)
        return super().destroy_node()


def main():
    """Run the rapid web console ROS adapter."""
    rclpy.init()
    node = RapidWebConsole()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
