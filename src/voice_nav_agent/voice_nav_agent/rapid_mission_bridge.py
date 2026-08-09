"""Minimal Nav2-backed Mission endpoint for the local rapid demo."""
import math
import secrets

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from voice_nav_interfaces.action import ExecuteMission
from voice_nav_interfaces.msg import MissionState, MissionStep
from voice_nav_interfaces.srv import StopMission

PLACES = {'home': (0.0, 0.0, 0.0), 'study': (-2.0, 1.8, math.pi), 'kitchen': (1.5, -1.8, -math.pi / 2.0)}


class RapidMissionBridge(Node):
    def __init__(self):
        super().__init__('rapid_mission_bridge')
        self.runtime_id, self.epoch = secrets.token_hex(16), 1
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MissionState, '/mission/state', qos)
        self.server = ActionServer(self, ExecuteMission, '/mission/execute', self.execute)
        self.stop = self.create_service(StopMission, '/mission/stop', self.stop_goal)
        self.publish_state()

    def publish_state(self):
        state = MissionState()
        state.runtime_instance_id, state.admission_epoch = self.runtime_id, self.epoch
        state.operating_mode, state.availability = MissionState.NAVIGATION, MissionState.AVAILABLE
        state.gate_state, state.supported_step_mask, state.max_steps = MissionState.GATE_INHIBITED, 7, 1
        state.named_place_ids = list(PLACES)
        self.publisher.publish(state)

    def execute(self, handle):
        goal, result = handle.request, ExecuteMission.Result()
        if goal.runtime_instance_id != self.runtime_id or goal.admission_epoch != self.epoch:
            result.code, result.detail = ExecuteMission.Result.STALE_REQUEST, 'stale runtime token'
            handle.abort(); return result
        if len(goal.steps) != 1 or goal.steps[0].kind != MissionStep.NAVIGATE_TO:
            result.code, result.failed_step, result.detail = ExecuteMission.Result.UNSUPPORTED_STEP, 0, 'one NAVIGATE_TO step required'
            handle.abort(); return result
        target = goal.steps[0].target_id
        if target not in PLACES:
            result.code, result.failed_step, result.detail = ExecuteMission.Result.UNKNOWN_TARGET, 0, 'unknown place'
            handle.abort(); return result
        x, y, yaw = PLACES[target]
        nav_goal = NavigateToPose.Goal(); nav_goal.pose = PoseStamped()
        nav_goal.pose.header.frame_id = 'map'; nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x, nav_goal.pose.pose.position.y = x, y
        nav_goal.pose.pose.orientation.z, nav_goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.nav.send_goal_async(nav_goal)
        result.code, result.detail = ExecuteMission.Result.SUCCEEDED, 'Nav2 goal forwarded'
        handle.succeed(); return result

    def stop_goal(self, _request, response):
        self.nav.cancel_all_goals_async(); self.epoch += 1; self.publish_state()
        response.code, response.runtime_instance_id = StopMission.Response.APPLIED, self.runtime_id
        response.admission_epoch, response.motion_inhibited = self.epoch, True
        response.detail = 'Nav2 goals canceled'; return response


def main():
    rclpy.init(); node = RapidMissionBridge()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
