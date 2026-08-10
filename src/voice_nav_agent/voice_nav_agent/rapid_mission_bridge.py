"""Small Nav2-backed sequential Mission executor for the local rapid demo."""

import math
import secrets

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from voice_nav_interfaces.action import ExecuteMission
from voice_nav_interfaces.msg import MissionState, MissionStep
from voice_nav_interfaces.srv import StopMission

PLACES = {
    'home': (0.0, 0.0, 0.0),
    'study': (-2.0, 1.8, math.pi),
    'kitchen': (1.5, -1.8, -math.pi / 2.0),
}
NAVIGATE_TO_MASK = 1 << (MissionStep.NAVIGATE_TO - 1)


class RapidMissionBridge(Node):
    def __init__(self):
        super().__init__('rapid_mission_bridge')
        self.runtime_id, self.epoch = secrets.token_hex(16), 1
        self.active_nav = None
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MissionState, '/mission/state', qos)
        self.server = ActionServer(self, ExecuteMission, '/mission/execute', self.execute)
        self.stop = self.create_service(StopMission, '/mission/stop', self.stop_goal)
        self.publish_state()
        self.create_timer(1.0, self.publish_state)

    def publish_state(self):
        state = MissionState()
        state.runtime_instance_id, state.admission_epoch = self.runtime_id, self.epoch
        state.operating_mode, state.availability = MissionState.NAVIGATION, MissionState.AVAILABLE
        state.gate_state = MissionState.GATE_INHIBITED
        state.supported_step_mask, state.max_steps = NAVIGATE_TO_MASK, 3
        state.named_place_ids = sorted(PLACES)
        self.publisher.publish(state)

    async def execute(self, handle):
        goal, result = handle.request, ExecuteMission.Result()
        if goal.runtime_instance_id != self.runtime_id or goal.admission_epoch != self.epoch:
            result.code, result.detail = ExecuteMission.Result.STALE_REQUEST, 'stale runtime token'
            handle.abort()
            return result
        invalid = self._invalid_step(goal.steps)
        if invalid is not None:
            result.code, result.failed_step, result.detail = invalid
            handle.abort()
            return result
        if not self.nav.server_is_ready():
            result.code, result.detail = ExecuteMission.Result.DEPENDENCY_UNAVAILABLE, 'Nav2 action unavailable'
            handle.abort()
            return result
        execution_epoch = self.epoch
        for index, step in enumerate(goal.steps):
            if execution_epoch != self.epoch:
                result.code, result.failed_step, result.detail = ExecuteMission.Result.STOPPED, index, 'stopped'
                handle.canceled()
                return result
            self._feedback(handle, index, len(goal.steps))
            self.active_nav = await self.nav.send_goal_async(self._nav_goal(step.target_id))
            if not self.active_nav.accepted:
                result.code, result.failed_step, result.detail = ExecuteMission.Result.EXECUTION_FAILED, index, 'Nav2 rejected goal'
                handle.abort()
                return result
            self.get_logger().info('Forwarded Mission step=%s place=%s' % (index, step.target_id))
            nav_result = await self.active_nav.get_result_async()
            self.active_nav = None
            if nav_result.status != GoalStatus.STATUS_SUCCEEDED:
                result.code = ExecuteMission.Result.STOPPED if execution_epoch != self.epoch else ExecuteMission.Result.EXECUTION_FAILED
                result.failed_step, result.detail = index, 'Nav2 goal did not succeed'
                handle.canceled() if execution_epoch != self.epoch else handle.abort()
                return result
        result.code, result.detail = ExecuteMission.Result.SUCCEEDED, 'all Nav2 steps completed'
        handle.succeed()
        return result

    @staticmethod
    def _invalid_step(steps):
        if not 1 <= len(steps) <= 3:
            return ExecuteMission.Result.UNSUPPORTED_STEP, 0, 'one to three steps required'
        for index, step in enumerate(steps):
            if step.kind != MissionStep.NAVIGATE_TO:
                return ExecuteMission.Result.UNSUPPORTED_STEP, index, 'only NAVIGATE_TO is supported'
            if step.target_id not in PLACES:
                return ExecuteMission.Result.UNKNOWN_TARGET, index, 'unknown place'
        return None

    def _feedback(self, handle, index, total):
        feedback = ExecuteMission.Feedback()
        feedback.phase, feedback.step_index = ExecuteMission.Feedback.EXECUTING, index
        feedback.progress = index / total
        handle.publish_feedback(feedback)

    def _nav_goal(self, target):
        x, y, yaw = PLACES[target]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return goal

    def stop_goal(self, _request, response):
        if self.active_nav is not None:
            self.active_nav.cancel_goal_async()
        self.epoch += 1
        self.publish_state()
        response.code, response.runtime_instance_id = StopMission.Response.APPLIED, self.runtime_id
        response.admission_epoch, response.motion_inhibited = self.epoch, True
        response.detail = 'Nav2 goal canceled'
        return response


def main():
    rclpy.init()
    node = RapidMissionBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
