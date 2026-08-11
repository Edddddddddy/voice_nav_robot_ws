"""Small direct Mission executor for the explicitly non-safe rapid demo."""

import math
from pathlib import Path
import re
import secrets
import shutil
import tempfile
import time

from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, TwistStamped

from nav2_msgs.action import NavigateToPose

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.task import Future

from slam_toolbox.srv import SaveMap, SerializePoseGraph

from voice_nav_interfaces.action import ExecuteMission
from voice_nav_interfaces.msg import MissionState, MissionStep
from voice_nav_interfaces.srv import StopMission

from .rapid_map_package import finish_map_package, load_places

MAPPING_MASK = 0b1011
NAVIGATION_MASK = 0b0100
LINEAR_SPEED = 0.25
ANGULAR_SPEED = 0.6


class RapidMissionBridge(Node):
    """Execute typed Missions against Nav2 or SLAM Toolbox with little policy."""

    def __init__(self):
        """Create one mode-specific rapid execution boundary."""
        super().__init__('rapid_mission_bridge')
        self.mode = self.declare_parameter('mode', 'navigation').value
        if self.mode not in {'mapping', 'navigation'}:
            raise ValueError('mode must be mapping or navigation')
        places_file = self.declare_parameter('named_places_file', '').value
        self.places = (
            load_places(places_file)
            if self.mode == 'navigation' and places_file else {}
        )
        if self.mode == 'navigation' and not self.places:
            raise ValueError('navigation mode requires named_places_file')
        self.map_root = Path(
            self.declare_parameter(
                'map_output_root', '/tmp/voice_nav_rapid_maps'
            ).value
        ).expanduser()
        self.runtime_id = secrets.token_hex(16)
        self.epoch = 1
        self.active_handle = None
        self.active_nav = None
        self.work_group = ReentrantCallbackGroup()
        self.nav = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            callback_group=self.work_group,
        )
        self.save_map = self.create_client(
            SaveMap,
            '/slam_toolbox/save_map',
            callback_group=self.work_group,
        )
        self.serialize_map = self.create_client(
            SerializePoseGraph,
            '/slam_toolbox/serialize_map',
            callback_group=self.work_group,
        )
        self.cmd = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10
        )
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            MissionState, '/mission/state', qos
        )
        self.server = ActionServer(
            self,
            ExecuteMission,
            '/mission/execute',
            self.execute,
            cancel_callback=self.cancel,
            callback_group=self.work_group,
        )
        self.stop = self.create_service(
            StopMission,
            '/mission/stop',
            self.stop_goal,
            callback_group=self.work_group,
        )
        self.publish_state()
        self.create_timer(1.0, self.publish_state)

    def publish_state(self):
        """Publish the current rapid Runtime snapshot."""
        state = MissionState()
        state.runtime_instance_id = self.runtime_id
        state.admission_epoch = self.epoch
        state.operating_mode = (
            MissionState.MAPPING
            if self.mode == 'mapping'
            else MissionState.NAVIGATION
        )
        state.availability = (
            MissionState.AVAILABLE
            if self.mode == 'mapping' or self.nav.server_is_ready()
            else MissionState.UNAVAILABLE
        )
        state.gate_state = MissionState.GATE_INHIBITED
        state.supported_step_mask = (
            MAPPING_MASK if self.mode == 'mapping' else NAVIGATION_MASK
        )
        state.max_steps = 3
        state.named_place_ids = (
            [] if self.mode == 'mapping' else sorted(self.places)
        )
        self.publisher.publish(state)

    async def execute(self, handle):
        """Execute one bounded typed Mission sequentially."""
        result = ExecuteMission.Result()
        goal = handle.request
        if self.active_handle is not None:
            return self._finish(
                handle,
                result,
                ExecuteMission.Result.BUSY,
                -1,
                'another rapid Mission is active',
            )
        if (
            goal.runtime_instance_id != self.runtime_id
            or goal.admission_epoch != self.epoch
        ):
            return self._finish(
                handle,
                result,
                ExecuteMission.Result.STALE_REQUEST,
                -1,
                'stale runtime token',
            )
        invalid = self._invalid_step(goal.steps)
        if invalid is not None:
            return self._finish(handle, result, *invalid)
        self.active_handle = handle
        execution_epoch = self.epoch
        try:
            for index, step in enumerate(goal.steps):
                if self._stopped(handle, execution_epoch):
                    return self._finish(
                        handle,
                        result,
                        ExecuteMission.Result.CANCELED,
                        index,
                        'Mission canceled or stopped',
                    )
                self._feedback(handle, index, len(goal.steps), 0.0)
                if step.kind == MissionStep.NAVIGATE_TO:
                    code, detail = await self._navigate(
                        handle, step.target_id, execution_epoch
                    )
                elif step.kind == MissionStep.SAVE_MAP:
                    code, detail = await self._save_map(step.target_id)
                else:
                    code, detail = await self._move(
                        handle, step, index, len(goal.steps), execution_epoch
                    )
                if code != ExecuteMission.Result.SUCCEEDED:
                    return self._finish(handle, result, code, index, detail)
            return self._finish(
                handle,
                result,
                ExecuteMission.Result.SUCCEEDED,
                -1,
                'all rapid Mission steps completed',
            )
        except Exception as error:
            self.get_logger().error(f'Rapid Mission execution failed: {error}')
            return self._finish(
                handle,
                result,
                ExecuteMission.Result.INTERNAL_ERROR,
                -1,
                'rapid executor internal error',
            )
        finally:
            self._zero_velocity()
            self.active_handle = None
            self.active_nav = None

    async def _navigate(self, handle, target, execution_epoch):
        if not self.nav.server_is_ready() and not self.nav.wait_for_server(
            timeout_sec=5.0
        ):
            return (
                ExecuteMission.Result.DEPENDENCY_UNAVAILABLE,
                'Nav2 action unavailable',
            )
        self.active_nav = await self.nav.send_goal_async(self._nav_goal(target))
        if not self.active_nav.accepted:
            return ExecuteMission.Result.EXECUTION_FAILED, 'Nav2 rejected goal'
        self.get_logger().info(f'Forwarded rapid Nav2 goal place={target}')
        nav_result = await self.active_nav.get_result_async()
        self.active_nav = None
        if self._stopped(handle, execution_epoch):
            return ExecuteMission.Result.CANCELED, 'Nav2 goal canceled'
        if nav_result.status != GoalStatus.STATUS_SUCCEEDED:
            return (
                ExecuteMission.Result.EXECUTION_FAILED,
                'Nav2 goal did not succeed',
            )
        return ExecuteMission.Result.SUCCEEDED, 'Nav2 goal completed'

    async def _move(self, handle, step, index, total, execution_epoch):
        linear = step.kind == MissionStep.MOVE_DISTANCE
        amount = step.distance_m if linear else step.angle_rad
        speed = LINEAR_SPEED if linear else ANGULAR_SPEED
        duration = abs(amount) / speed
        self.get_logger().info(
            'Rapid relative motion amount=%.3f speed=%.3f duration=%.3f'
            % (amount, speed, duration)
        )
        started = time.monotonic()
        cycles = 0
        while time.monotonic() - started < duration:
            if self._stopped(handle, execution_epoch):
                return ExecuteMission.Result.CANCELED, 'motion canceled'
            velocity = math.copysign(speed, amount)
            self._publish_velocity(velocity if linear else 0.0,
                                   0.0 if linear else velocity)
            cycles += 1
            progress = min((time.monotonic() - started) / duration, 1.0)
            self._feedback(handle, index, total, progress)
            await self._sleep(0.05)
        self._zero_velocity()
        self.get_logger().info(f'Rapid relative motion cycles={cycles}')
        return ExecuteMission.Result.SUCCEEDED, 'relative motion completed'

    async def _save_map(self, map_id):
        if not self.save_map.service_is_ready() or not (
            self.serialize_map.service_is_ready()
        ):
            return (
                ExecuteMission.Result.DEPENDENCY_UNAVAILABLE,
                'SLAM Toolbox save services unavailable',
            )
        self.map_root.mkdir(parents=True, exist_ok=True)
        target = self.map_root / map_id
        if target.exists():
            return ExecuteMission.Result.INVALID_PLAN, 'map already exists'
        staging = Path(tempfile.mkdtemp(prefix=f'.{map_id}-', dir=self.map_root))
        committed = False
        try:
            base = staging / 'map'
            save_request = SaveMap.Request()
            save_request.name.data = str(base)
            save_response = await self.save_map.call_async(save_request)
            if save_response.result != SaveMap.Response.RESULT_SUCCESS:
                return (
                    ExecuteMission.Result.EXECUTION_FAILED,
                    'occupancy save failed',
                )
            graph_request = SerializePoseGraph.Request()
            graph_request.filename = str(base)
            graph_response = await self.serialize_map.call_async(graph_request)
            if graph_response.result != SerializePoseGraph.Response.RESULT_SUCCESS:
                return (
                    ExecuteMission.Result.EXECUTION_FAILED,
                    'posegraph save failed',
                )
            finish_map_package(staging, map_id)
            if target.exists():
                return ExecuteMission.Result.INVALID_PLAN, 'map already exists'
            staging.rename(target)
            committed = True
            self.get_logger().info(f'Saved rapid Map Package under {target}')
            return ExecuteMission.Result.SUCCEEDED, f'map saved as {map_id}'
        finally:
            if not committed:
                shutil.rmtree(staging, ignore_errors=True)

    def _invalid_step(self, steps):
        if not 1 <= len(steps) <= 3:
            return (
                ExecuteMission.Result.UNSUPPORTED_STEP,
                0,
                'one to three steps required',
            )
        for index, step in enumerate(steps):
            if self.mode == 'navigation':
                if step.kind != MissionStep.NAVIGATE_TO:
                    return (
                        ExecuteMission.Result.UNSUPPORTED_STEP,
                        index,
                        'navigation mode accepts NAVIGATE_TO only',
                    )
                if step.target_id not in self.places:
                    return (
                        ExecuteMission.Result.UNKNOWN_TARGET,
                        index,
                        'unknown place',
                    )
            elif step.kind == MissionStep.MOVE_DISTANCE:
                if not 0.05 <= abs(step.distance_m) <= 2.0:
                    return ExecuteMission.Result.INVALID_PLAN, index, 'bad distance'
            elif step.kind == MissionStep.ROTATE_ANGLE:
                if not 0.05 <= abs(step.angle_rad) <= math.pi:
                    return ExecuteMission.Result.INVALID_PLAN, index, 'bad angle'
            elif step.kind == MissionStep.SAVE_MAP:
                if not re.fullmatch(r'[a-z][a-z0-9_-]{0,31}', step.target_id):
                    return ExecuteMission.Result.INVALID_PLAN, index, 'bad map ID'
            else:
                return (
                    ExecuteMission.Result.UNSUPPORTED_STEP,
                    index,
                    'mapping mode accepts MOVE, ROTATE, and SAVE_MAP',
                )
        return None

    @staticmethod
    def _feedback(handle, index, total, within_step):
        feedback = ExecuteMission.Feedback()
        feedback.phase = ExecuteMission.Feedback.EXECUTING
        feedback.step_index = index
        feedback.progress = (index + within_step) / total
        handle.publish_feedback(feedback)

    def _nav_goal(self, target):
        x, y, yaw = self.places[target]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        return goal

    def _publish_velocity(self, linear, angular):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_footprint'
        message.twist.linear.x = linear
        message.twist.angular.z = angular
        self.cmd.publish(message)

    def _zero_velocity(self):
        self._publish_velocity(0.0, 0.0)

    async def _sleep(self, seconds):
        """Yield a rapid Action using an rclpy timer instead of asyncio."""
        future = Future()

        def complete():
            if not future.done():
                future.set_result(None)

        timer = self.create_timer(
            seconds, complete, callback_group=self.work_group
        )
        try:
            await future
        finally:
            self.destroy_timer(timer)

    def _stopped(self, handle, execution_epoch):
        return execution_epoch != self.epoch or handle.is_cancel_requested

    def cancel(self, _request):
        """Accept local Action cancel and stop current output."""
        if self.active_nav is not None:
            self.active_nav.cancel_goal_async()
        self._zero_velocity()
        return CancelResponse.ACCEPT

    def stop_goal(self, _request, response):
        """Rotate the rapid epoch and stop current motion."""
        if self.active_nav is not None:
            self.active_nav.cancel_goal_async()
        self.epoch += 1
        self._zero_velocity()
        self.publish_state()
        response.code = StopMission.Response.APPLIED
        response.runtime_instance_id = self.runtime_id
        response.admission_epoch = self.epoch
        response.motion_inhibited = True
        response.detail = 'rapid goal canceled and zero velocity sent'
        return response

    @staticmethod
    def _finish(handle, result, code, failed_step, detail):
        result.code = code
        result.failed_step = failed_step
        result.detail = detail
        if code == ExecuteMission.Result.SUCCEEDED:
            handle.succeed()
        elif code == ExecuteMission.Result.CANCELED:
            handle.canceled()
        else:
            handle.abort()
        return result


def main():
    """Run the explicitly non-safe rapid Mission bridge."""
    rclpy.init()
    node = RapidMissionBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
