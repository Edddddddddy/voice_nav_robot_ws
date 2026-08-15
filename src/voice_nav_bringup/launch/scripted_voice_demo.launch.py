# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Bounded, simulation-only Scripted VoiceNav demonstration."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


CLARIFICATION_TEXT = '请说明需要前进多少米。'


class ScriptedLoopbackServer(ThreadingHTTPServer):
    """A loopback-only deterministic provider for this installed demo."""

    daemon_threads = True

    def __init__(self):
        super().__init__(('127.0.0.1', 0), ScriptedLoopbackHandler)
        self.requests = []
        self.response_kinds = []
        self.lock = threading.Lock()

    @property
    def endpoint(self):
        return f'http://127.0.0.1:{self.server_address[1]}'


class ScriptedLoopbackHandler(BaseHTTPRequestHandler):
    """Serve the frozen clarify, snapshot, and typed Mission sequence."""

    def do_POST(self):
        try:
            content_length = int(self.headers['Content-Length'])
            body = self.rfile.read(content_length)
            payload = json.loads(body.decode('utf-8'))
            content = json.loads(payload['messages'][-1]['content'])
        except (KeyError, TypeError, UnicodeDecodeError, ValueError):
            self.send_error(400, 'invalid scripted demo loopback request')
            return

        with self.server.lock:
            self.server.requests.append((self.path, body))
            request_index = len(self.server.requests)
        response_payload = self._response_for(request_index, content)
        if response_payload is None:
            self.send_error(400, 'unexpected scripted demo request sequence')
            return

        with self.server.lock:
            self.server.response_kinds.append(response_payload['kind'])
        response = json.dumps({
            'choices': [{
                'message': {
                    'content': json.dumps(response_payload, ensure_ascii=False),
                },
            }],
        }, ensure_ascii=False).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(response)))
        self.end_headers()
        self.wfile.write(response)

    @staticmethod
    def _response_for(request_index, content):
        if not isinstance(content, dict):
            return None
        turn = content.get('turn')
        if not isinstance(turn, dict):
            return None
        expected_text = ('绕到大厅', '半米', '半米')
        if request_index not in (1, 2, 3) or turn.get('text') != expected_text[request_index - 1]:
            return None
        if request_index == 1:
            return {'kind': 'clarify', 'text': CLARIFICATION_TEXT}
        if request_index == 2:
            return {
                'kind': 'tool',
                'tool_call': {
                    'name': 'read_runtime_snapshot',
                    'arguments': {},
                },
            }
        return {
            'kind': 'tool',
            'tool_call': {
                'name': 'propose_mission',
                'arguments': {
                    'kind': 'mission',
                    'steps': [{
                        'kind': 'move_distance',
                        'distance_m': 0.5,
                    }],
                },
            },
        }

    def log_message(self, _format, *_args):
        pass


def _stop_loopback(server, worker):
    server.shutdown()
    server.server_close()
    worker.join(5.0)


def create_scripted_voice_demo(
    *,
    headless='true',
    shutdown_on_gazebo_exit='true',
    shutdown_when_demo_exits=True,
):
    """Build the installed demo graph and return its test observation seams."""
    server = ScriptedLoopbackServer()
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()

    product = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('voice_nav_bringup'),
                'launch',
                'product_sim.launch.py',
            ])
        ),
        launch_arguments={
            'headless': headless,
            'shutdown_on_gazebo_exit': shutdown_on_gazebo_exit,
        }.items(),
    )
    agent = Node(
        package='voice_nav_agent',
        executable='agent_node',
        name='agent_node',
        output='screen',
        parameters=[{'llm_endpoint': server.endpoint}],
    )
    speech_driver = Node(
        package='voice_nav_audio',
        executable='scripted_voice_demo',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    actions = [product, agent, speech_driver]
    if shutdown_when_demo_exits:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=speech_driver,
            on_exit=[Shutdown(reason='Scripted simulation demo completed.')],
        )))

    def cleanup(_event, _context):
        _stop_loopback(server, worker)
        return []

    actions.append(RegisterEventHandler(OnShutdown(on_shutdown=cleanup)))
    return actions, {
        'agent': agent,
        'llm_server': server,
        'llm_thread': worker,
        'speech_driver': speech_driver,
    }


def generate_launch_description():
    """Launch the self-contained, headless-by-default simulation demo."""
    headless = LaunchConfiguration('headless')
    shutdown_on_gazebo_exit = LaunchConfiguration('shutdown_on_gazebo_exit')
    actions, _ = create_scripted_voice_demo(
        headless=headless,
        shutdown_on_gazebo_exit=shutdown_on_gazebo_exit,
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            choices=['true', 'false'],
            description='Run the scripted simulation demo without the Gazebo GUI.',
        ),
        DeclareLaunchArgument(
            'shutdown_on_gazebo_exit',
            default_value='true',
            choices=['true', 'false'],
            description='Fail closed when the required Gazebo simulation exits.',
        ),
        *actions,
    ])
