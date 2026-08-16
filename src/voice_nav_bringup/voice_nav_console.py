#!/usr/bin/env python3
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

"""Installed simulation-only line console for the existing command gateway."""

from __future__ import annotations

import argparse
import json
import sys
import time


COMMAND_TEXT_MAX_BYTES = 512
DEFAULT_TIMEOUT_S = 2.0
INVALID_COMMAND_REASON = (
    'command_text must contain non-whitespace UTF-8 text of at most 512 bytes'
)
INVALID_UTF8_REASON = 'command_text must be valid UTF-8'
PARAMETER_SERVICE = '/voice_nav_command_gateway/set_parameters'


def _stable_result(status: str, reason: str = '') -> dict[str, str]:
    return {'status': status, 'reason': reason}


def _write_result(stdout, result: dict[str, str]) -> None:
    stdout.write(
        json.dumps(
            result,
            ensure_ascii=False,
            sort_keys=True,
            separators=(',', ':'),
        )
        + '\n'
    )
    stdout.flush()


def _remove_line_ending(value: str | bytes) -> str | bytes:
    if value.endswith(b'\n' if isinstance(value, bytes) else '\n'):
        value = value[:-1]
    if value.endswith(b'\r' if isinstance(value, bytes) else '\r'):
        value = value[:-1]
    return value


def _decode_line(value: str | bytes) -> str:
    value = _remove_line_ending(value)
    if isinstance(value, bytes):
        return value.decode('utf-8')
    value.encode('utf-8')
    return value


def _validate_command(text: str) -> str | None:
    try:
        encoded = text.encode('utf-8')
    except UnicodeEncodeError:
        return INVALID_UTF8_REASON
    if not text.strip() or len(encoded) > COMMAND_TEXT_MAX_BYTES:
        return INVALID_COMMAND_REASON
    return None


def _reason(value) -> str:
    if value is None:
        return ''
    return value if isinstance(value, str) else str(value)


def _transport_result(response) -> dict[str, str]:
    if not isinstance(response, dict):
        return _stable_result(
            'unavailable', 'command transport returned an invalid response',
        )
    status = response.get('status')
    if status in {'accepted', 'rejected', 'unavailable'}:
        return _stable_result(status, _reason(response.get('reason')))
    if response.get('available') is False:
        return _stable_result('unavailable', _reason(response.get('reason')))
    accepted = response.get('accepted', response.get('successful'))
    if accepted is True:
        return _stable_result('accepted', _reason(response.get('reason')))
    if accepted is False:
        return _stable_result('rejected', _reason(response.get('reason')))
    return _stable_result(
        'unavailable', 'command transport returned an invalid response',
    )


def _call_set_parameters_with_budget(
    *, client, node, rclpy, request, timeout_s, clock=time.monotonic,
):
    """Call one parameter service with one discovery/response deadline."""
    deadline = clock() + timeout_s
    try:
        discovery_timeout = max(0.0, deadline - clock())
        if not client.wait_for_service(timeout_sec=discovery_timeout):
            return None, (
                'voice_nav_command_gateway set_parameters service unavailable'
            )
        remaining = deadline - clock()
        if remaining <= 0.0:
            return None, (
                'voice_nav_command_gateway set_parameters response timeout'
            )
        future = client.call_async(request)
        remaining = deadline - clock()
        if remaining <= 0.0:
            return None, (
                'voice_nav_command_gateway set_parameters response timeout'
            )
        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=remaining,
        )
        if not future.done():
            return None, (
                'voice_nav_command_gateway set_parameters response timeout'
            )
        return future.result(), ''
    except Exception as error:
        return None, _reason(error)


class RosParameterTransport:
    """One-shot SetParameters adapter for the fixed command gateway."""

    def __init__(self):
        """Create the fixed gateway parameter client."""
        import rclpy
        from rcl_interfaces.srv import SetParameters

        self._rclpy = rclpy
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init(args=None)
        self._node = rclpy.create_node('voice_nav_console')
        self._client = self._node.create_client(
            SetParameters,
            PARAMETER_SERVICE,
        )

    def submit(self, text: str, timeout_s: float) -> dict[str, str]:
        """Submit exactly one bounded command text to the gateway."""
        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter

        try:
            request = SetParameters.Request()
            request.parameters = [
                Parameter('command_text', value=text).to_parameter_msg(),
            ]
            response, unavailable_reason = _call_set_parameters_with_budget(
                client=self._client,
                node=self._node,
                rclpy=self._rclpy,
                request=request,
                timeout_s=timeout_s,
            )
        except Exception as error:
            return _stable_result('unavailable', _reason(error))
        if unavailable_reason:
            return _stable_result('unavailable', unavailable_reason)
        if response is None or len(response.results) != 1:
            return _stable_result(
                'unavailable',
                'voice_nav_command_gateway returned an invalid '
                'set_parameters response',
            )
        result = response.results[0]
        return _stable_result(
            'accepted' if result.successful else 'rejected',
            _reason(result.reason),
        )

    def close(self) -> None:
        """Release the client node and an owned rclpy context."""
        self._node.destroy_node()
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()


def _readline(stdin):
    stream = getattr(stdin, 'buffer', stdin)
    return stream.readline()


def _dispatch(text: str, transport, ensure_transport):
    invalid_reason = _validate_command(text)
    if invalid_reason is not None:
        return _stable_result('rejected', invalid_reason), 2
    try:
        response = ensure_transport(transport)
        return (
            _transport_result(response.submit(text, DEFAULT_TIMEOUT_S)),
            None,
        )
    except Exception as error:
        return _stable_result('unavailable', _reason(error)), 1


def main(
    argv: list[str] | None = None,
    *,
    transport=None,
    stdin=None,
    stdout=None,
) -> int:
    """Run command mode or stdin line mode through the existing gateway."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--command')
    arguments = parser.parse_args(argv)
    if stdin is None:
        stdin = sys.stdin
    if stdout is None:
        stdout = sys.stdout

    owned_transport = False

    def ensure_transport(current):
        nonlocal owned_transport, transport
        if current is not None:
            return current
        owned_transport = True
        transport = RosParameterTransport()
        return transport

    try:
        if arguments.command is not None:
            result, exit_code = _dispatch(
                arguments.command,
                transport,
                ensure_transport,
            )
            _write_result(stdout, result)
            return 0 if exit_code is None else exit_code

        exit_code = 0
        while True:
            raw_line = _readline(stdin)
            if raw_line in ('', b''):
                break
            try:
                text = _decode_line(raw_line)
            except UnicodeDecodeError:
                _write_result(
                    stdout,
                    _stable_result('rejected', INVALID_UTF8_REASON),
                )
                continue
            if text == ':quit':
                break
            result, line_exit_code = _dispatch(
                text,
                transport,
                ensure_transport,
            )
            _write_result(stdout, result)
            if line_exit_code == 1:
                exit_code = 1
        return exit_code
    finally:
        if owned_transport and transport is not None:
            transport.close()


if __name__ == '__main__':
    raise SystemExit(main())
