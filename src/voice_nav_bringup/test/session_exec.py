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

"""Package-private launch prefix that synchronizes an exclusive session."""

import os
import sys


_READY = b'VOICE_NAV_SESSION_READY\n'


def main():
    """Create the session before notifying the launch OnProcessStart hook."""
    arguments = sys.argv[1:]
    if len(arguments) < 3 or arguments[0] != '--ready-fifo':
        raise SystemExit('usage: session_exec.py --ready-fifo FIFO COMMAND [ARG]...')

    ready_fifo = arguments[1]
    command = arguments[2:]
    os.setsid()
    if os.getsid(0) != os.getpid() or os.getpgid(0) != os.getpid():
        raise RuntimeError('session_exec did not become its own session leader')

    ready_fd = os.open(ready_fifo, os.O_WRONLY)
    try:
        os.write(ready_fd, _READY)
    finally:
        os.close(ready_fd)
    os.execvp(command[0], command)


if __name__ == '__main__':
    main()
