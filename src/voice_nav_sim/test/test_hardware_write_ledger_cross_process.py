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

"""Cross-process behavior test for the hardware-write ledger mapping."""

import errno
import os
import subprocess
import sys

from hardware_write_ledger_test_support import HardwareWriteLedgerRegionOwner


def terminate_owned_process(process):
    """Bound cleanup to the exact child created by this test."""
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2.0)


def main():
    """Prove exec attach, unique claim, unlink, and mapped access."""
    if len(sys.argv) != 2:
        raise SystemExit('expected the attach probe executable')
    probe = os.path.abspath(sys.argv[1])

    with HardwareWriteLedgerRegionOwner(
        generation=73,
        segment_capacity=4,
        page_segment_limit=2,
    ) as owner:
        process = subprocess.Popen(
            [
                probe,
                owner.name,
                str(owner.owner_uid),
                str(owner.generation),
                str(owner.nonce_hi),
                str(owner.nonce_lo),
                str(owner.segment_capacity),
                str(owner.page_segment_limit),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            close_fds=True,
        )
        try:
            assert owner.wait_for_writer(process.pid) == process.pid
            assert process.stdout.readline().strip() == 'READY'
            owner.unlink_name()
            try:
                descriptor = owner.api.open_object(owner.name, os.O_RDWR)
            except OSError as error:
                assert error.errno == errno.ENOENT
            else:
                os.close(descriptor)
                raise AssertionError('retired ledger name remained openable')

            process.stdin.write('CHECK\n')
            process.stdin.flush()
            return_code = process.wait(timeout=3.0)
            stderr = process.stderr.read()
            assert return_code == 0, (
                f'attach probe exited {return_code}: {stderr}'
            )
        finally:
            terminate_owned_process(process)


if __name__ == '__main__':
    main()
