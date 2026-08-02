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
import selectors
import subprocess
import sys

from hardware_write_ledger_test_support import (
    BANK_STATE_ACTIVE,
    BANK_STATE_FREE,
    CONTROL_FLAG_ZERO_REQUIRED,
    CONTROL_OP_ARM,
    CONTROL_RESPONSE_INVALID,
    CONTROL_RESPONSE_OK,
    FAULT_PROTOCOL,
    FAULT_SEQUENCE,
    FAULT_SIM_STAMP,
    GLOBAL_ORACLE_FAULTS_WORD,
    HardwareWriteLedgerRegionOwner,
    WRITER_PID_WORD,
)


def require(condition, message):
    """Raise explicitly even when Python assertions are optimized out."""
    if not condition:
        raise AssertionError(message)


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


def close_owned_process_pipes(process):
    """Close only the three pipes allocated for the exact child."""
    for stream in (process.stdin, process.stdout, process.stderr):
        if stream is not None:
            stream.close()


def read_owned_line(process, description, timeout=3.0):
    """Bound one read from the exact child so local cleanup always runs."""
    selector = selectors.DefaultSelector()
    try:
        selector.register(process.stdout, selectors.EVENT_READ)
        if not selector.select(timeout):
            raise TimeoutError(f'timed out waiting for probe {description}')
        line = process.stdout.readline()
        if line == '':
            raise RuntimeError(
                f'probe closed stdout before {description}',
            )
        return line.strip()
    finally:
        selector.close()


def spawn_probe(probe, owner):
    """Start only the exact child used by one region test."""
    return subprocess.Popen(
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


def test_corrupt_header_is_rejected_before_claim(probe):
    """Prove an identity CRC mutation cannot obtain the Writer claim."""
    with HardwareWriteLedgerRegionOwner(
        generation=72,
        segment_capacity=4,
        page_segment_limit=2,
    ) as owner:
        owner.corrupt_header_checksum()
        process = spawn_probe(probe, owner)
        try:
            return_code = process.wait(timeout=3.0)
            stdout = process.stdout.read()
            stderr = process.stderr.read()
            require(return_code != 0, 'corrupt header probe unexpectedly passed')
            require(stdout == '', 'corrupt header probe published READY')
            require(
                owner.load_header_word(WRITER_PID_WORD) == 0,
                'corrupt header obtained the Writer claim',
            )
            require(
                'checksum mismatch' in stderr,
                f'corrupt header diagnostic changed: {stderr}',
            )
            descriptor = owner.api.open_object(owner.name, os.O_RDWR)
            os.close(descriptor)
        finally:
            terminate_owned_process(process)
            close_owned_process_pipes(process)


def test_exec_attach_survives_unlink(probe):
    """Prove exec attach, unique claim, unlink, and mapped access."""
    with HardwareWriteLedgerRegionOwner(
        generation=73,
        segment_capacity=4,
        page_segment_limit=2,
    ) as owner:
        process = spawn_probe(probe, owner)
        try:
            require(
                owner.wait_for_writer(process.pid) == process.pid,
                'attach probe Writer PID mismatch',
            )
            require(
                read_owned_line(process, 'READY') == 'READY',
                'attach probe did not publish READY',
            )
            owner.unlink_name()
            try:
                descriptor = owner.api.open_object(owner.name, os.O_RDWR)
            except OSError as error:
                require(
                    error.errno == errno.ENOENT,
                    f'retired name failed with errno {error.errno}',
                )
            else:
                os.close(descriptor)
                raise AssertionError('retired ledger name remained openable')

            process.stdin.write('CHECK\n')
            process.stdin.flush()
            return_code = process.wait(timeout=3.0)
            stderr = process.stderr.read()
            require(
                return_code == 0,
                f'attach probe exited {return_code}: {stderr}',
            )
        finally:
            terminate_owned_process(process)
            close_owned_process_pipes(process)


def test_arm_linearizes_before_first_write(probe):
    """Prove ARM fence, first sequence, receipt, and first active record."""
    with HardwareWriteLedgerRegionOwner(
        generation=74,
        segment_capacity=4,
        page_segment_limit=2,
    ) as owner:
        process = spawn_probe(probe, owner)
        try:
            require(
                owner.wait_for_writer(process.pid) == process.pid,
                'ARM probe Writer PID mismatch',
            )
            require(
                read_owned_line(process, 'READY') == 'READY',
                'ARM probe did not publish READY',
            )
            owner.unlink_name()
            request_ticket = owner.post_arm(
                interval_id=91,
                segment_budget=2,
                invocation_budget=4,
                require_zero_commands=True,
            )
            process.stdin.write(
                'WRITE 1000000 0 0 0 9223372036854775808\n',
            )
            process.stdin.flush()

            response = owner.wait_response(request_ticket)
            require(response[0] == CONTROL_OP_ARM, 'ARM operation changed')
            require(
                response[1] == CONTROL_FLAG_ZERO_REQUIRED,
                'ARM predicate flags changed',
            )
            require(
                response[9] == CONTROL_RESPONSE_OK,
                f'ARM response code was {response[9]}',
            )
            require(response[10] == 0, 'ARM selected the wrong bank')
            require(response[11] == 1, 'ARM bank epoch was not one')
            require(response[12] == 0, 'ARM fence was not zero')
            require(
                owner.wait_completed_write(1) == 1,
                'first ARM write did not publish sequence one',
            )

            bank = owner.snapshot_bank(response[10])
            expected_bank = (
                BANK_STATE_ACTIVE,
                1,
                91,
                0,
                0,
                2,
                4,
                CONTROL_FLAG_ZERO_REQUIRED,
                0,
                1,
                1,
                1,
                1,
                0,
                0,
                0,
            )
            require(
                bank == expected_bank,
                f'ARM bank mismatch: {bank!r}',
            )
            segment = owner.snapshot_segment(0, 0)
            expected_segment = (
                74,
                1,
                1,
                1,
                1000000,
                0,
                0,
                9223372036854775808,
            )
            require(
                segment == expected_segment,
                f'first ARM segment mismatch: {segment!r}',
            )
            require(
                read_owned_line(process, 'WROTE') == 'WROTE 1',
                'ARM probe did not report sequence one',
            )
            return_code = process.wait(timeout=3.0)
            stderr = process.stderr.read()
            require(
                return_code == 0,
                f'ARM probe exited {return_code}: {stderr}',
            )
        finally:
            terminate_owned_process(process)
            close_owned_process_pipes(process)


def test_arm_excludes_prior_write_and_preserves_multiple_records(probe):
    """Prove a nonzero ARM fence and lossless multi-write evidence."""
    with HardwareWriteLedgerRegionOwner(
        generation=75,
        segment_capacity=4,
        page_segment_limit=2,
    ) as owner:
        process = spawn_probe(probe, owner)
        try:
            require(
                owner.wait_for_writer(process.pid) == process.pid,
                'multi-write probe Writer PID mismatch',
            )
            require(
                read_owned_line(process, 'READY') == 'READY',
                'multi-write probe did not publish READY',
            )
            owner.unlink_name()

            process.stdin.write('BEGIN 500000\n')
            process.stdin.flush()
            require(
                read_owned_line(process, 'BEGAN') == 'BEGAN 1',
                'unarmed write did not receive sequence one',
            )
            process.stdin.write('FINISH 0 0 0 0\n')
            process.stdin.flush()
            require(
                read_owned_line(process, 'FINISHED') == 'FINISHED 1',
                'unarmed write did not finish sequence one',
            )
            require(
                owner.wait_completed_write(1) == 1,
                'unarmed write did not publish completion one',
            )

            request_ticket = owner.post_arm(
                interval_id=92,
                segment_budget=3,
                invocation_budget=4,
                require_zero_commands=True,
            )
            process.stdin.write('BEGIN 1000000\n')
            process.stdin.flush()
            require(
                read_owned_line(process, 'BEGAN') == 'BEGAN 2',
                'first included write did not receive sequence two',
            )
            response = owner.wait_response(request_ticket)
            require(
                response[9] == CONTROL_RESPONSE_OK,
                f'nonzero-fence ARM response code was {response[9]}',
            )
            require(response[10] == 0, 'nonzero-fence ARM selected wrong bank')
            require(response[11] == 1, 'nonzero-fence ARM epoch changed')
            require(response[12] == 1, 'ARM included the prior write')
            require(
                owner.snapshot_bank(0)[9:13] == (0, 0, 0, 0),
                'ARM response was delayed until finish or exposed a record',
            )

            process.stdin.write('FINISH 0 0 0 9223372036854775808\n')
            process.stdin.flush()
            require(
                read_owned_line(process, 'FINISHED') == 'FINISHED 2',
                'first included write did not finish sequence two',
            )
            require(
                owner.wait_completed_write(2) == 2,
                'first included write did not publish completion two',
            )

            process.stdin.write('BEGIN 2000000\n')
            process.stdin.flush()
            require(
                read_owned_line(process, 'BEGAN') == 'BEGAN 3',
                'second included write did not receive sequence three',
            )
            process.stdin.write('FINISH 1 0 9223372036854775808 0\n')
            process.stdin.flush()
            require(
                read_owned_line(process, 'FINISHED') == 'FINISHED 3',
                'second included write did not finish sequence three',
            )
            require(
                owner.wait_completed_write(3) == 3,
                'second included write did not publish completion three',
            )

            bank = owner.snapshot_bank(0)
            require(
                bank == (
                    BANK_STATE_ACTIVE,
                    1,
                    92,
                    1,
                    0,
                    3,
                    4,
                    CONTROL_FLAG_ZERO_REQUIRED,
                    0,
                    2,
                    2,
                    2,
                    3,
                    0,
                    0,
                    0,
                ),
                f'multi-write bank mismatch: {bank!r}',
            )
            require(
                owner.snapshot_segment(0, 0) == (
                    75,
                    2,
                    2,
                    1,
                    1000000,
                    0,
                    0,
                    9223372036854775808,
                ),
                'second write overwrote the first included segment',
            )
            require(
                owner.snapshot_segment(0, 1) == (
                    75,
                    3,
                    3,
                    1,
                    2000000,
                    1,
                    9223372036854775808,
                    0,
                ),
                'second included segment was not appended exactly',
            )

            process.stdin.write('EXIT\n')
            process.stdin.flush()
            return_code = process.wait(timeout=3.0)
            stderr = process.stderr.read()
            require(
                return_code == 0,
                f'multi-write probe exited {return_code}: {stderr}',
            )
        finally:
            terminate_owned_process(process)
            close_owned_process_pipes(process)


def test_same_ticket_different_payload_replay_latches_fault(probe):
    """Prove an acknowledged ticket cannot be reused for another payload."""
    with HardwareWriteLedgerRegionOwner(
        generation=76,
        segment_capacity=4,
        page_segment_limit=2,
    ) as owner:
        process = spawn_probe(probe, owner)
        try:
            require(
                owner.wait_for_writer(process.pid) == process.pid,
                'replay probe Writer PID mismatch',
            )
            require(
                read_owned_line(process, 'READY') == 'READY',
                'replay probe did not publish READY',
            )
            owner.unlink_name()
            request_ticket = owner.post_arm(
                interval_id=93,
                segment_budget=2,
                invocation_budget=3,
                require_zero_commands=False,
            )
            process.stdin.write('BEGIN 1000000\n')
            process.stdin.flush()
            require(
                read_owned_line(process, 'BEGAN') == 'BEGAN 1',
                'replay setup write did not begin',
            )
            response = owner.wait_response(request_ticket)
            require(
                response[9] == CONTROL_RESPONSE_OK,
                'replay setup ARM was not acknowledged',
            )
            process.stdin.write('FINISH 0 0 0 0\n')
            process.stdin.flush()
            require(
                read_owned_line(process, 'FINISHED') == 'FINISHED 1',
                'replay setup write did not finish',
            )
            require(owner.wait_completed_write(1) == 1, 'setup completion lost')

            owner.replay_arm_with_interval(request_ticket, interval_id=94)
            process.stdin.write('BEGIN 2000000\n')
            process.stdin.flush()
            require(
                read_owned_line(process, 'BEGAN') == 'BEGAN 2',
                'replay trigger write did not begin',
            )
            faults = owner.load_header_word(GLOBAL_ORACLE_FAULTS_WORD)
            require(
                faults & FAULT_PROTOCOL,
                'same-ticket different-payload replay was silently ignored',
            )
            require(
                owner.snapshot_bank(response[10])[2] == 93,
                'replay changed the acknowledged interval identity',
            )
            process.stdin.write('FINISH 0 0 0 0\n')
            process.stdin.flush()
            require(
                read_owned_line(process, 'FINISHED') == 'FINISHED 2',
                'replay trigger write did not finish',
            )
            process.stdin.write('EXIT\n')
            process.stdin.flush()
            return_code = process.wait(timeout=3.0)
            stderr = process.stderr.read()
            require(
                return_code == 0,
                f'replay probe exited {return_code}: {stderr}',
            )
        finally:
            terminate_owned_process(process)
            close_owned_process_pipes(process)


def test_decreasing_sim_stamp_latches_fault_without_overwrite(probe):
    """Prove time regression faults an interval and preserves both writes."""
    with HardwareWriteLedgerRegionOwner(
        generation=77,
        segment_capacity=4,
        page_segment_limit=2,
    ) as owner:
        process = spawn_probe(probe, owner)
        try:
            require(
                owner.wait_for_writer(process.pid) == process.pid,
                'stamp probe Writer PID mismatch',
            )
            require(
                read_owned_line(process, 'READY') == 'READY',
                'stamp probe did not publish READY',
            )
            owner.unlink_name()
            request_ticket = owner.post_arm(
                interval_id=95,
                segment_budget=3,
                invocation_budget=3,
                require_zero_commands=False,
            )

            process.stdin.write('BEGIN 200\n')
            process.stdin.flush()
            require(
                read_owned_line(process, 'BEGAN') == 'BEGAN 1',
                'stamp setup write did not begin',
            )
            response = owner.wait_response(request_ticket)
            process.stdin.write('FINISH 0 0 0 0\n')
            process.stdin.flush()
            require(
                read_owned_line(process, 'FINISHED') == 'FINISHED 1',
                'stamp setup write did not finish',
            )
            require(owner.wait_completed_write(1) == 1, 'stamp setup lost')

            process.stdin.write('BEGIN 100\n')
            process.stdin.flush()
            require(
                read_owned_line(process, 'BEGAN') == 'BEGAN 2',
                'regressed-stamp write did not begin',
            )
            process.stdin.write('FINISH 0 0 0 0\n')
            process.stdin.flush()
            require(
                read_owned_line(process, 'FINISHED') == 'FINISHED 2',
                'regressed-stamp write did not finish',
            )
            require(owner.wait_completed_write(2) == 2, 'stamp fault lost')

            global_faults = owner.load_header_word(GLOBAL_ORACLE_FAULTS_WORD)
            bank = owner.snapshot_bank(response[10])
            require(
                global_faults & FAULT_SIM_STAMP,
                'decreasing simulation stamp was accepted globally',
            )
            require(
                bank[13] & FAULT_SIM_STAMP,
                'decreasing simulation stamp did not fault the interval',
            )
            require(
                bank[9:13] == (2, 2, 1, 2),
                f'stamp fault lost invocation evidence: {bank!r}',
            )
            require(
                owner.snapshot_segment(response[10], 0)[4] == 200,
                'stamp fault overwrote the first segment',
            )
            require(
                owner.snapshot_segment(response[10], 1)[4] == 100,
                'stamp fault did not retain the offending segment',
            )

            process.stdin.write('EXIT\n')
            process.stdin.flush()
            return_code = process.wait(timeout=3.0)
            stderr = process.stderr.read()
            require(
                return_code == 0,
                f'stamp probe exited {return_code}: {stderr}',
            )
        finally:
            terminate_owned_process(process)
            close_owned_process_pipes(process)


def test_sequence_exhaustion_rejects_arm_before_bank_activation(probe):
    """Prove sequence exhaustion cannot leave an impossible ACTIVE bank."""
    with HardwareWriteLedgerRegionOwner(
        generation=82,
        segment_capacity=4,
        page_segment_limit=2,
    ) as owner:
        process = spawn_probe(probe, owner)
        try:
            require(
                owner.wait_for_writer(process.pid) == process.pid,
                'exhaustion probe Writer PID mismatch',
            )
            require(
                read_owned_line(process, 'READY') == 'READY',
                'exhaustion probe did not publish READY',
            )
            owner.unlink_name()
            owner.force_last_completed_write_seq((1 << 64) - 1)
            request_ticket = owner.post_arm(
                interval_id=96,
                segment_budget=2,
                invocation_budget=2,
                require_zero_commands=True,
            )
            process.stdin.write('BEGIN 100\n')
            process.stdin.flush()
            require(
                read_owned_line(process, 'BEGAN') == 'BEGAN 0',
                'sequence exhaustion fabricated a write ticket',
            )
            response = owner.wait_response(request_ticket)
            require(
                response[9] == CONTROL_RESPONSE_INVALID,
                f'exhausted ARM response was {response[9]}',
            )
            require(
                owner.snapshot_bank(0)[0] == BANK_STATE_FREE and
                owner.snapshot_bank(1)[0] == BANK_STATE_FREE,
                'sequence exhaustion left an impossible ACTIVE bank',
            )
            require(
                owner.load_header_word(GLOBAL_ORACLE_FAULTS_WORD) &
                FAULT_SEQUENCE,
                'sequence exhaustion did not latch its global fault',
            )
            process.stdin.write('EXIT\n')
            process.stdin.flush()
            return_code = process.wait(timeout=3.0)
            stderr = process.stderr.read()
            require(
                return_code == 0,
                f'exhaustion probe exited {return_code}: {stderr}',
            )
        finally:
            terminate_owned_process(process)
            close_owned_process_pipes(process)


def test_wrapped_request_cannot_lower_exhausted_response_ticket(probe):
    """Prove mailbox wrap faults without publishing response ticket zero."""
    with HardwareWriteLedgerRegionOwner(
        generation=83,
        segment_capacity=4,
        page_segment_limit=2,
    ) as owner:
        process = spawn_probe(probe, owner)
        try:
            require(
                owner.wait_for_writer(process.pid) == process.pid,
                'wrap probe Writer PID mismatch',
            )
            require(
                read_owned_line(process, 'READY') == 'READY',
                'wrap probe did not publish READY',
            )
            owner.unlink_name()
            request_ticket = owner.post_arm(
                interval_id=97,
                segment_budget=2,
                invocation_budget=3,
                require_zero_commands=False,
            )
            process.stdin.write('BEGIN 100\n')
            process.stdin.flush()
            require(
                read_owned_line(process, 'BEGAN') == 'BEGAN 1',
                'wrap setup write did not begin',
            )
            require(
                owner.wait_response(request_ticket)[9] == CONTROL_RESPONSE_OK,
                'wrap setup ARM was not acknowledged',
            )
            process.stdin.write('FINISH 0 0 0 0\n')
            process.stdin.flush()
            require(
                read_owned_line(process, 'FINISHED') == 'FINISHED 1',
                'wrap setup write did not finish',
            )
            require(owner.wait_completed_write(1) == 1, 'wrap setup lost')

            owner.force_wrapped_request_after_exhausted_response()
            process.stdin.write('BEGIN 200\n')
            process.stdin.flush()
            require(
                read_owned_line(process, 'BEGAN') == 'BEGAN 2',
                'wrap trigger write did not begin',
            )
            require(
                owner.load_response_ticket() == (1 << 64) - 1,
                'wrapped request lowered the response ticket to zero',
            )
            require(
                owner.load_header_word(GLOBAL_ORACLE_FAULTS_WORD) &
                FAULT_PROTOCOL,
                'wrapped request did not latch a protocol fault',
            )
            process.stdin.write('FINISH 0 0 0 0\n')
            process.stdin.flush()
            require(
                read_owned_line(process, 'FINISHED') == 'FINISHED 2',
                'wrap trigger write did not finish',
            )
            process.stdin.write('EXIT\n')
            process.stdin.flush()
            return_code = process.wait(timeout=3.0)
            stderr = process.stderr.read()
            require(
                return_code == 0,
                f'wrap probe exited {return_code}: {stderr}',
            )
        finally:
            terminate_owned_process(process)
            close_owned_process_pipes(process)


def main():
    """Run the bounded cross-process ledger behavior slices."""
    if len(sys.argv) != 2:
        raise SystemExit('expected the attach probe executable')
    probe = os.path.abspath(sys.argv[1])
    test_corrupt_header_is_rejected_before_claim(probe)
    test_exec_attach_survives_unlink(probe)
    test_arm_linearizes_before_first_write(probe)
    test_arm_excludes_prior_write_and_preserves_multiple_records(probe)
    test_same_ticket_different_payload_replay_latches_fault(probe)
    test_decreasing_sim_stamp_latches_fault_without_overwrite(probe)
    test_sequence_exhaustion_rejects_arm_before_bank_activation(probe)
    test_wrapped_request_cannot_lower_exhausted_response_ticket(probe)


if __name__ == '__main__':
    main()
