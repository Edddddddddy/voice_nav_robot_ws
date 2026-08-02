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

"""Cross-process contract test for the parent-owned Gate event journal."""

import os
from pathlib import Path
import subprocess
import sys
import time
import unittest

import gate_event_journal_test_support as journal_support


PHASE_COMMITTED = 2
KIND_OUTPUT_ATTEMPT = 2
SLOT_WORDS = 32
WAIT_TIMEOUT_SECONDS = 4.0
POLL_SECONDS = 0.005

EVENT_CODE = 41
REASON = 9
OUTPUT_ATTEMPT_SEQ = 17
INTENDED_OUTPUT_SEQ = 18
ROS_STAMP_SEC_BITS = 23
ROS_STAMP_NANOSEC = 456
GATE_INSTANCE_HI = 0x1111222233334444
GATE_INSTANCE_LO = 0x5555666677778888
CAUSE_TRANSITION_JOURNAL_SEQ = 3
FLAGS = 0xA5


class GateEventJournalCrossProcessTest(unittest.TestCase):
    """Exercise owner and writer as distinct OS processes."""

    @classmethod
    def setUpClass(cls):
        """Capture the attach probe supplied by the CTest command."""
        if len(sys.argv) != 2:
            raise RuntimeError('expected the attach probe executable path')
        cls.probe_path = Path(sys.argv[1])

    def test_parent_unlink_preserves_child_mapping_and_committed_record(self):
        """Keep the child mapping writable after unlinking its public name."""
        self.assertTrue(
            self.probe_path.is_file(),
            f'attach probe is missing: {self.probe_path}',
        )
        self.assertTrue(os.access(self.probe_path, os.X_OK))

        process = None
        with journal_support.GateEventJournalOwner(
            capacity=1,
            generation=73,
        ) as owner:
            command = [
                str(self.probe_path),
                owner.name,
                str(owner.owner_uid),
                str(owner.generation),
                str(owner.capacity),
                owner.nonce,
            ]
            process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            try:
                deadline = time.monotonic() + WAIT_TIMEOUT_SECONDS
                while time.monotonic() < deadline:
                    writer_pid = owner.load_header_word(13)
                    if writer_pid == process.pid:
                        break
                    if process.poll() is not None:
                        stdout, stderr = process.communicate()
                        self.fail(
                            'attach probe exited before writer claim: '
                            f'rc={process.returncode}, stdout={stdout!r}, '
                            f'stderr={stderr!r}',
                        )
                    time.sleep(POLL_SECONDS)
                else:
                    self.fail('timed out waiting for the child writer claim')

                descriptor = owner.open_existing()
                os.close(descriptor)
                owner.unlink_name()
                owner.assert_name_missing(self)

                stdout, stderr = process.communicate(
                    input=b'\x01',
                    timeout=WAIT_TIMEOUT_SECONDS,
                )
                self.assertEqual(process.returncode, 0, stderr.decode())
                self.assertEqual(stdout, b'')
                self.assertEqual(stderr, b'')

                self._assert_committed_generation(owner, process.pid)
                owner.assert_name_missing(self)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate(timeout=WAIT_TIMEOUT_SECONDS)

    def _assert_committed_generation(self, owner, child_pid):
        self.assertEqual(owner.load_slot_phase(), PHASE_COMMITTED)
        self.assertEqual(owner.load_header_word(11), 1)
        self.assertEqual(owner.load_header_word(12), 0)
        self.assertEqual(owner.load_header_word(13), child_pid)
        header, slot = owner.snapshot()

        expected_header = [
            journal_support.MAGIC,
            journal_support.ABI_VERSION,
            journal_support.HEADER_BYTES,
            journal_support.SLOT_BYTES,
            owner.region_bytes,
            owner.capacity,
            owner.owner_uid,
            owner.generation,
            owner.nonce_hi,
            owner.nonce_lo,
            journal_support.INIT_READY,
            1,
            0,
            child_pid,
            0,
            0,
        ]
        expected_header[14] = journal_support.header_checksum(expected_header)
        self.assertEqual(header, tuple(expected_header))

        expected_slot = [0] * SLOT_WORDS
        expected_slot[0] = PHASE_COMMITTED
        expected_slot[1] = KIND_OUTPUT_ATTEMPT
        expected_slot[2] = 1
        expected_slot[3] = owner.generation
        expected_slot[4] = 100
        expected_slot[6] = 200
        expected_slot[9] = EVENT_CODE
        expected_slot[10] = REASON
        expected_slot[15] = OUTPUT_ATTEMPT_SEQ
        expected_slot[16] = INTENDED_OUTPUT_SEQ
        expected_slot[17] = ROS_STAMP_SEC_BITS
        expected_slot[18] = ROS_STAMP_NANOSEC
        expected_slot[25] = GATE_INSTANCE_HI
        expected_slot[26] = GATE_INSTANCE_LO
        expected_slot[27] = CAUSE_TRANSITION_JOURNAL_SEQ
        expected_slot[28] = FLAGS
        for index, expected in enumerate(expected_slot):
            if index not in (7, 8):
                self.assertEqual(slot[index], expected, f'slot word {index}')
        self.assertNotEqual(slot[7], 0)
        self.assertEqual(slot[7], journal_support.intent_checksum(slot))
        self.assertNotEqual(slot[8], 0)
        self.assertEqual(slot[8], journal_support.commit_checksum(slot))


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]])
