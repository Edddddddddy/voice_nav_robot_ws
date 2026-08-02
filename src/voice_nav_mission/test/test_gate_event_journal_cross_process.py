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

import ctypes
import ctypes.util
import errno
import mmap
import os
from pathlib import Path
import struct
import subprocess
import sys
import time
import unittest
import uuid


MAGIC = 0x564E474154454A31
ABI_VERSION = 1
HEADER_BYTES = 128
SLOT_BYTES = 256
INIT_READY = 1
PHASE_COMMITTED = 2
KIND_OUTPUT_ATTEMPT = 2
CRC64_ECMA_POLYNOMIAL = 0x42F0E1EBA9EA3693
UINT64_MASK = (1 << 64) - 1
HEADER_WORDS = 16
SLOT_WORDS = 32
HEADER_FORMAT = '<16Q'
SLOT_FORMAT = '<32Q'
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


def crc64_ecma_words(words):
    """Calculate CRC64-ECMA over little-endian uint64 words independently."""
    checksum = 0
    for word in words:
        for byte in int(word).to_bytes(8, byteorder='little'):
            checksum ^= byte << 56
            for _ in range(8):
                high_bit_set = checksum & (1 << 63)
                checksum = (checksum << 1) & UINT64_MASK
                if high_bit_set:
                    checksum ^= CRC64_ECMA_POLYNOMIAL
    return checksum


def header_checksum(header):
    """Return the ABI-v1 header checksum without dynamic header words."""
    return crc64_ecma_words((*header[0:10], header[15]))


def intent_checksum(slot):
    """Return the ABI-v1 output-intent checksum."""
    indices = (
        1, 2, 3, 4, 9, 10, 11, 13, 15, 16,
        17, 18, 19, 20, 21, 22, 25, 26, 27, 28,
    )
    return crc64_ecma_words(slot[index] for index in indices)


def commit_checksum(slot):
    """Return the ABI-v1 committed-output checksum."""
    indices = (
        7, 1, 2, 3, 4, 9, 10, 11, 13, 15, 16, 17, 18, 19,
        20, 21, 22, 25, 26, 27, 28, 5, 6, 12, 14, 23, 24,
    )
    return crc64_ecma_words(slot[index] for index in indices)


class AtomicUint64:
    """Small ctypes binding for GCC's lock-free uint64 atomic ABI."""

    _ACQUIRE = 2
    _RELEASE = 3

    def __init__(self):
        library_name = ctypes.util.find_library('atomic')
        if library_name is None:
            raise RuntimeError('libatomic is required for the journal test')
        library = ctypes.CDLL(library_name)
        self._load = getattr(library, '__atomic_load_8')
        self._load.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self._load.restype = ctypes.c_uint64
        self._store = getattr(library, '__atomic_store_8')
        self._store.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.c_int,
        ]
        self._store.restype = None
        self._library = library

    @staticmethod
    def _word(region, offset):
        return ctypes.c_uint64.from_buffer(region, offset)

    def load_acquire(self, region, offset):
        """Acquire-load one aligned uint64 from an mmap."""
        word = self._word(region, offset)
        return int(self._load(ctypes.byref(word), self._ACQUIRE))

    def store_release(self, region, offset, value):
        """Release-store one aligned uint64 into an mmap."""
        word = self._word(region, offset)
        self._store(ctypes.byref(word), value, self._RELEASE)


class PosixSharedMemoryApi:
    """Direct libc POSIX shared-memory API without a resource tracker."""

    def __init__(self):
        library_name = ctypes.util.find_library('c')
        if library_name is None:
            raise RuntimeError('libc is required for the journal test')
        self._library = ctypes.CDLL(library_name, use_errno=True)
        self._shm_open = self._library.shm_open
        self._shm_open.argtypes = [
            ctypes.c_char_p,
            ctypes.c_int,
            ctypes.c_uint,
        ]
        self._shm_open.restype = ctypes.c_int
        self._shm_unlink = self._library.shm_unlink
        self._shm_unlink.argtypes = [ctypes.c_char_p]
        self._shm_unlink.restype = ctypes.c_int

    def open_object(self, name, flags, mode=0):
        """Open a POSIX shared-memory object or raise its OS error."""
        encoded_name = name.encode('ascii')
        ctypes.set_errno(0)
        descriptor = self._shm_open(encoded_name, flags, mode)
        if descriptor < 0:
            error_number = ctypes.get_errno()
            raise OSError(error_number, os.strerror(error_number), name)
        return descriptor

    def unlink(self, name):
        """Unlink a POSIX shared-memory name or raise its OS error."""
        ctypes.set_errno(0)
        if self._shm_unlink(name.encode('ascii')) != 0:
            error_number = ctypes.get_errno()
            raise OSError(error_number, os.strerror(error_number), name)


class GateEventJournalOwner:
    """Parent-side owner of one initialized journal generation."""

    def __init__(self, capacity, generation):
        self.capacity = capacity
        self.generation = generation
        self.owner_uid = os.geteuid()
        self.nonce = uuid.uuid4().hex
        self.nonce_hi = int(self.nonce[:16], 16)
        self.nonce_lo = int(self.nonce[16:], 16)
        self.name = f'/voice_nav_gate_{uuid.uuid4().hex}'
        self.region_bytes = HEADER_BYTES + capacity * SLOT_BYTES
        self.api = PosixSharedMemoryApi()
        self.atomic = AtomicUint64()
        self.fd = -1
        self.region = None
        self.linked = False

        try:
            flags = os.O_CREAT | os.O_EXCL | os.O_RDWR | os.O_CLOEXEC
            self.fd = self.api.open_object(self.name, flags, 0o600)
            self.linked = True
            os.fchmod(self.fd, 0o600)
            os.ftruncate(self.fd, self.region_bytes)
            self.region = mmap.mmap(
                self.fd,
                self.region_bytes,
                flags=mmap.MAP_SHARED,
                prot=mmap.PROT_READ | mmap.PROT_WRITE,
            )
            self._initialize_region()
        except BaseException:
            self.cleanup()
            raise

    def _initialize_region(self):
        self.region[:] = bytes(self.region_bytes)
        header = [
            MAGIC,
            ABI_VERSION,
            HEADER_BYTES,
            SLOT_BYTES,
            self.region_bytes,
            self.capacity,
            self.owner_uid,
            self.generation,
            self.nonce_hi,
            self.nonce_lo,
            0,
            0,
            0,
            0,
            0,
            0,
        ]
        header[14] = header_checksum(header)
        struct.pack_into(HEADER_FORMAT, self.region, 0, *header)
        self.atomic.store_release(self.region, 80, INIT_READY)

    def load_header_word(self, index):
        """Acquire-load a dynamic header word by ABI index."""
        return self.atomic.load_acquire(self.region, index * 8)

    def load_slot_phase(self):
        """Acquire-load the first slot's phase word."""
        return self.atomic.load_acquire(self.region, HEADER_BYTES)

    def snapshot(self):
        """Read header and slot after an acquire synchronization point."""
        header = struct.unpack_from(HEADER_FORMAT, self.region, 0)
        slot = struct.unpack_from(SLOT_FORMAT, self.region, HEADER_BYTES)
        return header, slot

    def open_existing(self):
        """Open the current name without creating it."""
        return self.api.open_object(self.name, os.O_RDWR | os.O_CLOEXEC)

    def unlink_name(self):
        """Idempotently retire the name while existing mappings survive."""
        if not self.linked:
            return
        try:
            self.api.unlink(self.name)
        except OSError as error:
            if error.errno != errno.ENOENT:
                raise
        self.linked = False

    def assert_name_missing(self, test_case):
        """Assert that opening the retired name fails with ENOENT."""
        with test_case.assertRaises(OSError) as context:
            descriptor = self.open_existing()
            os.close(descriptor)
        test_case.assertEqual(context.exception.errno, errno.ENOENT)

    def cleanup(self):
        """Idempotently release every parent-owned resource."""
        try:
            self.unlink_name()
        finally:
            if self.region is not None:
                self.region.close()
                self.region = None
            if self.fd >= 0:
                os.close(self.fd)
                self.fd = -1

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception, traceback):
        del exception_type, exception, traceback
        self.cleanup()


class GateEventJournalCrossProcessTest(unittest.TestCase):
    """Exercise owner and writer as distinct OS processes."""

    @classmethod
    def setUpClass(cls):
        if len(sys.argv) != 2:
            raise RuntimeError('expected the attach probe executable path')
        cls.probe_path = Path(sys.argv[1])

    def test_parent_unlink_preserves_child_mapping_and_committed_record(self):
        self.assertTrue(
            self.probe_path.is_file(),
            f'attach probe is missing: {self.probe_path}',
        )
        self.assertTrue(os.access(self.probe_path, os.X_OK))

        process = None
        with GateEventJournalOwner(capacity=1, generation=73) as owner:
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

                self.assertEqual(owner.load_slot_phase(), PHASE_COMMITTED)
                self._assert_committed_generation(owner, process.pid)
                owner.assert_name_missing(self)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate(timeout=WAIT_TIMEOUT_SECONDS)

    def _assert_committed_generation(self, owner, child_pid):
        self.assertEqual(owner.load_header_word(11), 1)
        self.assertEqual(owner.load_header_word(12), 0)
        self.assertEqual(owner.load_header_word(13), child_pid)
        header, slot = owner.snapshot()

        expected_header = [
            MAGIC,
            ABI_VERSION,
            HEADER_BYTES,
            SLOT_BYTES,
            owner.region_bytes,
            owner.capacity,
            owner.owner_uid,
            owner.generation,
            owner.nonce_hi,
            owner.nonce_lo,
            INIT_READY,
            1,
            0,
            child_pid,
            0,
            0,
        ]
        expected_header[14] = header_checksum(expected_header)
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
        self.assertEqual(slot[7], intent_checksum(slot))
        self.assertNotEqual(slot[8], 0)
        self.assertEqual(slot[8], commit_checksum(slot))


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]])
