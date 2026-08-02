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

"""Independent Parent support for hardware-write ledger process tests."""

import ctypes
import ctypes.util
import mmap
import os
import struct
import time
import uuid


MAGIC = 0x564E48574C444731
ABI_VERSION = 1
ENDIAN_TAG = 0x0102030405060708
HEADER_BYTES = 192
CONTROL_BYTES = 128
BANK_BYTES = 128
SEGMENT_BYTES = 64
PAGE_BYTES = 192
BANK_COUNT = 2
INIT_READY = 1
CRC64_ECMA_POLYNOMIAL = 0x42F0E1EBA9EA3693
UINT64_MASK = (1 << 64) - 1
HEADER_FORMAT = '<24Q'
CONTROL_FORMAT = '<16Q'
CONTROL_REQUEST_FORMAT = '<9Q'
BANK_FORMAT = '<16Q'
SEGMENT_FORMAT = '<8Q'
INIT_STATE_WORD = 17
WRITER_PID_WORD = 18
LAST_COMPLETED_WRITE_SEQ_WORD = 19
GLOBAL_ORACLE_FAULTS_WORD = 20
CONTROL_OFFSET = HEADER_BYTES
CONTROL_RESPONSE_TICKET_WORD = 14
CONTROL_REQUEST_TICKET_WORD = 15
BANK_STATE_ACTIVE = 1
CONTROL_OP_ARM = 1
CONTROL_FLAG_ZERO_REQUIRED = 1
CONTROL_RESPONSE_OK = 1
FAULT_SIM_STAMP = 1 << 3
FAULT_PROTOCOL = 1 << 7


def crc64_ecma_words(words):
    """Calculate CRC64-ECMA over little-endian words independently."""
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
    """Bind immutable ABI geometry, identity, features, and reserved zeros."""
    return crc64_ecma_words((*header[0:17], header[21], header[22]))


def control_request_checksum(owner, control, request_ticket):
    """Bind one Parent request to the region identity and ticket."""
    return crc64_ecma_words(
        (
            owner.owner_uid,
            owner.generation,
            owner.nonce_hi,
            owner.nonce_lo,
            *control[0:8],
            request_ticket,
        ),
    )


def control_response_checksum(owner, control, response_ticket):
    """Bind one Writer response to its request and region identity."""
    return crc64_ecma_words(
        (
            owner.owner_uid,
            owner.generation,
            owner.nonce_hi,
            owner.nonce_lo,
            control[8],
            response_ticket,
            *control[9:13],
        ),
    )


class AtomicUint64:
    """ctypes binding for the GCC lock-free uint64 atomic ABI."""

    _ACQUIRE = 2
    _RELEASE = 3

    def __init__(self):
        """Bind the uint64 operations shared with the C++ implementation."""
        library_name = ctypes.util.find_library('atomic')
        if library_name is None:
            raise RuntimeError('libatomic is required for the ledger test')
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
        """Acquire-load one aligned ABI word."""
        word = self._word(region, offset)
        return int(self._load(ctypes.byref(word), self._ACQUIRE))

    def store_release(self, region, offset, value):
        """Release-store one aligned ABI word."""
        word = self._word(region, offset)
        self._store(ctypes.byref(word), value, self._RELEASE)


class PosixSharedMemoryApi:
    """Direct libc POSIX shared-memory calls without a resource tracker."""

    def __init__(self):
        """Bind the two POSIX name operations used by the test owner."""
        library_name = ctypes.util.find_library('c')
        if library_name is None:
            raise RuntimeError('libc is required for the ledger test')
        library = ctypes.CDLL(library_name, use_errno=True)
        self._shm_open = library.shm_open
        self._shm_open.argtypes = [
            ctypes.c_char_p,
            ctypes.c_int,
            ctypes.c_uint,
        ]
        self._shm_open.restype = ctypes.c_int
        self._shm_unlink = library.shm_unlink
        self._shm_unlink.argtypes = [ctypes.c_char_p]
        self._shm_unlink.restype = ctypes.c_int
        self._library = library

    def open_object(self, name, flags, mode=0):
        """Open one exact object name or raise its OS error."""
        ctypes.set_errno(0)
        descriptor = self._shm_open(name.encode('ascii'), flags, mode)
        if descriptor < 0:
            error_number = ctypes.get_errno()
            raise OSError(error_number, os.strerror(error_number), name)
        return descriptor

    def unlink(self, name):
        """Retire one exact object name or raise its OS error."""
        ctypes.set_errno(0)
        if self._shm_unlink(name.encode('ascii')) != 0:
            error_number = ctypes.get_errno()
            raise OSError(error_number, os.strerror(error_number), name)


class HardwareWriteLedgerRegionOwner:
    """Own and initialize one cross-process hardware-write ledger region."""

    def __init__(self, generation, segment_capacity, page_segment_limit):
        """Create one exact 0600 region and release-publish READY."""
        self.generation = generation
        self.segment_capacity = segment_capacity
        self.page_segment_limit = page_segment_limit
        self.owner_uid = os.geteuid()
        self.nonce = uuid.uuid4().hex
        self.nonce_hi = int(self.nonce[:16], 16)
        self.nonce_lo = int(self.nonce[16:], 16)
        self.name = f'/voice_nav_hardware_{uuid.uuid4().hex[:16]}'
        self.bank_stride = BANK_BYTES + segment_capacity * SEGMENT_BYTES
        self.region_bytes = (
            HEADER_BYTES + CONTROL_BYTES + BANK_COUNT * self.bank_stride
        )
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
            ENDIAN_TAG,
            HEADER_BYTES,
            CONTROL_BYTES,
            BANK_BYTES,
            SEGMENT_BYTES,
            PAGE_BYTES,
            self.region_bytes,
            BANK_COUNT,
            self.segment_capacity,
            self.page_segment_limit,
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
            0,
            0,
        ]
        header[23] = header_checksum(header)
        struct.pack_into(HEADER_FORMAT, self.region, 0, *header)
        self.atomic.store_release(
            self.region,
            INIT_STATE_WORD * 8,
            INIT_READY,
        )

    def load_header_word(self, index):
        """Acquire-load one dynamic header word."""
        return self.atomic.load_acquire(self.region, index * 8)

    def corrupt_header_checksum(self):
        """Flip only the immutable header checksum for one mutation test."""
        header = list(struct.unpack_from(HEADER_FORMAT, self.region, 0))
        header[23] ^= 1
        struct.pack_into(HEADER_FORMAT, self.region, 0, *header)

    def wait_for_writer(self, expected_pid, timeout=3.0):
        """Wait until the exact child PID owns the Writer claim."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            writer_pid = self.load_header_word(WRITER_PID_WORD)
            if writer_pid == expected_pid:
                return writer_pid
            if writer_pid != 0:
                raise AssertionError(
                    f'unexpected ledger Writer PID {writer_pid}',
                )
            time.sleep(0.005)
        raise TimeoutError('ledger Writer did not claim the region')

    def post_arm(
        self,
        interval_id,
        segment_budget,
        invocation_budget,
        require_zero_commands,
    ):
        """Release-publish the first checksummed ARM request."""
        if self.atomic.load_acquire(
            self.region,
            CONTROL_OFFSET + CONTROL_REQUEST_TICKET_WORD * 8,
        ) != 0:
            raise AssertionError('test owner supports one request in this slice')
        flags = CONTROL_FLAG_ZERO_REQUIRED if require_zero_commands else 0
        control = [
            CONTROL_OP_ARM,
            flags,
            interval_id,
            0,
            0,
            segment_budget,
            invocation_budget,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ]
        request_ticket = 1
        control[8] = control_request_checksum(
            self,
            control,
            request_ticket,
        )
        struct.pack_into(CONTROL_FORMAT, self.region, CONTROL_OFFSET, *control)
        self.atomic.store_release(
            self.region,
            CONTROL_OFFSET + CONTROL_REQUEST_TICKET_WORD * 8,
            request_ticket,
        )
        return request_ticket

    def replay_arm_with_interval(self, request_ticket, interval_id):
        """Release-republish one ticket with a different valid payload."""
        control = list(
            struct.unpack_from(CONTROL_FORMAT, self.region, CONTROL_OFFSET),
        )
        response_ticket = self.atomic.load_acquire(
            self.region,
            CONTROL_OFFSET + CONTROL_RESPONSE_TICKET_WORD * 8,
        )
        published_ticket = self.atomic.load_acquire(
            self.region,
            CONTROL_OFFSET + CONTROL_REQUEST_TICKET_WORD * 8,
        )
        if response_ticket != request_ticket or published_ticket != request_ticket:
            raise AssertionError('replay requires one completed request ticket')
        control[2] = interval_id
        control[8] = control_request_checksum(
            self,
            control,
            request_ticket,
        )
        struct.pack_into(
            CONTROL_REQUEST_FORMAT,
            self.region,
            CONTROL_OFFSET,
            *control[0:9],
        )
        self.atomic.store_release(
            self.region,
            CONTROL_OFFSET + CONTROL_REQUEST_TICKET_WORD * 8,
            request_ticket,
        )

    def wait_response(self, request_ticket, timeout=3.0):
        """Acquire one exact Writer response and validate its checksum."""
        deadline = time.monotonic() + timeout
        response_offset = (
            CONTROL_OFFSET + CONTROL_RESPONSE_TICKET_WORD * 8
        )
        while time.monotonic() < deadline:
            response_ticket = self.atomic.load_acquire(
                self.region,
                response_offset,
            )
            if response_ticket == request_ticket:
                control = struct.unpack_from(
                    CONTROL_FORMAT,
                    self.region,
                    CONTROL_OFFSET,
                )
                if control[13] != control_response_checksum(
                    self,
                    control,
                    response_ticket,
                ):
                    raise AssertionError('ledger response checksum mismatch')
                return control
            if response_ticket != 0:
                raise AssertionError(
                    f'unexpected response ticket {response_ticket}',
                )
            time.sleep(0.005)
        raise TimeoutError('ledger Writer did not publish a response')

    def wait_completed_write(self, expected_write_seq, timeout=3.0):
        """Acquire the global completion publication for one write."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            write_seq = self.load_header_word(
                LAST_COMPLETED_WRITE_SEQ_WORD,
            )
            if write_seq == expected_write_seq:
                return write_seq
            if write_seq > expected_write_seq:
                raise AssertionError(f'unexpected completed seq {write_seq}')
            time.sleep(0.005)
        raise TimeoutError('ledger Writer did not complete the write')

    def bank_offset(self, bank_index):
        """Return one checked bank offset."""
        if bank_index < 0 or bank_index >= BANK_COUNT:
            raise IndexError('ledger bank index is outside the region')
        return HEADER_BYTES + CONTROL_BYTES + bank_index * self.bank_stride

    def snapshot_bank(self, bank_index):
        """Read one ACTIVE bank after an external acquire publication."""
        return struct.unpack_from(
            BANK_FORMAT,
            self.region,
            self.bank_offset(bank_index),
        )

    def snapshot_segment(self, bank_index, segment_index):
        """Read one preallocated segment after completion publication."""
        if segment_index < 0 or segment_index >= self.segment_capacity:
            raise IndexError('ledger segment index is outside the bank')
        offset = (
            self.bank_offset(bank_index)
            + BANK_BYTES
            + segment_index * SEGMENT_BYTES
        )
        return struct.unpack_from(SEGMENT_FORMAT, self.region, offset)

    def unlink_name(self):
        """Retire the POSIX name while retaining this mapping."""
        if self.linked:
            self.api.unlink(self.name)
            self.linked = False

    def cleanup(self):
        """Release only resources owned by this test object."""
        try:
            if self.linked:
                try:
                    self.api.unlink(self.name)
                except FileNotFoundError:
                    pass
                finally:
                    self.linked = False
        finally:
            try:
                if self.region is not None:
                    self.region.close()
                    self.region = None
            finally:
                if self.fd >= 0:
                    os.close(self.fd)
                    self.fd = -1

    def __enter__(self):
        return self

    def __exit__(self, _exception_type, _exception, _traceback):
        self.cleanup()
