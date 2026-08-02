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
from dataclasses import dataclass, field
import math
import mmap
import os
import struct
import time
import uuid


MAGIC = 0x564E48574C444731
PAGE_MAGIC = 0x564E485750414731
ABI_VERSION = 1
ENDIAN_TAG = 0x0102030405060708
HEADER_BYTES = 192
CONTROL_BYTES = 192
BANK_BYTES = 128
SEGMENT_BYTES = 64
PAGE_BYTES = 192
BANK_COUNT = 2
INIT_READY = 1
CRC64_ECMA_POLYNOMIAL = 0x42F0E1EBA9EA3693
UINT64_MASK = (1 << 64) - 1
INVALID_BANK_INDEX = UINT64_MASK
HEADER_FORMAT = '<24Q'
CONTROL_FORMAT = '<24Q'
CONTROL_REQUEST_FORMAT = '<10Q'
BANK_FORMAT = '<16Q'
SEGMENT_FORMAT = '<8Q'
PAGE_FORMAT = '<24Q'
INIT_STATE_WORD = 17
WRITER_PID_WORD = 18
LAST_COMPLETED_WRITE_SEQ_WORD = 19
GLOBAL_ORACLE_FAULTS_WORD = 20
CONTROL_OFFSET = HEADER_BYTES
CONTROL_REQUEST_TICKET_WORD = 9
CONTROL_REQUEST_STATE_WORD = 10
CONTROL_RESPONSE_REQUEST_CHECKSUM_WORD = 15
CONTROL_RESPONSE_CHECKSUM_WORD = 16
CONTROL_RESPONSE_TICKET_WORD = 17
CONTROL_REQUEST_IDLE = 0
CONTROL_REQUEST_WRITING = 1
CONTROL_REQUEST_READY = 2
CONTROL_REQUEST_READING = 3
BANK_STATE_ACTIVE = 1
BANK_STATE_FREE = 0
BANK_STATE_SEALED_OK = 2
BANK_STATE_SEALED_FAULT = 3
CONTROL_OP_ARM = 1
CONTROL_OP_SEAL = 2
CONTROL_FLAG_ZERO_REQUIRED = 1
CONTROL_FLAG_EXACT_SEAL_STAMP = 2
CONTROL_RESPONSE_OK = 1
CONTROL_RESPONSE_INVALID = 2
FAULT_SEQUENCE = 1 << 0
FAULT_GENERATION = 1 << 1
FAULT_NONFINITE = 1 << 2
FAULT_SIM_STAMP = 1 << 3
FAULT_CAPACITY = 1 << 4
FAULT_ZERO_REQUIRED = 1 << 5
FAULT_OBSERVATION = 1 << 6
FAULT_PROTOCOL = 1 << 7
KNOWN_FAULTS = (
    FAULT_SEQUENCE |
    FAULT_GENERATION |
    FAULT_NONFINITE |
    FAULT_SIM_STAMP |
    FAULT_CAPACITY |
    FAULT_ZERO_REQUIRED |
    FAULT_OBSERVATION |
    FAULT_PROTOCOL
)
KNOWN_PREDICATE_FLAGS = (
    CONTROL_FLAG_ZERO_REQUIRED |
    CONTROL_FLAG_EXACT_SEAL_STAMP
)


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
            control[CONTROL_RESPONSE_REQUEST_CHECKSUM_WORD],
            response_ticket,
            *control[11:15],
        ),
    )


@dataclass(frozen=True)
class HardwareWriteLedgerSnapshotPage:
    """One immutable Parent copy of a bounded sealed evidence page."""

    page_magic: int
    abi_version: int
    page_bytes: int
    segment_bytes: int
    bank_index: int
    bank_epoch: int
    generation: int
    interval_id: int
    arm_fence_write_seq: int
    seal_fence_write_seq: int
    seal_not_before_sim_stamp_ns_bits: int
    predicate_flags: int
    page_index: int
    page_count: int
    total_segment_count: int
    total_invocation_count: int
    page_segment_count: int
    page_invocation_count: int
    page_first_write_seq: int
    page_last_write_seq: int
    previous_page_checksum: int
    oracle_faults: int
    bank_checksum: int
    page_checksum: int
    segments: tuple


@dataclass(frozen=True)
class SealedHardwareWriteLedgerInterval:
    """Complete validated evidence retained until one exact ACK."""

    generation: int
    interval_id: int
    bank_index: int
    bank_epoch: int
    terminal_state: int
    arm_fence_write_seq: int
    seal_fence_write_seq: int
    bank_checksum: int
    oracle_faults: int
    pages: tuple
    _owner_token: object = field(repr=False, compare=False)
    _ack_token: object = field(repr=False, compare=False)
    _bank_words: tuple = field(repr=False, compare=False)
    _segments: tuple = field(repr=False, compare=False)


class AtomicUint64:
    """ctypes binding for the GCC lock-free uint64 atomic ABI."""

    _ACQUIRE = 2
    _RELEASE = 3
    _ACQ_REL = 4

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
        self._compare_exchange = getattr(
            library,
            '__atomic_compare_exchange_8',
        )
        self._compare_exchange.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.c_bool,
            ctypes.c_int,
            ctypes.c_int,
        ]
        self._compare_exchange.restype = ctypes.c_bool
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

    def compare_exchange_acq_rel(self, region, offset, expected, desired):
        """Claim one exact word and return success plus observed value."""
        word = self._word(region, offset)
        observed = ctypes.c_uint64(expected)
        exchanged = self._compare_exchange(
            ctypes.byref(word),
            ctypes.byref(observed),
            desired,
            False,
            self._ACQ_REL,
            self._ACQUIRE,
        )
        return bool(exchanged), int(observed.value)


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
        self._evidence_owner_token = object()
        self._registered_snapshots = {}

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

    @staticmethod
    def _signed_int64(bits):
        """Interpret one ABI uint64 word as its signed int64 payload."""
        if bits & (1 << 63):
            return bits - (1 << 64)
        return bits

    @staticmethod
    def _finite_command_bits(bits):
        """Return whether exact IEEE-754 command bits encode a finite value."""
        value = struct.unpack('<d', struct.pack('<Q', bits))[0]
        return math.isfinite(value)

    @staticmethod
    def _zero_command_bits(bits):
        """Accept both signed IEEE-754 zero encodings."""
        return (bits & 0x7FFFFFFFFFFFFFFF) == 0

    @staticmethod
    def _require_uint64(value, field_name, nonzero=False):
        """Reject values that cannot be one exact ABI uint64 identity word."""
        if not isinstance(value, int) or value < 0 or value > UINT64_MASK:
            raise AssertionError(f'{field_name} is not one uint64 word')
        if nonzero and value == 0:
            raise AssertionError(f'{field_name} must be non-zero')

    def _validate_static_header(self):
        """Validate the complete immutable region identity and geometry."""
        if self.region is None:
            raise AssertionError('ledger region is closed')
        header = struct.unpack_from(HEADER_FORMAT, self.region, 0)
        expected_prefix = (
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
        )
        if header[0:17] != expected_prefix:
            raise AssertionError('ledger immutable header identity changed')
        if self.page_segment_limit <= 0:
            raise AssertionError('ledger page segment limit is zero')
        if self.page_segment_limit > self.segment_capacity:
            raise AssertionError('ledger page segment limit exceeds capacity')
        if self.segment_capacity <= 0:
            raise AssertionError('ledger segment capacity is zero')
        if (self.nonce_hi | self.nonce_lo) == 0:
            raise AssertionError('ledger nonce is zero')
        if header[21:23] != (0, 0):
            raise AssertionError('ledger header reserved words changed')
        if header[23] != header_checksum(header):
            raise AssertionError('ledger header checksum mismatch')
        if self.atomic.load_acquire(
            self.region,
            INIT_STATE_WORD * 8,
        ) != INIT_READY:
            raise AssertionError('ledger region is not READY')

    def _validate_terminal_bank(self, bank_index, terminal_state, bank):
        """Validate one copied sealed bank before any segment traversal."""
        if bank[0] != terminal_state:
            raise AssertionError('ledger bank state changed during copy')
        if bank[1] == 0 or bank[2] == 0:
            raise AssertionError('ledger sealed identity contains zero')
        if bank[3] == UINT64_MASK or bank[4] <= bank[3]:
            raise AssertionError('ledger seal fence does not follow ARM')
        if bank[5] == 0 or bank[5] > self.segment_capacity:
            raise AssertionError('ledger segment budget is outside capacity')
        if bank[6] == 0:
            raise AssertionError('ledger invocation budget is zero')
        if bank[7] & ~KNOWN_PREDICATE_FLAGS:
            raise AssertionError('ledger predicate flags are unknown')
        if self._signed_int64(bank[8]) < 0:
            raise AssertionError('ledger seal stamp is negative')
        if bank[9] > bank[5] or bank[9] > self.segment_capacity:
            raise AssertionError('ledger segment count exceeds capacity')
        if bank[10] == 0 or bank[10] > bank[6]:
            raise AssertionError('ledger invocation count exceeds its budget')
        if bank[11] != bank[3] + 1:
            raise AssertionError('ledger first write does not follow ARM')
        if bank[12] != bank[4]:
            raise AssertionError('ledger last write does not equal SEAL')
        if bank[10] != bank[4] - bank[3]:
            raise AssertionError('ledger attempt count does not cover fences')
        if bank[13] & ~KNOWN_FAULTS:
            raise AssertionError('ledger bank contains unknown fault bits')
        if terminal_state == BANK_STATE_SEALED_OK and bank[13] != 0:
            raise AssertionError('SEALED_OK bank contains sticky faults')
        if terminal_state == BANK_STATE_SEALED_FAULT and bank[13] == 0:
            raise AssertionError('SEALED_FAULT bank has no sticky fault')
        expected_page_count = max(
            1,
            (bank[9] + self.page_segment_limit - 1) //
            self.page_segment_limit,
        )
        if bank[14] != expected_page_count:
            raise AssertionError('ledger page count does not match geometry')
        if bank_index < 0 or bank_index >= BANK_COUNT:
            raise AssertionError('ledger bank index is outside the region')

    def _validate_segments(self, bank, segments):
        """Validate bounded stored tuples without inventing missing attempts."""
        recorded_invocations = 0
        previous_segment = None
        previous_stamp = None
        clean_bank = bank[13] == 0
        for segment_index, segment in enumerate(segments):
            if segment[0] != self.generation:
                raise AssertionError('ledger segment generation changed')
            if segment[3] == 0 or segment[2] < segment[1]:
                raise AssertionError('ledger segment range is empty')
            if segment[3] != segment[2] - segment[1] + 1:
                raise AssertionError('ledger segment count differs from range')
            if segment[1] < bank[11] or segment[2] > bank[12]:
                raise AssertionError('ledger segment escapes attempt range')
            if previous_segment is not None:
                if segment[1] <= previous_segment[2]:
                    raise AssertionError('ledger segments overlap or regress')
                if clean_bank and segment[1] != previous_segment[2] + 1:
                    raise AssertionError('clean ledger segment coverage has gap')
            elif clean_bank and segment[1] != bank[11]:
                raise AssertionError('clean ledger coverage starts after first')
            if recorded_invocations > bank[10] - segment[3]:
                raise AssertionError('ledger recorded count exceeds attempts')
            recorded_invocations += segment[3]

            stamp = self._signed_int64(segment[4])
            if (
                previous_stamp is not None and
                stamp < previous_stamp and
                not bank[13] & FAULT_SIM_STAMP
            ):
                raise AssertionError('ledger stamp regressed without fault')
            if segment[5] > 0xFF:
                raise AssertionError('ledger stored a non-VALID observation')
            if not self._finite_command_bits(segment[6]):
                raise AssertionError('ledger left command is non-finite')
            if not self._finite_command_bits(segment[7]):
                raise AssertionError('ledger right command is non-finite')
            if bank[7] & CONTROL_FLAG_ZERO_REQUIRED:
                if (
                    not self._zero_command_bits(segment[6]) or
                    not self._zero_command_bits(segment[7])
                ):
                    raise AssertionError('ledger stored a non-zero predicate')
            if (
                previous_segment is not None and
                segment[1] == previous_segment[2] + 1 and
                segment[4:8] == previous_segment[4:8]
            ):
                raise AssertionError('ledger retained an unfoldable tuple')
            previous_segment = segment
            previous_stamp = stamp

        if clean_bank:
            if not segments or segments[-1][2] != bank[12]:
                raise AssertionError('clean ledger coverage ends before SEAL')
            if recorded_invocations != bank[10]:
                raise AssertionError('clean ledger omitted invocation evidence')
            final_stamp = self._signed_int64(segments[-1][4])
            seal_stamp = self._signed_int64(bank[8])
            if final_stamp < seal_stamp:
                raise AssertionError('clean ledger sealed before threshold')
            if (
                bank[7] & CONTROL_FLAG_EXACT_SEAL_STAMP and
                final_stamp != seal_stamp
            ):
                raise AssertionError('clean exact seal stamp does not match')
        elif recorded_invocations > bank[10]:
            raise AssertionError('fault ledger records too many invocations')

    def _build_snapshot_pages(self, bank_index, bank, segments):
        """Build one local immutable CRC chain from exact sealed evidence."""
        pages = []
        previous_page_checksum = 0
        page_count = bank[14]
        for page_index in range(page_count):
            first_segment_index = page_index * self.page_segment_limit
            page_segments = segments[
                first_segment_index:
                first_segment_index + self.page_segment_limit
            ]
            if not segments or page_index == 0:
                page_first_write_seq = bank[11]
            else:
                page_first_write_seq = page_segments[0][1]
            if page_index + 1 < page_count:
                next_segment_index = (
                    first_segment_index + len(page_segments)
                )
                page_last_write_seq = (
                    segments[next_segment_index][1] - 1
                )
            else:
                page_last_write_seq = bank[12]
            if page_last_write_seq < page_first_write_seq:
                raise AssertionError('ledger page attempt range is empty')
            page_invocation_count = (
                page_last_write_seq - page_first_write_seq + 1
            )
            header_words = (
                PAGE_MAGIC,
                ABI_VERSION,
                PAGE_BYTES,
                SEGMENT_BYTES,
                bank_index,
                bank[1],
                self.generation,
                bank[2],
                bank[3],
                bank[4],
                bank[8],
                bank[7],
                page_index,
                page_count,
                bank[9],
                bank[10],
                len(page_segments),
                page_invocation_count,
                page_first_write_seq,
                page_last_write_seq,
                previous_page_checksum,
                bank[13],
                bank[15],
            )
            page_checksum = crc64_ecma_words(
                (
                    *header_words,
                    *(
                        word
                        for segment in page_segments
                        for word in segment
                    ),
                ),
            )
            page = HardwareWriteLedgerSnapshotPage(
                *header_words,
                page_checksum,
                tuple(page_segments),
            )
            pages.append(page)
            previous_page_checksum = page_checksum
        if sum(page.page_segment_count for page in pages) != bank[9]:
            raise AssertionError('ledger pages omit stored segments')
        if sum(page.page_invocation_count for page in pages) != bank[10]:
            raise AssertionError('ledger pages omit attempted invocations')
        return tuple(pages)

    def _copy_validated_terminal_evidence(
        self,
        interval_id,
        bank_index,
        bank_epoch,
        seal_fence_write_seq,
    ):
        """Copy, validate, and acquire-recheck one exact terminal bank."""
        self._require_uint64(interval_id, 'interval_id', nonzero=True)
        self._require_uint64(bank_index, 'bank_index')
        self._require_uint64(bank_epoch, 'bank_epoch', nonzero=True)
        self._require_uint64(
            seal_fence_write_seq,
            'seal_fence_write_seq',
            nonzero=True,
        )
        self._validate_static_header()
        bank_offset = self.bank_offset(bank_index)
        terminal_state = self.atomic.load_acquire(
            self.region,
            bank_offset,
        )
        if terminal_state not in (
            BANK_STATE_SEALED_OK,
            BANK_STATE_SEALED_FAULT,
        ):
            raise AssertionError('ledger bank is not terminal')
        bank = struct.unpack_from(BANK_FORMAT, self.region, bank_offset)
        self._validate_terminal_bank(bank_index, terminal_state, bank)
        if (
            bank[1] != bank_epoch or
            bank[2] != interval_id or
            bank[4] != seal_fence_write_seq
        ):
            raise AssertionError('ledger sealed identity does not match')
        segments = tuple(
            self.snapshot_segment(bank_index, segment_index)
            for segment_index in range(bank[9])
        )
        self._validate_segments(bank, segments)
        calculated_bank_checksum = crc64_ecma_words(
            (
                *bank[1:15],
                *(word for segment in segments for word in segment),
            ),
        )
        if bank[15] != calculated_bank_checksum:
            raise AssertionError('ledger bank checksum mismatch')
        pages = self._build_snapshot_pages(bank_index, bank, segments)

        rechecked_bank = struct.unpack_from(
            BANK_FORMAT,
            self.region,
            bank_offset,
        )
        rechecked_segments = tuple(
            self.snapshot_segment(bank_index, segment_index)
            for segment_index in range(bank[9])
        )
        rechecked_state = self.atomic.load_acquire(
            self.region,
            bank_offset,
        )
        if (
            rechecked_state != terminal_state or
            rechecked_bank != bank or
            rechecked_segments != segments
        ):
            raise AssertionError('ledger evidence changed during Parent copy')
        return terminal_state, bank, segments, pages

    def read_sealed_interval(
        self,
        interval_id,
        bank_index,
        bank_epoch,
        seal_fence_write_seq,
    ):
        """Return a complete immutable snapshot without releasing its bank."""
        terminal_state, bank, segments, pages = (
            self._copy_validated_terminal_evidence(
                interval_id,
                bank_index,
                bank_epoch,
                seal_fence_write_seq,
            )
        )
        ack_token = object()
        snapshot = SealedHardwareWriteLedgerInterval(
            generation=self.generation,
            interval_id=interval_id,
            bank_index=bank_index,
            bank_epoch=bank_epoch,
            terminal_state=terminal_state,
            arm_fence_write_seq=bank[3],
            seal_fence_write_seq=seal_fence_write_seq,
            bank_checksum=bank[15],
            oracle_faults=bank[13],
            pages=pages,
            _owner_token=self._evidence_owner_token,
            _ack_token=ack_token,
            _bank_words=bank,
            _segments=segments,
        )
        self._registered_snapshots[bank_index] = snapshot
        return snapshot

    def acknowledge(self, snapshot):
        """Release one exact fully validated snapshot with a single CAS."""
        if not isinstance(snapshot, SealedHardwareWriteLedgerInterval):
            return False
        if snapshot._owner_token is not self._evidence_owner_token:
            return False
        if self._registered_snapshots.get(snapshot.bank_index) is not snapshot:
            return False
        try:
            terminal_state, bank, segments, pages = (
                self._copy_validated_terminal_evidence(
                    snapshot.interval_id,
                    snapshot.bank_index,
                    snapshot.bank_epoch,
                    snapshot.seal_fence_write_seq,
                )
            )
        except (AssertionError, IndexError, struct.error):
            self._registered_snapshots.pop(snapshot.bank_index, None)
            return False
        if (
            snapshot.generation != self.generation or
            snapshot.terminal_state != terminal_state or
            snapshot.arm_fence_write_seq != bank[3] or
            snapshot.bank_checksum != bank[15] or
            snapshot.oracle_faults != bank[13] or
            snapshot.pages != pages or
            snapshot._bank_words != bank or
            snapshot._segments != segments
        ):
            self._registered_snapshots.pop(snapshot.bank_index, None)
            return False
        self._registered_snapshots.pop(snapshot.bank_index, None)
        exchanged, _observed = self.atomic.compare_exchange_acq_rel(
            self.region,
            self.bank_offset(snapshot.bank_index),
            terminal_state,
            BANK_STATE_FREE,
        )
        return exchanged

    def load_header_word(self, index):
        """Acquire-load one dynamic header word."""
        return self.atomic.load_acquire(self.region, index * 8)

    def corrupt_header_checksum(self):
        """Flip only the immutable header checksum for one mutation test."""
        header = list(struct.unpack_from(HEADER_FORMAT, self.region, 0))
        header[23] ^= 1
        struct.pack_into(HEADER_FORMAT, self.region, 0, *header)

    def force_last_completed_write_seq(self, write_seq):
        """Release-store a sequence boundary for exhaustion fault injection."""
        self.atomic.store_release(
            self.region,
            LAST_COMPLETED_WRITE_SEQ_WORD * 8,
            write_seq,
        )

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

    def _claim_request_mailbox(self):
        state_offset = (
            CONTROL_OFFSET + CONTROL_REQUEST_STATE_WORD * 8
        )
        claimed, observed = self.atomic.compare_exchange_acq_rel(
            self.region,
            state_offset,
            CONTROL_REQUEST_IDLE,
            CONTROL_REQUEST_WRITING,
        )
        if not claimed:
            raise AssertionError(
                f'ledger request mailbox is owned in state {observed}',
            )

    def _release_request_mailbox(self, state):
        self.atomic.store_release(
            self.region,
            CONTROL_OFFSET + CONTROL_REQUEST_STATE_WORD * 8,
            state,
        )

    def _write_owned_request(self, request):
        struct.pack_into(
            CONTROL_REQUEST_FORMAT,
            self.region,
            CONTROL_OFFSET,
            *request,
        )

    def _publish_request(self, request):
        self._write_owned_request(request)
        self._release_request_mailbox(CONTROL_REQUEST_READY)

    def _begin_request_preparation(self, request_fields):
        """Own and populate one request without publishing it to Writer."""
        self._claim_request_mailbox()
        try:
            control = struct.unpack_from(
                CONTROL_FORMAT,
                self.region,
                CONTROL_OFFSET,
            )
            response_ticket = control[CONTROL_RESPONSE_TICKET_WORD]
            if response_ticket == UINT64_MASK:
                raise OverflowError('ledger request ticket is exhausted')
            if control[CONTROL_REQUEST_TICKET_WORD] != response_ticket:
                raise AssertionError(
                    'ledger mailbox tickets differ while IDLE',
                )
            request_ticket = response_ticket + 1
            request = [*request_fields, 0, request_ticket]
            request[8] = control_request_checksum(
                self,
                request,
                request_ticket,
            )
            self._write_owned_request(request)
            return request_ticket
        except BaseException:
            self._release_request_mailbox(CONTROL_REQUEST_IDLE)
            raise

    def begin_arm_preparation(
        self,
        interval_id,
        segment_budget,
        invocation_budget,
        require_zero_commands,
    ):
        """Write one ARM request while retaining Parent mailbox ownership."""
        flags = CONTROL_FLAG_ZERO_REQUIRED if require_zero_commands else 0
        return self._begin_request_preparation(
            (
                CONTROL_OP_ARM,
                flags,
                interval_id,
                0,
                0,
                segment_budget,
                invocation_budget,
                0,
            ),
        )

    def commit_prepared_request(self, request_ticket):
        """Release-publish the exact request currently owned by Parent."""
        control = struct.unpack_from(
            CONTROL_FORMAT,
            self.region,
            CONTROL_OFFSET,
        )
        request_state = self.atomic.load_acquire(
            self.region,
            CONTROL_OFFSET + CONTROL_REQUEST_STATE_WORD * 8,
        )
        if (
            request_state != CONTROL_REQUEST_WRITING or
            control[CONTROL_REQUEST_TICKET_WORD] != request_ticket
        ):
            raise AssertionError('no matching prepared ledger request')
        self._release_request_mailbox(CONTROL_REQUEST_READY)

    def post_arm(
        self,
        interval_id,
        segment_budget,
        invocation_budget,
        require_zero_commands,
    ):
        """Prepare and release-publish one checksummed ARM request."""
        request_ticket = self.begin_arm_preparation(
            interval_id,
            segment_budget,
            invocation_budget,
            require_zero_commands,
        )
        self.commit_prepared_request(request_ticket)
        return request_ticket

    def post_seal(
        self,
        interval_id,
        bank_index,
        bank_epoch,
        not_before_sim_stamp_ns,
        require_exact_stamp,
    ):
        """Prepare and release-publish one checksummed SEAL request."""
        flags = (
            CONTROL_FLAG_EXACT_SEAL_STAMP if require_exact_stamp else 0
        )
        return self.post_seal_fields(
            flags=flags,
            interval_id=interval_id,
            bank_index=bank_index,
            bank_epoch=bank_epoch,
            segment_budget=0,
            invocation_budget=0,
            not_before_sim_stamp_ns=not_before_sim_stamp_ns,
        )

    def post_seal_fields(
        self,
        flags,
        interval_id,
        bank_index,
        bank_epoch,
        segment_budget,
        invocation_budget,
        not_before_sim_stamp_ns,
        corrupt_checksum=False,
    ):
        """Publish one owned SEAL fixture with explicit ABI fields."""
        request_ticket = self._begin_request_preparation(
            (
                CONTROL_OP_SEAL,
                flags,
                interval_id,
                bank_index,
                bank_epoch,
                segment_budget,
                invocation_budget,
                int(not_before_sim_stamp_ns) & UINT64_MASK,
            ),
        )
        if corrupt_checksum:
            request = list(
                struct.unpack_from(
                    CONTROL_REQUEST_FORMAT,
                    self.region,
                    CONTROL_OFFSET,
                ),
            )
            request[8] ^= 1
            self._write_owned_request(request)
        self.commit_prepared_request(request_ticket)
        return request_ticket

    def post_seal_with_segment_budget(
        self,
        interval_id,
        bank_index,
        bank_epoch,
        not_before_sim_stamp_ns,
        segment_budget,
    ):
        """Publish one checksummed SEAL with an invalid unused budget."""
        return self.post_seal_fields(
            flags=CONTROL_FLAG_EXACT_SEAL_STAMP,
            interval_id=interval_id,
            bank_index=bank_index,
            bank_epoch=bank_epoch,
            segment_budget=segment_budget,
            invocation_budget=0,
            not_before_sim_stamp_ns=not_before_sim_stamp_ns,
        )

    def post_seal_with_corrupt_checksum(
        self,
        interval_id,
        bank_index,
        bank_epoch,
        not_before_sim_stamp_ns,
    ):
        """Publish one otherwise valid SEAL with a corrupted checksum."""
        return self.post_seal_fields(
            flags=CONTROL_FLAG_EXACT_SEAL_STAMP,
            interval_id=interval_id,
            bank_index=bank_index,
            bank_epoch=bank_epoch,
            segment_budget=0,
            invocation_budget=0,
            not_before_sim_stamp_ns=not_before_sim_stamp_ns,
            corrupt_checksum=True,
        )

    def replay_arm_with_interval(self, request_ticket, interval_id):
        """Release-republish one ticket with a different valid payload."""
        self._claim_request_mailbox()
        try:
            control = list(
                struct.unpack_from(
                    CONTROL_FORMAT,
                    self.region,
                    CONTROL_OFFSET,
                ),
            )
            response_ticket = control[CONTROL_RESPONSE_TICKET_WORD]
            published_ticket = control[CONTROL_REQUEST_TICKET_WORD]
            if (
                response_ticket != request_ticket or
                published_ticket != request_ticket
            ):
                raise AssertionError(
                    'replay requires one completed request ticket',
                )
            control[2] = interval_id
            control[8] = control_request_checksum(
                self,
                control,
                request_ticket,
            )
            self._publish_request(control[0:10])
        except BaseException:
            self._release_request_mailbox(CONTROL_REQUEST_IDLE)
            raise

    def force_wrapped_request_after_exhausted_response(self):
        """Inject response MAX followed by wrapped request zero."""
        self._claim_request_mailbox()
        try:
            control = list(
                struct.unpack_from(
                    CONTROL_FORMAT,
                    self.region,
                    CONTROL_OFFSET,
                ),
            )
            control[2] += 1
            control[CONTROL_REQUEST_TICKET_WORD] = 0
            control[8] = control_request_checksum(self, control, 0)
            self.atomic.store_release(
                self.region,
                CONTROL_OFFSET + CONTROL_RESPONSE_TICKET_WORD * 8,
                UINT64_MASK,
            )
            self._publish_request(control[0:10])
        except BaseException:
            self._release_request_mailbox(CONTROL_REQUEST_IDLE)
            raise

    def load_response_ticket(self):
        """Acquire-load the monotonic Writer response publication."""
        return self.atomic.load_acquire(
            self.region,
            CONTROL_OFFSET + CONTROL_RESPONSE_TICKET_WORD * 8,
        )

    def load_request_state(self):
        """Acquire-load current request-envelope ownership."""
        return self.atomic.load_acquire(
            self.region,
            CONTROL_OFFSET + CONTROL_REQUEST_STATE_WORD * 8,
        )

    def force_bank_segment_count(self, bank_index, segment_count):
        """Inject one impossible segment count for a bounded fault test."""
        self.atomic.store_release(
            self.region,
            self.bank_offset(bank_index) + 9 * 8,
            segment_count,
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
                if (
                    control[CONTROL_REQUEST_TICKET_WORD] != request_ticket or
                    control[CONTROL_RESPONSE_REQUEST_CHECKSUM_WORD] !=
                    control[8]
                ):
                    raise AssertionError(
                        'ledger response does not bind its exact request',
                    )
                if control[CONTROL_RESPONSE_CHECKSUM_WORD] != (
                    control_response_checksum(
                        self,
                        control,
                        response_ticket,
                    )
                ):
                    raise AssertionError('ledger response checksum mismatch')
                return (
                    *control[0:9],
                    *control[11:15],
                    control[CONTROL_RESPONSE_CHECKSUM_WORD],
                    response_ticket,
                    control[CONTROL_REQUEST_TICKET_WORD],
                )
            if response_ticket > request_ticket:
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
        self._registered_snapshots.clear()
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
        """Return this live owner to a context manager."""
        return self

    def __exit__(self, _exception_type, _exception, _traceback):
        """Release the exact mapping and POSIX object owned here."""
        self.cleanup()
