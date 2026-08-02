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
INIT_STATE_WORD = 17
WRITER_PID_WORD = 18


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

    def unlink_name(self):
        """Retire the POSIX name while retaining this mapping."""
        if self.linked:
            self.api.unlink(self.name)
            self.linked = False

    def cleanup(self):
        """Release only resources owned by this test object."""
        if self.region is not None:
            self.region.close()
            self.region = None
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1
        if self.linked:
            try:
                self.api.unlink(self.name)
            except FileNotFoundError:
                pass
            self.linked = False

    def __enter__(self):
        return self

    def __exit__(self, _exception_type, _exception, _traceback):
        self.cleanup()
