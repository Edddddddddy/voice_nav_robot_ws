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

"""Reusable parent-side support for Gate event journal process tests."""

import ctypes
import ctypes.util
import errno
import mmap
import os
import struct
import uuid


MAGIC = 0x564E474154454A31
ABI_VERSION = 1
HEADER_BYTES = 128
SLOT_BYTES = 256
INIT_READY = 1
CRC64_ECMA_POLYNOMIAL = 0x42F0E1EBA9EA3693
UINT64_MASK = (1 << 64) - 1
HEADER_FORMAT = '<16Q'
SLOT_FORMAT = '<32Q'


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
        """Bind the GCC uint64 atomic operations used by the journal ABI."""
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
        """Bind the libc POSIX shared-memory operations."""
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
        """Create and initialize one parent-owned journal generation."""
        self.capacity = capacity
        self.generation = generation
        self.owner_uid = os.geteuid()
        self.nonce = uuid.uuid4().hex
        self.nonce_hi = int(self.nonce[:16], 16)
        self.nonce_lo = int(self.nonce[16:], 16)
        self.name = f'/voice_nav_gate_{uuid.uuid4().hex}'
        self.descriptor = (
            f'v1:{self.owner_uid}:{self.generation}:'
            f'{self.capacity}:{self.nonce}'
        )
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
        """Return the initialized owner for a managed test scope."""
        return self

    def __exit__(self, exception_type, exception, traceback):
        """Release the journal name, mapping, and descriptor."""
        del exception_type, exception, traceback
        self.cleanup()
