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
#
# Package-private, transient-local MotionGate state snapshot.

uint8 INHIBITED=0
uint8 PREPARED=1
uint8 ARMED=2
uint8 FAULTED=3

uint16 NONE=0
uint16 INVALID_REQUEST=1
uint16 STALE_GATE=2
uint16 STALE_SEQUENCE=3
uint16 INVALID_STATE=4
uint16 STALE_LEASE=5
uint16 REQUEST_ID_COLLISION=6
uint16 PREPARE_EXPIRED=7
uint16 AUTHORITY_EXPIRED=8
uint16 CANDIDATE_EXPIRED=9
uint16 WRITER_UNAVAILABLE=10
uint16 WRITER_AMBIGUOUS=11
uint16 WRITER_MISMATCH=12
uint16 WRITER_STILL_PRESENT=13
uint16 INVALID_CANDIDATE=14
uint16 SEQUENCE_EXHAUSTED=15
uint16 CONFIGURATION_INVALID=16
uint16 PUBLISH_FAILED=17
uint16 INTERNAL_FAILURE=18

string<=36 gate_instance_id
uint64 state_seq
uint64 control_seq
uint8 state
string<=36 lease_id
string<=128 candidate_topic
uint8[16] bound_writer_gid
bool motion_inhibited
bool authority_live
bool candidate_fresh
bool writer_bound
bool zero_selected
uint64 output_publish_seq
uint64 zero_publish_seq
uint16 reason
string<=160 detail
