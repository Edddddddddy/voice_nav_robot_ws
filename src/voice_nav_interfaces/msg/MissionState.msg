uint8 MAPPING=1
uint8 NAVIGATION=2

uint8 UNAVAILABLE=0
uint8 AVAILABLE=1
uint8 BUSY=2
uint8 FAULTED=3

uint8 GATE_INHIBITED=0
uint8 GATE_ARMED=1
uint8 GATE_FAULTED=2

string<=36 runtime_instance_id
uint64 admission_epoch
uint8 operating_mode
uint8 availability
uint8 gate_state
uint32 active_step
uint32 supported_step_mask
uint8 max_steps
string<=64[<=32] named_place_ids
