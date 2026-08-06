uint8 COMMAND=1
uint8 STOP=2

string<=36 voice_instance_id
uint64 voice_seq
string<=36 session_id
string<=36 turn_id
uint8 kind
string<=512 text
float32 confidence
bool during_playback
