# speech_node -> interaction/speech_status (latched), on every change.

uint8 IDLE = 0
uint8 SPEAKING = 1
uint8 DONE = 2
uint8 FAILED = 3
uint8 CANCELLED = 4

std_msgs/Header header
uint32 id                 # utterance this status is about (from Say), 0: none yet
uint8 state
string text
string language
uint32 queued             # utterances waiting after this one
string engine             # piper, espeak, none
string message
