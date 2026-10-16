# speech_node -> latched interaction/speech_status, on every change.

std_msgs/Header header
uint32 id                 # utterance this status is about (from Say), 0: none yet
string state              # idle, speaking, done, failed, cancelled
string text
string language
uint32 queued             # utterances waiting after this one
string engine             # piper, espeak, none
string message
