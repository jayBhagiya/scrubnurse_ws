# What the voice node heard or did.

string KIND_READY=ready      # microphone open, listening
string KIND_HEARD=heard      # a phrase was transcribed
string KIND_ERROR=error      # microphone or speech recognition failed

builtin_interfaces/Time stamp
string kind
string text          # transcript (heard) or error message (error)
bool wake_word       # the phrase contained a wake word (always true outside live mode)
string tool_id       # requested tool, empty if none was matched or the phrase was ignored
string tool_name
