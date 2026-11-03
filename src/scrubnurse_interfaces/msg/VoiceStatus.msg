# State of the voice node, published (latched) on every change.

string STATE_LISTENING=listening    # live: waiting for a phrase with a wake word
string STATE_MUTED=muted            # live, but phrases are ignored
string STATE_COMMAND=command        # push-to-talk: the next phrase needs no wake word
string STATE_PROCESSING=processing  # transcribing a phrase
string STATE_IDLE=idle              # not live: waiting for ~/listen
string STATE_ERROR=error            # microphone failed; see message

builtin_interfaces/Time stamp
string state
string message        # error text, empty otherwise
bool live             # always listening (wake word) or only on ~/listen
bool muted
string input_device   # the microphone in use; empty: the system default
string[] devices      # microphones on this machine, for choosing one (parameter input_device)
string[] wake_words
