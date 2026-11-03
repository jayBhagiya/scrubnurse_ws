import numpy as np
from scrubnurse_voice.audio import hardware_inputs, split_phrases

QUIET, LOUD = np.zeros(4), np.full(4, 0.5)


def phrases(pattern):
    """Split a string of q(uiet)/L(oud) blocks; return each phrase as a q/L string."""
    blocks = [LOUD if c == 'L' else QUIET for c in pattern]
    found = split_phrases(blocks, level=0.1, end_silence_blocks=3, max_blocks=10,
                          pre_roll_blocks=2)
    return [''.join('L' if b > 0.1 else 'q' for b in p[::4]) for p in found]


def test_phrase_has_pre_roll_and_ends_after_silence():
    assert phrases('qqqqLLqLqqqqqLqqq') == ['qqLLqLqqq', 'qqLqqq']


def test_long_speech_is_cut_at_max_length():
    assert phrases('LLLLLLLLLLLLqqq') == ['LLLLLLLLLL', 'LLqqq']


def test_unfinished_phrase_is_not_yielded():
    assert phrases('qqLL') == []


def test_hardware_inputs_skip_outputs_and_alsa_plugins():
    devices = [
        {'name': 'Jabra Link 370: USB Audio (hw:2,0)', 'max_input_channels': 1},
        {'name': 'HDA Intel PCH: HDMI 0 (hw:0,3)', 'max_input_channels': 0},
        {'name': 'pulse', 'max_input_channels': 32},
        {'name': 'default', 'max_input_channels': 32},
    ]
    assert hardware_inputs(devices) == ['Jabra Link 370: USB Audio (hw:2,0)']
