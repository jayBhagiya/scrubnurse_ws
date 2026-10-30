import numpy as np
from scrubnurse_voice.audio import split_phrases

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
