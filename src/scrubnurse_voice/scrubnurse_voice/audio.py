"""Microphone recording and audio file loading at Whisper's sample rate."""

from collections import deque
from math import gcd
import queue

import numpy as np

WHISPER_RATE = 16000
BLOCK_SECONDS = 0.03


def to_whisper_input(samples, rate):
    """Return mono float32 audio resampled to 16 kHz."""
    from scipy.signal import resample_poly

    audio = np.asarray(samples, dtype=np.float32)
    if audio.ndim == 2:
        audio = audio.mean(axis=1)
    if rate != WHISPER_RATE:
        g = gcd(int(rate), WHISPER_RATE)
        audio = resample_poly(audio, WHISPER_RATE // g, int(rate) // g).astype(np.float32)
    return audio


def load_audio_file(path):
    """Load a wav/flac/mp3 file as Whisper input."""
    import soundfile

    samples, rate = soundfile.read(path, dtype='float32', always_2d=True)
    return to_whisper_input(samples, rate)


def hardware_inputs(devices):
    """Names of the microphones among sounddevice.query_devices() entries (no ALSA plugins)."""
    return [d['name'] for d in devices if d['max_input_channels'] > 0 and '(hw:' in d['name']]


def input_devices():
    """Microphones on this machine, e.g. 'Jabra Link 370: USB Audio (hw:2,0)'."""
    import sounddevice

    return hardware_inputs(sounddevice.query_devices())


def record(seconds, device=None):
    """Record `seconds` of audio from the microphone as Whisper input."""
    import sounddevice

    samples = sounddevice.rec(int(seconds * WHISPER_RATE), samplerate=WHISPER_RATE,
                              channels=1, dtype='float32', device=device)
    sounddevice.wait()
    return samples[:, 0]


def split_phrases(blocks, level, end_silence_blocks, max_blocks, pre_roll_blocks=10):
    """
    Group audio blocks into phrases and yield each as one array.

    A phrase starts at the first block whose RMS exceeds `level` (with a few
    quiet blocks before it, so the first syllable is not cut) and ends after
    `end_silence_blocks` quiet blocks or at `max_blocks`.
    """
    before = deque(maxlen=pre_roll_blocks)
    phrase, quiet = [], 0
    for block in blocks:
        loud = np.sqrt(np.mean(np.square(block))) > level
        if not phrase:
            if loud:
                phrase, quiet = [*before, block], 0
            else:
                before.append(block)
            continue
        phrase.append(block)
        quiet = 0 if loud else quiet + 1
        if quiet >= end_silence_blocks or len(phrase) >= max_blocks:
            yield np.concatenate(phrase)
            phrase = []
            before.clear()


def phrases(running, level, end_silence=0.8, max_seconds=8.0, device=None):
    """Yield each spoken phrase from the microphone as Whisper input while `running()`."""
    import sounddevice

    blocks = queue.Queue()

    def mic_blocks():
        while running():
            try:
                yield blocks.get(timeout=0.5)
            except queue.Empty:
                pass

    with sounddevice.InputStream(samplerate=WHISPER_RATE, channels=1, dtype='float32',
                                 blocksize=int(BLOCK_SECONDS * WHISPER_RATE), device=device,
                                 callback=lambda data, *_: blocks.put(data[:, 0].copy())):
        yield from split_phrases(mic_blocks(), level, round(end_silence / BLOCK_SECONDS),
                                 round(max_seconds / BLOCK_SECONDS))
