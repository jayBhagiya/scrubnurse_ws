"""Microphone recording and audio file loading at Whisper's sample rate."""

from math import gcd

import numpy as np

WHISPER_RATE = 16000


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


def record(seconds, device=None):
    """Record `seconds` of audio from the microphone as Whisper input."""
    import sounddevice

    samples = sounddevice.rec(int(seconds * WHISPER_RATE), samplerate=WHISPER_RATE,
                              channels=1, dtype='float32', device=device)
    sounddevice.wait()
    return samples[:, 0]
