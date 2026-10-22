"""
End-to-end check of Whisper plus name matching on recorded tool names.

Needs the workspace venv (openai-whisper, soundfile) and downloads the model on
first use, so it only runs when SCRUBNURSE_WHISPER_TESTS=1. The model defaults
to medium.en (as on the robot); override with SCRUBNURSE_WHISPER_MODEL.
"""

import glob
import os

import pytest
from scrubnurse_voice import audio
from scrubnurse_voice.tool_catalog import DEFAULT_TOOL_IDS, DEFAULT_TOOL_NAMES, ToolCatalog

SAMPLES = sorted(glob.glob(os.path.join(os.path.dirname(__file__), 'data', 'tool_names', '*')))

pytestmark = pytest.mark.skipif(os.environ.get('SCRUBNURSE_WHISPER_TESTS') != '1',
                                reason='set SCRUBNURSE_WHISPER_TESTS=1 to run Whisper tests')


@pytest.fixture(scope='module')
def model():
    import torch
    import whisper

    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    return whisper.load_model(os.environ.get('SCRUBNURSE_WHISPER_MODEL', 'medium.en'),
                              device=device), device != 'cpu'


@pytest.mark.parametrize('path', SAMPLES, ids=os.path.basename)
def test_recording_maps_to_tool(model, path):
    whisper_model, fp16 = model
    text = whisper_model.transcribe(audio.load_audio_file(path), language='en', fp16=fp16)['text']
    expected = os.path.basename(path).split('_')[0]
    assert ToolCatalog(DEFAULT_TOOL_IDS, DEFAULT_TOOL_NAMES).match(text) == expected, text
