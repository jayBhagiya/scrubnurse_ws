import pytest
from scrubnurse_voice.tool_catalog import DEFAULT_TOOL_IDS, DEFAULT_TOOL_NAMES
from scrubnurse_voice.tool_catalog import has_wake_word, normalize, ToolCatalog


@pytest.fixture
def catalog():
    return ToolCatalog(DEFAULT_TOOL_IDS, DEFAULT_TOOL_NAMES)


@pytest.mark.parametrize('text, expected', [
    (' Scalpel.', 'D'),
    ('Needle holder!', 'B'),
    ('POINTED   scissors', 'A'),
    ('Surgical forceps.', 'F'),
    ('anatomical forceps', 'E'),
    ('Mosquito', 'C'),
])
def test_exact_names(catalog, text, expected):
    assert catalog.match(text) == expected


@pytest.mark.parametrize('text, expected', [
    ('Can I have the scalpel please', 'D'),
    ('give me the surgical forceps', 'F'),
])
def test_name_inside_sentence(catalog, text, expected):
    assert catalog.match(text) == expected


@pytest.mark.parametrize('text, expected', [
    ('Needle holders.', 'B'),
    ('pointed scisors', 'A'),
    ('mosquitto', 'C'),
    ('anatomic forceps', 'E'),
    ('let me the analytic forceps', 'E'),  # heard on the real headset
    ('give me the scalp all', 'D'),
])
def test_close_spellings(catalog, text, expected):
    assert catalog.match(text) == expected


@pytest.mark.parametrize('text', ['', '   ', 'Caesar', 'forceps', 'hello robot'])
def test_no_match(catalog, text):
    assert catalog.match(text) is None


def test_normalize():
    assert normalize('  Needle-holder,  please! ') == 'needleholder please'


def test_mismatched_lists_rejected():
    with pytest.raises(ValueError):
        ToolCatalog(['A'], ['scalpel', 'mosquito'])


WAKE = ['robot', 'scrub nurse', 'scrubnurse']


# Mishearings from the Jabra headset on the real cell.
@pytest.mark.parametrize('text', ['Robot, scalpel.', 'robert scalpel',
                                  'scrub nurse get me scalpel',
                                  'Grub Nuts get me anatomical forceps',
                                  'Hey scrub nerds can you get me mosquito clamp?',
                                  'Scrubnurse, scalpel'])
def test_wake_word_heard(text):
    assert has_wake_word(text, WAKE, 0.65)


@pytest.mark.parametrize('text', ['', 'get me the scalpel', 'we will not need the needle holder',
                                  'about the scalpel', 'can you get me surgical forceps'])
def test_no_wake_word(text):
    assert not has_wake_word(text, WAKE, 0.65)
