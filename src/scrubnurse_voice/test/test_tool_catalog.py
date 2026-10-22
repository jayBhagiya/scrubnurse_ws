import pytest
from scrubnurse_voice.tool_catalog import DEFAULT_TOOL_IDS, DEFAULT_TOOL_NAMES
from scrubnurse_voice.tool_catalog import normalize, ToolCatalog


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
