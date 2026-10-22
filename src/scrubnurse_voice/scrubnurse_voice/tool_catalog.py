"""Map spoken tool names to tool ids."""

from difflib import SequenceMatcher
import re

DEFAULT_TOOL_IDS = ['A', 'B', 'C', 'D', 'E', 'F']
DEFAULT_TOOL_NAMES = ['pointed scissors', 'needle holder', 'mosquito', 'scalpel',
                      'anatomical forceps', 'surgical forceps']


def normalize(text):
    """Lower-case `text`, keep only letters, digits and single spaces."""
    return ' '.join(re.sub(r'[^a-zA-Z0-9\s]', '', text).lower().split())


class ToolCatalog:
    """Spoken names of the tools the robot can hand over, keyed by tool id."""

    def __init__(self, ids, names, threshold=0.75):
        if len(ids) != len(names):
            raise ValueError('tool_ids and tool_names must have the same length')
        self._ids_by_name = {normalize(name): tool_id for tool_id, name in zip(ids, names)}
        self._threshold = threshold

    def match(self, text):
        """
        Return the tool id for a transcript, or None.

        Tries, in order: the exact name, a name contained in the transcript
        ("the scalpel please"), and the most similar name above the similarity
        threshold, which absorbs Whisper spelling slips ("needle holders").
        """
        spoken = normalize(text)
        if not spoken:
            return None
        if spoken in self._ids_by_name:
            return self._ids_by_name[spoken]

        contained = [n for n in self._ids_by_name if re.search(rf'\b{re.escape(n)}\b', spoken)]
        if contained:
            return self._ids_by_name[max(contained, key=len)]

        def similarity(name):
            return SequenceMatcher(None, spoken, name).ratio()

        best = max(self._ids_by_name, key=similarity)
        return self._ids_by_name[best] if similarity(best) > self._threshold else None
