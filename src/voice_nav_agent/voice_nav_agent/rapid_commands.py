"""Small pure helpers used by the local rapid voice endpoint."""

import re
from dataclasses import dataclass

PLACE_ALIASES = {
    '\u53a8\u623f': 'kitchen',
    '\u4e66\u623f': 'study',
    '\u5bb6': 'home',
}


def normalize_rapid_command(text):
    """Translate the three demo place names to the Agent's stable IDs."""
    if text.strip() in ('\u56de\u5bb6', '\u56de\u5230\u5bb6'):
        return '\u53bbhome'
    for spoken_name, place_id in PLACE_ALIASES.items():
        text = text.replace(spoken_name, place_id)
    return text


@dataclass
class WakeGate:
    """Accept one command after a wake phrase without audio-thread coupling."""

    wake_word: str = '\u5c0f\u667a'
    timeout_s: float = 8.0
    deadline: float = 0.0

    def accept(self, text, now):
        """Return an authorized command or retain one short wake window."""
        text = text.strip()
        pattern = re.escape(self.wake_word)
        if self.wake_word == '\u5c0f\u667a':
            pattern = r'(?:\u5c0f\s*[\u667a\u5fd7]|\u6653\s*\u667a)'
        match = re.search(pattern, text)
        if match is not None:
            self.deadline = now + self.timeout_s
            command = text[match.end():].lstrip(' ,\uFF0C\u3002')
            return command or None
        if now <= self.deadline:
            self.deadline = 0.0
            return text or None
        return None
