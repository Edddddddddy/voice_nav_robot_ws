"""Small pure helpers used by the local rapid voice endpoint."""

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
        text = text.strip()
        index = text.find(self.wake_word)
        if index >= 0:
            self.deadline = now + self.timeout_s
            command = text[index + len(self.wake_word):].lstrip(' ,\uFF0C\u3002')
            return command or None
        if now <= self.deadline:
            self.deadline = 0.0
            return text or None
        return None
