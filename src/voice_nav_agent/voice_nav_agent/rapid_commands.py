"""Small pure helpers used by the local rapid voice endpoint."""

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
