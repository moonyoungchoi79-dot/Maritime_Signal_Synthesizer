import re

def sanitize_filename(name: str) -> str:
    s = re.sub(r'[\\/*?:"<>|]', "", name)
    s = s.replace(" ", "_")
    return s if s else "Unnamed"
