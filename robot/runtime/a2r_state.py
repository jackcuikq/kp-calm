import json
from pathlib import Path
STATE = Path("/run/a2r-security-demo/state.json")

def read_state(path=None):
    path = STATE if path is None else path
    try:
        data = json.loads(path.read_text())
        return dict(active=True, locked=bool(data.get("locked")),
                    message=str(data.get("message", "HACKED"))[:160])
    except FileNotFoundError:
        return dict(active=False, locked=False, message="")
    except (OSError, ValueError, TypeError, AttributeError):
        return dict(active=True, locked=True, message="HACKED")
