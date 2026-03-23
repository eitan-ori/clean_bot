from __future__ import annotations

import sys
from pathlib import Path


def _prepend_sys_path(path: Path) -> None:
    p = str(path)
    if p in sys.path:
        sys.path.remove(p)
    sys.path.insert(0, p)


# Force tests to import from workspace source, not stale build artifacts.
_repo_root = Path(__file__).resolve().parents[3]
_prepend_sys_path(_repo_root / 'src' / 'clean_bot_mission')
