"""Convenient entry point for the standalone English PyQt console."""

from __future__ import annotations

import sys
from pathlib import Path


RUNTIME_ROOT = Path(__file__).resolve().parent.parent
if str(RUNTIME_ROOT) not in sys.path:
    sys.path.insert(0, str(RUNTIME_ROOT))

from Bronchialtree_identification.recognition_gui_en.launcher import main


if __name__ == "__main__":
    main()
