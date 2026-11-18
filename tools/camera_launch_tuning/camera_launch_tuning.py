#!/usr/bin/env python3
"""Entry point: python camera_launch_tuning.py --help"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from clt.cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
