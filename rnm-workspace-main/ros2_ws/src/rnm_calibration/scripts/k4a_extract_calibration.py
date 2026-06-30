#!/usr/bin/env python3
"""Run the Azure Kinect SDK calibration extractor without ROS."""

from __future__ import annotations

from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from rnm_calibration.k4a_factory_calibration import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
