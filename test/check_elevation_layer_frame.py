#!/usr/bin/env python3

import re
import sys
from pathlib import Path


def main() -> int:
    source_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(
        "src/elevation_layer.cpp")
    source = source_path.read_text(encoding="utf-8")

    if 'lookupTransform(\n        "odom",' in source:
        print("NG: lookupTransform() still uses a hard-coded odom target frame.")
        return 1

    if "layered_costmap_->getGlobalFrameID()" not in source:
        print("NG: costmap global frame is not used.")
        return 1

    lookup_transform_uses_global_frame = re.search(
        r"lookupTransform\s*\(\s*(?:global_frame|layered_costmap_->getGlobalFrameID\(\))\s*,",
        source)
    if not lookup_transform_uses_global_frame:
        print("NG: lookupTransform() does not use the costmap global frame as target frame.")
        return 1

    print("OK: point cloud target frame follows the costmap global frame.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
