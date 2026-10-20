#!/usr/bin/env python3
"""Zips the BeamNG mod folder into the layout BeamNG expects (mod folders at the zip root).

usage: make-mod-zip.py MOD_DIR OUTPUT_ZIP
"""

import os
import sys
import zipfile


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    source, output = sys.argv[1], sys.argv[2]

    entries = []
    for root, _, files in os.walk(source):
        for name in files:
            path = os.path.join(root, name)
            entries.append((path, os.path.relpath(path, source)))

    # Sorted, with fixed timestamps, so rebuilding gives an identical zip
    with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as archive:
        for path, arcname in sorted(entries, key=lambda e: e[1]):
            info = zipfile.ZipInfo(arcname.replace(os.sep, "/"), date_time=(2020, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o644 << 16
            with open(path, "rb") as f:
                archive.writestr(info, f.read())


if __name__ == "__main__":
    main()
