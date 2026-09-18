#!/usr/bin/env python3
"""Gzip a web asset at build time.

The readable source is what lives in git; only the compressed copy is embedded
in the firmware. mtime is pinned to zero so the output is reproducible and does
not force a relink on every build.

CMake's add_custom_command has no shell, so a redirection such as
"gzip -c src > dst" cannot be expressed there. Hence this helper.
"""

import gzip
import shutil
import sys


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: gzip_asset.py <source> <destination>", file=sys.stderr)
        return 2

    source, destination = sys.argv[1], sys.argv[2]

    with open(source, "rb") as src, gzip.GzipFile(destination, "wb", compresslevel=9, mtime=0) as dst:
        shutil.copyfileobj(src, dst)

    return 0


if __name__ == "__main__":
    sys.exit(main())
