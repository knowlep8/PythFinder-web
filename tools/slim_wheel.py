#!/usr/bin/env python3
"""Strip a PythFinder wheel down to what a browser needs.

The published wheel carries every menu image, the field pictures, the interface
font and the documentation PDFs -- about 13MB, all of it for the simulator
window. The web planner imports the planning half only: it never opens a
window, never draws a menu, and draws the field itself in the page.

Sending all that to a school laptop over the school's wifi, on top of Pyodide,
is worth avoiding, so the browser build installs a slimmed copy instead. The
desktop wheel on PyPI is untouched -- it still needs its images.

    python tools/slim_wheel.py dist/pythfinder-0.0.5.2-py3-none-any.whl
    python tools/slim_wheel.py dist/*.whl --out-dir web/public

The RECORD file is rewritten rather than copied, so the result is a valid
wheel and not one that merely happens to install.
"""

import argparse
import base64
import csv
import hashlib
import io
import os
import sys
import zipfile


# everything under these, inside the package, is for the interface
DROP_DIRECTORIES = ("Images", "Documentation", "Screenshots", "Font")


def is_wanted(name: str) -> bool:
    parts = name.split("/")

    if len(parts) > 1 and parts[1] in DROP_DIRECTORIES:
        # the __init__.py files are kept: they are what make the directories
        # importable, and something may still import the package path
        return parts[-1] == "__init__.py"

    return True


def record_line(name: str, data: bytes):
    """A RECORD row: path, sha256 hash, size -- as the wheel spec wants them."""
    digest = hashlib.sha256(data).digest()
    encoded = base64.urlsafe_b64encode(digest).rstrip(b"=").decode("ascii")

    return [name, "sha256={0}".format(encoded), str(len(data))]


def slim(wheel_path: str, out_dir: str) -> str:
    source = zipfile.ZipFile(wheel_path)

    record_name = None
    for name in source.namelist():
        if name.endswith(".dist-info/RECORD"):
            record_name = name

    if record_name is None:
        raise SystemExit("{0}: no RECORD, is this a wheel?".format(wheel_path))

    kept = []
    rows = []

    for item in source.infolist():
        if item.filename == record_name or not is_wanted(item.filename):
            continue

        data = source.read(item.filename)
        kept.append((item, data))
        rows.append(record_line(item.filename, data))

    rows.append([record_name, "", ""])

    written = io.StringIO()
    csv.writer(written, lineterminator = "\n").writerows(rows)

    os.makedirs(out_dir, exist_ok = True)
    out_path = os.path.join(out_dir, os.path.basename(wheel_path))

    with zipfile.ZipFile(out_path, "w", zipfile.ZIP_DEFLATED) as slimmed:
        for item, data in kept:
            slimmed.writestr(item, data)

        slimmed.writestr(record_name, written.getvalue())

    return out_path


def main(argv = None):
    parser = argparse.ArgumentParser(description = __doc__)
    parser.add_argument("wheels", nargs = "+", help = "wheel(s) to slim")
    parser.add_argument("--out-dir", default = "dist-web",
                        help = "where to write them (default: dist-web)")
    args = parser.parse_args(argv)

    for wheel in args.wheels:
        before = os.path.getsize(wheel)
        out_path = slim(wheel, args.out_dir)
        after = os.path.getsize(out_path)

        print("{0}\n    -> {1}\n    {2:.1f}MB -> {3:.2f}MB  ({4:.0f}% smaller)"
              .format(wheel, out_path, before / 1e6, after / 1e6,
                      100 * (1 - after / before)))

    return 0


if __name__ == "__main__":
    sys.exit(main())
