#!/usr/bin/env python3

"""
Rebuilds the FLL field image from FIRST's official season PDF.

The BIOGLOW 'Wireframe & Grid' PDF holds the robot game field as a raster drawing.
Page 1 carries a 20cm reference grid over the top of it, page 2 carries the plain
drawing at a higher resolution - page 2 is the one we ship.

Two things need care, and they are the whole reason this script exists:

  * FIRST draws the mission model outlines and the launch area markings as 1px
    lines at roughly 20% contrast (tone ~203 on white). The field is displayed at
    about a third of the source width, and averaging a 1px line of that weight
    across three pixels of white erases it. So the faint lines are deepened and
    widened before the image is reduced, and the reduction keeps the darkest
    pixel of each block rather than averaging.

  * The table size in constants.py is derived from this drawing, not quoted from
    any FIRST document - none of the published material states the mat
    dimensions. Run with --calibrate to re-derive it (see fll_table_width_cm).

Usage:
    python tools/build_field_image.py                  # download, rebuild in place
    python tools/build_field_image.py --pdf local.pdf  # use an already downloaded PDF
    python tools/build_field_image.py --calibrate      # also re-derive the table size

Needs numpy, which arrives with matplotlib.
"""

import argparse
import os
import re
import struct
import sys
import zlib

import numpy as np

PDF_URL = (
    "https://firstinspires.blob.core.windows.net/fll/challenge/2026-27/"
    "fll-challenge-bioglow-wireframe-grid.pdf"
)

DEFAULT_OUT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "pythfinder", "Images", "Field", "FLL_table_BG.png",
)

FIELD_ASPECT = 1.754   # the drawing's width / height, both pages agree on it
GRID_CELL_CM = 20.0    # the reference grid on page 1 is labelled 20cm per cell

FAINT_FLOOR = 140      # below this is a heavy mission outline, leave it alone
INK_CEILING = 250      # above this is bare mat
FAINT_TONE = 120       # what the faint lines are deepened to
FAINT_GROW = 2         # how far they are widened, in source pixels


def read_pdf(path: str | None) -> bytes:
    if path:
        with open(path, "rb") as f:
            return f.read()

    from urllib.request import urlopen
    print("downloading {0}".format(PDF_URL))
    with urlopen(PDF_URL) as response:
        return response.read()


def iter_images(pdf: bytes):
    """Yield (header, start, end) for every image XObject stream in the file."""

    pattern = rb"<<(?:[^<>]|<<(?:[^<>]|<<[^>]*>>)*>>)*?>>\s*stream\r?\n"

    for match in re.finditer(pattern, pdf):
        header = match.group(0)
        if b"/Image" not in header:
            continue

        width = re.search(rb"/Width\s+(\d+)", header)
        height = re.search(rb"/Height\s+(\d+)", header)
        if not width or not height:
            continue

        start = match.end()
        end = pdf.find(b"endstream", start)
        if end < 0:
            continue

        yield header, int(width.group(1)), int(height.group(1)), start, end


def find_field_raster(pdf: bytes) -> np.ndarray:
    """The plain drawing from page 2: the largest RGB image at the field's aspect."""

    best = None

    for header, width, height, start, end in iter_images(pdf):
        if b"/DeviceRGB" not in header or b"/SMask" in header:
            continue
        if abs(width / height - FIELD_ASPECT) > 0.01:
            continue
        if best is None or width > best[0]:
            best = (width, height, start, end)

    if best is None:
        sys.exit("no field raster found - has the PDF layout changed?")

    width, height, start, end = best
    raw = zlib.decompress(pdf[start:end])

    expected = width * height * 3
    if len(raw) != expected:
        sys.exit(
            "field raster is {0} bytes, expected {1} - it is probably using a "
            "PNG predictor now, which this script does not undo".format(len(raw), expected)
        )

    print("field raster: {0} x {1}".format(width, height))
    return np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 3)


def find_grid_mask(pdf: bytes):
    """Page 1's reference grid, carried as the soft mask of the overlay layer."""

    for header, width, height, start, end in iter_images(pdf):
        if b"/DeviceGray" not in header:
            continue
        if abs(width / height - FIELD_ASPECT) > 0.01:
            continue

        raw = zlib.decompress(pdf[start:end])
        if len(raw) != width * height:
            continue

        return np.frombuffer(raw, dtype=np.uint8).reshape(height, width)

    return None


def calibrate(pdf: bytes, field: np.ndarray):
    """Re-derive the table size from the 20cm grid, for constants.py."""

    mask = find_grid_mask(pdf)
    if mask is None:
        print("no grid mask found, skipping calibration")
        return

    height, width = mask.shape
    row = mask[int(height * 0.30)]

    lines, run = [], None
    for x, value in enumerate(row > 40):
        if value and run is None:
            run = x
        elif not value and run is not None:
            lines.append((run + x - 1) // 2)
            run = None

    if len(lines) < 3:
        print("could not read the grid, skipping calibration")
        return

    gaps = [lines[i + 1] - lines[i] for i in range(len(lines) - 1)]
    pitch = sum(gaps) / len(gaps)
    px_per_cm = pitch / GRID_CELL_CM

    print("\ngrid: {0} lines, {1:.1f}px per {2:.0f}cm cell".format(len(lines), pitch, GRID_CELL_CM))
    print("constants.py should read:")
    print("    fll_table_width_cm  = {0:.1f}".format(width / px_per_cm))
    print("    fll_table_height_cm = {0:.1f}".format(height / px_per_cm))
    print("  (field raster is {0} x {1}, same physical extent)\n".format(
        field.shape[1], field.shape[0]))


def enhance(field: np.ndarray) -> np.ndarray:
    """Deepen and widen the faint lines so the reduction cannot lose them."""

    out = field.copy()
    lum = out.min(axis=2)

    solid = lum < FAINT_FLOOR
    faint = (lum >= FAINT_FLOOR) & (lum < INK_CEILING)

    grown = faint.copy()
    for radius in range(1, FAINT_GROW + 1):
        for shift in ((radius, 0), (-radius, 0), (0, radius), (0, -radius)):
            grown |= np.roll(faint, shift, axis=(0, 1))

    out[grown & ~solid] = FAINT_TONE
    print("faint lines: {0:,} px deepened, {1:,} px after widening".format(
        int(faint.sum()), int((grown & ~solid).sum())))
    return out


def reduce_min(field: np.ndarray) -> np.ndarray:
    """Halve the image keeping the darkest pixel of each 2x2 block."""

    height, width = field.shape[0] // 2 * 2, field.shape[1] // 2 * 2
    field = field[:height, :width]
    return field.reshape(height // 2, 2, width // 2, 2, 3).min(axis=(1, 3))


def write_png(path: str, image: np.ndarray):
    height, width = image.shape[:2]

    def chunk(kind: bytes, payload: bytes) -> bytes:
        body = kind + payload
        return struct.pack(">I", len(payload)) + body + struct.pack(
            ">I", zlib.crc32(body) & 0xFFFFFFFF)

    rows = np.hstack([np.zeros((height, 1), dtype=np.uint8),
                      image.reshape(height, width * 3)])

    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(rows.tobytes(), 9))
           + chunk(b"IEND", b""))

    with open(path, "wb") as f:
        f.write(png)

    print("wrote {0} ({1} x {2}, {3:,} bytes)".format(path, width, height, len(png)))


def main():
    parser = argparse.ArgumentParser(description = __doc__,
                                     formatter_class = argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pdf", help = "local copy of the wireframe PDF (default: download)")
    parser.add_argument("--out", default = DEFAULT_OUT, help = "output PNG")
    parser.add_argument("--calibrate", action = "store_true",
                        help = "also re-derive the table size in cm")
    args = parser.parse_args()

    pdf = read_pdf(args.pdf)
    field = find_field_raster(pdf)

    if args.calibrate:
        calibrate(pdf, field)

    write_png(args.out, reduce_min(enhance(field)))


if __name__ == "__main__":
    main()
