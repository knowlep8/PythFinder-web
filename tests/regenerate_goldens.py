#!/usr/bin/env python3
"""Rewrite tests/golden/*.txt from the current code.

    uv run python tests/regenerate_goldens.py            # all of them
    uv run python tests/regenerate_goldens.py turn_ccw   # just these

Run this only when a change to the motion output is intended, and read
`git diff tests/golden/` before committing. During the phase 1 refactor in
docs/web-planner.md, nothing here should change at all.
"""

import os
import sys
from pathlib import Path

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
sys.path.insert(0, str(Path(__file__).parent))

import pygame                                                   # noqa: E402

import pythfinder                                               # noqa: E402
from golden_runs import GOLDEN_DIR, GOLDEN_RUNS                 # noqa: E402


def regenerate(names):
    GOLDEN_DIR.mkdir(exist_ok = True)
    summary = []

    for name in names:
        build, steps = GOLDEN_RUNS[name]

        sim = pythfinder.Simulator()
        trajectory = build(sim)
        trajectory.generate(str(GOLDEN_DIR / name), steps = steps)

        summary.append((name,
                        len(trajectory.STATES),
                        trajectory.TIME,
                        [marker.time for marker in trajectory.MARKERS],
                        (GOLDEN_DIR / "{0}.txt".format(name)).stat().st_size))

    return summary


def main(argv):
    names = argv[1:] or list(GOLDEN_RUNS)

    unknown = [name for name in names if name not in GOLDEN_RUNS]
    if unknown:
        print("\nnot a golden run: {0}".format(", ".join(unknown)))
        print("known runs: {0}".format(", ".join(GOLDEN_RUNS)))
        return 1

    summary = regenerate(names)
    pygame.quit()

    print("\n\n{0:<24} {1:>8} {2:>9} {3:>8}  {4}"
          .format("run", "states", "time", "bytes", "markers"))

    for name, states, time_ms, markers, size in summary:
        print("{0:<24} {1:>8} {2:>8}ms {3:>8}  {4}"
              .format(name, states, time_ms, size,
                      ", ".join(str(each) for each in markers) or "-"))

    print("\nwrote {0} file(s) into {1}".format(len(summary), GOLDEN_DIR))
    print("now read the diff:  git diff tests/golden/")

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
