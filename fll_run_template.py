"""
Starter template for an FLL run.

Copy this file once per run (run_a.py, run_b.py, ...), edit the two marked
sections, and you have a trajectory the SPIKE hub can follow.

There are three steps, and they happen on two different machines:

    1. THIS FILE, on your computer
       Builds the path, shows it in the simulator, and writes a .txt export.

    2. tools/txt_to_py.py, on your computer
       Turns that .txt into a Python module, because the SPIKE hub has no
       filesystem and cannot open() a file while a program is running.

           python3 tools/txt_to_py.py Trajectory/TXT/run_a.txt

    3. runs.py, on the hub
       Imports the generated module and binds your attachment motors to the
       markers. See ON THE HUB at the bottom of this file.

Run this file with:

    python fll_run_template.py            # watch it, then export
    python fll_run_template.py --export   # export only, no window
"""

import argparse
import os

import pygame
import pythfinder
from pythfinder import Pose, TrajectoryBuilder


# ---------------------------------------------------------------------------
# EDIT ME 1 -- where things go
# ---------------------------------------------------------------------------

# Name of this run. The export becomes <RUN_NAME>.txt, and the hub module
# becomes traj_<RUN_NAME>.py.
RUN_NAME = "run_a"

# Where your pythfinder-EV3-quick-start clone lives. The export is written
# into its Trajectory/TXT/ folder so the converter can find it.
QUICK_START = os.path.expanduser("~/pythfinder-EV3-quick-start")

# Milliseconds per exported state. Motion is calculated every 1ms, so 6 keeps
# every sixth sample. Smaller means a bigger file on a hub with a small heap;
# larger means coarser motion. 6 is a good default, 10 is still fine.
STEPS = 6

# Where the robot starts on the mat, in centimetres from the middle of the
# field, with the heading in degrees. Drive the robot around the simulator with
# a controller and read these numbers off the bottom of the window.
START_POSE = Pose(x = -47, y = 97, head = -45)

FLL_FIELD = 1   # preset 1 is the FLL field, preset 2 is FTC


# ---------------------------------------------------------------------------
# EDIT ME 2 -- the run itself
# ---------------------------------------------------------------------------

def build(sim):
    """Describe the run. Distances are centimetres, angles are degrees.

    Markers are how attachment motors get triggered. The function you pass here
    runs ONLY in the simulator - it is there so you can see the marker fire.
    What actually crosses over to the hub is the marker's *timestamp*. The real
    motor code is bound in runs.py, in the same order the markers happen in
    TIME. See ON THE HUB below.
    """

    return (TrajectoryBuilder(sim, START_POSE, FLL_FIELD)

            .inLineCM(75)
                # 35cm into this 75cm move
                .addRelativeDisplacementMarker(35, lambda: print("marker 1: arm down"))

            .wait(600)

            .turnToDeg(90)

            .inLineCM(-30)
                # -1 means "1ms before the end of this move", so negative values
                # count back from the end
                .addRelativeTemporalMarker(-1, lambda: print("marker 2: arm up"))

            .build())


# ---------------------------------------------------------------------------
# you should not need to edit below here
# ---------------------------------------------------------------------------

def export(trajectory):
    out_dir = os.path.join(QUICK_START, "Trajectory", "TXT")

    if not os.path.isdir(out_dir):
        raise SystemExit(
            "cannot find {0}\n"
            "fix QUICK_START at the top of this file so it points at your "
            "pythfinder-EV3-quick-start clone".format(out_dir))

    # generate() appends the .txt itself, so hand it the path without one
    trajectory.generate(os.path.join(out_dir, RUN_NAME), steps = STEPS)

    print("\nnow convert it for the hub:")
    print("    cd {0}".format(QUICK_START))
    print("    python3 tools/txt_to_py.py Trajectory/TXT/{0}.txt".format(RUN_NAME))
    print("\nthen in runs.py:")
    print("    import traj_{0}".format(RUN_NAME))


def main():
    parser = argparse.ArgumentParser(description = "build and export an FLL run")
    parser.add_argument("--export", action = "store_true",
                        help = "skip the simulator, just write the file")
    args = parser.parse_args()

    sim = pythfinder.Simulator()

    try:
        trajectory = build(sim)
        export(trajectory)

        if args.export:
            return

        # watch it. close the window when you are done - you can drive the
        # robot around with a controller first to find coordinates.
        trajectory.follow(sim, wait = True)

        while sim.RUNNING():
            sim.update()

    except pygame.error as e:
        if "display Surface quit" not in str(e):
            raise
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()


# ---------------------------------------------------------------------------
# ON THE HUB -- what goes in runs.py, next to your other runs
# ---------------------------------------------------------------------------
#
#   import traj_run_a
#
#   core = Robot()
#
#   def arm_down():
#       core.leftTask.run(500)
#
#   def arm_up():
#       core.leftTask.run(-500)
#
#   # The tuple is in TIME order, not the order you wrote the markers above.
#   # build() sorts markers by when they happen, so if you add a marker at
#   # second 8 before one at second 2, the hub still fires second 2 first.
#   trajectory1 = Trajectory(traj_run_a).withMarkers((arm_down, arm_up))
#
#   def run_a():
#       trajectory1.follow(core)
#
# Keep marker functions SHORT. On the SPIKE hub there is no threading, so
# markers run inline on the loop that is driving the robot - a blocking call
# like run_angle(500, 90) stalls the whole path until the arm finishes. Use
# run(speed), or run_angle(..., wait=False), and let it finish in its own time.
# follow() calls stopTaskMotors() when the path ends.
