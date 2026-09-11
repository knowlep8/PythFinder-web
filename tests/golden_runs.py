"""The reference runs that pin down how the robot moves.

Each run here is exported with the current code into tests/golden/<name>.txt
and committed. That .txt is what becomes the file on the hub, so a changed byte
is a changed robot: these files are what lets the phase 1 refactor in
docs/web-planner.md prove it did not alter any motion.

Between them the runs cover every segment type the team can use, both marker
kinds, constraints markers and interrupts.

To add a run: write the function, register it in GOLDEN_RUNS, then run

    uv run python tests/regenerate_goldens.py

Regenerate deliberately, never just to make a failing test pass. Read the diff
first -- if a run you did not touch changed, the motion changed.
"""

from pathlib import Path

from pythfinder import Point, Pose, TrajectoryBuilder
from pythfinder.Trajectory.constraints import Constraints, Constraints2D
from pythfinder.Trajectory.robotConfig import FLL_ROBOT


GOLDEN_DIR = Path(__file__).parent / "golden"

# The left launch area, the same start pose as fll_run_template.py.
START = Pose(x = -46, y = -83, head = 0)

# Preset 1 is the FLL table: the team's measured robot and the BIOGLOW mat.
FLL_PRESET = 1

# ms per exported state, matching the hub default in fll_run_template.py.
STEPS = 6

# The team's track width, for the constraints marker run below. Repeated here
# on purpose: a golden run should describe itself, not inherit a default that
# may later change.
TRACK_WIDTH_CM = 16


def _start(sim, pose = START):
    """Begin a run, with or without a simulator.

    Passing sim = None builds through the interface-free signature, which is
    how the web planner will call it. Both ways have to produce the same
    trajectory, and test_builds_without_a_simulator checks exactly that.
    """
    if sim is None:
        return TrajectoryBuilder(pose, robot = FLL_ROBOT)

    return TrajectoryBuilder(sim, pose, FLL_PRESET)


def _action():
    """Stands in for an attachment motor.

    Markers only contribute their timestamp to the export, so what this does
    is irrelevant -- but it must exist, since a marker with no function is a
    different code path.
    """


# --- one segment at a time --------------------------------------------------

def line_forward(sim):
    return _start(sim).inLineCM(75).build()


def line_backward(sim):
    return _start(sim).inLineCM(-40).build()


def line_merged(sim):
    """Consecutive same-direction lines are combined by the builder.

    Should come out identical to line_forward; test_goldens.py asserts it.
    """
    return _start(sim).inLineCM(30).inLineCM(45).build()


def wait_merged(sim):
    """Consecutive waits are combined the same way."""
    return _start(sim).inLineCM(20).wait(300).wait(400).build()


def turn_ccw(sim):
    return _start(sim).turnToDeg(90).build()


def turn_cw(sim):
    return _start(sim).turnToDeg(-90).build()


def turn_reversed(sim):
    return _start(sim).turnToDeg(90, reversed = True).build()


# --- driving to a place -----------------------------------------------------
#
# The heading variants all collapse into one another on this robot, and their
# goldens are byte-identical on purpose. A tank drive is NON_HOLONOMIC -- it
# cannot move sideways -- so PointSegment and PoseSegment force tangent = True
# and linear_head = False regardless of what was asked for
# (pointSegment.py:34, poseSegment.py:35). The heading has to follow the line
# of travel, because there is no other way for the robot to get there.
#
# They are kept as separate runs because that collapse is itself worth
# pinning: if a refactor dropped the NON_HOLONOMIC check, these files would
# stop matching each other. test_goldens.py asserts it directly.
#
# What does still change the motion: reversed, and the final turn that toPose
# adds over toPoint.

def to_point(sim):
    return _start(sim).toPoint(Point(0, -40)).build()


def to_point_tangent_head(sim):
    """Identical to to_point on a tank drive -- see the note above."""
    return _start(sim).toPointTangentHead(Point(0, -40)).build()


def to_point_reversed(sim):
    """Arrives driving backwards, which is a genuinely different path."""
    return _start(sim).toPoint(Point(0, -40), reversed = True).build()


def to_pose(sim):
    """Like to_point, plus a turn to the requested heading once there."""
    return _start(sim).toPose(Pose(0, -40, 90)).build()


def to_pose_tangent_head(sim):
    """Identical to to_pose on a tank drive -- see the note above."""
    return _start(sim).toPoseTangentHead(Pose(0, -40, 90)).build()


def to_pose_linear_head(sim):
    """Identical to to_pose on a tank drive -- see the note above.

    linear_head asks for the turn and the straight at the same time, which a
    tank drive cannot do.
    """
    return _start(sim).toPoseLinearHead(Pose(0, -40, 90)).build()


# --- markers ----------------------------------------------------------------

def markers_relative(sim):
    """Displacement and temporal markers, relative to their own segment.

    -1 ms counts back from the end of the segment, which is the idiom the run
    template teaches.
    """
    return (_start(sim)
            .inLineCM(75)
                .addRelativeDisplacementMarker(35, _action)
                .addRelativeTemporalMarker(-1, _action)
            .build())


def markers_absolute(sim):
    """Markers placed against the whole trajectory rather than a segment."""
    return (_start(sim)
            .inLineCM(50)
            .turnToDeg(90)
            .addTemporalMarker(500, _action)
            .addDisplacementMarker(-5, _action)
            .build())


def constraints_marker(sim):
    """Slow the robot down partway through a straight."""
    slow = Constraints2D(linear = Constraints(20, 20, 20),
                         track_width = TRACK_WIDTH_CM)

    return (_start(sim)
            .inLineCM(80)
                .addRelativeDisplacementConstraints(30, slow)
            .build())


def interrupt_marker(sim):
    """Cut a segment short, as a sensor would."""
    return (_start(sim)
            .inLineCM(80)
                .interruptDisplacement(40)
            .build())


# --- the real thing ---------------------------------------------------------

def template_run(sim):
    """The run in fll_run_template.py, the one the team actually drives.

    Its export should match Trajectory/TXT/run_a.txt in the quick-start repo,
    which is the file currently on the hub.
    """
    return (_start(sim)
            .inLineCM(75)
                .addRelativeDisplacementMarker(35, _action)
            .wait(600)
            .turnToDeg(90)
            .inLineCM(30)
                .addRelativeTemporalMarker(-1, _action)
            .toPose(START)
            .build())


# name -> (builder, ms per exported state)
GOLDEN_RUNS = {
    "line_forward": (line_forward, STEPS),
    "line_backward": (line_backward, STEPS),
    "line_merged": (line_merged, STEPS),
    "wait_merged": (wait_merged, STEPS),

    "turn_ccw": (turn_ccw, STEPS),
    "turn_cw": (turn_cw, STEPS),
    "turn_reversed": (turn_reversed, STEPS),

    "to_point": (to_point, STEPS),
    "to_point_tangent_head": (to_point_tangent_head, STEPS),
    "to_point_reversed": (to_point_reversed, STEPS),
    "to_pose": (to_pose, STEPS),
    "to_pose_tangent_head": (to_pose_tangent_head, STEPS),
    "to_pose_linear_head": (to_pose_linear_head, STEPS),

    "markers_relative": (markers_relative, STEPS),
    "markers_absolute": (markers_absolute, STEPS),
    "constraints_marker": (constraints_marker, STEPS),
    "interrupt_marker": (interrupt_marker, STEPS),

    "template_run": (template_run, STEPS),

    # Every millisecond kept, so downsampling cannot hide a difference.
    "template_run_full": (template_run, 1),
}
