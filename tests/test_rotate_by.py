"""rotate_by reflected as well as rotating (docs/web-planner.md, "Found along
the way").

mathEx.Point.rotate_by and the free mathEx.rotate_by both computed
`y = x*sin(rad) - y*cos(rad)`, where a rotation needs `y = x*sin(rad) +
y*cos(rad)`. The x half of the formula does not involve that sign, so a point
that starts with y = 0 rotates the same way whether the bug is there or not --
test_rotate_by_is_not_a_reflection below uses a point with both coordinates
nonzero, which is where the mirror actually shows up.

trajectoryBuilder.py's __how_far_off_the_field already uses the correct
`forward * cos - left * sin, forward * sin + left * cos` for the robot's
corners, rather than depend on rotate_by -- that is the convention these
tests pin: head 0 is the robot's own +x (forward), increasing head turns
toward the robot's own +y (left, per that same method's comment), and radians
behave the ordinary mathematical way in between.
"""

import math

from pythfinder import Point, Pose
from pythfinder.Components.BetterClasses.mathEx import rotate_by


def test_rotate_by_matches_the_librarys_convention():
    """A quarter turn takes "forward" to "left" -- see the module docstring."""
    point = Point(1, 0).rotate_by(math.radians(90))

    assert round(point.x, 9) == 0
    assert round(point.y, 9) == 1


def test_rotate_by_is_not_a_reflection():
    """Both coordinates nonzero, so the old sign bug cannot hide.

    Before the fix this returned (-0.366, 0.366) -- the y the bug computed by
    subtracting the cos term instead of adding it.
    """
    point = Point(1, 1).rotate_by(math.radians(60))

    assert round(point.x, 9) == round(-0.3660254037844385, 9)
    assert round(point.y, 9) == round(1.3660254037844388, 9)


def test_rotate_by_mutates_and_returns_self():
    """Matches round(), negate() and set() -- the rest of Point mutates and
    returns self, and robot.py's one real caller already copies first in
    order to rely on that, so the fix keeps it rather than switching to a
    copy no caller asked for."""
    point = Point(1, 0)
    result = point.rotate_by(math.radians(90))

    assert result is point
    assert round(point.x, 9) == 0
    assert round(point.y, 9) == 1


def test_pose_rotate_by_keeps_its_head():
    """Pose.rotate_by wraps Point.rotate_by and should rotate the position
    without disturbing the heading."""
    pose = Pose(1, 0, 45).rotate_by(math.radians(90))

    assert round(pose.x, 9) == 0
    assert round(pose.y, 9) == 1
    assert pose.head == 45


def test_free_rotate_by_matches_the_method():
    """The module-level rotate_by(rad, point) had the same sign bug, fixed
    the same way. It has no real callers today, but mirrors Point.rotate_by's
    formula, so it should keep agreeing with it."""
    point = rotate_by(math.radians(60), Point(1, 1))

    assert round(point.x, 9) == round(-0.3660254037844385, 9)
    assert round(point.y, 9) == round(1.3660254037844388, 9)


def test_free_rotate_by_does_not_mutate_its_argument():
    """Unlike the method, the free function already returned a new Point --
    the fix only touched the formula, not that."""
    original = Point(1, 1)
    rotate_by(math.radians(60), original)

    assert original.x == 1 and original.y == 1
