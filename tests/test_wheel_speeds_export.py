"""The swerve branch of the wheel-speeds export (docs/web-planner.md,
"Found along the way").

TrajectoryGenerator.wheel_speeds_text() has carried a swerve-only branch since
step 1.4, untouched because nothing here runs a swerve robot: the team drives
tank, so every golden in tests/golden/ goes through the plain branch instead.
That let a formatting bug sit unseen -- `(power, 2)` is a tuple, not
`round(power, 2)` -- so a swerve export's per-module text came out
"(31.1, 2) 0.0 " instead of "31.1 0.0 ". No existing golden exercises this
branch, so this file builds a tiny swerve run directly, without the
TrajectoryBuilder/Simulator machinery the real goldens use, and pins its
export text.
"""

from pythfinder import Point, Pose
from pythfinder.Trajectory.Kinematics.generic import ChassisState
from pythfinder.Trajectory.Kinematics.SwerveKinematics import SwerveKinematics
from pythfinder.Trajectory.Segments.Primitives.generic import MotionState
from pythfinder.Trajectory.constraints import Constraints2D
from pythfinder.Trajectory.robotConfig import RobotConfig
from pythfinder.Trajectory.trajectoryGenerator import TrajectoryGenerator


def swerve_generator():
    """One state, driving straight at a round number, on a square chassis.

    real_max_velocity = 20 with a field velocity of (20, 0) makes every
    module's motor power exactly 100.0, so the expected text below is exact
    rather than rounded off some less tidy number.
    """
    robot = RobotConfig(
        kinematics = SwerveKinematics(track_width = 10, track_length = 10),
        constraints = Constraints2D(),
        real_max_velocity = 20)

    states = [MotionState(time = 0,
                          field_vel = ChassisState(Point(20, 0), 0),
                          pose = Pose(0, 0, 0))]

    return TrajectoryGenerator(states, [], robot)


def test_swerve_export_has_no_stray_tuple():
    """The exact bug: `(power, 2)` formats as a tuple, not a rounded number."""
    text = swerve_generator().wheel_speeds_text(steps = 1)

    assert "(" not in text and ")" not in text


def test_swerve_export_matches_expected_line():
    """Pins the fixed format whole. Before the fix this read:

    '\\n1\\n(100.0, 2) 0.0 (100.0, 2) 0.0 (100.0, 2) 0.0 (100.0, 2) 0.0 0 1 '
    """
    text = swerve_generator().wheel_speeds_text(steps = 1)

    assert text == "\n1\n100.0 0.0 100.0 0.0 100.0 0.0 100.0 0.0 0 1 "


def test_tank_export_is_unaffected():
    """The swerve branch is an `isinstance` check away from tank's -- make
    sure fixing it did not touch the branch every golden actually exercises."""
    from pythfinder.Trajectory.Kinematics.TankKinematics import TankKinematics

    robot = RobotConfig(
        kinematics = TankKinematics(track_width = 10),
        constraints = Constraints2D(),
        real_max_velocity = 20)

    states = [MotionState(time = 0,
                          field_vel = ChassisState(Point(20, 0), 0),
                          pose = Pose(0, 0, 0))]

    text = TrajectoryGenerator(states, [], robot).wheel_speeds_text(steps = 1)

    assert text == "\n1\n100.0 100.0 0 1 "
