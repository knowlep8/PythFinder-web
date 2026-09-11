"""What the robot is, with no interface attached.

Planning a path needs four things about a robot: how its wheels are arranged,
how fast it is allowed to accelerate and turn, how fast it can actually go, and
what counts as full power. Until now those lived in
Components/Constants/constants.py, which imports pygame and loads every menu
image the moment it is imported.

They live here instead, in a module that imports nothing but maths, so the
trajectory code -- and the web planner running in a browser -- can describe a
robot without an interface. constants.py imports these values back, so the
simulator and existing team scripts see them at the names they always used.

See docs/web-planner.md, step 1.3.
"""

from pythfinder.Components.BetterClasses.mathEx import Point
from pythfinder.Trajectory.Kinematics.TankKinematics import TankKinematics
from pythfinder.Trajectory.constraints import Constraints2D


class RobotConfig():
    def __init__(self,
                 kinematics,
                 constraints: Constraints2D,
                 real_max_velocity: float,
                 max_power: int | float = 100,
                 width_cm: float = 0,
                 length_cm: float = 0):
        """
        Args:
            kinematics: wheel layout, e.g. TankKinematics.
            constraints: planned speed and acceleration limits.
            real_max_velocity: what the robot actually does at full power, cm/s.
            max_power: what "full power" means to the hub. 100 for a motor
                driven by a percentage, 1 where power is a fraction.
            width_cm: side to side, across the wheels. Drawing only.
            length_cm: front to back. Drawing only.
        """

        self.kinematics = kinematics
        self.constraints = constraints

        self.REAL_MAX_VEL = real_max_velocity
        self.MAX_POWER = max_power

        self.WIDTH_CM = width_cm
        self.LENGTH_CM = length_cm

    def to_motor_power(self, velocity: float):
        """Turn a planned velocity in cm/s into a power for the hub.

        This is the one place the plan meets the hardware: if REAL_MAX_VEL is
        wrong, every straight comes out the wrong length.
        """
        return round(velocity * self.MAX_POWER / self.REAL_MAX_VEL, 2)

    def copy(self):
        return RobotConfig(self.kinematics.copy(),
                           self.constraints.copy(),
                           self.REAL_MAX_VEL,
                           self.MAX_POWER,
                           self.WIDTH_CM,
                           self.LENGTH_CM)


# --- the team's robot, measured on the build --------------------------------

fll_robot_width_cm = 19    # side to side, across the wheels
fll_robot_length_cm = 14   # front to back

# center_offset points from the robot's geometric centre to the centre it turns
# about, in robot coordinates with +x forward. The axle sits 3.5cm from the back
# of a 14cm robot, so 3.5cm behind the centre.
fll_center_offset = Point(-3.5, 0)

# distance between the two drive wheels, centre to centre - measured on the
# build. Every turn is scaled by this, so a wrong value makes turns
# consistently over- or under-shoot.
fll_track_width_cm = 16

# top speed at full power, in cm/s. This converts a planned speed into a motor
# power, so if it is wrong every straight comes out the wrong length. Measured
# on the robot with measure_max_velocity.py: three runs averaging 64.3 cm/s
# with a 1.8 cm/s spread, at 8.23V.
#
# This is the robot's ceiling, not the speed runs are planned at - that is the
# linear constraint, deliberately left lower so the heading PID has power left
# to correct with.
fll_real_max_velocity = 64.3


FLL_ROBOT = RobotConfig(
    kinematics = TankKinematics(fll_track_width_cm,
                                center_offset = fll_center_offset),
    constraints = Constraints2D(track_width = fll_track_width_cm),
    real_max_velocity = fll_real_max_velocity,
    width_cm = fll_robot_width_cm,
    length_cm = fll_robot_length_cm)
