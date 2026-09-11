from pythfinder.Trajectory.Kinematics.SwerveKinematics import *
from pythfinder.Trajectory.Segments import *
from pythfinder.Trajectory.Markers import *
from pythfinder.Trajectory.robotConfig import RobotConfig

# Turns motion states into the text the hub reads.
#
# This used to take the whole Simulator, for two things it could get from a
# RobotConfig instead: the wheel layout, and the conversion from a planned
# velocity to a motor power. It takes the robot itself now, so exporting a
# trajectory needs no window -- see docs/web-planner.md, step 1.4.
#
# Each method comes in two halves: one that builds the text, and one that
# writes it to a file. The web planner hands the text straight to a download
# instead, since a browser has no filesystem to write to.


class TrajectoryGenerator():
    def __init__(self,
                 motion_states: List[MotionState],
                 markers: List[FunctionMarker],
                 robot: RobotConfig
                 ):

        self.STATES = motion_states
        self.MARKERS = markers
        self.robot = robot

    def __header(self, steps: int) -> str:
        # first layer --> marker times
        first_string = ''
        for marker in self.MARKERS:
            first_string += '{0} '.format(marker.time)

        # second layer --> steps
        return first_string + '\n' + "{0}\n".format(steps)

    def wheel_speed_groups(self, steps: int = 1):
        """Walk the states, collapsing runs of identical ones.

        Yields (wheel_states, head, copies): the speed of each wheel, the
        heading to hold, and how many consecutive states said the same thing.

        Both exports are built from this -- the .txt the simulator writes and
        the hub module -- so the two cannot end up describing different motion.
        """
        rg = int(len(self.STATES) / steps)
        t = 0

        while t <= rg:
            try: # basically you're out of the list
                current_state = self.STATES[t * steps]
            except: break

            consecutive = 0
            try: # same stuff here
                while current_state.is_like(self.STATES[t * steps]):
                    consecutive += 1
                    t += 1
            except: pass

            # velocities for each wheel, acording to the kinematics
            robot_centric_vel = current_state.velocities.field_to_robot(current_state.pose)
            wheel_states = self.robot.kinematics.inverse(robot_centric_vel)

            yield wheel_states, current_state.pose.head, consecutive

    def wheel_speeds_text(self, steps: int = 1, separate_lines: bool = False) -> str:
        text = self.__header(steps)

        # third layer --> wheel velocities -- head -- nr of consecutive copies
        for wheel_states, head, consecutive in self.wheel_speed_groups(steps):

            if isinstance(self.robot.kinematics, SwerveKinematics):
                # add module angles too
                line = ''.join("{0} {1} ".format((self.robot.to_motor_power(state.VELOCITY), 2), round(state.ANGLE, 2)) for state in wheel_states)
            else:
                line = ''.join(str(round(self.robot.to_motor_power(state.VELOCITY), 2)) + " " for state in wheel_states)

            line = line + "{0} {1} ".format(
                round(head, 2),
                consecutive)

            if separate_lines:
                line += "\n"

            text += line

        return text

    def chassis_speeds_text(self, steps: int = 1, separate_lines: bool = False) -> str:
        text = self.__header(steps)

        # third layer --> VEL_X -- VEL_Y -- ANG_VEL -- head -- nr of consecutive copies
        rg = int(len(self.STATES) / steps)
        t = 0

        while t <= rg:
            try: # basically you're out of the list
                current_state = self.STATES[t * steps]
            except: break

            consecutive = 0
            try: # same stuff here
                while current_state.is_like(self.STATES[t * steps]):
                    consecutive += 1
                    t += 1
            except: break

            robot_centric_vel = current_state.velocities.field_to_robot(current_state.pose)

            # write velocities for each wheel, acording to the kinematics
            line = '{0} {1} {2} {3} {4}'.format(
                round(robot_centric_vel.VEL.x, 2),
                round(robot_centric_vel.VEL.y, 2),
                round(robot_centric_vel.ANG_VEL,2),
                round(current_state.pose.head, 2),
                consecutive)

            if separate_lines:
                line += "\n"

            text += line

        return text

    def generate_wheel_speeds(self, file_name: str, steps: int = 1, separate_lines: bool = False):
        with open('{0}.txt'.format(file_name), "w") as f:
            f.write(self.wheel_speeds_text(steps, separate_lines))

    def generate_chassis_speeds(self, file_name: str, steps: int = 1, separate_lines: bool = False):
        with open('{0}.txt'.format(file_name), "w") as f:
            f.write(self.chassis_speeds_text(steps, separate_lines))
