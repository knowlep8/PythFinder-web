from pythfinder.Trajectory.Segments.Primitives.generic import *
from pythfinder.Trajectory.Markers import *

from pythfinder.Trajectory.trajectoryGenerator import *
from pythfinder.Trajectory.robotConfig import RobotConfig

# safe to import at the top: the hub export needs nothing but struct
from pythfinder.Export.hubModule import hub_module_text

# A built trajectory: the motion states, the markers, and the robot they were
# planned for.
#
# Exporting needs none of the interface, so the generator is built here. The
# follower needs a Simulator to animate in, and the grapher needs matplotlib,
# so both are imported inside the methods that use them -- that way importing
# this module does not require either. See docs/web-planner.md, step 1.4.


class Trajectory():
    def __init__(self,
                 motion_states: List[MotionState],
                 markers: List[FunctionMarker],
                 robot: RobotConfig,
                 sim = None) -> None:

        self.STATES = motion_states
        self.MARKERS = markers
        self.robot = robot
        self.sim = sim

        self.TIME = self.STATES[-1].time

        self.trajGenerator = TrajectoryGenerator(motion_states, markers, robot)
        self.trajFollower = None
        self.trajGrapher = None

    # the module the hub imports, as text, without writing it
    def hub_module(self, name: str = "trajectory", steps: int = 1) -> str:
        """Ready to save as <name>.py and upload to the hub.

        Replaces the separate tools/txt_to_py.py step: no .txt has to exist
        first, which is what lets a browser produce a finished file.
        """
        return hub_module_text(self.trajGenerator, name, steps)

    # the text that would be written to the .txt, without writing it
    def text(self,
             steps: int = 1,
             wheel_speeds: bool = True,
             separate_lines: bool = False) -> str:

        if wheel_speeds:
            return self.trajGenerator.wheel_speeds_text(steps, separate_lines)
        return self.trajGenerator.chassis_speeds_text(steps, separate_lines)

    # generates a .txt file with wheel velocities
    def generate(self,
                 file_name: str,
                 steps: int = 1,
                 wheel_speeds: bool = True,
                 separate_lines: bool = False):

        if self.TIME <= 0:
            print("\n\ncan't generate data from an empty trajectory")
            return None

        print('\n\nwriting precious values into * {0}.txt * ...'.format(file_name))

        if wheel_speeds:
            self.trajGenerator.generate_wheel_speeds(file_name, steps, separate_lines)
        else: self.trajGenerator.generate_chassis_speeds(file_name, steps, separate_lines)

        print('\n\ndone writing in * {0}.txt * file :o'.format(file_name))

    def graph(self,
              connect: bool = False,
              velocity: bool = True,
              acceleration: bool = True,
              wheel_speeds: bool = True,
              sim = None):

        if self.TIME <= 0:
            print("\n\ncan't graph an empty trajectory")
            return None

        if not (velocity or acceleration):
            print("\n\nno velocity: checked")
            print("no acceleration: checked")
            print("wait... what am I supposed to graph then???")
            print("\nyour greatest dreams and some pizza, right?")
            return None

        from pythfinder.Trajectory.trajectoryGrapher import TrajectoryGrapher

        self.trajGrapher = TrajectoryGrapher(self.__need_sim(sim, "graph"), self.STATES)

        print('\n\ncomputing graph...')

        if wheel_speeds:
            self.trajGrapher.graph_wheel_speeds(connect, velocity, acceleration)
        else: self.trajGrapher.graph_chassis_speeds(connect, velocity, acceleration)

    def follow(self,
               sim = None,
               perfect: bool = None,
               wait: bool = None,
               steps: int = None):
        """Animate this trajectory in a simulator window.

        The arguments default to None rather than to their real defaults so
        that the old signature keeps working: follow() used to be
        follow(perfect, wait, steps), and a bool where the simulator goes means
        somebody's existing script is calling it that way. Everything then
        shifts along one place, which needs to tell "not passed" from "passed
        False" -- hence the sentinels.
        """
        if isinstance(sim, bool):
            sim, perfect, wait, steps = None, sim, perfect, wait

        perfect = True if perfect is None else perfect
        wait = True if wait is None else wait

        if self.TIME == 0:
            print("\n\ncan't follow an empty trajectory")
            return None

        from pythfinder.Trajectory.trajectoryFollower import TrajectoryFollower

        self.trajFollower = TrajectoryFollower(self.__need_sim(sim, "follow"),
                                               self.STATES, self.MARKERS)

        print('\n\nFOLLOWING TRAJECTORY...')

        self.trajFollower.follow(perfect, wait, steps)

        print('\n\nTRAJECTORY COMPLETED! ;)')

    def __need_sim(self, sim, what: str):
        """The simulator to draw in: the one passed, else the one that built this."""
        sim = self.sim if sim is None else sim

        if sim is None:
            raise ValueError(
                "'{0}' needs a Simulator to draw in. This trajectory was built "
                "without one, so pass it: trajectory.{0}(sim)".format(what))

        return sim
