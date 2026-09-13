from pythfinder.Trajectory.Control.feedforward import *
from pythfinder.Trajectory.Markers.generic import *
from pythfinder.Trajectory.trajectory import *
from pythfinder.Trajectory.Segments import *

from pythfinder.Trajectory.robotConfig import (FLL_ROBOT,
                                               RobotConfig,
                                               robot_from_constants)
from pythfinder.Trajectory.diagnostics import Diagnostic
from pythfinder.Trajectory.field import Field, FLL_FIELD

# for marker sorting
type_priority = {
    'constraints': 0,
    'interrupt': 1,
    'function': 2
}


class TrajectoryBuilder():
    def __init__(self,
                 sim = None,
                 start_pose: Pose | None = None,
                 preset: int = 1,
                 robot: RobotConfig | None = None):
        """Start describing a run.

        Two ways to call this:

            TrajectoryBuilder(sim, START_POSE, preset)   # with a simulator
            TrajectoryBuilder(START_POSE, robot = FLL_ROBOT)

        The first is what team scripts have always used: the robot is taken
        from the preset, and the trajectory can be followed in the window
        afterwards. The second needs no interface at all, which is what lets
        the web planner build the same trajectory in a browser.
        """

        # called the second way, with the pose where the simulator usually goes
        if isinstance(sim, Pose):
            sim, start_pose = None, sim

        self.sim = sim

        if sim is None:
            self.robot_config = FLL_ROBOT.copy() if robot is None else robot

            self.preset = None
            self.preset_nr = None

            self.kinematics = self.robot_config.kinematics
            self.CONSTRAINTS = self.robot_config.constraints

        else:
            self.robot = sim.robot

            self.preset_nr = preset

            self.sim.presets.on(self.preset_nr)
            self.preset = self.sim.presets.get(self.preset_nr)



            if preset is None:
                self.kinematics = self.sim.constants.kinematics
                self.CONSTRAINTS = self.sim.constants.constraints

            else:
                self.kinematics = self.preset.preset_constants.kinematics
                self.CONSTRAINTS = self.preset.preset_constants.constraints

                self.sim.constants.REAL_MAX_VEL = self.preset.preset_constants.REAL_MAX_VEL
                self.sim.constants.MAX_POWER = self.preset.preset_constants.MAX_POWER

            self.sim.constants.kinematics = self.kinematics.copy()
            self.sim.constants.constraints = self.CONSTRAINTS.copy()

            # captured now rather than read at export time, so that building a
            # second trajectory cannot change what this one exports
            self.robot_config = (robot if robot is not None
                                 else robot_from_constants(self.sim.constants))



        # problems found while building, kept as data: the simulator prints
        # them, the web planner puts them next to the step that caused them
        self.diagnostics: List[Diagnostic] = []
        self.print_diagnostics = True

        # which step is being processed, for diagnostics to point at
        self.__step = None

        # the mat a path is expected to stay on
        self.field = self.__field_from_preset()

        self.START_POSE = Pose() if start_pose is None else start_pose

        self.segments: List[MotionSegment] = []
        self.relative_markers: List[Marker] = []
        self.final_markers: List[FunctionMarker] = []
        self.absolute_markers: List[ConstraintsMarker] = []

        self.last_state = MotionState()

        self.pose = self.START_POSE

        self.states: List[MotionState] = []
        self.TRAJ_TIME = 0

    

    def inLineCM(self, cm: float):
        if self.__is_eligible_to_combine_linear(cm): # combine consecutive line segments
            last_linear_sgm: LinearSegment = self.segments[-1]
            self.segments[-1] = last_linear_sgm.add_cm(cm)

            return self

        self.segments.append(LinearSegment(last_state = None,
                                           constraints = self.CONSTRAINTS.linear,
                                           point_cm = cm)) # calculate the point when building the trajectory
        self.relative_markers.append(None)

        return self
    
    # does nothing, yet
    def inSpline(self, end: SplineTarget, tangent: bool = True, reversed: bool = False):
        '''self.segments.append(SplineSegment(last_state = None,
                                           kinematics = self.kinematics,
                                           constraints2d = self.CONSTRAINTS,
                                           end = end,
                                           tangent = tangent,
                                           reversed = reversed))
        
        self.relative_markers.append(None)'''

        return self
    

    
    def wait(self, ms: int):
        ms = abs(ms)
        if self.__is_eligible_to_combine_wait(ms):
            last_wait_sgm: WaitSegment = self.segments[-1]
            self.segments[-1] = last_wait_sgm.add_ms(ms)
            
            return self
        
        self.segments.append(WaitSegment(last_state = None,
                                         ms = ms))
        self.relative_markers.append(None)

        return self

    def turnToDeg(self, deg: float, reversed: bool = False):
        self.segments.append(AngularSegment(last_state = None,
                                            constraints = self.CONSTRAINTS.angular,
                                            kinematics = self.kinematics,
                                            angle_deg = deg,
                                            reversed = reversed))
        self.relative_markers.append(None)

        return self



    def toPoint(self, point: Point, reversed: bool = False):
        self.segments.append(PointSegment(last_state = None,
                                          kinematics = self.kinematics,
                                          constraints2d = self.CONSTRAINTS,
                                          point = point,
                                          tangent = False,
                                          reversed = reversed))
        self.relative_markers.append(None)

        return self

    def toPointTangentHead(self, point: Point, reversed: bool = False):
        self.segments.append(PointSegment(last_state = None,
                                          kinematics = self.kinematics,
                                          constraints2d = self.CONSTRAINTS,
                                          point = point,
                                          tangent = True,
                                          reversed = reversed))
        self.relative_markers.append(None)

        return self
    


    def toPose(self, pose: Pose, reversed: bool = False):
        self.segments.append(PoseSegment(last_state = None,
                                         kinematics = self.kinematics,
                                         constraints2d = self.CONSTRAINTS,
                                         pose = pose.normalize_degreez(),
                                         tangent = False,
                                         linear_head = False,
                                         reversed = reversed))
        self.relative_markers.append(None)

        return self

    def toPoseTangentHead(self, pose: Pose, reversed: bool = False):
        self.segments.append(PoseSegment(last_state = None,
                                         kinematics = self.kinematics,
                                         constraints2d = self.CONSTRAINTS,
                                         pose = pose.normalize_degreez(),
                                         tangent = True,
                                         linear_head = False,
                                         reversed = reversed))
        self.relative_markers.append(None)

        return self

    def toPoseLinearHead(self, pose: Pose, reversed: bool = False):
        self.segments.append(PoseSegment(last_state = None,
                                         kinematics = self.kinematics,
                                         constraints2d = self.CONSTRAINTS,
                                         pose = pose.normalize_degreez(),
                                         tangent = False,
                                         linear_head = True,
                                         reversed = reversed))
        self.relative_markers.append(None)

        return self



    def addRelativeDisplacementConstraints(self, cm: float, constraints2d: Constraints2D):
        marker = (ConstraintsMarker(time = None,
                                    displacement = cm,
                                    constraints = constraints2d,
                                    relative = True))
    
        self.__add_relative_marker(marker)
        return self

    def addRelativeTemporalConstraints(self, ms: int, constraints2d: Constraints2D):
        marker = (ConstraintsMarker(time = ms,
                                    displacement = None,
                                    constraints = constraints2d,
                                    relative = True))
        
        self.__add_relative_marker(marker)
        return self

    def addRelativeDisplacementMarker(self, cm: float, fun = idle):
        marker = (FunctionMarker(time = None,
                                displacement = cm,
                                function = fun,
                                relative = True))
        
        self.__add_relative_marker(marker)
        return self

    def addRelativeTemporalMarker(self, ms: int, fun = idle):
        marker = (FunctionMarker(time = ms,
                                displacement = None,
                                function = fun,
                                relative = True))
        
        self.__add_relative_marker(marker)
        return self

    def interruptDisplacement(self, cm: float):
        marker = (InterruptMarker(time = None,
                                  displacement = cm,
                                  relative = True))
        
        self.__add_relative_marker(marker)
        return self

    def interruptTemporal(self, ms: float):
        marker = (InterruptMarker(time = ms,
                                  displacement = None,
                                  relative = True))
        
        self.__add_relative_marker(marker)
        return self



    def addDisplacementMarker(self, cm: float, fun = idle):
        self.final_markers.append(FunctionMarker(time = None,
                                                 displacement = cm,
                                                 function = fun,
                                                 relative = False))
        return self
    
    def addTemporalMarker(self, ms: int, fun = idle):
        self.final_markers.append(FunctionMarker(time = ms,
                                                 displacement = None,
                                                 function = fun,
                                                 relative = False))
        return self



    def build(self) -> Trajectory:

        self.diagnostics = []

        # when each segment ends, in trajectory time. Public because the web
        # planner needs it to say which step is running at a given moment
        self.step_ends = []

        self.last_state = MotionState(pose = self.START_POSE)
        self.pose = self.START_POSE.copy()

        self.states = []
        self.segment_number = len(self.segments)
        self.TRAJ_TIME = 0


        for i in range(self.segment_number):
            self.__step = i

            sgm: MotionSegment = self.segments[i]
            markers: List[Marker] | None = self.relative_markers[i]

            # recursively compleate each
            if sgm.last_state is None:
                sgm: MotionSegment = sgm.copy(self.last_state.copy(), self.CONSTRAINTS)

            # generate the values for each
            sgm.generate()

            if markers is not None:
                _, sgm = self.__process_relative_markers(markers, sgm)

            # combine states from the primitive into one state
            self.states += sgm.get_all()
            self.TRAJ_TIME += sgm.total_time
            self.step_ends.append(self.TRAJ_TIME)

            self.last_state = sgm.states[-1]

        self.__step = None

        # nothing to drive, and nothing below would survive an empty list
        if not self.states:
            self.__report(Diagnostic.error(
                "this run has no steps in it, so there is nothing to drive",
                suggestion = "add a move, a turn or a wait"))

            return Trajectory(self.states, self.final_markers,
                              self.robot_config, self.sim, self.diagnostics)

        self.__process_final_function_markers()
        self.__check_the_path_stays_on_the_field()

        return Trajectory(self.states, self.final_markers,
                          self.robot_config, self.sim, self.diagnostics)



    # every fifth state is enough: at full speed the robot covers about 3mm in
    # 5ms, so nothing slips across the edge and back between two samples
    FIELD_CHECK_EVERY = 5

    def __check_the_path_stays_on_the_field(self):
        """Warn if any corner of the robot ends up off the mat."""
        half_length = self.robot_config.LENGTH_CM / 2
        half_width = self.robot_config.WIDTH_CM / 2

        if half_length == 0 and half_width == 0:
            return  # nothing is known about the robot's size

        worst = 0
        worst_time = None

        for i in range(0, len(self.states), self.FIELD_CHECK_EVERY):
            state = self.states[i]
            over = self.__how_far_off_the_field(state.pose, half_length, half_width)

            if over > worst:
                worst = over
                worst_time = state.time

        if worst_time is None:
            return

        self.__report(Diagnostic.warning(
            "the robot goes off the {0} during this run, by {1}cm at the worst point"
                .format(self.field.name if self.field.name else "field",
                        round(worst, 1)),
            step = self.__step_at_time(worst_time),
            time_ms = worst_time,
            suggestion = "keep the path further from the edge, or start further in"))

    def __how_far_off_the_field(self, pose: Pose, half_length: float, half_width: float) -> float:
        """How far the worst corner of the robot lies past an edge, in cm.

        The corners are rotated here rather than with mathEx.rotate_by, which
        reflects as well as rotates -- see docs/web-planner.md.
        """
        radians = math.radians(pose.head)
        cos, sin = math.cos(radians), math.sin(radians)

        over = 0

        for forward in (half_length, -half_length):     # robot +x is forwards
            for left in (half_width, -half_width):      # robot +y is to its left
                corner = Point(pose.x + forward * cos - left * sin,
                               pose.y + forward * sin + left * cos)

                over = max(over, self.field.outside_by(corner))

        return over

    def __step_at_time(self, time_ms: int):
        """Which step was running at this point in the trajectory.

        The comparison includes the end of a step: a segment's last state is
        timed at the running total, so the final moment of the run would
        otherwise belong to no step at all.
        """
        for step, ends_at in enumerate(self.step_ends):
            if time_ms <= ends_at:
                return step

        return None


        
    def __add_relative_marker(self, marker: Marker):
        if self.relative_markers[-1] is None:
            self.relative_markers[-1] = [marker]
        else: self.relative_markers[-1].append(marker)



    def __marker_priority(self, marker: Marker):
        class_name = marker.__class__.__name__.lower()

        for key in type_priority.keys():    # loop through the priority dictionary
            if key in class_name:           # check if the string is in the class name
                return type_priority[key]   # get the aferent value
            
        return float('inf')                 # Fallback if no type found
    
    def __marker_sort_key(self, marker: Marker):
        priority = self.__marker_priority(marker)
        is_displacement = 0 if marker.displacement is not None else 1
        value = marker.time if marker.time is not None else marker.displacement

        return (priority,                               # sort by priority
                is_displacement,                        # displacement in front of time
                value if value >= 0 else float('inf'),  # put negatives in the back 
                value)                                  # normal sort, by value
    
    def __sort_markers(self, markers: List[Marker]) -> List[Marker]:
        return sorted(markers, 
                      key = self.__marker_sort_key) 



    def __process_negative_into_positive_displacement(self, markers: List[Marker], segment: MotionSegment) -> List[Marker]:
        remove = []
        removed = 0

        for i in range(len(markers)):
            marker = markers[i]

            if marker.displacement is None: continue
            if marker.displacement < 0:
                positive = round(segment.states[-1].displacement + marker.displacement, 3)

                if positive >= segment.states[0].displacement:    # check if it's still in the segment
                    marker.displacement = positive   # if so, update marker
                else:
                    remove.append(i)                 # else remove marker, because it's impossible

                    self.__report(Diagnostic.warning(
                        "{0} {1}cm from the end of this step was dropped, "
                        "because that is {2}cm before the step begins"
                        .format(self.__marker_phrase(marker), abs(marker.displacement),
                                round(segment.states[0].displacement - positive, 2)),
                        step = self.__step,
                        suggestion = "count back a smaller distance, or make the step longer"))
            
        for index in remove:
            markers.pop(index - removed)
            removed += 1
            
        return markers

    def __process_relatives_into_absolutes_displacement(self, markers: List[Marker], segment: MotionSegment) -> List[Marker]:
        for i in range(len(markers)):

            if markers[i].displacement is not None:
                    if markers[i].displacement >= 0:
                        markers[i].displacement = round(markers[i].displacement + segment.states[0].displacement, 3)
        
        return markers



    def __find_displacement_from_segm_time(self, time: int, segment: MotionSegment) -> float:
        state = segment.get_segm_time(time)

        if state is None: return None
        return state.displacement

    def __find_segm_time_from_displacement(self, displacement: float, segment: MotionSegment) -> int:
        last = segment.states[-1].displacement

        # A 40cm drive does not end at 40cm. The profile integrates to
        # 39.99999997, so asking for "40" -- the step's own length, the number
        # the planner shows -- is 26 nanometres past the end and a strict test
        # refuses it. Markers are rounded to 3 decimals on the way in
        # (__process_relatives_into_absolutes_displacement), while segment
        # displacements carry full float error, so the two sides are quantised
        # differently by construction. The tolerance has to be coarser than
        # that rounding, which is why it is 0.001cm -- ten microns, far below
        # anything a robot can be asked to do, and far above the drift.
        if displacement > last and displacement - last <= 0.001:
            displacement = last

        # Closed, not open: a marker sitting exactly on the segment's first or
        # last state belongs to it. An open test made "at the start of this
        # step" -- what the planner's action button produces by default --
        # impossible to express, and dropped it silently.
        if not in_closed_interval(displacement,
                              left = segment.states[0].displacement,
                              right = last):
            return None

        # The last state needs saying out loud. binary_search narrows with
        # `while left + 1 < right` and returns `left`, so it can never return
        # the final index -- ask it for the end of a 40cm drive and it answers
        # with the state before it.
        if displacement >= last:
            return len(segment.states) - 1

        return binary_search(displacement, segment.states, "displacement")[0]
    


    def __separate_markers_by_type(self, markers: List[Marker]) -> Tuple[list]:
        constraints_markers = [m for m in markers if isinstance(m, ConstraintsMarker)]
        interrupt_markers = [m for m in markers if isinstance(m, InterruptMarker)]
        function_markers = [m for m in markers if isinstance(m, FunctionMarker)]

        return (markers, constraints_markers, interrupt_markers, function_markers)

    def __separate_markers_by_value(self, markers: List[Marker]) -> Tuple[list]:
        displacement = [m for m in markers if m.displacement is not None]
        temporal = [m for m in markers if m.time is not None]

        return (markers, temporal, displacement)



    def __find_the_first_in_displacement_order(self, markers: List[Marker], segment: MotionSegment) -> int:
        min_disp = math.inf
        min_index = -1

        for i in range(len(markers)):
            marker = markers[i]

            if marker.displacement is None:
                current_disp = self.__find_displacement_from_segm_time(marker.time, segment)
            else: current_disp = marker.displacement

            if current_disp is None: # marker is not in the segment
                continue

            if current_disp < min_disp:
                min_disp = current_disp
                min_index = i
        
        return min_index



    def __process_relative_constraints(self, markers: List[ConstraintsMarker], segment: MotionSegment) -> MotionSegment:
        while markers:
            index = self.__find_the_first_in_displacement_order(markers, segment)

            the_chosen_one = markers.pop(index)
            time = (the_chosen_one.time if the_chosen_one.time is not None else
                    self.__find_segm_time_from_displacement(the_chosen_one.displacement, segment))
            
            if not segment.time_in_segment_segm_time(time): # marker is not in the segment
                self.__report_marker_not_in_segment(the_chosen_one, segment)
                continue

            self.CONSTRAINTS = the_chosen_one.constraints
            segment.add_constraints_segm_time(time, the_chosen_one.constraints)
            
        
        return segment
    
    def __process_relative_interrupt(self, markers: List[InterruptMarker], segment: MotionSegment) -> MotionSegment:
        # use just the earliest interrupter, obviously
        # earliest in the segment, obviously
        marker_found = False

        while not marker_found and markers:
            effective = self.__find_the_first_in_displacement_order(markers, segment)

            the_chosen_one = markers.pop(effective)
            time = (the_chosen_one.time if the_chosen_one.time is not None else
                    self.__find_segm_time_from_displacement(the_chosen_one.displacement, segment))
            
            if not segment.time_in_segment_segm_time(time): # marker is not in the segment
                self.__report_marker_not_in_segment(the_chosen_one, segment)
                continue

            segment.interrupt_segm_time(time)
            marker_found = True
        

        markers = []
        return segment

    def __process_relative_function(self, markers: List[FunctionMarker], segment: MotionSegment):
        while markers:
            current = markers.pop()
            time = (current.time if current.time is not None else
                    self.__find_segm_time_from_displacement(current.displacement, segment))
            
            if not segment.time_in_segment_segm_time(time): # marker is not in the segment
                self.__report_marker_not_in_segment(current, segment)
                continue

            self.final_markers.append(FunctionMarker(time = segment.states[time].time,
                                                        function = current.function))
        
        return segment


    # big boss function
    def __process_relative_markers(self, markers: List[Marker], segment: MotionSegment):
        markers = self.__process_relatives_into_absolutes_displacement(markers, segment)
        markers = self.__process_negative_into_positive_displacement(markers, segment)
        markers = self.__sort_markers(markers)

        # now we are working with relative time and absolute displacement

        markers, constraints, interrupt, function = self.__separate_markers_by_type(markers)
        segment = self.__process_relative_constraints(constraints, segment)
        segment = self.__process_relative_interrupt(interrupt, segment)
        self.__process_relative_function(function, segment)

        return markers, segment

    def __process_final_function_markers(self):
        self.__process_negative_into_positive_all()
        self.__transform_displacement_into_temporal()
        self.__sort_final_function_markers()

    

    def __process_negative_into_positive_all(self):
        remove = []
        removed = 0

        for i in range(len(self.final_markers)):
            marker = self.final_markers[i]

            if marker.time is None: 
                if marker.displacement < 0:
                    positive = round(self.states[-1].displacement + marker.displacement, 3)

                    if positive >= 0:                    # check if it's still in the segment
                        marker.displacement = positive   # if so, update marker
                    else: 
                        remove.append(i)                 # else remove marker, because it's impossible
                        self.__report_marker_not_in_trajectory(marker)
            
            else:
                if marker.time < 0:
                    positive = self.states[-1].time + marker.time

                    if positive >= 0:
                        marker.time = positive
                    else:
                        remove.append(i)
                        self.__report_marker_not_in_trajectory(marker)
                        
        for index in remove:
            self.final_markers.pop(index - removed)
            removed += 1

    def __transform_displacement_into_temporal(self):
        for marker in self.final_markers:
            if marker.displacement is None: 
                continue

            index, _ = binary_search(marker.displacement, self.states, "displacement")

            marker.displacement = None
            marker.time = self.states[index].time

    def __sort_final_function_markers(self):
        self.final_markers = self.__sort_markers(self.final_markers)



    def __is_eligible_to_combine_linear(self, cm: float) -> bool:
        if len(self.segments) == 0:
            return False
        
        if not isinstance(self.segments[-1], LinearSegment):
            return False
        
        if isinstance(self.segments[-1].target, Point):
            return False
        
        if not signum(cm) == signum(self.segments[-1].target):
            return False
        
        return True

    def __is_eligible_to_combine_wait(self, ms: float) -> bool:
        if len(self.segments) == 0:
            return False
        
        if not isinstance(self.segments[-1], WaitSegment):
            return False
        
        return True



    def __field_from_preset(self) -> Field:
        """The mat to measure against: the preset's, or the FLL table."""
        if self.preset is None or self.preset.img_size_cm is None:
            return FLL_FIELD

        return Field(self.preset.img_size_cm.width,
                     self.preset.img_size_cm.height,
                     self.preset.name)

    def __report(self, diagnostic: Diagnostic):
        """Record a problem, and say it out loud unless asked not to."""
        self.diagnostics.append(diagnostic)

        if self.print_diagnostics:
            print("\n\n{0}".format(diagnostic))

    def __marker_phrase(self, marker: Marker) -> str:
        """What to call a dropped marker, in words a team member chose.

        A constraints marker is a speed limit and an interrupt is, well, an
        interrupt -- to a kid neither is "an action", which is what these
        messages called every marker before step 4.5 made a constraints
        marker something the browser could actually produce.
        """
        if isinstance(marker, ConstraintsMarker):
            return "a speed limit"

        if isinstance(marker, InterruptMarker):
            return "an interrupt"

        return "an action"

    def __report_marker_not_in_segment(self, marker: Marker, segment: MotionSegment):
        if marker.time is not None:
            asked_for = "{0}ms".format(marker.time)
            as_long_as = "{0}ms".format(len(segment.states) - 1)
        else:
            asked_for = "{0}cm".format(round(marker.displacement - segment.states[0].displacement, 2))
            as_long_as = "{0}cm".format(round(segment.states[-1].displacement - segment.states[0].displacement, 2))

        self.__report(Diagnostic.warning(
            "{0} {1} into this step was dropped, because the step only "
            "goes as far as {2}".format(self.__marker_phrase(marker), asked_for, as_long_as),
            step = self.__step,
            suggestion = "put it before {0}, or make the step longer"
                         .format(as_long_as)))

    def __report_marker_not_in_trajectory(self, marker: Marker):
        if marker.time is not None:
            asked_for = "{0}ms".format(marker.time)
            as_long_as = "{0}ms".format(len(self.states) - 1)
        else:
            asked_for = "{0}cm".format(round(marker.displacement, 2))
            as_long_as = "{0}cm".format(round(self.states[-1].displacement, 2))

        self.__report(Diagnostic.warning(
            "an action {0} into the run was dropped, because the whole run only "
            "goes as far as {1}".format(asked_for, as_long_as),
            suggestion = "put the action before {0}, or add more steps"
                         .format(as_long_as)))



    def __print_markers_debugging(self, markers: List[Marker]):
        print("\n\n")

        for marker in markers:
            class_type = marker.__class__.__name__

            print("{0}: time ({1})   displacement ({2})"
                  .format(class_type,
                          marker.time,
                          marker.displacement))
    
    def __write_segment_in_file_debugging(self, segment: MotionSegment, file_name: str = "test"):
        with open("{0}.txt".format(file_name), "w") as f:
            for state in segment.states:
                f.write("{0} {1} {2} {3}\n".format(round(state.velocities.get_velocity_magnitude(), 1),
                                             state.velocities.ANG_VEL,
                                             state.time,
                                             round(state.displacement, 3)))
    
