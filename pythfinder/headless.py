"""Build a run from a plain description, with no interface anywhere.

This is the door the web planner knocks on. It takes the run as JSON-shaped
data -- the same shape the browser saves and reloads -- and hands back
everything needed to draw it, complain about it, and download it:

    {
      "ok": True,
      "total_ms": 15641,
      "poses": [ {"t": 0, "x": -46.0, "y": -83.0, "head": 0.0}, ... ],
      "markers": [ {"id": "a1", "step": 0, "time_ms": 1764}, ... ],
      "diagnostics": [ {"level": "warning", "message": ..., "step": 1, ...} ],
      "module_text": "<the .py file to save and upload to the hub>"
    }

Nothing here raises for a badly described run: a run a child typed wrong is
ordinary, not exceptional, so problems come back as diagnostics next to the
step that caused them.

The run description looks like this:

    {
      "version": 1,
      "name": "run_a",
      "steps_ms": 6,
      "robot": "fll_team",
      "start": {"x": -46, "y": -83, "head": 0},
      "steps": [
        {"type": "drive", "cm": 75,
         "actions": [{"id": "a1", "at": {"cm": 35}, "label": "arm down"}]},
        {"type": "wait", "ms": 600},
        {"type": "turn", "deg": 90},
        {"type": "toPose", "x": -46, "y": -83, "head": 0}
      ]
    }

See docs/web-planner.md, step 1.7.
"""

from pythfinder.Components.BetterClasses.mathEx import Point, Pose
from pythfinder.Trajectory.Kinematics.TankKinematics import TankKinematics
from pythfinder.Trajectory.constraints import Constraints2D
from pythfinder.Trajectory.diagnostics import Diagnostic
from pythfinder.Trajectory.robotConfig import FLL_ROBOT, RobotConfig
from pythfinder.Trajectory.trajectoryBuilder import TrajectoryBuilder


VERSION = 1

# how often a pose is kept for drawing. The states are one per millisecond,
# which is far more than a path on a screen can show.
DEFAULT_POSE_EVERY_MS = 20

STEP_TYPES = ("drive", "wait", "turn", "toPoint", "toPose", "armStep")

# A sequential arm step is given at least this long, so that a small sweep
# still reads as a step of its own on the timeline.
LEAST_ARM_MS = 150


def arm_step_ms(step: dict) -> int:
    """How long an arm step is expected to take, in milliseconds.

    From the motor's own terms: turning `angle` degrees at `speed` degrees per
    second takes angle/speed seconds. It is an estimate and nothing more -- the
    robot waits for the motor itself, not for this number -- but the run length
    on screen is built from it, so it should not be silly.

    `run_until_stalled` has no angle to work from, so it gets a plain guess.
    """
    speed = abs(float(step.get("speed", 0))) or 1

    if step.get("call") == "run_until_stalled":
        return int(step.get("expected_ms", 1000))

    angle = abs(float(step.get("angle", 0)))

    return max(LEAST_ARM_MS, int(angle / speed * 1000))


def robot_from_description(description) -> RobotConfig:
    """The robot to plan for: the team's, or one described by its numbers."""
    if description is None or description == "fll_team":
        return FLL_ROBOT.copy()

    if isinstance(description, str):
        raise ValueError("unknown robot '{0}'".format(description))

    track_width = float(description["track_width_cm"])
    offset = float(description.get("center_offset_cm", 0))

    return RobotConfig(
        kinematics = TankKinematics(track_width, center_offset = Point(offset, 0)),
        constraints = Constraints2D(track_width = track_width),
        real_max_velocity = float(description["max_velocity_cm_s"]),
        max_power = float(description.get("max_power", 100)),
        width_cm = float(description.get("width_cm", 0)),
        length_cm = float(description.get("length_cm", 0)))


def pose_from_description(description) -> Pose:
    if description is None:
        return Pose()

    return Pose(x = float(description.get("x", 0)),
                y = float(description.get("y", 0)),
                head = float(description.get("head", 0)))


def build_run(run: dict, pose_every_ms: int = DEFAULT_POSE_EVERY_MS) -> dict:
    """Turn a described run into something to draw, check and download."""
    diagnostics = []

    name = run.get("name", "trajectory")
    steps_ms = int(run.get("steps_ms", 6))
    steps = run.get("steps", [])

    try:
        robot = robot_from_description(run.get("robot"))
    except (ValueError, KeyError, TypeError) as problem:
        diagnostics.append(Diagnostic.error(
            "this robot cannot be used: {0}".format(problem),
            suggestion = "check the robot settings"))

        return _nothing_to_drive(name, diagnostics)

    builder = TrajectoryBuilder(pose_from_description(run.get("start")),
                                robot = robot)
    builder.print_diagnostics = False

    # which described step each builder segment came from. They can differ:
    # consecutive drives in the same direction, and consecutive waits, are
    # merged into one segment, so a later step may add to an earlier segment.
    segment_owner = []
    action_steps = {}
    action_code = {}

    # How far into the current run of merged steps we are: milliseconds for
    # waits, centimetres for drives.
    #
    # Merging is right for the motion and wrong for what is attached to it.
    # Consecutive waits become one segment, and arm steps are waits, so two arm
    # steps in a row share one; consecutive drives in the same direction become
    # a single acceleration profile. A marker is placed relative to the
    # *segment*, which belongs to whichever step created it -- so without these,
    # "at the start of this step" means the start of the step before it.
    #
    # Left uncorrected it is the worse kind of wrong. Both arm steps fire on the
    # same millisecond, the arm told to do two things at once. An action on the
    # second of two drives fires up to three seconds early, with no diagnostic
    # at all: the file downloads, looks right, and moves the arm confidently at
    # the wrong place on the mat.
    into_wait = 0
    into_line = 0.0

    for index, step in enumerate(steps):
        kind = step.get("type")

        if kind not in ("wait", "armStep"):
            into_wait = 0

        if kind != "drive":
            into_line = 0.0

        _add_step(builder, step, index, diagnostics)

        while len(segment_owner) < len(builder.segments):
            segment_owner.append(index)

            # a new segment: this step begins it, so offsets start again
            into_wait = 0
            into_line = 0.0

        for action in step.get("actions", []):
            _check_code(action, index, diagnostics)

            at = _at_within_step(action.get("at"), step, into_wait, into_line)

            if _add_action(builder, dict(action, at = at), index, diagnostics):
                action_steps[action.get("id")] = index

                if action.get("do") is not None:
                    action_code[action.get("id")] = action["do"]
                elif kind == "armStep":
                    action_code[action.get("id")] = _arm_command(step)

        into_wait += _step_own_ms(step)
        into_line += _step_own_cm(step)

    trajectory = builder.build()

    for problem in trajectory.diagnostics:
        problem.step = _described_step(problem.step, segment_owner)
        diagnostics.append(problem)

    # in the order they fire, which is the order the hub binds them -- not the
    # order the steps were written
    firing_order = [dict(action_code.get(marker.function, {}),
                         id = marker.function,
                         label = _label_for(marker.function, steps))
                    for marker in trajectory.MARKERS]

    module_text = _hub_module(trajectory, name, steps_ms, diagnostics,
                              firing_order)

    return {"version": VERSION,
            "name": name,
            "ok": not any(problem.is_error() for problem in diagnostics),
            "total_ms": trajectory.TIME,
            "steps": _step_times(builder, segment_owner, steps),
            "poses": _poses(trajectory, pose_every_ms),
            # already in the order they fire, which is the order the hub
            # expects the actions to be bound in
            "markers": [{"id": marker.function,
                         "step": action_steps.get(marker.function),
                         "time_ms": marker.time}
                        for marker in trajectory.MARKERS],
            "diagnostics": [problem.as_dict() for problem in diagnostics],
            "module_text": module_text}


def _step_own_ms(step: dict) -> int:
    """How much time this step adds to a run of merged waits. 0 if it is not one."""
    kind = step.get("type")

    if kind == "armStep":
        return arm_step_ms(step)

    if kind == "wait":
        try:
            return int(step["ms"])
        except (KeyError, TypeError, ValueError):
            return 0

    return 0


def _step_own_cm(step: dict) -> float:
    """How much distance this step adds to a merged drive. 0 if it is not one.

    Always positive: displacement along the path grows whichever way the robot
    faces, and a reversed drive starts a new segment anyway -- the builder only
    combines drives whose signs match.
    """
    if step.get("type") != "drive":
        return 0.0

    try:
        return abs(float(step["cm"]))
    except (KeyError, TypeError, ValueError):
        return 0.0


def _at_within_step(at, step: dict, into_wait: int, into_line: float):
    """Move an action's placement from "into this step" to "into this segment".

    Steps merge; markers do not know it. Everything here is the difference
    between the two, and it applies to every action alike -- an earlier version
    offset only the arm step's own implicit marker, which left an explicit one
    on the same step firing inside the step before it.

    A negative value counts back from the end of *this* step, not the merged
    segment's, so it is resolved here against the step's own length rather than
    left to the builder, which knows only the segment.
    """
    kind = step.get("type")

    if at is None:
        if kind != "armStep":
            return at

        # its own step: the arm starts as the robot comes to rest. The +1 keeps
        # it just inside the segment, and the goldens pin these times.
        return {"ms": into_wait + 1}

    if "cm" in at and kind == "drive":
        value = float(at["cm"])
        within = _step_own_cm(step) + value if value < 0 else value

        return dict(at, cm = into_line + within)

    if "ms" in at and kind in ("wait", "armStep"):
        value = int(at["ms"])
        within = _step_own_ms(step) + value if value < 0 else value

        return dict(at, ms = into_wait + within)

    return at


def _add_step(builder: TrajectoryBuilder, step: dict, index: int, diagnostics: list):
    kind = step.get("type")

    try:
        if kind == "drive":
            builder.inLineCM(float(step["cm"]))

        elif kind == "wait":
            builder.wait(int(step["ms"]))

        elif kind == "armStep":
            # The robot stops and the arm runs to completion before the next
            # step. On the hub that waiting happens on the follow loop, which
            # discounts it -- so the length here only has to be a fair guess at
            # how long the motor takes, not a promise.
            builder.wait(arm_step_ms(step))

        elif kind == "turn":
            builder.turnToDeg(float(step["deg"]), bool(step.get("reversed", False)))

        elif kind == "toPoint":
            builder.toPoint(Point(float(step["x"]), float(step["y"])),
                            bool(step.get("reversed", False)))

        elif kind == "toPose":
            builder.toPose(Pose(float(step["x"]), float(step["y"]),
                                float(step.get("head", 0))),
                           bool(step.get("reversed", False)))

        else:
            diagnostics.append(Diagnostic.error(
                "'{0}' is not something the robot knows how to do".format(kind),
                step = index,
                suggestion = "use one of: {0}".format(", ".join(STEP_TYPES))))

    except (KeyError, TypeError, ValueError) as problem:
        diagnostics.append(Diagnostic.error(
            "this step is missing something, or has the wrong kind of value: {0}"
                .format(problem),
            step = index))


def _code_problems(code: str) -> list:
    """Simple, readable checks for code that would block the whole robot.

    Not real analysis -- substring checks, same as the plan asks for. A
    parallel action's code runs on the follow loop itself, which has no
    threading: anything that blocks here stalls driving, not just whatever
    the action was meant to do.
    """
    problems = []

    if "wait(" in code:
        problems.append((
            "this action's code calls wait(), which blocks the whole robot "
            "while it runs",
            "remove it, or use a sequential 'Move arm' step instead"))

    if "while" in code:
        problems.append((
            "this action's code contains a while loop, which can block the "
            "whole robot",
            "remove it, or use a sequential 'Move arm' step instead"))

    # run_angle and run_target both wait by default -- see _call_for in
    # hubModule.py, "the ones that finish". The plan names only run_angle;
    # run_target is the same class of call for the same reason.
    for call in ("run_angle(", "run_target("):
        if call in code and "wait=False" not in code:
            problems.append((
                "this action's code calls {0}() without wait=False, which "
                "blocks the whole robot".format(call[:-1]),
                "add wait=False, or use a sequential 'Move arm' step instead"))

    return problems


def _check_code(action: dict, index: int, diagnostics: list):
    """Syntax-check a custom action's code, and warn on obvious blocking.

    `compile()` only proves the code parses -- it cannot run on a hub that
    is not there, so it cannot know whether the names it calls exist, or
    what the robot actually does. A stub proves what code says, never what
    the platform has; see step 3.1 for the fuller version of this lesson.

    A blank action is not an error: it is one mid-way through being typed,
    the same allowance an unfinished arm step gets. _actions_source in
    hubModule.py turns it into a plain `pass`.
    """
    do = action.get("do")

    if not isinstance(do, dict) or "code" not in do:
        return

    code = str(do.get("code") or "")

    if not code.strip():
        return

    try:
        compile(code, "<action>", "exec")
    except SyntaxError as problem:
        diagnostics.append(Diagnostic.error(
            "this action's code will not run: {0}".format(problem.msg),
            step = index,
            suggestion = ("check the Python -- line {0}".format(problem.lineno)
                         if problem.lineno else None)))
        return   # a syntax error makes the blocking check unreliable

    for message, suggestion in _code_problems(code):
        diagnostics.append(Diagnostic.warning(message, step = index,
                                              suggestion = suggestion))


def _add_action(builder: TrajectoryBuilder, action: dict, index: int,
                diagnostics: list) -> bool:
    """Attach one action to the step just added. True if it took."""
    if not builder.segments:
        diagnostics.append(Diagnostic.warning(
            "an action was dropped, because nothing has moved yet",
            step = index,
            suggestion = "put a move or a turn before it"))
        return False

    at = action.get("at", {})

    # the action's id is handed over where a function would normally go. It is
    # never called here -- the hub binds the real motor code to it later -- but
    # it rides along on the marker, so the order markers come back in is the
    # order the actions fire.
    identifier = action.get("id")

    if "cm" in at:
        builder.addRelativeDisplacementMarker(float(at["cm"]), identifier)
    elif "ms" in at:
        builder.addRelativeTemporalMarker(int(at["ms"]), identifier)
    else:
        diagnostics.append(Diagnostic.error(
            "an action does not say when it should happen",
            step = index,
            suggestion = "give it a distance into the step, or a time"))
        return False

    return True


def _arm_command(step: dict) -> dict:
    """An arm step's motor call, in the shape the generated file needs.

    `wait` is the whole point of a sequential step: the hub blocks here until
    the motor is done, and the follow loop discounts the time.
    """
    return {"motor": step.get("motor", "leftTask"),
            "call": step.get("call", "run_angle"),
            "speed": step.get("speed", 500),
            "angle": step.get("angle"),
            "wait": True}


def _described_step(segment_index, segment_owner):
    """Translate a builder segment number back into a described step number."""
    if segment_index is None:
        return None

    if 0 <= segment_index < len(segment_owner):
        return segment_owner[segment_index]

    return None


def _step_times(builder, segment_owner: list, steps: list) -> list:
    """When each described step runs, in trajectory time.

    The page needs this to say which step is happening at a given moment, and
    to light up one step's share of the path.

    Merged steps are the awkward case, and the two kinds of merge want
    different answers.

    Consecutive drives in one direction become a single acceleration profile.
    The second drive genuinely has no slice to point at -- the robot never
    slows between them -- so it reports the moment the segment finishes as both
    its start and its end, and the row says "joined to the step above".

    Consecutive arm steps also share a segment, because they are waits. But
    they are not one motion: they happen strictly one after another, and the
    segment is already as long as their estimates together. Giving the second
    one zero length would tell a team member that a step the robot genuinely
    waits for is free, so each takes its own share by its own estimate.
    """
    ends = {}

    for segment, owner in enumerate(segment_owner):
        if segment < len(builder.step_ends):
            ends[owner] = builder.step_ends[segment]

    timed = []
    previous = 0

    for index, step in enumerate(steps):
        kind = step.get("type") if isinstance(step, dict) else None

        if index in ends:
            ends_at = ends[index]
        elif kind == "armStep":
            # merged into an earlier wait: take our own share of it
            ends_at = previous + arm_step_ms(step)
        else:
            ends_at = previous

        timed.append({"index": index,
                      "type": kind,
                      "starts_ms": previous,
                      "ends_ms": ends_at})

        previous = ends_at

    return timed


def _label_for(identifier, steps: list) -> str:
    """What the team called this action, for the comment in the file."""
    for step in steps:
        if not isinstance(step, dict):
            continue

        for action in step.get("actions", []):
            if action.get("id") == identifier:
                return action.get("label") or identifier

    return identifier if identifier is not None else "action"


def _hub_module(trajectory, name: str, steps_ms: int, diagnostics: list,
                actions: list = None):
    if trajectory.TIME <= 0:
        return None

    try:
        return trajectory.hub_module(name, steps_ms, actions)

    except ValueError as problem:
        # a speed too large for the hub's format, or a robot that is not a
        # tank drive. Either way there is no file to hand over
        diagnostics.append(Diagnostic.error(
            "this run cannot be sent to the hub: {0}".format(problem)))

        return None


def _poses(trajectory, every_ms: int) -> list:
    states = trajectory.STATES

    if not states:
        return []

    kept = list(states[::max(1, every_ms)])

    if kept[-1] is not states[-1]:
        kept.append(states[-1])

    return [{"t": state.time,
             "x": round(state.pose.x, 2),
             "y": round(state.pose.y, 2),
             "head": round(state.pose.head, 2)}
            for state in kept]


def _nothing_to_drive(name: str, diagnostics: list) -> dict:
    return {"version": VERSION,
            "name": name,
            "ok": False,
            "total_ms": 0,
            "steps": [],
            "poses": [],
            "markers": [],
            "diagnostics": [problem.as_dict() for problem in diagnostics],
            "module_text": None}
