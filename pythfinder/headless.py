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
      "module_text": "<the .py file to save and upload to the hub>",
      "builder_source": "<the equivalent TrajectoryBuilder chain, for the desktop tool>"
    }

Nothing here raises for a badly described run: a run a child typed wrong is
ordinary, not exceptional, so problems come back as diagnostics next to the
step that caused them.

Step 5.7: "module_text" is the DriveBase file (driveModule.py, steps 5.4/5.5)
-- straight(), turn_to(), wait() and the run's own actions, a few hundred
bytes of readable Python -- not the older recording of wheel powers every few
milliseconds (hubModule.py). That format stays in the library for a run
already on the hub; this function just stops handing it out. A diagnostic
that belongs to the run as a whole rather than any one step (today, only "no
DriveBase numbers of its own") is folded into "diagnostics" above with
step: None; see build_run's own body for why a *step*-specific one from the
move-list compiler is not folded in the same way.

The run description looks like this:

    {
      "version": 2,
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

"robot" is "fll_team", a bare object of planning numbers (the pre-5.3 shape),
or a named profile with its planning numbers nested under "planning" and a
"driveBase" alongside them -- see robot_from_description. "start" may carry a
"name" too, step 5.3's own addition; neither this function nor anything below
reads it, since it exists only for the browser's own pickers.

See docs/web-planner.md, steps 1.7 and 5.3.
"""

from pythfinder.Components.BetterClasses.mathEx import Point, Pose
from pythfinder.Trajectory.Kinematics.TankKinematics import TankKinematics
from pythfinder.Trajectory.constraints import Constraints2D
from pythfinder.Trajectory.diagnostics import Diagnostic
from pythfinder.Trajectory.robotConfig import FLL_ROBOT, RobotConfig
from pythfinder.Trajectory.trajectoryBuilder import TrajectoryBuilder


VERSION = 2

# how often a pose is kept for drawing. The states are one per millisecond,
# which is far more than a path on a screen can show.
DEFAULT_POSE_EVERY_MS = 20

STEP_TYPES = ("drive", "wait", "turn", "toPoint", "toPose", "armStep")

# A sequential arm step is given at least this long, so that a small sweep
# still reads as a step of its own on the timeline.
LEAST_ARM_MS = 150

# Below this many cm, a turn action's own placement is the same as cm: 0 --
# nothing a person typed on purpose, only float error left by AngularSegment.
TURN_ACTION_CM_EPSILON = 1e-6


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
    """The robot to plan for: the team's, or one described by its numbers.

    Step 5.3 adds a third shape: a named profile, with its planning numbers
    nested under "planning" and a "driveBase" alongside them --

        {"name": "Team robot",
         "planning": {"track_width_cm": 16, "max_velocity_cm_s": 64.3, ...},
         "driveBase": {"wheel_diameter_mm": 56, ...}}

    "driveBase" is read by nothing here: those numbers matter once the hub
    file is generated from a DriveBase (5.5/5.6), not for planning a path
    today. `description.get("planning", description)` is what makes the old,
    flat shape and the new, nested one both work with the same code below --
    a run without a "planning" key is the pre-5.3 shape, and its own numbers
    sit where "planning"'s would otherwise be.
    """
    if description is None or description == "fll_team":
        return FLL_ROBOT.copy()

    if isinstance(description, str):
        raise ValueError("unknown robot '{0}'".format(description))

    numbers = description.get("planning", description)

    track_width = float(numbers["track_width_cm"])
    offset = float(numbers.get("center_offset_cm", 0))

    return RobotConfig(
        kinematics = TankKinematics(track_width, center_offset = Point(offset, 0)),
        constraints = Constraints2D(track_width = track_width),
        real_max_velocity = float(numbers["max_velocity_cm_s"]),
        max_power = float(numbers.get("max_power", 100)),
        width_cm = float(numbers.get("width_cm", 0)),
        length_cm = float(numbers.get("length_cm", 0)))


def pose_from_description(description) -> Pose:
    if description is None:
        return Pose()

    return Pose(x = float(description.get("x", 0)),
                y = float(description.get("y", 0)),
                head = float(description.get("head", 0)))


def build_run(run: dict, pose_every_ms: int = DEFAULT_POSE_EVERY_MS) -> dict:
    """Turn a described run into something to draw, check and download."""
    # Imported here, not at module level: driveProgram.py imports STEP_TYPES,
    # _arm_command and pose_from_description back out of *this* module, so an
    # import at the top would be a cycle that fails while headless.py is
    # still executing its own top-level statements. By the time build_run is
    # actually called, headless.py has finished loading and the cycle
    # resolves fine.
    from pythfinder.Export.driveModule import drive_module_text

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

    # Step 4.5: whether the chain needs to say Constraints/Constraints2D are
    # not among fll_run_template.py's own imports -- only worth the note when
    # a speed limit actually placed one.
    used_speed_limits = False

    # Step 3.4's desktop-tool view, one entry per described step: its own
    # .method(...) line, then one further-indented marker line per action.
    # Built in this same loop rather than a second pass over `steps`, so it
    # can only ever place a marker exactly where _add_action just did -- the
    # `at` computed a line below is the one both of them use.
    chain_blocks = []

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

        block = [_chain_step_line(step)]

        for action in step.get("actions", []):
            _check_code(action, index, diagnostics)

            raw_at = action.get("at")

            if _placed_by_time(raw_at):
                diagnostics.append(Diagnostic.error(
                    "an action can only be placed by distance now, not a time",
                    step = index,
                    suggestion = "give it a distance into the step instead"))
                continue

            if kind == "turn":
                _warn_if_turn_action_off_start(raw_at, index, diagnostics)

            at = _at_within_step(raw_at, step, into_wait, into_line)

            if _add_action(builder, dict(action, at = at), index, diagnostics):
                action_steps[action.get("id")] = index

            marker_line = _chain_marker_line(action, at)

            if marker_line is not None:
                block.append("    " + marker_line)

        for limit in step.get("speedLimits", []):
            chain_lines = _add_speed_limit(builder, step, limit, index, diagnostics,
                                           robot.constraints, into_wait, into_line)
            block.extend("    " + line for line in chain_lines)
            used_speed_limits = used_speed_limits or bool(chain_lines)

        chain_blocks.append(block)

        into_wait += _step_own_ms(step)
        into_line += _step_own_cm(step)

    builder_source = _builder_chain_text(run.get("start") or {}, chain_blocks,
                                         used_speed_limits)

    trajectory = builder.build()

    for problem in trajectory.diagnostics:
        problem.step = _described_step(problem.step, segment_owner)
        diagnostics.append(problem)

    ok = not any(problem.is_error() for problem in diagnostics)

    # Step 5.7: the file to hand over is no longer this trajectory's own
    # recorded-powers rendering (hubModule.py) -- it is the DriveBase move
    # list compiled straight from the run description (driveProgram.py /
    # driveModule.py, steps 5.4/5.5). hubModule.py and the hub's recorded
    # trajectory.py both stay in the library -- a run already on the hub can
    # still import Trajectory, and step 5.8 decides whether that ever goes
    # away -- this function just stops handing that format out.
    drive_result = drive_module_text(run)

    if ok and not drive_result["ok"]:
        # A shape the loop above thought was fine, but the drive compiler
        # alone refuses. Rare -- step 5.2 unified the placement rules the two
        # enforce -- but a hand-edited or pre-5.2 saved run can still reach
        # one, and every diagnostic here is genuinely new, so all of them are
        # kept, against whichever step compile_drive_program itself named.
        new_diagnostics = drive_result["diagnostics"]
    else:
        # Otherwise, fold in only the run-wide ones -- there is only one
        # today, "no DriveBase numbers of its own". A *step*-specific one
        # here would very likely just be the loop above's own problem
        # again, in different words and quite possibly against a different
        # step number: compile_drive_program does not know which steps
        # TrajectoryBuilder merged into one acceleration profile, so it
        # checks each described step's own placement on its own, not the
        # merged segment's -- see docs/web-planner.md, step 5.4. Showing
        # both would tell a team member the same mistake twice, worded two
        # different ways, on two different steps.
        new_diagnostics = [raw for raw in drive_result["diagnostics"]
                           if raw["step"] is None]

    for raw in new_diagnostics:
        diagnostics.append(Diagnostic(level = raw["level"], message = raw["message"],
                                      step = raw["step"], suggestion = raw["suggestion"],
                                      time_ms = raw["time_ms"]))

    ok = ok and drive_result["ok"]
    module_text = drive_result["module_text"] if drive_result["ok"] else None

    return {"version": VERSION,
            "name": name,
            "ok": ok,
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
            # the DriveBase file to save -- see the note above
            "module_text": module_text,
            # step 3.4: the equivalent TrajectoryBuilder chain, for the
            # desktop tool's own simulator -- read-only, not needed to drive
            # the robot
            "builder_source": builder_source}


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


def _placed_by_time(at) -> bool:
    """True for a raw, person-written (or old-saved-run) `at` using "ms".

    Step 5.2: timed triggers are gone from the run format -- an action split
    a drive for the DriveBase (5.4/5.5), and a DriveBase has no notion of "so
    many milliseconds in". Checked against the *raw* value a caller was
    handed, before _at_within_step has a chance to synthesise its own
    internal "ms" (the armStep's implicit marker, a turn's own start) --
    those are never what this is guarding against.
    """
    return isinstance(at, dict) and "ms" in at


def _warn_if_turn_action_off_start(at, index: int, diagnostics: list):
    """A turn covers no distance, so cm: 0 is the only value that means
    anything -- anything else is a step ahead of itself. Still placed (at
    the turn's own start, by _at_within_step), just with a warning rather
    than a silent surprise: see driveProgram.py's _process_turn_actions,
    which reaches the identical rule from the flat move list's own side.
    """
    if not isinstance(at, dict) or "cm" not in at:
        return

    try:
        off_start = abs(float(at["cm"])) > TURN_ACTION_CM_EPSILON
    except (TypeError, ValueError):
        return

    if off_start:
        diagnostics.append(Diagnostic.warning(
            "an action on a turn only ever fires at its start",
            step = index,
            suggestion = "move it to the next step"))


def _at_within_step(at, step: dict, into_wait: int, into_line: float):
    """Move an action's placement from "into this step" to "into this segment".

    Steps merge; markers do not know it. Everything here is the difference
    between the two, and it applies to every action alike -- an earlier version
    offset only the arm step's own implicit marker, which left an explicit one
    on the same step firing inside the step before it.

    A negative value counts back from the end of *this* step, not the merged
    segment's, so it is resolved here against the step's own length rather than
    left to the builder, which knows only the segment.

    Step 5.2 removed `ms` placement from what a person can type -- every
    caller now rejects a raw `at` containing "ms" before this function ever
    sees it. Only two internal uses of "ms" survive, both synthesised here,
    never supplied: the armStep's own implicit marker below, and a turn's
    "cm" action, converted to "ms" because a turn's own *displacement* is a
    razor-thin float sliver an AngularSegment leaves behind (see
    _warn_if_turn_action_off_start) where its *elapsed time* is not.
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

    if "cm" in at and kind == "turn":
        # A turn covers no distance, so the only moment a distance can mean
        # is the turn's own start -- routed through the segment's elapsed
        # time, which AngularSegment tracks properly, rather than its
        # displacement, which does not (see the note above).
        return {"ms": into_wait}

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


# Matches FINISHING_CALLS's own labels in runEditor.ts, so the wording an
# arm step shows in the browser is the wording it shows in this comment.
_ARM_CALL_WORDS = {
    "run_angle": "turn by {angle}° at {speed}°/s",
    "run_target": "turn to {angle}° at {speed}°/s",
    "run_until_stalled": "run at {speed}°/s until it stops",
}


def _chain_step_line(step: dict) -> str:
    """The .method(...) call this step becomes in the desktop-tool chain.

    Named here and nowhere else: the *values* -- offsets, which segment a
    marker lands in -- come from _at_within_step and the accumulators in
    build_run's own loop, the same ones real placement uses. This only says
    which TrajectoryBuilder method the step is.
    """
    kind = step.get("type")

    if kind == "drive":
        return ".inLineCM({0})".format(step.get("cm", 0))

    if kind == "wait":
        return ".wait({0})".format(step.get("ms", 0))

    if kind == "armStep":
        words = _ARM_CALL_WORDS.get(step.get("call", "run_angle"), "{call}").format(
            call = step.get("call", "run_angle"),
            angle = step.get("angle", 0),
            speed = step.get("speed", 500))

        # a step here, not a builder method of its own -- see _add_step
        return ".wait({0})   # {1}: {2} (estimated)".format(
            arm_step_ms(step), step.get("motor", "leftTask"), words)

    reversed_arg = ", reversed = True)" if step.get("reversed") else ")"

    if kind == "turn":
        return ".turnToDeg({0}{1}".format(step.get("deg", 0), reversed_arg)

    if kind == "toPoint":
        return ".toPoint(Point({0}, {1}){2}".format(
            step.get("x", 0), step.get("y", 0), reversed_arg)

    if kind == "toPose":
        return ".toPose(Pose({0}, {1}, {2}){3}".format(
            step.get("x", 0), step.get("y", 0), step.get("head", 0), reversed_arg)

    return "# '{0}' has no equivalent here".format(kind)


def _chain_marker_line(action: dict, at) -> str:
    """The .addRelative...Marker(...) call one action becomes, or None.

    Every action becomes a print() of its own label, whichever kind of action
    it is -- a motor picker or step 3.3's own code. fll_run_template.py says
    up front that a marker only ever runs in the simulator, which has no
    motors to call, so the real code has nowhere to go here; it lives in the
    generated run() instead (see hub_module_code_text).
    """
    if not isinstance(at, dict):
        return None

    label = str(action.get("label") or action.get("id") or "action")
    printed = label.replace("\\", "\\\\").replace('"', '\\"')

    if "cm" in at:
        return '.addRelativeDisplacementMarker({0}, lambda: print("{1}"))'.format(
            at["cm"], printed)

    if "ms" in at:
        return '.addRelativeTemporalMarker({0}, lambda: print("{1}"))'.format(
            at["ms"], printed)

    return None


def _builder_chain_text(start: dict, blocks: list, needs_constraints: bool = False) -> str:
    """The desktop tool's own idiom, built from this run -- step 3.4.

    For pasting into fll_run_template.py's build(sim), replacing "EDIT ME 2".
    Uses the template's own call form, TrajectoryBuilder(sim, Pose, preset),
    rather than the sim-free form build_run itself uses -- this text is meant
    to run in the simulator, not headlessly. The pose is written out in full
    rather than naming START_POSE, so pasting this is correct even when the
    file's own START_POSE is something else.

    needs_constraints (step 4.5): a speed limit's Constraints2D/Constraints
    calls are real, not a print() stand-in -- unlike an action, there is
    nothing about them that only makes sense in the simulator, so they are
    the actual code, verbatim. But fll_run_template.py's own imports do not
    include them (`from pythfinder import Pose, TrajectoryBuilder`), so a
    leading note says what to add rather than leaving a pasted NameError to
    explain itself.
    """
    pose = start or {}
    preamble = "(TrajectoryBuilder(sim, Pose({0}, {1}, {2}), FLL_FIELD)".format(
        pose.get("x", 0), pose.get("y", 0), pose.get("head", 0))

    note = (
        "# needs: from pythfinder import Constraints, Constraints2D\n"
        if needs_constraints else ""
    )

    if not blocks:
        return note + preamble + "\n\n        .build())\n"

    body = "\n\n".join(
        "\n".join("        " + line for line in block)
        for block in blocks)

    return note + preamble + "\n\n" + body + "\n\n        .build())\n"


def _speed_limit_key(at) -> str:
    """"cm" or "ms", or None if `at` does not look like either."""
    if not isinstance(at, dict):
        return None

    if "cm" in at:
        return "cm"

    if "ms" in at:
        return "ms"

    return None


def _chain_speed_limit_lines(resolved_from: dict, resolved_to: dict, cm_s: float) -> list:
    """The two real .addRelativeDisplacementConstraints(...) calls a limit
    becomes -- step 5.2 made cm the only shape a limit's from/to can take, so
    there is only ever the one call to name here.

    Unlike an action's print() stand-in, these are the actual call the run
    itself makes -- a constraint is not simulator-only, so there is nothing
    to substitute. Constraints2D() bare restores the library's own defaults,
    which is what a headless build's default robot already carries too (see
    FLL_ROBOT's own constraints in robotConfig.py).
    """
    call = "addRelativeDisplacementConstraints"

    return [
        ".{0}({1}, Constraints2D(linear = Constraints(vel = {2})))".format(
            call, resolved_from["cm"], cm_s),
        ".{0}({1}, Constraints2D())".format(call, resolved_to["cm"]),
    ]


def _add_speed_limit(builder: TrajectoryBuilder, step: dict, spec: dict, index: int,
                     diagnostics: list, normal_constraints: Constraints2D,
                     into_wait: int, into_line: float) -> list:
    """Place one slow-then-normal pair of constraints markers -- step 4.5.

    Returns the chain lines the two markers become, or an empty list if the
    limit could not be placed at all (an error was recorded instead).

    Two markers, not one: setting a constraint changes the robot's planned
    speed ceiling from that point *forward for the rest of the run*, with no
    automatic reset (see __process_relative_constraints in
    trajectoryBuilder.py, `self.CONSTRAINTS = the_chosen_one.constraints`).
    A "speed limit" reads as a section with a start and an end, so it needs a
    marker that slows down where it starts and one that restores the robot's
    own normal speed where it ends -- both placed through the same
    _at_within_step every action already uses, so a limit on a merged step
    lands exactly where 3.2 already proved an action does.
    """
    kind = step.get("type")

    if kind not in ("drive", "toPoint", "toPose"):
        diagnostics.append(Diagnostic.error(
            "a speed limit only makes sense on a step that drives somewhere",
            step = index,
            suggestion = "move it to a Drive, Go to point, or Go to pose step"))
        return []

    from_at = spec.get("from")
    to_at = spec.get("to")
    from_key = _speed_limit_key(from_at)
    to_key = _speed_limit_key(to_at)

    if from_key == "ms" or to_key == "ms":
        # step 5.2: the same rule an action's `at` follows, applied here --
        # a speed limit's from/to are cm-only now, not "cm, or both ms".
        diagnostics.append(Diagnostic.error(
            "a speed limit can only be placed by distance now, not a time",
            step = index,
            suggestion = "give both ends a distance into the step instead"))
        return []

    if from_key is None or to_key is None:
        diagnostics.append(Diagnostic.error(
            "a speed limit's start and end have to both be a distance into "
            "the step",
            step = index,
            suggestion = "give both a distance in cm"))
        return []

    try:
        cm_s = float(spec["cm_s"])
    except (KeyError, TypeError, ValueError):
        diagnostics.append(Diagnostic.error(
            "a speed limit needs a speed", step = index))
        return []

    if cm_s <= 0:
        diagnostics.append(Diagnostic.error(
            "a speed limit has to be a positive speed",
            step = index,
            suggestion = "pick a speed above 0 cm/s"))
        return []

    resolved_from = _at_within_step(from_at, step, into_wait, into_line)
    resolved_to = _at_within_step(to_at, step, into_wait, into_line)

    if resolved_from["cm"] >= resolved_to["cm"]:
        diagnostics.append(Diagnostic.error(
            "a speed limit has to end after it starts",
            step = index,
            suggestion = "move the end further into the step"))
        return []

    slow = normal_constraints.copy()
    slow.linear.set(vel = cm_s)

    builder.addRelativeDisplacementConstraints(resolved_from["cm"], slow)
    builder.addRelativeDisplacementConstraints(resolved_to["cm"], normal_constraints.copy())

    return _chain_speed_limit_lines(resolved_from, resolved_to, cm_s)


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
            "module_text": None,
            "builder_source": None}
