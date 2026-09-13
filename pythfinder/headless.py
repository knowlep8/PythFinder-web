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
      "code_text": "<the same file, its DATA payload elided -- for reading>",
      "builder_source": "<the equivalent TrajectoryBuilder chain, for the desktop tool>"
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

            at = _at_within_step(action.get("at"), step, into_wait, into_line)

            if _add_action(builder, dict(action, at = at), index, diagnostics):
                action_steps[action.get("id")] = index

                if action.get("do") is not None:
                    action_code[action.get("id")] = action["do"]
                elif kind == "armStep":
                    action_code[action.get("id")] = _arm_command(step)

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

    # in the order they fire, which is the order the hub binds them -- not the
    # order the steps were written
    firing_order = [dict(action_code.get(marker.function, {}),
                         id = marker.function,
                         label = _label_for(marker.function, steps))
                    for marker in trajectory.MARKERS]

    module_text = _hub_module(trajectory, name, steps_ms, diagnostics,
                              firing_order)

    # only when the real file built: nothing legitimate to elide the payload
    # from otherwise, and _hub_module already recorded why
    code_text = (_hub_module_code(trajectory, name, steps_ms, firing_order)
                if module_text is not None else None)

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
            "module_text": module_text,
            # step 3.4: the same file with its payload elided, and the
            # TrajectoryBuilder chain it is equivalent to -- both read-only,
            # neither needed to drive the robot
            "code_text": code_text,
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
    """The two real .addRelative...Constraints(...) calls a limit becomes.

    Unlike an action's print() stand-in, these are the actual call the run
    itself makes -- a constraint is not simulator-only, so there is nothing
    to substitute. Constraints2D() bare restores the library's own defaults,
    which is what a headless build's default robot already carries too (see
    FLL_ROBOT's own constraints in robotConfig.py).
    """
    if "cm" in resolved_from:
        call, start, end = "addRelativeDisplacementConstraints", resolved_from["cm"], resolved_to["cm"]
    else:
        call, start, end = "addRelativeTemporalConstraints", resolved_from["ms"], resolved_to["ms"]

    return [
        ".{0}({1}, Constraints2D(linear = Constraints(vel = {2})))".format(call, start, cm_s),
        ".{0}({1}, Constraints2D())".format(call, end),
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

    if from_key is None or to_key is None or from_key != to_key:
        diagnostics.append(Diagnostic.error(
            "a speed limit's start and end have to both be a distance into "
            "the step, or both a time",
            step = index,
            suggestion = "use cm in for both, or ms for both"))
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

    if resolved_from[from_key] >= resolved_to[to_key]:
        diagnostics.append(Diagnostic.error(
            "a speed limit has to end after it starts",
            step = index,
            suggestion = "move the end further into the step"))
        return []

    slow = normal_constraints.copy()
    slow.linear.set(vel = cm_s)

    if from_key == "cm":
        builder.addRelativeDisplacementConstraints(resolved_from["cm"], slow)
        builder.addRelativeDisplacementConstraints(resolved_to["cm"], normal_constraints.copy())
    else:
        builder.addRelativeTemporalConstraints(resolved_from["ms"], slow)
        builder.addRelativeTemporalConstraints(resolved_to["ms"], normal_constraints.copy())

    return _chain_speed_limit_lines(resolved_from, resolved_to, cm_s)


def _hub_module_code(trajectory, name: str, steps_ms: int, actions: list = None):
    """The learning half of step 3.4 -- see hub_module_code_text."""
    if trajectory.TIME <= 0:
        return None

    try:
        return trajectory.hub_module_code(name, steps_ms, actions)

    except ValueError:
        # _hub_module already recorded the real diagnostic for this trajectory
        return None


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
            "module_text": None,
            "code_text": None,
            "builder_source": None}
