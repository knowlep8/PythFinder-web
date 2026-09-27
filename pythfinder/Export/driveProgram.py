"""Compile a run description into a flat move list for Pybricks' DriveBase.

Step 5.4 of docs/web-planner.md. Nothing the planner can build is a curve --
on a tank drive, PointSegment and PoseSegment force tangent = True and
linear_head = False (docs/web-planner.md, step 1.1's own finding), so every
step is already straight lines and turns in place. `build_run` (headless.py)
turns a run description into a recorded trajectory -- wheel powers and a
heading every few milliseconds, played back against the clock. This module
turns the same description into the *moves themselves*: `straight(mm)`,
`turn_to(deg)`, `wait(ms)`, an attachment action, an arm call, or a
`settings()` speed change. Generating the Python text those moves become is
step 5.5, not this one -- everything here is plain, JSON-shaped dicts.

Pose is tracked here on the PC, by plain trigonometry, in the planner's own
field convention (head 0 faces up the field, increasing clockwise as drawn --
docs/web-planner.md, step 2.3). A `turn_to` move always carries the absolute
heading to turn *to*; turning *by* an amount, and which way to spin to get
there, is the hub's own job -- step 5.6.

No RobotConfig, no TrajectoryBuilder, no acceleration profile: unlike
build_run, this never has to know how fast the robot can go, only where it
ends up. That is the whole point of handing motion to the DriveBase -- the
firmware picks the speeds; the planner only has to describe the shape.

`build_run` is the reference this is checked against: tests/test_drive_program.py
builds the same run both ways and compares the end pose and the order actions
fire in.
"""

import math

from pythfinder.Components.BetterClasses.mathEx import normalize_degres
from pythfinder.Trajectory.diagnostics import Diagnostic
from pythfinder.headless import STEP_TYPES, _arm_command, pose_from_description


# Below this many mm / ms / degrees, two numbers are the same number: nothing
# a robot could act on, only the float error atan2 and hypot leave behind.
MM_EPSILON = 1e-6
CM_EPSILON = 1e-6
MS_EPSILON = 1e-6
HEADING_EPSILON_DEG = 1e-6

# mm/s a run drives at once nothing overrides it. Read from the robot's own
# DriveBase numbers when it has them (step 5.3's nested "driveBase" shape);
# this is the fallback for a run whose "robot" is "fll_team", the pre-5.3
# bare-numbers shape, or a named profile saved before 5.3 -- none of which
# carry a straight_speed to read. It is the placeholder team robot's own
# number (web/src/profiles.ts, TEAM_ROBOT.driveBase.straight_speed), itself
# not yet a measurement -- see that file's own comment.
DEFAULT_STRAIGHT_SPEED_MM_S = 200.0


def compile_drive_program(run: dict) -> dict:
    """Turn a described run into a flat move list, plus what went wrong.

    Takes the same run description build_run takes. Returns

        {"ok": bool, "moves": [...], "diagnostics": [...], "end_pose": {...}}

    -- the diagnostics-as-data shape build_run itself returns (step 1.6),
    because most of what can go wrong here is an ordinary planning mistake,
    not something to raise an exception for. "moves" is the payload, a flat
    list of plain dicts:

        {"op": "straight", "mm": 250.0, "then": "none" | "stop"}
        {"op": "turn_to", "deg": 90.0}
        {"op": "wait", "ms": 600}
        {"op": "action", "id": "a1", "do": {...}, "label": "Left arm down"}
        {"op": "arm", "motor": "leftTask", "call": "run_angle",
         "speed": 500, "angle": 90, "wait": True, "label": None}
        {"op": "settings", "straight_speed": 200.0}

    An action move carries its own "do" and "label" -- unlike a build_run
    marker, which is only ever an id, leaving the hub file (build_run's own
    caller) to look the rest up afterwards -- so this list is everything
    step 5.5 needs to generate code, with no second walk over the original
    run.

    "end_pose" is the planned pose this run finishes at -- not part of the
    plan's own move shape, but cheap to return and exactly what
    tests/test_drive_program.py checks against build_run's own end pose.

    One thing this deliberately does not do, that build_run does: an action
    or a speed limit placed in *time* on a step that moves the robot (drive,
    turn, toPoint, toPose) is refused with an error diagnostic rather than
    guessed at. Step 5.2 removes time placement from the format entirely; it
    is not done yet, so an old saved run can still describe one, and the
    honest answer here is "not supported", not a number made up to fit. Time
    placement inside a wait or an arm step is unaffected -- see _emit_wait.
    """
    diagnostics = []
    moves = _Moves()

    start = pose_from_description(run.get("start"))
    x, y, head = start.x, start.y, normalize_degres(start.head)

    steps = run.get("steps", [])

    if not steps:
        diagnostics.append(Diagnostic.error(
            "this run has no steps in it, so there is nothing to drive",
            suggestion = "add a move, a turn or a wait").as_dict())

        return {"ok": False, "moves": [], "diagnostics": diagnostics,
                "end_pose": {"x": x, "y": y, "head": head}}

    normal_speed = _normal_straight_speed_mm_s(run.get("robot"))

    for index, step in enumerate(steps):
        if not isinstance(step, dict):
            diagnostics.append(Diagnostic.error(
                "this step is not something the robot knows how to do",
                step = index).as_dict())
            continue

        kind = step.get("type")

        try:
            if kind == "drive":
                cm = float(step["cm"])
                sign = -1 if cm < 0 else 1

                _emit_leg(moves, diagnostics, step, index, abs(cm), sign,
                         head, normal_speed)

                rad = math.radians(head)
                x += cm * math.cos(rad)
                y += cm * math.sin(rad)

            elif kind == "turn":
                # 'reversed' only ever picks which way the hub spins to get
                # there (5.6) -- a turn always ends up facing 'deg', however
                # it gets there, so the move list only ever needs the target
                target = normalize_degres(float(step["deg"]))

                _reject_speed_limits(diagnostics, step, index)
                _process_turn_actions(moves, diagnostics, step, index)

                if not _headings_match(head, target):
                    moves.turn_to(target)

                head = target

            elif kind == "wait":
                ms_len = int(step["ms"])

                _reject_speed_limits(diagnostics, step, index)
                _emit_wait(moves, diagnostics, step, index, ms_len)

            elif kind == "armStep":
                _reject_speed_limits(diagnostics, step, index)
                _emit_arm(moves, diagnostics, step, index)

            elif kind == "toPoint":
                target_x, target_y = float(step["x"]), float(step["y"])
                reversed_ = bool(step.get("reversed", False))

                head = _turn_toward(moves, x, y, target_x, target_y, head,
                                    reversed_)

                distance = math.hypot(target_x - x, target_y - y)
                sign = -1 if reversed_ else 1

                _emit_leg(moves, diagnostics, step, index, distance, sign,
                         head, normal_speed)

                x, y = target_x, target_y

            elif kind == "toPose":
                target_x, target_y = float(step["x"]), float(step["y"])
                final_head = normalize_degres(float(step.get("head", 0)))
                reversed_ = bool(step.get("reversed", False))

                head = _turn_toward(moves, x, y, target_x, target_y, head,
                                    reversed_)

                distance = math.hypot(target_x - x, target_y - y)
                sign = -1 if reversed_ else 1

                _emit_leg(moves, diagnostics, step, index, distance, sign,
                         head, normal_speed)

                x, y = target_x, target_y

                if not _headings_match(head, final_head):
                    moves.turn_to(final_head)

                head = final_head

            else:
                diagnostics.append(Diagnostic.error(
                    "'{0}' is not something the robot knows how to do".format(kind),
                    step = index,
                    suggestion = "use one of: {0}".format(", ".join(STEP_TYPES))
                ).as_dict())

        except (KeyError, TypeError, ValueError) as problem:
            diagnostics.append(Diagnostic.error(
                "this step is missing something, or has the wrong kind of "
                "value: {0}".format(problem),
                step = index).as_dict())

    ok = not any(problem["level"] == Diagnostic.ERROR for problem in diagnostics)

    return {"ok": ok,
            "moves": moves.list,
            "diagnostics": diagnostics,
            "end_pose": {"x": x, "y": y, "head": head}}


class _Moves:
    """Accumulates the flat move list, and decides `then` for each straight.

    A straight gets then = "none" when it flows directly out of the one
    before it -- the same heading, the same direction of travel -- so the
    two read as one continuous roll to the hub, exactly as the planner's own
    LinearSegment merging already treats them (trajectoryBuilder.py's
    __is_eligible_to_combine_linear). Anything else -- a turn, a wait, an arm
    call, or the end of the run -- gets "stop", because the planned
    trajectory itself comes to rest there too. An action or a settings()
    change between two straights does not break that continuity; only a real
    turn, wait or arm call does.
    """

    def __init__(self):
        self.list = []

        # the most recently appended straight move, and the heading/sign it
        # travelled at, so the next one can tell whether it flows out of it.
        # None once something has stopped the robot.
        self._open_move = None
        self._open_heading = None
        self._open_sign = None

    def straight(self, mm: float, heading_deg: float):
        if abs(mm) < MM_EPSILON:
            return

        sign = 1 if mm > 0 else -1

        if (self._open_move is not None and self._open_sign == sign
                and _headings_match(self._open_heading, heading_deg)):
            self._open_move["then"] = "none"

        move = {"op": "straight", "mm": round(mm, 3), "then": "stop"}
        self.list.append(move)

        self._open_move = move
        self._open_heading = heading_deg
        self._open_sign = sign

    def turn_to(self, deg: float):
        self.list.append({"op": "turn_to", "deg": round(deg, 4)})
        self.stop()

    def wait(self, ms: float):
        if ms > MS_EPSILON:
            self.list.append({"op": "wait", "ms": int(round(ms))})

    def action(self, description: dict):
        self.list.append({"op": "action",
                          "id": description.get("id"),
                          "do": description.get("do"),
                          "label": description.get("label")})

    def arm(self, command: dict):
        self.list.append(dict(command, op = "arm"))
        self.stop()

    def settings(self, straight_speed_mm_s: float):
        self.list.append({"op": "settings",
                          "straight_speed": round(straight_speed_mm_s, 3)})

    def stop(self):
        """The robot has genuinely come to rest -- a turn, a wait, or an arm
        call -- so whatever straight comes next cannot flow out of the last
        one, whatever heading and direction it happens to share with it."""
        self._open_move = None
        self._open_heading = None
        self._open_sign = None


def _headings_match(a: float, b: float, eps: float = HEADING_EPSILON_DEG) -> bool:
    diff = abs(normalize_degres(a - b))
    return diff <= eps or diff >= 360 - eps


def _normal_straight_speed_mm_s(description) -> float:
    """See DEFAULT_STRAIGHT_SPEED_MM_S for which runs fall back to it."""
    if isinstance(description, dict):
        drive_base = description.get("driveBase")

        if isinstance(drive_base, dict):
            try:
                return float(drive_base["straight_speed"])
            except (KeyError, TypeError, ValueError):
                pass

    return DEFAULT_STRAIGHT_SPEED_MM_S


def _tangent_face(x: float, y: float, target_x: float, target_y: float,
                  current_head: float) -> float:
    """The heading a straight line from here to the target points along --
    pointSegment.py's own `head = atan2(dy, dx)`, the same tangent a tank
    drive is always forced onto (docs/web-planner.md, step 1.1). If the two
    points coincide there is no line to point along, so the robot keeps
    whatever heading it already has, the same as an untouched AngularSegment
    would."""
    dx, dy = target_x - x, target_y - y

    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return current_head

    return normalize_degres(math.degrees(math.atan2(dy, dx)))


def _turn_toward(moves: _Moves, x: float, y: float, target_x: float,
                 target_y: float, head: float, reversed_: bool) -> float:
    """toPoint/toPose's first turn: face the target, or its mirror image if
    driving there backwards -- pointSegment.py's own `head = tangent; if
    reversed: head = 180 + head`. Returns the heading afterwards, whether or
    not a turn_to move was actually needed to reach it."""
    face = _tangent_face(x, y, target_x, target_y, head)

    if reversed_:
        face = normalize_degres(180 + face)

    if not _headings_match(head, face):
        moves.turn_to(face)

    return face


def _reject_speed_limits(diagnostics: list, step: dict, index: int):
    """A speed limit touches linear speed only -- offered on a turn, it
    would look like it did something and silently not (see 4.5's own note,
    unchanged here)."""
    if step.get("speedLimits"):
        diagnostics.append(Diagnostic.error(
            "a speed limit only makes sense on a step that drives somewhere",
            step = index,
            suggestion = "move it to a Drive, Go to point, or Go to pose step"
        ).as_dict())


def _emit_leg(moves: _Moves, diagnostics: list, step: dict, index: int,
             leg_len_cm: float, sign: int, heading_deg: float,
             normal_speed_mm_s: float):
    """One straight leg -- a whole `drive` step, or the straight part of a
    `toPoint`/`toPose` -- split at every action and speed-limit edge that
    falls on it.

    leg_len_cm is always positive, and is what a negative "cm" ("from the
    end") resolves against -- the same rule _at_within_step applies in
    headless.py, simplified because a leg here has no merged builder segment
    to worry about: this walks the description directly, one leg at a time,
    rather than through TrajectoryBuilder's own combined profiles. Actions on
    toPoint/toPose measure "cm" along this same straight part, which is why
    they can share this one function with a plain drive.
    """
    cuts = []   # (position_cm, order, kind, payload); order breaks ties

    for order, action in enumerate(step.get("actions", [])):
        at = action.get("at") or {}

        if "ms" in at:
            diagnostics.append(Diagnostic.error(
                "an action placed in time on a moving step is not "
                "supported here",
                step = index,
                suggestion = "give it a distance into the step instead"
            ).as_dict())
            continue

        if "cm" not in at:
            diagnostics.append(Diagnostic.error(
                "an action does not say when it should happen",
                step = index).as_dict())
            continue

        value = float(at["cm"])
        position = leg_len_cm + value if value < 0 else value

        if position < -CM_EPSILON or position > leg_len_cm + CM_EPSILON:
            diagnostics.append(Diagnostic.warning(
                "an action {0}cm into this step was dropped, because the "
                "step only goes as far as {1}cm"
                .format(round(value, 2), round(leg_len_cm, 2)),
                step = index,
                suggestion = "put it before {0}cm, or make the step longer"
                             .format(round(leg_len_cm, 2))).as_dict())
            continue

        cuts.append((min(max(position, 0.0), leg_len_cm), order, "action", action))

    for order, limit in enumerate(step.get("speedLimits", [])):
        from_at = limit.get("from") or {}
        to_at = limit.get("to") or {}

        if "cm" not in from_at or "cm" not in to_at:
            diagnostics.append(Diagnostic.error(
                "a speed limit's start and end have to both be a distance "
                "into the step",
                step = index).as_dict())
            continue

        try:
            cm_s = float(limit["cm_s"])
        except (KeyError, TypeError, ValueError):
            diagnostics.append(Diagnostic.error(
                "a speed limit needs a speed", step = index).as_dict())
            continue

        if cm_s <= 0:
            diagnostics.append(Diagnostic.error(
                "a speed limit has to be a positive speed",
                step = index,
                suggestion = "pick a speed above 0 cm/s").as_dict())
            continue

        from_value, to_value = float(from_at["cm"]), float(to_at["cm"])
        from_pos = leg_len_cm + from_value if from_value < 0 else from_value
        to_pos = leg_len_cm + to_value if to_value < 0 else to_value

        if from_pos >= to_pos:
            diagnostics.append(Diagnostic.error(
                "a speed limit has to end after it starts",
                step = index,
                suggestion = "move the end further into the step").as_dict())
            continue

        from_pos = min(max(from_pos, 0.0), leg_len_cm)
        to_pos = min(max(to_pos, 0.0), leg_len_cm)

        cuts.append((from_pos, order, "limit_start", cm_s))
        cuts.append((to_pos, order, "limit_end", None))

    cuts.sort(key = lambda cut: (cut[0], cut[1]))

    cursor = 0.0

    for position, _order, kind, payload in cuts:
        moves.straight((position - cursor) * sign * 10, heading_deg)
        cursor = position

        if kind == "action":
            moves.action(payload)
        elif kind == "limit_start":
            moves.settings(payload * 10)   # cm/s -> mm/s
        else:
            moves.settings(normal_speed_mm_s)

    moves.straight((leg_len_cm - cursor) * sign * 10, heading_deg)


def _emit_wait(moves: _Moves, diagnostics: list, step: dict, index: int,
               ms_len: int):
    """Split a wait step's own length around every action on it: wait(a),
    action, wait(b), for however many actions fall inside it -- the "trivial"
    half of step 5.4's own note on timed placement. armStep does not need
    this: see _emit_arm."""
    cuts = []

    for order, action in enumerate(step.get("actions", [])):
        at = action.get("at") or {}

        if "ms" not in at:
            diagnostics.append(Diagnostic.error(
                "an action does not say when it should happen",
                step = index).as_dict())
            continue

        value = int(at["ms"])
        position = ms_len + value if value < 0 else value

        if position < -MS_EPSILON or position > ms_len + MS_EPSILON:
            diagnostics.append(Diagnostic.warning(
                "an action {0}ms into this step was dropped, because the "
                "step only goes as far as {1}ms".format(value, ms_len),
                step = index,
                suggestion = "put it before {0}ms, or make the step longer"
                             .format(ms_len)).as_dict())
            continue

        cuts.append((min(max(position, 0), ms_len), order, action))

    cuts.sort(key = lambda cut: (cut[0], cut[1]))

    cursor = 0

    for position, _order, action in cuts:
        moves.wait(position - cursor)
        moves.action(action)
        cursor = position

    moves.wait(ms_len - cursor)

    if ms_len > 0:
        moves.stop()


def _process_turn_actions(moves: _Moves, diagnostics: list, step: dict, index: int):
    """A turn covers no distance, so the only moment an action on one can
    mean is its very start, before any turning happens -- cm: 0. Anything
    else is a step ahead of itself, and gets a warning rather than being
    silently moved (5.2's own rule, applied here ahead of that step landing).
    """
    for action in step.get("actions", []):
        at = action.get("at") or {}

        if "ms" in at:
            diagnostics.append(Diagnostic.error(
                "an action placed in time is not supported here",
                step = index,
                suggestion = "give it a distance into the step instead"
            ).as_dict())
            continue

        if "cm" not in at:
            diagnostics.append(Diagnostic.error(
                "an action does not say when it should happen",
                step = index).as_dict())
            continue

        if abs(float(at["cm"])) > CM_EPSILON:
            diagnostics.append(Diagnostic.warning(
                "an action on a turn only ever fires at its start",
                step = index,
                suggestion = "move it to the next step").as_dict())

        moves.action(action)


def _emit_arm(moves: _Moves, diagnostics: list, step: dict, index: int):
    """An arm step: the robot is already stopped, and the motor's own call
    blocks (wait=True) for as long as it takes -- see docs/web-planner.md's
    "why dropping timed triggers matters". The wait segment and the implicit
    marker the old recorded format needed for this are both gone: step 5.2's
    own note already called that marker "internal ... and goes away in 5.4
    anyway", and there is nothing left here for a marker to be relative to.

    An armStep's "actions" list only ever held that implicit marker (an
    id and nothing else) -- but if it carries an explicit "do", that is a
    deliberate override of the step's own motor call (the same precedence
    _arm_command's caller in headless.py gives it), kept here for whoever
    relies on it. More than one action, or one with its own "at", cannot be
    placed against a single blocking call, and is reported rather than
    guessed at.
    """
    explicit = step.get("actions", [])
    command = _arm_command(step)
    label = step.get("label")

    if explicit:
        first = explicit[0]

        if first.get("do") is not None:
            override = dict(first["do"])

            if "code" not in override:
                override["wait"] = True

            command = override

        label = first.get("label") or label

        if len(explicit) > 1 or first.get("at") is not None:
            diagnostics.append(Diagnostic.warning(
                "an arm step only ever does the one thing it already "
                "implies -- anything else attached to it is ignored",
                step = index).as_dict())

    moves.arm(dict(command, label = label))
