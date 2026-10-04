"""Turn a run into a module the hub can import -- as DriveBase calls.

Step 5.5 of docs/web-planner.md. The recorded format (hubModule.py) ships a
wheel power and a heading every few milliseconds and plays them back against
the clock. This ships what the run *is*: drive this far, turn to face that
way, run the arm. Pybricks' DriveBase does the motion itself, in firmware,
closing the loop on the wheel encoders and the gyro -- which the recorded
format never did for distance -- and the file is a few hundred bytes of
readable Python instead of kilobytes of encoded numbers.

The file keeps step 3.1's shape, so runs.py does not change: the actions are
functions (written by the same code as before, hubModule.action_functions),
and `run(core)` is the only thing runs.py calls. What run() needs from `core`
is the hub side's job, step 5.6:

    core.configure(...)       -> the DriveBase, set up with this run's robot
    core.set_heading(deg)     -> where the gyro should say the robot faces now
    core.turn_to(deg, by)     -> turn to face `deg`, the way `by` planned it
    core.stopTaskMotors()     -> as the recorded format's follow() always did

Everything geometric was already worked out on the PC by driveProgram.py
(step 5.4), so this is only a matter of writing each move as a line.
"""

from pythfinder.Export.driveProgram import compile_drive_program
from pythfinder.Export.hubModule import _call_for, _code_lines, action_functions


# The DriveBase numbers, in the order core.configure() takes them, and the
# name each one has in a robot profile's driveBase (step 5.3).
_DRIVE_BASE_ARGS = ("wheel_diameter_mm", "axle_track_mm", "straight_speed",
                    "straight_acceleration", "turn_rate", "turn_acceleration",
                    "use_gyro")


def _number(value, places: int = 2) -> str:
    """A number as a person would write it: 250, not 250.0; 54.6, not
    54.60000000000001. Rounding here is display only -- driveProgram already
    rounded more finely than any motor can act on."""
    value = round(float(value), places)

    if value == int(value):
        return str(int(value))

    return repr(value)


def _drive_base(robot) -> dict:
    """The run's own DriveBase numbers, if it carries any.

    Only a step-5.3 profile does. "fll_team" and the older bare-numbers shape
    carry none, and inventing some here would mean a file that quietly
    configures a robot nobody measured -- so those get robot.py's own
    defaults instead, and a warning saying so.
    """
    if isinstance(robot, dict) and isinstance(robot.get("driveBase"), dict):
        return robot["driveBase"]

    return None


def _configure_lines(drive_base: dict) -> list:
    """core.configure(...), two numbers to a line -- they come in pairs
    (size, then straight, then turn), and one long line is unreadable in the
    Python view."""
    args = []

    for key in _DRIVE_BASE_ARGS:
        if key not in drive_base:
            continue

        value = drive_base[key]
        name = key[:-3] if key.endswith("_mm") else key

        args.append("{0}={1}".format(
            name, repr(bool(value)) if key == "use_gyro" else _number(value)))

    pairs = [", ".join(args[at:at + 2]) for at in range(0, len(args), 2)]

    return (["    drive = core.configure("] +
            ["        " + pair + "," for pair in pairs[:-1]] +
            ["        " + pairs[-1] + ")"])


def _arm_lines(move: dict) -> list:
    """An arm step: the robot has stopped, and this blocks until the motor is
    done. The same guard as a parallel action -- an attachment that is not
    plugged in is None on the hub, and skipping it beats crashing mid-match.
    A code override is the team's own, and gets no guard, as in step 3.3."""
    label = move.get("label")
    lines = ["", "    # {0}".format(label or "arm step")]

    if move.get("code") is not None:
        lines.extend(_code_lines(move["code"]))
        return lines + [""]

    lines.append("    if core.{0} is not None:".format(move.get("motor", "leftTask")))
    lines.append("        " + _call_for(dict(move, wait = True)))

    return lines + [""]


def _run_lines(moves: list) -> list:
    lines = []
    fired = 0

    for move in moves:
        op = move["op"]

        if op == "straight":
            then = ", then=Stop.NONE" if move["then"] == "none" else ""
            lines.append("    drive.straight({0}{1})".format(_number(move["mm"], 1), then))

        elif op == "turn_to":
            lines.append("    core.turn_to({0}, {1})".format(
                _number(move["deg"]), _number(move["by"])))

        elif op == "wait":
            lines.append("    wait({0})".format(int(move["ms"])))

        elif op == "action":
            fired += 1
            label = move.get("label")
            comment = "        # " + label if label else ""
            lines.append("    _action_{0}(core){1}".format(fired, comment))

        elif op == "settings":
            lines.append("    drive.settings(straight_speed={0})".format(
                _number(move["straight_speed"], 1)))

        elif op == "arm":
            lines.extend(_arm_lines(move))

    return lines


def _flattened(move: dict) -> dict:
    """An action move, in the shape action_functions takes: the `do` body
    alongside the label and id, as build_run's firing order carries it."""
    return dict(move.get("do") or {}, id = move.get("id"), label = move.get("label"))


def drive_module_text(run: dict) -> dict:
    """The hub file for this run, and anything wrong with it.

    Returns {"ok", "module_text", "diagnostics"}, build_run's own shape
    (step 1.6). `module_text` is None when the move list has an error --
    a file that drives somewhere other than the plan is worse than none.
    """
    program = compile_drive_program(run)
    diagnostics = list(program["diagnostics"])

    if not program["ok"]:
        return {"ok": False, "module_text": None, "diagnostics": diagnostics}

    name = run.get("name", "run")
    moves = program["moves"]
    actions = [_flattened(move) for move in moves if move["op"] == "action"]

    robot = run.get("robot")
    drive_base = _drive_base(robot)

    if drive_base is None:
        diagnostics.append({
            "level": "warning",
            "message": "this run has no DriveBase numbers of its own, so the "
                       "hub will use robot.py's defaults",
            "step": None,
            "suggestion": "pick a robot in the Robot picker",
            "time_ms": None})

    ops = {move["op"] for move in moves}

    imports = []

    if "straight" in ops and any(move.get("then") == "none" for move in moves):
        imports.append("from pybricks.parameters import Stop")

    if "wait" in ops:
        imports.append("from pybricks.tools import wait")

    # two blank lines before the first def, whether or not anything was
    # imported -- a run with no waits and no rolling joins imports nothing
    lines = ['"""{0}, generated by PythFinder -- do not edit."""'.format(name)]

    if imports:
        lines.extend([""] + imports)

    lines.extend(["", ""])
    lines.extend(action_functions(actions))

    start = run.get("start", {})

    lines.append("def run(core):")
    lines.append('    """Drive this run. The only thing runs.py needs to call."""')

    if drive_base is not None:
        lines.append("    # {0}".format(robot.get("name") or "robot"))
        lines.extend(_configure_lines(drive_base))
    else:
        lines.append("    drive = core.configure()")

    lines.append("    core.set_heading({0})".format(_number(start.get("head", 0))))
    lines.append("")
    lines.extend(_run_lines(moves))

    # an arm step ends with its own blank line; one is enough
    while lines[-1] == "":
        lines.pop()

    lines.append("")
    lines.append("    core.stopTaskMotors()")

    return {"ok": True,
            "module_text": "\n".join(lines) + "\n",
            "diagnostics": diagnostics}
