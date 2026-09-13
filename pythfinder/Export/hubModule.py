"""Turn a trajectory into a module the hub can import.

The SPIKE Prime hub has no filesystem, so a program cannot open() a trajectory
while it is running. The data has to arrive as a Python module that Pybricks
bundles alongside the program at download time.

That conversion has lived in the quick-start repo as tools/txt_to_py.py, a
second step the team runs by hand after exporting a .txt. This does the same
job in one step, straight from the trajectory, which is what lets the web
planner hand somebody a finished file to download.

The output is deliberately identical to what txt_to_py.py produces, apart from
the first line saying where it came from -- tests/test_hub_module.py checks
that against the module currently on the hub.

Format:
    STEPS   int, ms per state
    MARKERS tuple[int], marker timestamps in ms
    COUNT   int, number of expanded states
    DATA    bytes, COUNT * 3 little-endian int16:
            left * 100, right * 100, heading * 10

DATA is one bytes constant rather than a list of numbers on purpose: a list of
several thousand ints would be compiled into bytecode and boxed individually on
the hub's small heap, while a bytes literal stays one compact object that
struct.unpack_from can index in place.
"""

import struct


# int16 range; powers scale by 100 (0.01% resolution), heading by 10 (0.1 deg).
POWER_SCALE = 100
HEAD_SCALE = 10
INT16_MIN, INT16_MAX = -32768, 32767

STATE_FORMAT = "<hhh"
BYTES_PER_LINE = 24


def _clamp(value: int, name: str, source: str) -> int:
    if not INT16_MIN <= value <= INT16_MAX:
        raise ValueError(
            "{0}: scaled {1} value {2} does not fit in an int16".format(
                source, name, value))
    return value


def _bytes_literal(payload, per_line: int = BYTES_PER_LINE) -> str:
    """Render the payload as chunked, implicitly concatenated bytes literals."""
    chunks = []

    for start in range(0, len(payload), per_line):
        chunks.append("    " + repr(bytes(payload[start:start + per_line])))

    return "\n".join(chunks)


def hub_payload(generator, name: str = "trajectory", steps: int = 1):
    """Pack the run into bytes, two drive motors at a time.

    The powers are rounded to two decimals before being scaled, because that is
    what the .txt export writes and therefore what the hub has always been fed.
    Skipping that rounding would shift the occasional value by one unit.
    """
    payload = bytearray()
    count = 0

    for wheel_states, head, copies in generator.wheel_speed_groups(steps):
        if not len(wheel_states) == 2:
            raise ValueError(
                "{0}: the hub format carries two drive motors, but this robot "
                "has {1} wheels. Only a tank drive can be exported this way."
                .format(name, len(wheel_states)))

        powers = [round(generator.robot.to_motor_power(state.VELOCITY), 2)
                  for state in wheel_states]

        left = _clamp(round(powers[0] * POWER_SCALE), "left", name)
        right = _clamp(round(powers[1] * POWER_SCALE), "right", name)
        heading = _clamp(round(round(head, 2) * HEAD_SCALE), "heading", name)

        payload += struct.pack(STATE_FORMAT, left, right, heading) * copies
        count += copies

    return payload, count


def _call_for(action: dict) -> str:
    """One motor command, as the line of Python the hub will run.

    `wait` is what separates the two kinds of action. A sequential arm step
    blocks here until the motor is done -- the follow loop discounts the time,
    so the path is not skipped. A parallel action must never block, because it
    fires while the robot is driving.
    """
    motor = action.get("motor", "leftTask")
    call = action.get("call", "run")
    speed = action.get("speed", 500)
    angle = action.get("angle")
    blocking = bool(action.get("wait", False))

    if call in ("run", "run_time"):
        return "core.{0}.{1}({2})".format(motor, call, speed)

    if call == "run_until_stalled":
        return "core.{0}.run_until_stalled({1})".format(motor, speed)

    if call in ("stop", "brake", "hold"):
        return "core.{0}.{1}()".format(motor, call)

    # run_angle / run_target: the ones that finish
    return "core.{0}.{1}({2}, {3}, wait={4})".format(
        motor, call, speed, angle if angle is not None else 0, blocking)


def _actions_source(actions: list) -> str:
    """The action functions, and the tuple that binds them in firing order."""
    if not actions:
        return ""

    lines = ["", ""]

    for number, action in enumerate(actions, start = 1):
        label = action.get("label") or action.get("id") or "action"

        lines.append("def _action_{0}(core):".format(number))
        lines.append('    """{0}"""'.format(label))

        if action.get("motor") is None:
            lines.append("    pass        # nothing bound to this one yet")
        else:
            # guard: an attachment that is not plugged in is None on the hub
            lines.append("    if core.{0} is not None:".format(action["motor"]))
            lines.append("        " + _call_for(action))

        lines.append("")
        lines.append("")

    lines.append("def run(core):")
    lines.append('    """Drive this run. The only thing runs.py needs to call."""')
    lines.append("    trajectory = Trajectory.fromValues(STEPS, MARKERS, COUNT, DATA)")
    lines.append("")
    lines.append("    # bound in the order they FIRE, which is not necessarily")
    lines.append("    # the order the steps were written")
    lines.append("    trajectory.withMarkers((")

    for number in range(1, len(actions) + 1):
        lines.append("        lambda: _action_{0}(core),".format(number))

    lines.append("    ))")
    lines.append("")
    lines.append("    trajectory.follow(core)")

    return "\n".join(lines) + "\n"


def hub_module_text(generator, name: str = "trajectory", steps: int = 1,
                    actions: list = None) -> str:
    """The text of a module the hub can import, ready to be saved.

    With `actions` -- one per marker, in firing order -- the file also carries
    the attachment motor code and a run() of its own, so runs.py needs only
    `import run_a` and `run_a.run`.
    """
    markers = tuple(int(marker.time) for marker in generator.MARKERS)
    payload, count = hub_payload(generator, name, steps)

    heading = '"""{0}, generated by PythFinder -- do not edit."""\n'.format(name)

    if actions:
        heading += "\nfrom trajectory import Trajectory\n"

    return (
        heading +
        "\n"
        "STEPS = {0}\n"
        "MARKERS = {1}\n"
        "COUNT = {2}\n"
        "\n"
        "# {2} states * 3 little-endian int16 (left*100, right*100, head*10)\n"
        "DATA = (\n{3}\n)\n"
    ).format(steps, markers, count, _bytes_literal(payload)) + _actions_source(
        actions or [])
