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

STEP_TYPES = ("drive", "wait", "turn", "toPoint", "toPose")


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

    for index, step in enumerate(steps):
        _add_step(builder, step, index, diagnostics)

        while len(segment_owner) < len(builder.segments):
            segment_owner.append(index)

        for action in step.get("actions", []):
            if _add_action(builder, action, index, diagnostics):
                action_steps[action.get("id")] = index

    trajectory = builder.build()

    for problem in trajectory.diagnostics:
        problem.step = _described_step(problem.step, segment_owner)
        diagnostics.append(problem)

    module_text = _hub_module(trajectory, name, steps_ms, diagnostics)

    return {"version": VERSION,
            "name": name,
            "ok": not any(problem.is_error() for problem in diagnostics),
            "total_ms": trajectory.TIME,
            "poses": _poses(trajectory, pose_every_ms),
            # already in the order they fire, which is the order the hub
            # expects the actions to be bound in
            "markers": [{"id": marker.function,
                         "step": action_steps.get(marker.function),
                         "time_ms": marker.time}
                        for marker in trajectory.MARKERS],
            "diagnostics": [problem.as_dict() for problem in diagnostics],
            "module_text": module_text}


def _add_step(builder: TrajectoryBuilder, step: dict, index: int, diagnostics: list):
    kind = step.get("type")

    try:
        if kind == "drive":
            builder.inLineCM(float(step["cm"]))

        elif kind == "wait":
            builder.wait(int(step["ms"]))

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


def _described_step(segment_index, segment_owner):
    """Translate a builder segment number back into a described step number."""
    if segment_index is None:
        return None

    if 0 <= segment_index < len(segment_owner):
        return segment_owner[segment_index]

    return None


def _hub_module(trajectory, name: str, steps_ms: int, diagnostics: list):
    if trajectory.TIME <= 0:
        return None

    try:
        return trajectory.hub_module(name, steps_ms)

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
            "poses": [],
            "markers": [],
            "diagnostics": [problem.as_dict() for problem in diagnostics],
            "module_text": None}
