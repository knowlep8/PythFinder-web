"""Step 3.4: two read-only views of what a run already computed.

`builder_source` is the TrajectoryBuilder chain this run is equivalent to, for
pasting into fll_run_template.py's own build(sim). `code_text` is the same
file the hub gets, with its DATA payload -- thousands of bytes nobody can
read -- elided.

The one property that actually matters for `builder_source`: it has to be
*runnable*, not just plausible-looking text. A chain that reads correctly but
places a marker in the wrong segment once pasted would reproduce the exact bug
step 3.2 spent a session fixing, just one layer further from the tests that
would catch it. So the tests below don't just check the text -- they exec()
it, through the real TrajectoryBuilder, and compare what it built against what
build_run itself built for the identical steps.
"""

from pythfinder import Point, Pose, TrajectoryBuilder
from pythfinder.Trajectory.robotConfig import FLL_ROBOT
from pythfinder.headless import build_run


def build(steps, start=None):
    return build_run({
        "version": 1,
        "name": "chain_check",
        "steps_ms": 6,
        "robot": "fll_team",
        "start": start or {"x": 0, "y": 0, "head": 0},
        "steps": steps,
    })


def run_chain(builder_source: str):
    """Actually execute the chain, the way a team member pasting it would.

    The displayed text uses the desktop form -- TrajectoryBuilder(sim, Pose,
    preset) -- because it is meant to run inside fll_run_template.py's own
    simulator. Swapped here for the sim-free form so the test needs no
    pygame window, exactly the substitution step 1.2 made this library
    support in the first place.
    """
    headless = (builder_source
                .replace("TrajectoryBuilder(sim, ", "TrajectoryBuilder(")
                .replace(", FLL_FIELD)", ", robot=FLL_ROBOT)"))

    namespace = {
        "TrajectoryBuilder": TrajectoryBuilder,
        "Pose": Pose,
        "Point": Point,
        "FLL_ROBOT": FLL_ROBOT,
        "print": lambda *args: None,   # the markers' own lambdas call this
    }

    exec("_trajectory = " + headless, namespace)

    return namespace["_trajectory"]


def test_builder_source_is_present_and_runs():
    result = build([{"type": "drive", "cm": 40}])

    assert result["builder_source"] is not None
    assert "TrajectoryBuilder(sim," in result["builder_source"]
    assert ".inLineCM(40)" in result["builder_source"]

    trajectory = run_chain(result["builder_source"])
    assert trajectory.TIME == result["total_ms"]


def test_the_chain_matches_build_run_on_the_template_run():
    """The run every team member starts from, not a toy case."""
    result = build([
        {"type": "drive", "cm": 75, "actions": [
            {"id": "a1", "at": {"cm": 35}, "label": "Left arm down",
             "do": {"motor": "leftTask", "call": "run", "speed": 500}}]},
        {"type": "wait", "ms": 600},
        {"type": "turn", "deg": 90},
        {"type": "drive", "cm": 30, "actions": [
            {"id": "a2", "at": {"ms": -1}, "label": "Left arm up",
             "do": {"motor": "leftTask", "call": "run", "speed": -500}}]},
        {"type": "toPose", "x": 0, "y": 0, "head": 0},
    ], start={"x": -46, "y": -83, "head": 0})

    assert result["ok"], result["diagnostics"]

    trajectory = run_chain(result["builder_source"])

    assert trajectory.TIME == result["total_ms"]
    assert sorted(m.time for m in trajectory.MARKERS) == sorted(
        m["time_ms"] for m in result["markers"])


def test_a_merged_drive_places_its_marker_in_the_right_segment():
    """The exact case step 3.2 fixed: an action on the second of two merged
    drives must land relative to the *segment*, not the step, once the real
    builder merges them -- which is what pasting this chain and running it
    for real would do."""
    result = build([
        {"type": "drive", "cm": 30},
        {"type": "drive", "cm": 30, "actions": [
            {"id": "a1", "at": {"cm": 15}, "label": "Grab",
             "do": {"motor": "leftTask", "call": "run", "speed": 500}}]},
    ])

    assert result["ok"], result["diagnostics"]

    trajectory = run_chain(result["builder_source"])

    assert trajectory.TIME == result["total_ms"]
    assert [m.time for m in trajectory.MARKERS] == [
        m["time_ms"] for m in result["markers"]]


def test_two_arm_steps_in_the_chain_still_fire_one_after_the_other():
    result = build([
        {"type": "drive", "cm": 20},
        {"type": "armStep", "motor": "leftTask", "call": "run_angle",
         "angle": 90, "speed": 500, "actions": [{"id": "down"}]},
        {"type": "armStep", "motor": "leftTask", "call": "run_angle",
         "angle": -90, "speed": 500, "actions": [{"id": "up"}]},
    ])

    assert result["ok"], result["diagnostics"]
    assert ".wait(" in result["builder_source"]
    assert "turn by 90° at 500°/s" in result["builder_source"]
    assert "turn by -90° at 500°/s" in result["builder_source"]

    trajectory = run_chain(result["builder_source"])

    assert trajectory.TIME == result["total_ms"]
    assert [m.time for m in trajectory.MARKERS] == [
        m["time_ms"] for m in result["markers"]]


def test_a_quote_in_a_label_does_not_break_the_chain():
    """The label becomes a Python string literal -- it has to survive one."""
    result = build([
        {"type": "drive", "cm": 30, "actions": [
            {"id": "a1", "at": {"cm": 0}, "label": 'the "big" arm',
             "do": {"motor": "leftTask", "call": "run", "speed": 500}}]},
    ])

    assert result["ok"], result["diagnostics"]
    # must not raise -- a naive f-string embed would produce invalid Python
    run_chain(result["builder_source"])


def test_an_empty_run_still_has_a_pasteable_chain():
    result = build([])

    assert result["builder_source"] is not None
    trajectory = run_chain(result["builder_source"])
    assert trajectory.TIME == 0


def test_code_text_elides_the_payload_but_keeps_everything_else():
    result = build([{"type": "drive", "cm": 40, "actions": [
        {"id": "a1", "at": {"cm": 0}, "label": "Grab",
         "do": {"motor": "leftTask", "call": "run", "speed": 500}}]}])

    assert result["ok"], result["diagnostics"]

    module = result["module_text"]
    code = result["code_text"]

    assert code is not None
    # the structural "DATA = (...)" wrapper is deliberately kept -- only the
    # bytes literal inside it is elided
    assert "DATA = (\n" in code
    assert "b'" not in code, "the payload bytes themselves must not be there"
    assert "elided" in code
    assert "def _action_1(core):" in code
    assert "def run(core):" in code

    # everything that is not the payload must match the real file exactly --
    # this can never drift, because both come from the same _module_text.
    # rsplit rather than split: the closing "\n)\n" is the template's own,
    # and only the last occurrence is guaranteed to be it
    assert module.split("DATA = (")[0] == code.split("DATA = (")[0]
    assert module.rsplit("\n)\n", 1)[-1] == code.rsplit("\n)\n", 1)[-1], (
        "the action defs and run() must be identical")


def test_code_text_is_much_smaller_than_the_real_file():
    """A sanity check that the elision actually elided something."""
    result = build([{"type": "drive", "cm": 400}])   # a long run, lots of DATA

    assert len(result["code_text"]) < len(result["module_text"]) / 4


def test_both_views_are_none_when_there_is_nothing_to_drive():
    result = build_run({
        "version": 1, "name": "broken", "steps_ms": 6,
        "robot": {"track_width_cm": "not a number"},
        "start": {"x": 0, "y": 0, "head": 0}, "steps": [],
    })

    assert result["module_text"] is None
    assert result["code_text"] is None
    assert result["builder_source"] is None
