"""Check the door the web planner knocks on.

build_run takes a run as data and hands back what a browser needs: poses to
draw, markers in firing order, problems to show, and the file to download. A
badly described run has to come back as diagnostics rather than an exception --
a child typing a wrong number is ordinary, not exceptional.
"""

import pytest

from pythfinder.headless import build_run

from golden_runs import GOLDEN_DIR, GOLDEN_RUNS


TEMPLATE_RUN = {
    "version": 1,
    "name": "run_a",
    "steps_ms": 6,
    "robot": "fll_team",
    "start": {"x": -46, "y": -83, "head": 0},
    "steps": [
        {"type": "drive", "cm": 75,
         "actions": [{"id": "arm_down", "at": {"cm": 35}, "label": "arm down"}]},
        {"type": "wait", "ms": 600},
        {"type": "turn", "deg": 90},
        # was {"ms": -1} -- "1ms before the end" -- until step 5.2 removed
        # time placement from the format. {"cm": -1}, "1cm before the end",
        # is the distance-based idiom that replaces it; see
        # test_the_template_run_described_as_data_gives_the_golden_file for
        # what that changes about the golden comparison below.
        {"type": "drive", "cm": 30,
         "actions": [{"id": "arm_up", "at": {"cm": -1}, "label": "arm up"}]},
        {"type": "toPose", "x": -46, "y": -83, "head": 0},
    ],
}


def body(module_text):
    """Everything but the opening docstring line, which names the source."""
    return module_text.split("\n", 1)[1]


def data_only(module_text):
    """Just the numbers the robot drives on.

    Picked out by name rather than by cutting the file at the first `def` or
    `from`: the generated module puts `from trajectory import Trajectory`
    directly under its docstring, so cutting there left nothing but the
    docstring on one side and the whole file on the other.

    The docstring itself is excluded on purpose -- the fixture was written by
    tools/txt_to_py.py and says so, while ours says PythFinder.
    """
    keep = ("STEPS", "MARKERS", "COUNT", "DATA")
    lines = module_text.split("\n")

    return "\n".join(
        line for line in lines
        if line.startswith(keep) or line.startswith("    b")
    )


def motion_only(module_text):
    """Just the states the robot drives on -- not MARKERS.

    Step 5.2 removed time placement, so TEMPLATE_RUN's "arm up" action moved
    from {"ms": -1} to {"cm": -1} -- see TEMPLATE_RUN's own comment. That is
    a different moment (7673ms into the run, not 7942), so MARKERS no longer
    matches the pinned hub file byte for byte; the motion itself does not
    depend on where a marker sits, so STEPS/COUNT/DATA still should. The
    byte-for-byte proof of MARKERS against that same file lives on
    unaffected in test_hub_module.py, built through golden_runs.py's own
    TrajectoryBuilder chain -- which still places that action with
    .addRelativeTemporalMarker(-1, ...), a call this format no longer
    exposes but the library itself still has.
    """
    keep = ("STEPS", "COUNT", "DATA")
    lines = module_text.split("\n")

    return "\n".join(
        line for line in lines
        if line.startswith(keep) or line.startswith("    b")
    )


def test_the_template_run_described_as_data_gives_the_golden_file():
    """The same run the team drives, described as JSON instead of Python.

    Compared on the motion alone -- not the full data_only() block this test
    used to check byte for byte. The file on the hub predates step 3.2, which
    adds the attachment motor code and a run() below the constants, so the
    numbers the robot *drives on* (STEPS/COUNT/DATA) must still match it
    exactly; the code section is new and is checked separately.

    MARKERS is deliberately excluded now. The hub file's second marker was
    placed with a temporal trigger ({"ms": -1}, "1ms before the end") that
    step 5.2 removed from the run-description format; TEMPLATE_RUN's
    equivalent is now {"cm": -1}, which lands at a different, but equally
    valid, moment (see TEMPLATE_RUN's own comment and motion_only's). The
    byte-for-byte MARKERS proof against this same golden file still lives at
    the library level, in test_hub_module.py, built through golden_runs.py's
    TrajectoryBuilder chain -- a path this format change does not touch.
    """
    result = build_run(TEMPLATE_RUN)

    assert result["ok"]
    assert result["total_ms"] == 15641

    on_the_hub = (GOLDEN_DIR / "hub" / "template_run.py").read_text()
    assert motion_only(result["module_text"]) == motion_only(on_the_hub)

    # the marker moved, but is still there, still ordered second, and still
    # lands inside the step it belongs to
    assert [m["id"] for m in result["markers"]] == ["arm_down", "arm_up"]
    assert result["markers"][1]["time_ms"] < result["total_ms"]


def test_the_template_run_now_carries_its_own_code():
    """What 3.2 adds on top: the actions, and a run() to bind them."""
    module = build_run(TEMPLATE_RUN)["module_text"]

    assert "from trajectory import Trajectory" in module
    assert "def _action_1(core):" in module
    assert "def _action_2(core):" in module
    assert "def run(core):" in module

    # bound in firing order, which is what makes the tuple in runs.py
    # unnecessary
    first = module.index("_action_1(core),")
    second = module.index("_action_2(core),")
    assert first < second


def test_a_run_with_no_actions_still_gets_a_run_function():
    """A plain run -- no arm, no code, nothing attached -- is not a smaller
    case of step 3.1's self-contained file, just a shorter one.

    hub_module_text's `actions` argument used to double as two different
    signals: None meant "the caller never described actions at all" (the
    pre-3.1 export, wired by hand into a runs.py with its own marker
    functions), and an empty list meant "this run has none" -- but both were
    falsy, so both skipped run() entirely. build_run always passes a list,
    even an empty one, so a plain drive-and-turn run downloaded from the
    planner came back with nothing runs.py could call: the docstring and the
    data, and nothing else. Caught from a real download -- a "square" test
    run with no actions -- that did exactly this on the hub.
    """
    run = {
        "version": 1,
        "name": "run_square",
        "steps_ms": 6,
        "robot": "fll_team",
        "start": {"x": 0, "y": 0, "head": 0},
        "steps": [
            {"type": "drive", "cm": 30},
            {"type": "turn", "deg": 90},
        ],
    }

    module = build_run(run)["module_text"]

    assert "from trajectory import Trajectory" in module
    assert "def run(core):" in module
    assert "trajectory.follow(core)" in module

    # nothing to bind, so nothing here should even mention it
    assert "withMarkers" not in module


def test_the_template_run_overhangs_the_table_while_turning_home():
    """A real 7mm overhang in the run the team drives, not a false alarm.

    Turning on the spot at the launch area swings a corner 11.8cm from the
    middle of the robot -- sqrt(7**2 + 9.5**2) -- and x = -46 leaves only
    11.15cm to the edge of a 114.3cm table. It is a warning, not an error: the
    run still works, and the robot still drives.
    """
    result = build_run(TEMPLATE_RUN)

    assert [problem["level"] for problem in result["diagnostics"]] == ["warning"]

    overhang = result["diagnostics"][0]

    assert "table" in overhang["message"]
    assert overhang["step"] == 4              # the drive back to the start
    assert overhang["time_ms"] > 0


def test_markers_come_back_in_the_order_they_fire():
    """Not the order they were written: the hub binds actions by time."""
    run = dict(TEMPLATE_RUN)
    run["steps"] = [
        # the action here happens late in the run...
        {"type": "drive", "cm": 75,
         "actions": [{"id": "second", "at": {"cm": 70}}]},
        # ...and this one, written later, happens sooner after it. A turn
        # only ever fires at its own start -- step 5.2 -- so cm: 0 is the
        # only placement that does not warn.
        {"type": "turn", "deg": 90,
         "actions": [{"id": "third", "at": {"cm": 0}}]},
    ]

    result = build_run(run)

    assert [marker["id"] for marker in result["markers"]] == ["second", "third"]
    assert result["markers"][0]["time_ms"] < result["markers"][1]["time_ms"]

    # and each says which step it belongs to
    assert [marker["step"] for marker in result["markers"]] == [0, 1]


def test_each_step_says_when_it_runs():
    """What the page lights up one step's share of the path with."""
    result = build_run(TEMPLATE_RUN)
    steps = result["steps"]

    assert [step["index"] for step in steps] == [0, 1, 2, 3, 4]
    assert [step["type"] for step in steps] == ["drive", "wait", "turn",
                                                "drive", "toPose"]

    # they run back to back, from the beginning to the end of the run
    assert steps[0]["starts_ms"] == 0
    assert steps[-1]["ends_ms"] == result["total_ms"]

    for earlier, later in zip(steps, steps[1:]):
        assert earlier["ends_ms"] == later["starts_ms"]

    # the action 35cm into the first step fires while that step is running
    arm_down = result["markers"][0]
    assert steps[0]["starts_ms"] <= arm_down["time_ms"] <= steps[0]["ends_ms"]


def test_a_merged_step_has_no_time_of_its_own():
    """Two drives in one direction are one move, and the times say so.

    Not a fudge: the second 20cm has no separate acceleration profile to point
    at, because the builder planned 40cm in one go.
    """
    run = dict(TEMPLATE_RUN)
    run["steps"] = [{"type": "drive", "cm": 20},
                    {"type": "drive", "cm": 20}]

    steps = build_run(run)["steps"]
    total = build_run(run)["total_ms"]

    assert steps[0]["starts_ms"] == 0
    assert steps[0]["ends_ms"] == total
    assert steps[1]["starts_ms"] == steps[1]["ends_ms"] == total


def test_poses_are_thinned_for_drawing_but_keep_the_ending():
    result = build_run(TEMPLATE_RUN, pose_every_ms = 20)

    poses = result["poses"]

    assert len(poses) < result["total_ms"] / 10

    # the states are numbered from 1, so the run starts at t = 1 and the last
    # one is timed at the total
    assert poses[0]["t"] == 1
    assert poses[-1]["t"] == result["total_ms"]

    # and it starts where it was told to
    assert round(poses[0]["x"]) == -46
    assert round(poses[0]["y"]) == -83

    # the run finishes back where it started
    assert round(poses[-1]["x"]) == -46
    assert round(poses[-1]["y"]) == -83


def test_an_unknown_step_is_a_diagnostic_not_an_exception():
    run = dict(TEMPLATE_RUN)
    run["steps"] = [{"type": "teleport", "cm": 5}]

    result = build_run(run)

    assert not result["ok"]
    assert result["module_text"] is None

    levels = [problem["level"] for problem in result["diagnostics"]]
    assert "error" in levels

    first = result["diagnostics"][0]
    assert first["step"] == 0
    assert "teleport" in first["message"]


def test_a_step_missing_its_number_is_a_diagnostic():
    run = dict(TEMPLATE_RUN)
    run["steps"] = [{"type": "drive"}]

    result = build_run(run)

    assert not result["ok"]
    assert result["diagnostics"][0]["step"] == 0


def test_problems_point_at_the_described_step_not_the_merged_one():
    """Consecutive drives merge into one segment, and the step numbers must not.

    Two 20cm drives become a single 40cm move inside the builder. An action
    placed past the end belongs to the step the person wrote, not to the
    segment the builder happens to have made.
    """
    run = dict(TEMPLATE_RUN)
    run["steps"] = [
        {"type": "drive", "cm": 20},
        {"type": "drive", "cm": 20,
         "actions": [{"id": "late", "at": {"cm": 300}}]},
    ]

    result = build_run(run)

    dropped = [problem for problem in result["diagnostics"]
               if problem["level"] == "warning"]

    assert len(dropped) == 1
    assert dropped[0]["step"] == 0    # both drives are one segment, the first
    assert result["markers"] == []


def test_an_empty_run_says_so():
    run = dict(TEMPLATE_RUN)
    run["steps"] = []

    result = build_run(run)

    assert not result["ok"]
    assert result["total_ms"] == 0
    assert result["poses"] == []
    assert result["module_text"] is None


def test_a_robot_can_be_described_by_its_numbers():
    """What the mentor-only settings panel will send."""
    run = dict(TEMPLATE_RUN)
    run["robot"] = {"track_width_cm": 16,
                    "max_velocity_cm_s": 64.3,
                    "center_offset_cm": -3.5,
                    "width_cm": 19,
                    "length_cm": 14}

    result = build_run(run)

    assert result["ok"]
    assert body(result["module_text"]) == body(build_run(TEMPLATE_RUN)["module_text"])


def test_an_unknown_robot_is_reported():
    run = dict(TEMPLATE_RUN)
    run["robot"] = "somebody_elses_robot"

    result = build_run(run)

    assert not result["ok"]
    assert "somebody_elses_robot" in result["diagnostics"][0]["message"]


# --- step 5.2: timed triggers are gone ---------------------------------------

def build(step, start=None):
    return build_run({
        "version": 2,
        "name": "time_placement",
        "steps_ms": 6,
        "robot": "fll_team",
        "start": start or {"x": 0, "y": 0, "head": 0},
        "steps": [step],
    })


@pytest.mark.parametrize("step", [
    {"type": "drive", "cm": 40,
     "actions": [{"id": "a1", "at": {"ms": 100}}]},
    {"type": "turn", "deg": 90,
     "actions": [{"id": "a1", "at": {"ms": 100}}]},
    {"type": "toPoint", "x": 0, "y": 40,
     "actions": [{"id": "a1", "at": {"ms": 100}}]},
    {"type": "toPose", "x": 0, "y": 40, "head": 90,
     "actions": [{"id": "a1", "at": {"ms": 100}}]},
])
def test_an_action_placed_by_time_is_rejected_on_every_step_that_moves(step):
    """The one rule that now applies everywhere: build_run only ever places
    an action by distance. Not just drives -- a turn, a toPoint and a toPose
    all refuse "ms" the same way, with the same clear diagnostic."""
    result = build(step)

    assert not result["ok"]
    problems = [d for d in result["diagnostics"] if d["level"] == "error"]
    assert len(problems) == 1
    assert problems[0]["step"] == 0
    assert "time" in problems[0]["message"]
    assert problems[0]["suggestion"] == "give it a distance into the step instead"
    assert result["markers"] == []


def test_an_old_saved_run_with_time_placement_is_flagged_not_guessed():
    """The migration step 5.2 asks for: a run saved before this rule existed
    (version 1, the editor never wrote "ms" but a hand-edited or imported
    file could carry one) still loads and builds -- it just comes back
    flagged on its own step, rather than the planner silently inventing a
    distance for something it was never told."""
    run = {
        "version": 1,
        "name": "old_run",
        "steps_ms": 6,
        "robot": "fll_team",
        "start": {"x": 0, "y": 0, "head": 0},
        "steps": [
            {"type": "drive", "cm": 30},
            {"type": "drive", "cm": 30,
             "actions": [{"id": "old", "at": {"ms": 500}, "label": "old action"}]},
        ],
    }

    result = build_run(run)

    assert not result["ok"]
    assert result["markers"] == []

    flagged = [d for d in result["diagnostics"] if d["level"] == "error"]
    assert len(flagged) == 1
    assert flagged[0]["step"] == 1
    assert "time" in flagged[0]["message"]

    # the rest of the run is unaffected -- only the one action is refused
    assert result["total_ms"] > 0


def test_a_turn_action_at_its_start_fires_with_no_warning():
    result = build({"type": "turn", "deg": 90,
                    "actions": [{"id": "a1", "at": {"cm": 0}}]})

    assert result["ok"], result["diagnostics"]
    assert [m["id"] for m in result["markers"]] == ["a1"]
    assert result["diagnostics"] == []


def test_a_turn_action_off_its_start_still_fires_but_warns():
    """A turn covers no distance, so cm: 0 is the only real placement -- but
    an action written with some other cm still has to fire *somewhere*
    rather than vanish, so it fires at the only moment a turn has (its
    start), with a warning pointing at the step it probably belongs to."""
    result = build({"type": "turn", "deg": 90,
                    "actions": [{"id": "a1", "at": {"cm": 15}}]})

    assert result["ok"]
    assert [m["id"] for m in result["markers"]] == ["a1"]

    warnings = [d for d in result["diagnostics"] if d["level"] == "warning"]
    assert len(warnings) == 1
    assert warnings[0]["step"] == 0
    assert "only ever fires at its start" in warnings[0]["message"]
    assert warnings[0]["suggestion"] == "move it to the next step"


def test_a_speed_limit_can_only_be_placed_by_distance():
    result = build({"type": "drive", "cm": 80, "speedLimits": [
        {"id": "s1", "from": {"ms": 100}, "to": {"ms": 500}, "cm_s": 10}]})

    assert not result["ok"]
    problems = [d for d in result["diagnostics"] if d["level"] == "error"]
    assert len(problems) == 1
    assert problems[0]["step"] == 0
    assert "not a time" in problems[0]["message"]


def test_toPoint_action_is_measured_along_the_straight_part_only():
    """The plan's own claim (docs/web-planner.md, step 5.2): a toPoint's
    initial turn-to-face adds no displacement, so "cm" measures only the
    straight leg. Pinned by comparing two runs that both drive 40cm in a
    straight line to the same action, differing only in how much turning it
    took to get facing that way first: the *time left after the action*
    (ends_ms - the marker's own time) has to be identical, because that
    remainder is entirely within the straight part, which neither run's
    turning touches.
    """
    def marker_to_end_gap(start_head):
        result = build({"type": "toPoint", "x": 0, "y": 40,
                        "actions": [{"id": "a1", "at": {"cm": 10}}]},
                       start = {"x": 0, "y": 0, "head": start_head})

        assert result["ok"], result["diagnostics"]

        return result["steps"][0]["ends_ms"] - result["markers"][0]["time_ms"]

    no_turn = marker_to_end_gap(90)       # already facing the target
    small_turn = marker_to_end_gap(60)
    big_turn = marker_to_end_gap(0)
    reverse_turn = marker_to_end_gap(180)

    assert small_turn == big_turn == reverse_turn
    assert abs(no_turn - small_turn) <= 1   # rounding at the segment boundary
