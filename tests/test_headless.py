"""Check the door the web planner knocks on.

build_run takes a run as data and hands back what a browser needs: poses to
draw, markers in firing order, problems to show, and the file to download. A
badly described run has to come back as diagnostics rather than an exception --
a child typing a wrong number is ordinary, not exceptional.
"""

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
        {"type": "drive", "cm": 30,
         "actions": [{"id": "arm_up", "at": {"ms": -1}, "label": "arm up"}]},
        {"type": "toPose", "x": -46, "y": -83, "head": 0},
    ],
}


def body(module_text):
    """Everything but the opening docstring line, which names the source."""
    return module_text.split("\n", 1)[1]


def test_the_template_run_described_as_data_gives_the_golden_file():
    """The same run the team drives, described as JSON instead of Python."""
    result = build_run(TEMPLATE_RUN)

    assert result["ok"]
    assert result["total_ms"] == 15641

    on_the_hub = (GOLDEN_DIR / "hub" / "template_run.py").read_text()
    assert body(result["module_text"]) == body(on_the_hub)


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
        # ...and this one, written later, happens sooner after it
        {"type": "turn", "deg": 90,
         "actions": [{"id": "third", "at": {"ms": 1}}]},
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
