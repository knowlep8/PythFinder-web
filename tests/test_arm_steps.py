"""Sequential arm steps: the robot stops, the arm finishes, then the next step.

Written before the feature. The case that matters is two arm steps in a row:
the builder merges consecutive waits into one segment, so without care both
actions land on the same millisecond and fire together -- which is precisely
what a sequential step exists to prevent, and it passed every other check in
this suite.

See docs/web-planner.md, step 3.2.
"""

import pytest

from pythfinder.headless import build_run


def arm(identifier, angle=90, speed=500, motor="leftTask"):
    """One sequential arm step, as the page will describe it."""
    return {
        "type": "armStep",
        "motor": motor,
        "call": "run_angle",
        "speed": speed,
        "angle": angle,
        "actions": [{"id": identifier}],
    }


def run_with(steps):
    return build_run({
        "version": 1,
        "name": "arms",
        "steps_ms": 6,
        "robot": "fll_team",
        "start": {"x": 0, "y": 0, "head": 0},
        "steps": steps,
    })


def test_an_arm_step_stops_the_robot():
    """It is a step, so the robot arrives at a standstill and waits."""
    result = run_with([
        {"type": "drive", "cm": 20},
        arm("lift"),
        {"type": "drive", "cm": 20},
    ])

    assert result["ok"], result["diagnostics"]

    stopped = result["steps"][1]
    assert stopped["type"] == "armStep"

    # it occupies real time, sized from the motor's own speed and angle
    assert stopped["ends_ms"] > stopped["starts_ms"]

    # and the action fires while the robot is standing still
    lift = next(m for m in result["markers"] if m["id"] == "lift")
    assert stopped["starts_ms"] <= lift["time_ms"] <= stopped["ends_ms"]


def test_two_arm_steps_in_a_row_happen_one_after_the_other():
    """The regression this file exists for.

    Consecutive waits merge into one segment. If both arm steps put their
    action at the start of it, the arm is told to do two things at the same
    instant and the generated file binds them in the wrong order.
    """
    result = run_with([
        {"type": "drive", "cm": 20},
        arm("first", angle=90),
        arm("second", angle=180),
        {"type": "drive", "cm": 20},
    ])

    assert result["ok"], result["diagnostics"]

    times = [marker["time_ms"] for marker in result["markers"]]
    names = [marker["id"] for marker in result["markers"]]

    assert names == ["first", "second"], "they must come back in firing order"
    assert times[0] < times[1], "and at different moments, not together"

    # the second waits for the first: its action falls after the first arm
    # step's estimated duration, not at the same instant
    assert times[1] - times[0] >= 100


def test_each_arm_step_is_given_its_own_time():
    """Two arm steps need room for both, not for the first one only.

    They share a segment, because consecutive waits merge. The segment has to
    be as long as both estimates together, or the run's length is a fiction and
    the second arm step appears to take no time at all -- which is precisely
    the wrong thing to tell a child about a step whose whole purpose is that
    the robot waits for it.
    """
    one = run_with([arm("only", angle=90, speed=500)])            # 180ms
    two = run_with([arm("first", angle=90, speed=500),            # 180ms
                    arm("second", angle=300, speed=500)])         # 600ms

    assert one["ok"] and two["ok"]

    # the pair must be longer than the single by about the second estimate
    assert two["total_ms"] - one["total_ms"] >= 500

    second = two["steps"][1]
    assert second["ends_ms"] > second["starts_ms"], (
        "a sequential step showing zero length tells the team it is free")


def test_a_longer_sweep_is_given_longer():
    """The estimate comes from the motor: twice the angle, about twice the wait."""
    short = run_with([arm("a", angle=90, speed=500)])
    long = run_with([arm("b", angle=180, speed=500)])

    assert short["ok"] and long["ok"]
    assert long["total_ms"] > short["total_ms"] * 1.5


def test_a_faster_motor_is_given_less_time():
    slow = run_with([arm("a", angle=180, speed=250)])
    fast = run_with([arm("b", angle=180, speed=1000)])

    assert slow["total_ms"] > fast["total_ms"]


def test_an_arm_step_carries_its_motor_command_to_the_file():
    """What 3.2 generates has to know which motor to move, and how."""
    result = run_with([
        {"type": "drive", "cm": 20},
        arm("lift", motor="rightTask", angle=45, speed=300),
    ])

    assert result["ok"], result["diagnostics"]

    module = result["module_text"]
    assert module is not None

    # the action becomes real code in the downloaded file
    assert "rightTask" in module
    assert "run_angle" in module
    assert "def run(core)" in module


def test_a_parallel_action_still_fires_while_driving():
    """The other kind, unchanged: attached to a move, robot keeps going."""
    result = run_with([
        {"type": "drive", "cm": 40,
         "actions": [{"id": "grab", "at": {"cm": 20},
                      "do": {"motor": "leftTask", "call": "run", "speed": 500}}]},
    ])

    assert result["ok"], result["diagnostics"]

    grab = next(m for m in result["markers"] if m["id"] == "grab")
    driving = result["steps"][0]

    assert driving["starts_ms"] < grab["time_ms"] < driving["ends_ms"]
