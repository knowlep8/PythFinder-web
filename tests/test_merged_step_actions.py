"""An action on a merged step is placed against the wrong step.

Consecutive drives merge into one acceleration profile -- two drives really
are one motion, which is why the editor reports the second as taking no time
of its own. But a distance-based action attached to the *second* drive is
measured from the start of the merged profile, not from where that step
begins, so it fires early:

    cm: 15 on the second of two 30cm drives  ->  1042ms
    cm: 30 on the second of two 30cm drives  ->  1584ms
    cm: 30 on a single 60cm drive            ->  1584ms

The last two are identical, which is the whole bug in one line.

The second step spans (3167, 3167) -- no duration at all -- so every action
attached to it fires during the first step, up to three seconds early, with no
diagnostic. The file downloads and looks correct. On the field it is an arm
that moves at the wrong place on the mat, which is harder to diagnose than an
arm that does not move.

These are xfail rather than fixes: the placement wants its own change, and
pinning the numbers now means that change has something to prove itself
against. Delete the xfail marks when it lands.
"""

import pytest

from pythfinder.headless import build_run


def build(steps):
    return build_run({
        "version": 1,
        "name": "merged",
        "steps_ms": 6,
        "robot": "fll_team",
        "start": {"x": 0, "y": 0, "head": 0},
        "steps": steps,
    })


def action(at, ident="second"):
    return {"id": ident, "at": at, "label": "Grab",
            "do": {"motor": "leftTask", "call": "run", "speed": 500}}


def two_drives(at):
    return build([
        {"type": "drive", "cm": 30},
        {"type": "drive", "cm": 30, "actions": [action(at)]},
    ])


def test_the_second_of_two_merged_drives_reports_no_time_of_its_own():
    """Not a bug -- two drives are one profile. It is why the rest goes wrong."""
    result = two_drives({"cm": 0})
    second = result["steps"][1]

    assert second["starts_ms"] == second["ends_ms"]


@pytest.mark.xfail(reason="placed against the merged profile, not the step")
def test_an_action_on_a_merged_step_is_measured_from_that_step():
    """15cm into the second drive should be 45cm into the merged motion."""
    merged = two_drives({"cm": 15})["markers"][0]["time_ms"]
    solo = build([{"type": "drive", "cm": 60, "actions": [action({"cm": 45})]}])

    assert merged == solo["markers"][0]["time_ms"]


@pytest.mark.xfail(reason="placed against the merged profile, not the step")
def test_an_action_on_a_merged_step_does_not_fire_during_the_step_before():
    """Whatever the offset, it must not fire before its own step begins."""
    result = two_drives({"cm": 15})

    starts = result["steps"][1]["starts_ms"]
    fires = result["markers"][0]["time_ms"]

    assert fires >= starts, (
        "the action fires {0}ms before its step begins".format(starts - fires))


@pytest.mark.xfail(reason="placed against the merged profile, not the step")
def test_the_end_of_a_merged_step_is_the_end_of_that_step():
    """cm: 30 on the second drive is the end of the run, not its middle."""
    result = two_drives({"cm": 30})

    assert result["markers"][0]["time_ms"] == pytest.approx(
        result["total_ms"], abs=50)
