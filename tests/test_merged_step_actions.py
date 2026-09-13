"""An action must be placed against its own step, even when steps merge.

The builder combines consecutive drives in one direction into a single
acceleration profile -- two drives really are one motion, which is why the
second reports no time of its own. It combines consecutive waits the same way,
and arm steps are waits.

That merging is right for the motion and was wrong for the actions attached to
it. `segment_owner` records only the step that *created* a segment, so a merged
step's action was placed relative to the segment -- which belongs to the step
before it -- while its window came from step order. The two disagreed:

    cm: 15 on the second of two 30cm drives  ->  1042ms   (was)
    cm: 30 on the second of two 30cm drives  ->  1584ms   (was)
    cm: 30 on a single 60cm drive            ->  1584ms

The last two being identical was the whole bug: "15cm into this step" meant
15cm into the merged motion, so the action fired during the step before it --
up to three seconds early, with no diagnostic. The file downloaded and looked
correct. On the field that is an arm moving confidently at the wrong place on
the mat, which is harder to diagnose than an arm that does not move.

Three merged drives were worse still: all three actions landed on the same
instant and came back as c, b, a, so the hub bound them backwards.

Arm steps already carried an offset (`into_wait` in headless.py) so that two in
a row do not fire together, but it reached only the arm step's own implicit
marker -- an explicit action on the same step still fired inside the step
before. Drives had no offset at all. Both now go through `_at_within_step`.
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


def window(result, index):
    step = result["steps"][index]
    return step["starts_ms"], step["ends_ms"]


def only_marker(result):
    assert len(result["markers"]) == 1, result["diagnostics"]
    return result["markers"][0]["time_ms"]


def test_the_second_of_two_merged_drives_reports_no_time_of_its_own():
    """Not a bug -- two drives are one profile. It is why the rest goes wrong."""
    result = two_drives({"cm": 0})
    starts, ends = window(result, 1)

    assert starts == ends


def test_an_action_on_a_merged_step_is_measured_from_that_step():
    """15cm into the second drive is 45cm into the merged motion."""
    merged = only_marker(two_drives({"cm": 15}))
    solo = build([{"type": "drive", "cm": 60, "actions": [action({"cm": 45})]}])

    assert merged == only_marker(solo)


def test_an_action_at_the_start_of_a_merged_step_is_not_the_start_of_the_run():
    """cm: 0 on the second drive is 30cm in, not the very beginning."""
    merged = only_marker(two_drives({"cm": 0}))
    solo = build([{"type": "drive", "cm": 60, "actions": [action({"cm": 30})]}])

    assert merged == only_marker(solo)


def test_an_action_on_a_merged_step_does_not_fire_during_the_step_before():
    """Whatever the offset, it must not fire before its own step begins.

    A merged step's reported window is a single instant -- the robot never
    slows between the two drives -- so there is no window to test against.
    The honest boundary is where the first step's distance runs out: 30cm of
    the merged 60cm profile. Anything earlier is firing during the drive
    before, which is the bug this file exists for.
    """
    fires = only_marker(two_drives({"cm": 15}))

    first_step_ends = only_marker(build([
        {"type": "drive", "cm": 60, "actions": [action({"cm": 30})]}]))
    run_ends = only_marker(build([
        {"type": "drive", "cm": 60, "actions": [action({"cm": 60})]}]))

    assert fires >= first_step_ends, (
        "fires {0}ms before its own step begins".format(first_step_ends - fires))
    assert fires <= run_ends


def test_a_negative_offset_counts_back_from_its_own_step():
    """cm: -5 on the second drive is 5cm before *that* step ends, not the run."""
    merged = only_marker(two_drives({"cm": -5}))
    solo = build([{"type": "drive", "cm": 60, "actions": [action({"cm": 55})]}])

    assert merged == only_marker(solo)


def test_three_merged_drives_each_place_their_own_action():
    """The offset has to accumulate, not just apply once."""
    result = build([
        {"type": "drive", "cm": 20, "actions": [action({"cm": 0}, "a")]},
        {"type": "drive", "cm": 20, "actions": [action({"cm": 0}, "b")]},
        {"type": "drive", "cm": 20, "actions": [action({"cm": 0}, "c")]},
    ])

    times = [marker["time_ms"] for marker in result["markers"]]
    names = [marker["id"] for marker in result["markers"]]

    assert names == ["a", "b", "c"], "in firing order"
    assert times[0] < times[1] < times[2], "at three distinct moments"


def test_a_drive_that_does_not_merge_is_unaffected():
    """A turn between them means two segments, and no offset should apply."""
    result = build([
        {"type": "drive", "cm": 30, "actions": [action({"cm": 0}, "a")]},
        {"type": "turn", "deg": 90},
        {"type": "drive", "cm": 30, "actions": [action({"cm": 0}, "b")]},
    ])

    for marker in result["markers"]:
        starts, ends = window(result, marker["step"])
        assert starts <= marker["time_ms"] <= ends, (
            "{0} fired outside its own step".format(marker["id"]))


def test_two_arm_steps_in_a_row_still_fire_one_after_the_other():
    """The wait-side offset already worked: it must keep working."""
    result = build([
        {"type": "drive", "cm": 20},
        {"type": "armStep", "motor": "leftTask", "call": "run_angle",
         "angle": 90, "speed": 500, "actions": [{"id": "down"}]},
        {"type": "armStep", "motor": "leftTask", "call": "run_angle",
         "angle": -90, "speed": 500, "actions": [{"id": "up"}]},
        {"type": "drive", "cm": 20},
    ])

    names = [marker["id"] for marker in result["markers"]]
    times = [marker["time_ms"] for marker in result["markers"]]

    assert names == ["down", "up"]
    assert times[0] < times[1]


@pytest.mark.xfail(reason="a merged drive's time split is not known before build()")
def test_a_time_based_action_on_a_merged_drive_is_measured_from_that_step():
    """The one form the offset does not cover, recorded rather than hidden.

    `cm` on a drive and `ms` on a wait are both offsets the step itself knows:
    a drive declares its distance, a wait its duration. A *time* offset into a
    merged drive is different -- two 30cm drives become one 60cm acceleration
    profile, and when the second one "begins" in that profile is not knowable
    until the trajectory is built.

    Unreachable from the planner, which only ever creates `cm` actions (see
    addAction in runEditor.ts), so nobody can hit it today. It is pinned here
    so that whoever adds a time field to the action row finds this first.
    """
    merged = build([
        {"type": "drive", "cm": 30},
        {"type": "drive", "cm": 30, "actions": [action({"ms": 0})]},
    ])

    first_ends = merged["steps"][0]["ends_ms"]

    assert only_marker(merged) >= first_ends, (
        "fires during the drive before it")
