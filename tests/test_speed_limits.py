"""Step 4.5: a slow, careful section within one step.

The library already has this -- addRelativeDisplacementConstraints, proven by
the constraints_marker golden -- it has just never been reachable from a
described run before. Setting a constraint changes the robot's planned speed
ceiling from that point *forward for the rest of the run*, with no automatic
reset (trajectoryBuilder.py:589, `self.CONSTRAINTS = the_chosen_one.
constraints`). A "speed limit" block is therefore always two markers under
one name: one that slows down where it starts, and one that restores the
robot's own normal speed where it ends. Get the second one wrong -- drop it,
misplace it, or forget it -- and the robot stays slow for the rest of the run,
which is a worse and quieter failure than the limit never applying at all.
"""

from pythfinder import Constraints, Constraints2D, Point, Pose, TrajectoryBuilder
from pythfinder.Trajectory.robotConfig import FLL_ROBOT
from pythfinder.headless import build_run


def build(steps):
    return build_run({
        "version": 1,
        "name": "speed_limit",
        "steps_ms": 6,
        "robot": "fll_team",
        "start": {"x": 0, "y": 0, "head": 0},
        "steps": steps,
    })


def limit(from_cm=None, to_cm=None, cm_s=20, ident="s1", from_at=None, to_at=None):
    return {
        "id": ident,
        "from": from_at if from_at is not None else {"cm": from_cm},
        "to": to_at if to_at is not None else {"cm": to_cm},
        "cm_s": cm_s,
    }


def drive(cm, limits):
    return {"type": "drive", "cm": cm, "speedLimits": limits}


def test_a_speed_limit_slows_the_run_down():
    plain = build([{"type": "drive", "cm": 80}])
    slowed = build([drive(80, [limit(30, 60, cm_s=10)])])

    assert plain["ok"] and slowed["ok"], (plain["diagnostics"], slowed["diagnostics"])
    assert slowed["total_ms"] > plain["total_ms"]


def test_speed_is_restored_after_the_limit_ends():
    """The whole point of a *section*: normal speed either side of it.

    Proved by comparing three runs of the same total distance -- a limit that
    covers only the middle third must cost less time than one left open with
    no restore, and more than none at all. If the restore marker were being
    dropped, "middle only" and "open-ended" would take the same time.
    """
    plain = build([{"type": "drive", "cm": 90}])
    middle_only = build([drive(90, [limit(30, 60, cm_s=10)])])
    open_ended = build([{"type": "drive", "cm": 90, "speedLimits": [
        {"id": "s1", "from": {"cm": 30}, "to": {"cm": 89.999}, "cm_s": 10}]}])

    assert plain["ok"] and middle_only["ok"] and open_ended["ok"]
    assert plain["total_ms"] < middle_only["total_ms"] < open_ended["total_ms"]


def test_two_separate_slow_zones_on_one_step_both_apply():
    one_zone = build([drive(120, [limit(20, 40, cm_s=10)])])
    two_zones = build([drive(120, [limit(20, 40, cm_s=10), limit(70, 90, cm_s=10)])])

    assert one_zone["ok"] and two_zones["ok"]
    assert two_zones["total_ms"] > one_zone["total_ms"]


def test_a_limit_on_the_second_of_two_merged_drives_still_lands_correctly():
    """The exact machinery 3.2 fixed for actions -- speed limits ride the
    same _at_within_step, and must be placed against the segment the same
    way once the builder merges the two drives into one."""
    merged = build([
        {"type": "drive", "cm": 30},
        drive(30, [limit(0, 15, cm_s=10)]),
    ])
    solo = build([drive(60, [limit(30, 45, cm_s=10)])])

    assert merged["ok"] and solo["ok"], (merged["diagnostics"], solo["diagnostics"])
    assert merged["total_ms"] == solo["total_ms"]


def test_backwards_range_is_an_error():
    result = build([drive(80, [limit(60, 30, cm_s=10)])])

    assert not result["ok"]
    assert any("end after it starts" in d["message"] for d in result["diagnostics"])


def test_equal_from_and_to_is_an_error():
    result = build([drive(80, [limit(40, 40, cm_s=10)])])

    assert not result["ok"]


def test_zero_speed_is_an_error():
    result = build([drive(80, [limit(30, 60, cm_s=0)])])

    assert not result["ok"]
    assert any("positive speed" in d["message"] for d in result["diagnostics"])


def test_negative_speed_is_an_error():
    result = build([drive(80, [limit(30, 60, cm_s=-5)])])

    assert not result["ok"]


def test_mismatched_units_is_an_error():
    result = build([drive(80, [limit(from_at={"cm": 30}, to_at={"ms": 500}, cm_s=10)])])

    assert not result["ok"]
    assert any("both" in d["message"] for d in result["diagnostics"])


def test_a_turn_step_cannot_carry_a_speed_limit():
    """Turning uses angular constraints, which a speed limit does not touch --
    offering it on a turn would silently do nothing, so it is refused instead."""
    result = build([{"type": "turn", "deg": 90, "speedLimits": [limit(0, 45, cm_s=10)]}])

    assert not result["ok"]
    assert any("drive" in d["message"] for d in result["diagnostics"])


def test_a_wait_step_cannot_carry_a_speed_limit():
    result = build([{"type": "wait", "ms": 500, "speedLimits": [limit(0, 100, cm_s=10)]}])

    assert not result["ok"]


def test_a_range_past_the_end_of_the_step_is_a_warning_not_an_error():
    """Out of range is the same class of mistake as a dropped action -- a
    warning, so the rest of the run still builds; the wording says what it
    actually is, not "an action" (trajectoryBuilder.py's shared message)."""
    result = build([drive(80, [limit(30, 200, cm_s=10)])])

    assert result["ok"], result["diagnostics"]

    warnings = [d["message"] for d in result["diagnostics"] if d["level"] == "warning"]
    assert any("speed limit" in message for message in warnings)
    assert not any(message.startswith("an action") for message in warnings)


def test_a_toPoint_step_can_carry_a_speed_limit():
    result = build([{"type": "toPoint", "x": 0, "y": 60,
                     "speedLimits": [limit(10, 30, cm_s=10)]}])

    assert result["ok"], result["diagnostics"]


def run_chain(builder_source: str):
    """Actually execute step 3.4's chain view, the way test_python_view.py's
    own run_chain does -- duplicated rather than imported, since this file
    also has to strip the "# needs: ..." note a speed limit adds, which
    test_python_view.py's own runs never trigger."""
    lines = [line for line in builder_source.split("\n") if not line.startswith("#")]
    headless = ("\n".join(lines)
                .replace("TrajectoryBuilder(sim, ", "TrajectoryBuilder(")
                .replace(", FLL_FIELD)", ", robot=FLL_ROBOT)"))

    namespace = {
        "TrajectoryBuilder": TrajectoryBuilder, "Pose": Pose, "Point": Point,
        "FLL_ROBOT": FLL_ROBOT, "Constraints": Constraints,
        "Constraints2D": Constraints2D, "print": lambda *args: None,
    }

    exec("_trajectory = " + headless, namespace)
    return namespace["_trajectory"]


def test_the_chain_view_includes_a_working_speed_limit():
    """3.4's whole promise is that the chain is equivalent, not just similar
    -- a speed limit has to prove that the same way everything else in
    test_python_view.py does: run it for real and compare."""
    result = build([drive(80, [limit(30, 60, cm_s=10)])])

    assert result["ok"], result["diagnostics"]
    assert "needs: from pythfinder import Constraints, Constraints2D" in result["builder_source"]
    assert "addRelativeDisplacementConstraints" in result["builder_source"]

    trajectory = run_chain(result["builder_source"])
    assert trajectory.TIME == result["total_ms"]


def test_the_chain_note_is_absent_without_a_speed_limit():
    result = build([{"type": "drive", "cm": 80}])

    assert "needs:" not in result["builder_source"]


def test_a_speed_limit_reaches_the_hub_file_as_slower_numbers_only():
    """A speed limit changes the DATA payload's own numbers -- the wheel
    speeds baked in -- and adds no code of its own to the hub file. Nothing
    in module_text should even hint a limit was involved."""
    result = build([drive(80, [limit(30, 60, cm_s=10)])])

    assert result["ok"], result["diagnostics"]
    assert result["module_text"] is not None
    assert "MARKERS = ()" in result["module_text"]
    assert result["code_text"] is not None


def test_a_bare_step_with_no_speed_limits_is_unaffected():
    """A run with the key absent must build exactly as it always has."""
    with_key = build([{"type": "drive", "cm": 80, "speedLimits": []}])
    without_key = build([{"type": "drive", "cm": 80}])

    assert with_key["total_ms"] == without_key["total_ms"]
