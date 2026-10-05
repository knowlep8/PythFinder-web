"""Step 6.1: a speed limit is the speed for one whole step.

Step 4.5 made a limit a from/to section within a step, which was never what
was wanted -- and in the hub file a section placed a sliver into a leg came
out as drive.settings(slow), drive.straight(0), drive.settings(normal): a
limit that did nothing at all. Now a step carries at most one
`speedLimit_cm_s`, covering its whole straight part.

Setting a constraint changes the robot's planned speed ceiling from that
point *forward for the rest of the run*, with no automatic reset
(trajectoryBuilder.py's __process_relative_constraints, `self.CONSTRAINTS =
the_chosen_one.constraints`) -- and a DriveBase's settings() speed lasts
until the next settings() too. So a limit is one marker where its step
starts, and the next step that drives at the robot's own speed is what ends
it. Get that wrong and the robot stays slow for the rest of the run, which
is a worse and quieter failure than the limit never applying at all.
"""

from pythfinder import Constraints, Constraints2D, Point, Pose, TrajectoryBuilder
from pythfinder.Export.driveProgram import compile_drive_program
from pythfinder.Trajectory.robotConfig import FLL_ROBOT
from pythfinder.headless import build_run


def run_of(steps):
    return {
        "version": 3,
        "name": "speed_limit",
        "steps_ms": 6,
        "robot": "fll_team",
        "start": {"x": 0, "y": 0, "head": 0},
        "steps": steps,
    }


def build(steps):
    return build_run(run_of(steps))


def drive(cm, cm_s=None):
    step = {"type": "drive", "cm": cm}

    if cm_s is not None:
        step["speedLimit_cm_s"] = cm_s

    return step


def errors(result):
    return [d["message"] for d in result["diagnostics"] if d["level"] == "error"]


def moves_of(steps):
    compiled = compile_drive_program(run_of(steps))
    assert compiled["ok"], compiled["diagnostics"]

    return [(m["op"], m.get("mm", m.get("straight_speed"))) for m in compiled["moves"]
            if m["op"] in ("straight", "settings")]


# --- the hub file ------------------------------------------------------------

def test_a_limit_is_one_settings_change_either_side_of_its_straight():
    """The bug 6.1 fixed: no straight(0) between the two settings() calls,
    and the leg is not split -- the whole step drives at the limit."""
    assert moves_of([drive(80, cm_s=10)]) == [
        ("settings", 100.0), ("straight", 800.0), ("settings", 200.0)]


def test_the_next_unlimited_step_hands_the_normal_speed_back():
    assert moves_of([drive(30, cm_s=10), {"type": "turn", "deg": 90}, drive(20)]) == [
        ("settings", 100.0), ("straight", 300.0),
        ("settings", 200.0), ("straight", 200.0)]


def test_two_limited_steps_in_a_row_do_not_restore_between_them():
    assert moves_of([drive(30, cm_s=10), {"type": "turn", "deg": 90},
                     drive(20, cm_s=10)]) == [
        ("settings", 100.0), ("straight", 300.0), ("straight", 200.0),
        ("settings", 200.0)]


def test_a_different_limit_on_the_next_step_changes_straight_to_it():
    assert moves_of([drive(30, cm_s=10), drive(20, cm_s=5)]) == [
        ("settings", 100.0), ("straight", 300.0),
        ("settings", 50.0), ("straight", 200.0),
        ("settings", 200.0)]


def test_a_run_without_limits_has_no_settings_calls():
    assert moves_of([drive(30), {"type": "turn", "deg": 90}, drive(20)]) == [
        ("straight", 300.0), ("straight", 200.0)]


def test_a_limit_on_toPoint_slows_its_straight_not_its_turn():
    """The turn to face the point comes first, at the robot's own turn rate
    -- a limit is linear speed only -- so settings() lands after it."""
    compiled = compile_drive_program(run_of([
        {"type": "toPoint", "x": 60, "y": 60, "speedLimit_cm_s": 10}]))
    assert compiled["ok"], compiled["diagnostics"]

    assert [m["op"] for m in compiled["moves"]] == [
        "turn_to", "settings", "straight", "settings"]


def test_actions_still_split_a_limited_leg():
    compiled = compile_drive_program(run_of([{
        "type": "drive", "cm": 60, "speedLimit_cm_s": 10,
        "actions": [{"id": "a1", "at": {"cm": 20}}]}]))
    assert compiled["ok"], compiled["diagnostics"]

    assert [m["op"] for m in compiled["moves"]] == [
        "settings", "straight", "action", "straight", "settings"]


def test_the_restore_reads_the_robots_own_straight_speed():
    run = dict(run_of([drive(80, cm_s=10)]), robot = {
        "name": "practice bot",
        "planning": {"track_width_cm": 16, "max_velocity_cm_s": 64.3},
        "driveBase": {"straight_speed": 350}})

    settings = [m for m in compile_drive_program(run)["moves"] if m["op"] == "settings"]
    assert [m["straight_speed"] for m in settings] == [100.0, 350.0]


def test_a_speed_limit_reaches_build_runs_hub_file():
    """driveProgram.py proves the move list above; what matters here is only
    that build_run's own wiring hands the same file over."""
    result = build([drive(80, cm_s=10)])

    assert result["ok"], result["diagnostics"]
    assert ("    drive.settings(straight_speed=100)\n"
            "    drive.straight(800)\n"
            "    drive.settings(straight_speed=200)\n") in result["module_text"]


# --- the planner's own preview ----------------------------------------------

def test_a_speed_limit_slows_the_run_down():
    plain = build([drive(80)])
    slowed = build([drive(80, cm_s=10)])

    assert plain["ok"] and slowed["ok"], (plain["diagnostics"], slowed["diagnostics"])
    assert slowed["total_ms"] > plain["total_ms"]


def test_speed_is_restored_on_the_next_unlimited_step():
    """If the restore were dropped, the second drive would stay slow, and
    the run would take as long as one where both are limited."""
    first_only = build([drive(40, cm_s=10), {"type": "turn", "deg": 90}, drive(40)])
    both = build([drive(40, cm_s=10), {"type": "turn", "deg": 90}, drive(40, cm_s=10)])
    neither = build([drive(40), {"type": "turn", "deg": 90}, drive(40)])

    assert first_only["ok"] and both["ok"] and neither["ok"]
    assert neither["total_ms"] < first_only["total_ms"] < both["total_ms"]


def test_a_limit_on_the_second_of_two_merged_drives_starts_where_it_does():
    """Two drives the builder merges into one segment: the limit has to land
    at the second one's start within that segment, through the same
    _at_within_step every action uses -- not at the segment's own start."""
    second_only = build([drive(30), drive(30, cm_s=10)])
    both = build([drive(30, cm_s=10), drive(30, cm_s=10)])

    assert second_only["ok"] and both["ok"]
    assert second_only["total_ms"] < both["total_ms"]


def test_a_toPoint_step_can_carry_a_speed_limit():
    plain = build([{"type": "toPoint", "x": 0, "y": 60}])
    slowed = build([{"type": "toPoint", "x": 0, "y": 60, "speedLimit_cm_s": 10}])

    assert slowed["ok"], slowed["diagnostics"]
    assert slowed["total_ms"] > plain["total_ms"]


def test_a_toPose_step_can_carry_a_speed_limit():
    """Point and pose segments both used to drop a constraint added to them
    -- generate() re-copied their built primitives with the primitives' own
    original constraints -- so before 6.1 a limit on either built fine and
    changed nothing in the preview."""
    plain = build([{"type": "toPose", "x": 40, "y": 60, "head": 90}])
    slowed = build([{"type": "toPose", "x": 40, "y": 60, "head": 90,
                     "speedLimit_cm_s": 10}])

    assert slowed["ok"], slowed["diagnostics"]
    assert slowed["total_ms"] > plain["total_ms"]


def test_a_bare_step_with_no_speed_limit_is_unaffected():
    assert build([drive(80)])["total_ms"] == build([
        {"type": "drive", "cm": 80, "speedLimit_cm_s": None}])["total_ms"]


# --- what is refused ---------------------------------------------------------

def test_zero_speed_is_an_error():
    result = build([drive(80, cm_s=0)])

    assert not result["ok"]
    assert any("positive speed" in message for message in errors(result))


def test_negative_speed_is_an_error():
    assert not build([drive(80, cm_s=-5)])["ok"]


def test_a_speed_that_is_not_a_number_is_an_error():
    result = build([drive(80, cm_s="fast")])

    assert not result["ok"]
    assert any("needs a speed" in message for message in errors(result))


def test_a_turn_step_cannot_carry_a_speed_limit():
    """Turning uses angular speed, which a limit does not touch -- offering
    it on a turn would silently do nothing, so it is refused instead."""
    result = build([{"type": "turn", "deg": 90, "speedLimit_cm_s": 10}])

    assert not result["ok"]
    assert any("drives somewhere" in message for message in errors(result))


def test_a_wait_step_cannot_carry_a_speed_limit():
    assert not build([{"type": "wait", "ms": 500, "speedLimit_cm_s": 10}])["ok"]


def test_a_limit_no_slower_than_normal_is_a_warning():
    """The default team robot drives at 200 mm/s -- 4.5's own default limit
    of 20 cm/s was exactly that, so an untouched limit visibly did nothing."""
    result = build([drive(80, cm_s=20)])

    assert result["ok"], result["diagnostics"]
    assert any("no slower than the robot's normal speed of 20 cm/s" in d["message"]
               for d in result["diagnostics"] if d["level"] == "warning")


def test_an_old_from_to_limit_is_refused_not_guessed_at():
    """The web page converts these when a run is opened (upgradeRun in
    store.ts), so only a hand-edited file reaches this."""
    result = build([{"type": "drive", "cm": 80, "speedLimits": [
        {"id": "s1", "from": {"cm": 30}, "to": {"cm": 60}, "cm_s": 10}]}])

    assert not result["ok"]
    assert any("old format" in message for message in errors(result))


# --- step 3.4's chain view ---------------------------------------------------

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
    -- run it for real and compare."""
    result = build([drive(30), drive(30, cm_s=10), {"type": "turn", "deg": 90}, drive(30)])

    assert result["ok"], result["diagnostics"]
    assert "needs: from pythfinder import Constraints, Constraints2D" in result["builder_source"]
    assert result["builder_source"].count("addRelativeDisplacementConstraints") == 2

    trajectory = run_chain(result["builder_source"])
    assert trajectory.TIME == result["total_ms"]


def test_the_chain_note_is_absent_without_a_speed_limit():
    assert "needs:" not in build([drive(80)])["builder_source"]
