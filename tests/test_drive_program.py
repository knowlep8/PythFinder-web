"""Step 5.4: a run description compiled into a flat DriveBase move list.

`build_run` is the reference throughout. Nothing here should end up
anywhere the robot has never effectively been: for every golden run that has
a JSON shape at all, the pose this module predicts at the end of the run has
to match build_run's own end pose to within 1mm and 0.1 degree, and where
both agree on what fires, they have to agree on the order.

Some of golden_runs.py's own runs have no run-description equivalent at all
-- markers_absolute, constraints_marker and interrupt_marker are built
straight against TrajectoryBuilder, using calls the JSON run format never
exposes (addTemporalMarker/addDisplacementMarker are "final" markers with no
"at" shape in the format at all; a bare, unpaired constraints call and a raw
interrupt have no run-description encoding either -- the format only offers
a paired "speed limit"). Those are recorded below as skips, with the reason,
rather than silently left out.
"""

import math

import pytest

from pythfinder.headless import build_run
from pythfinder.Export.driveProgram import (DEFAULT_STRAIGHT_SPEED_MM_S,
                                             compile_drive_program)


START = {"x": -46, "y": -83, "head": 0}


def run_with(steps, start=None, robot="fll_team"):
    return {"version": 2, "name": "x", "steps_ms": 6, "robot": robot,
            "start": start or dict(START), "steps": steps}


def ops(result, op=None):
    moves = result["moves"]
    return [m for m in moves if op is None or m["op"] == op]


def action_ids(result):
    return [m["id"] for m in ops(result, "action")]


def heading_difference(a, b):
    """The smallest angle between two headings, in degrees, 0..180."""
    return abs(((a - b) + 180) % 360 - 180)


def assert_end_poses_match(run, tolerance_mm=1.0, tolerance_deg=0.1):
    """The one check every comparable golden run has to pass: given the same
    description, this module's tracked end pose and build_run's own end pose
    -- taken from the real trajectory it built -- have to agree."""
    reference = build_run(run)
    compiled = compile_drive_program(run)

    assert reference["ok"], reference["diagnostics"]
    assert compiled["ok"], compiled["diagnostics"]

    expected = reference["poses"][-1]
    actual = compiled["end_pose"]

    # 1mm = 0.1cm; the library works in cm throughout.
    assert abs(expected["x"] - actual["x"]) <= tolerance_mm / 10
    assert abs(expected["y"] - actual["y"]) <= tolerance_mm / 10
    assert heading_difference(expected["head"], actual["head"]) <= tolerance_deg

    return reference, compiled


# --- every golden run that has a JSON shape at all --------------------------

COMPARABLE_RUNS = {
    "line_forward": [{"type": "drive", "cm": 75}],
    "line_backward": [{"type": "drive", "cm": -40}],
    "line_merged": [{"type": "drive", "cm": 30}, {"type": "drive", "cm": 45}],
    "wait_merged": [{"type": "drive", "cm": 20},
                    {"type": "wait", "ms": 300}, {"type": "wait", "ms": 400}],
    "turn_ccw": [{"type": "turn", "deg": 90}],
    "turn_cw": [{"type": "turn", "deg": -90}],
    "turn_reversed": [{"type": "turn", "deg": 90, "reversed": True}],
    "to_point": [{"type": "toPoint", "x": 0, "y": -40}],
    "to_point_reversed": [{"type": "toPoint", "x": 0, "y": -40, "reversed": True}],
    "to_pose": [{"type": "toPose", "x": 0, "y": -40, "head": 90}],
    "to_pose_reversed": [{"type": "toPose", "x": 0, "y": -40, "head": 90,
                          "reversed": True}],
}


@pytest.mark.parametrize("name", sorted(COMPARABLE_RUNS))
def test_end_pose_matches_build_run(name):
    assert_end_poses_match(run_with(COMPARABLE_RUNS[name]))


def test_to_point_tangent_head_and_to_pose_variants_are_not_separately_tested():
    """Step 1.1 already proved toPointTangentHead/toPoseTangentHead/
    toPoseLinearHead collapse into toPoint/toPose on a tank drive -- the
    run-description format has no field to ask for the other headings in the
    first place, so there is nothing extra for this module to get right."""
    assert True


@pytest.mark.parametrize("name,reason", [
    ("markers_absolute",
     "addTemporalMarker/addDisplacementMarker are final markers with no "
     "'at' shape in the run-description format -- every action the format "
     "can describe is relative to a step"),
    ("constraints_marker",
     "a bare addRelativeDisplacementConstraints with no matching restore "
     "has no run-description encoding -- the format only ever offers a "
     "paired 'speed limit', from and to"),
    ("interrupt_marker",
     "interruptDisplacement has no run-description encoding at all; the "
     "editor never offers one"),
])
def test_golden_with_no_run_description_equivalent(name, reason):
    pytest.skip("{0}: {1}".format(name, reason))


def test_template_run_end_pose_still_matches_despite_a_rejected_action():
    """The real run the team drives has one action build_run still accepts
    that this module does not: "arm up" is placed with {"ms": -1} on a
    drive step (see docs/web-planner.md, step 5.2's still-open work) --
    exactly the "timed action on a moving step" case this module refuses
    rather than guesses at. That does not touch the geometry, only whether a
    marker exists, so the end pose still has to agree; the full action-order
    comparison golden_runs.py's other runs get is not attempted here, because
    build_run fires "arm up" and this module never does -- there being
    nothing for the two lists to agree about for that one action.
    """
    run = run_with([
        {"type": "drive", "cm": 75,
         "actions": [{"id": "arm_down", "at": {"cm": 35}, "label": "arm down"}]},
        {"type": "wait", "ms": 600},
        {"type": "turn", "deg": 90},
        {"type": "drive", "cm": 30,
         "actions": [{"id": "arm_up", "at": {"ms": -1}, "label": "arm up"}]},
        {"type": "toPose", "x": -46, "y": -83, "head": 0},
    ])

    reference = build_run(run)
    compiled = compile_drive_program(run)

    assert reference["ok"]
    assert not compiled["ok"], "an ms-placed action on a drive should be refused"

    rejected = [d for d in compiled["diagnostics"] if d["level"] == "error"]
    assert len(rejected) == 1
    assert rejected[0]["step"] == 3
    assert "not supported" in rejected[0]["message"]

    expected = reference["poses"][-1]
    actual = compiled["end_pose"]
    assert abs(expected["x"] - actual["x"]) <= 0.1
    assert abs(expected["y"] - actual["y"]) <= 0.1
    assert heading_difference(expected["head"], actual["head"]) <= 0.1

    # the one action this module *can* place still comes through, and still
    # comes through first
    assert action_ids(compiled) == ["arm_down"]


def test_markers_relative_cm_only_variant_matches_in_full():
    """markers_relative's own two markers are {"cm": 35} and {"ms": -1} on
    the same drive step -- the second hits the same "ms on a moving step" gap
    as the template run's "arm up". This variant keeps only the cm-based
    marker, which both functions handle the same way, and checks the two
    agree on it completely: pose, and firing order."""
    run = run_with([{"type": "drive", "cm": 75,
                     "actions": [{"id": "m1", "at": {"cm": 35}}]}])

    reference, compiled = assert_end_poses_match(run)

    assert [m["id"] for m in reference["markers"]] == ["m1"]
    assert action_ids(compiled) == ["m1"]


# --- splitting ---------------------------------------------------------------

def test_a_drive_is_split_at_every_action():
    run = run_with([{"type": "drive", "cm": 40, "actions": [
        {"id": "a1", "at": {"cm": 15}, "label": "one"},
        {"id": "a2", "at": {"cm": 30}, "label": "two"}]}])

    compiled = compile_drive_program(run)
    assert compiled["ok"], compiled["diagnostics"]

    straights = ops(compiled, "straight")
    assert [m["mm"] for m in straights] == [150.0, 150.0, 100.0]
    assert action_ids(compiled) == ["a1", "a2"]

    # the split pieces flow into each other; only the last one stops
    assert [m["then"] for m in straights] == ["none", "none", "stop"]


def test_a_drive_is_split_at_speed_limit_edges():
    run = run_with([{"type": "drive", "cm": 80, "speedLimits": [
        {"id": "s1", "from": {"cm": 30}, "to": {"cm": 60}, "cm_s": 10}]}])

    compiled = compile_drive_program(run)
    assert compiled["ok"], compiled["diagnostics"]

    assert [m["op"] for m in compiled["moves"]] == [
        "straight", "settings", "straight", "settings", "straight"]

    straights = ops(compiled, "straight")
    assert [m["mm"] for m in straights] == [300.0, 300.0, 200.0]
    assert [m["then"] for m in straights] == ["none", "none", "stop"]

    settings = ops(compiled, "settings")
    assert settings[0]["straight_speed"] == 100.0   # 10cm/s -> mm/s
    assert settings[1]["straight_speed"] == DEFAULT_STRAIGHT_SPEED_MM_S


def test_a_speed_limit_reads_the_robots_own_straight_speed():
    robot = {"name": "practice bot",
            "planning": {"track_width_cm": 16, "max_velocity_cm_s": 64.3},
            "driveBase": {"straight_speed": 350}}

    run = run_with([{"type": "drive", "cm": 80, "speedLimits": [
        {"id": "s1", "from": {"cm": 30}, "to": {"cm": 60}, "cm_s": 10}]}],
        robot = robot)

    settings = ops(compile_drive_program(run), "settings")
    assert settings[1]["straight_speed"] == 350.0


def test_a_speed_limit_only_makes_sense_on_a_step_that_drives():
    run = run_with([{"type": "turn", "deg": 90, "speedLimits": [
        {"id": "s1", "from": {"cm": 0}, "to": {"cm": 10}, "cm_s": 10}]}])

    result = compile_drive_program(run)
    assert not result["ok"]
    assert any("drives somewhere" in d["message"] for d in result["diagnostics"])


# --- negative cm ("from the end") --------------------------------------------

def test_negative_cm_counts_back_from_the_ends_own_length():
    run = run_with([{"type": "drive", "cm": 60,
                     "actions": [{"id": "a", "at": {"cm": -5}}]}])

    compiled = compile_drive_program(run)
    straights = ops(compiled, "straight")

    assert [m["mm"] for m in straights] == [550.0, 50.0]


def test_negative_cm_on_a_to_point_leg_counts_back_from_its_own_length():
    """toPoint/toPose measure "cm" along the straight part only -- a leg
    that has no other length to speak of."""
    run = run_with([{"type": "toPoint", "x": 0, "y": -40,
                     "actions": [{"id": "a", "at": {"cm": -10}}]}])

    reference = build_run(run)
    compiled = compile_drive_program(run)

    assert reference["ok"] and compiled["ok"]
    assert [m["id"] for m in reference["markers"]] == ["a"]
    assert action_ids(compiled) == ["a"]


# --- reversed drives ----------------------------------------------------------

def test_a_reversed_to_point_faces_away_and_drives_a_negative_distance():
    run = run_with([{"type": "toPoint", "x": 0, "y": -40, "reversed": True}])
    compiled = compile_drive_program(run)
    assert compiled["ok"], compiled["diagnostics"]

    turn = ops(compiled, "turn_to")[0]
    straight = ops(compiled, "straight")[0]

    unreversed = compile_drive_program(
        run_with([{"type": "toPoint", "x": 0, "y": -40}]))
    unreversed_turn = ops(unreversed, "turn_to")[0]
    unreversed_straight = ops(unreversed, "straight")[0]

    # facing +180 from the unreversed tangent, driving the same distance
    # backward -- docs/web-planner.md, step 5.4's own rule
    assert heading_difference(turn["deg"], unreversed_turn["deg"] + 180) < 1e-6
    assert straight["mm"] == pytest.approx(-unreversed_straight["mm"])


def test_a_reversed_drive_is_just_a_negative_cm_no_reversed_field():
    """A plain drive step has no "reversed" key at all -- the sign of "cm"
    already says which way, and the heading never changes."""
    run = run_with([{"type": "drive", "cm": -40}])
    compiled = compile_drive_program(run)

    assert ops(compiled, "turn_to") == []
    assert [m["mm"] for m in ops(compiled, "straight")] == [-400.0]


# --- toPoint/toPose decomposition --------------------------------------------

def test_to_point_becomes_a_turn_then_a_straight():
    run = run_with([{"type": "toPoint", "x": 0, "y": -40}], start = {"x": -46, "y": -83, "head": 0})
    compiled = compile_drive_program(run)

    assert [m["op"] for m in compiled["moves"]] == ["turn_to", "straight"]

    dx, dy = 0 - (-46), -40 - (-83)
    expected_face = math.degrees(math.atan2(dy, dx)) % 360
    expected_distance = math.hypot(dx, dy) * 10   # cm -> mm

    turn, straight = compiled["moves"]
    # the module rounds a turn to 4 decimal places on the way out
    assert turn["deg"] == pytest.approx(expected_face, abs = 1e-4)
    assert straight["mm"] == pytest.approx(expected_distance, abs = 1e-3)


def test_to_pose_becomes_turn_straight_turn():
    run = run_with([{"type": "toPose", "x": 0, "y": -40, "head": 90}],
                   start = {"x": -46, "y": -83, "head": 0})
    compiled = compile_drive_program(run)

    assert [m["op"] for m in compiled["moves"]] == ["turn_to", "straight", "turn_to"]
    assert compiled["moves"][-1]["deg"] == pytest.approx(90.0)


def test_to_pose_final_turn_is_unaffected_by_reversed():
    """reversed only changes how the robot gets there, never the heading it
    ends up at."""
    plain = compile_drive_program(run_with(
        [{"type": "toPose", "x": 0, "y": -40, "head": 90}]))
    reversed_ = compile_drive_program(run_with(
        [{"type": "toPose", "x": 0, "y": -40, "head": 90, "reversed": True}]))

    assert plain["end_pose"]["head"] == reversed_["end_pose"]["head"] == 90.0


def test_a_toPoint_at_the_current_position_keeps_the_current_heading():
    """No line to point along when start and target coincide -- the same
    thing an untouched AngularSegment would do."""
    run = run_with([{"type": "toPoint", "x": -46, "y": -83}],
                   start = {"x": -46, "y": -83, "head": 33})
    compiled = compile_drive_program(run)

    assert ops(compiled, "turn_to") == []
    assert ops(compiled, "straight") == []
    assert compiled["end_pose"]["head"] == 33


# --- skipping a turn that changes nothing ------------------------------------

def test_a_turn_to_the_current_heading_is_skipped():
    run = run_with([{"type": "turn", "deg": 0}])
    compiled = compile_drive_program(run)

    assert compiled["moves"] == []
    assert compiled["end_pose"]["head"] == 0


def test_turns_reversed_flag_never_changes_the_final_heading():
    """'reversed' on a turn only ever picks which way the hub spins to get
    there (5.6) -- the target heading in the move list is the same either
    way."""
    plain = compile_drive_program(run_with([{"type": "turn", "deg": 137}]))
    reversed_ = compile_drive_program(
        run_with([{"type": "turn", "deg": 137, "reversed": True}]))

    assert plain["moves"] == reversed_["moves"]


def test_a_skipped_turn_does_not_break_a_straight_run_either_side():
    """Two drives either side of a no-op turn are still one continuous roll,
    the same as if the turn were not written at all."""
    with_noop_turn = compile_drive_program(run_with([
        {"type": "drive", "cm": 20}, {"type": "turn", "deg": 0},
        {"type": "drive", "cm": 20}]))
    without = compile_drive_program(run_with([
        {"type": "drive", "cm": 20}, {"type": "drive", "cm": 20}]))

    assert with_noop_turn["moves"] == without["moves"]


# --- then: none / stop --------------------------------------------------------

def test_consecutive_same_direction_drives_flow_into_each_other():
    compiled = compile_drive_program(
        run_with([{"type": "drive", "cm": 30}, {"type": "drive", "cm": 30}]))

    straights = ops(compiled, "straight")
    assert [m["then"] for m in straights] == ["none", "stop"]


def test_opposite_direction_drives_stop_in_between():
    compiled = compile_drive_program(
        run_with([{"type": "drive", "cm": 30}, {"type": "drive", "cm": -30}]))

    straights = ops(compiled, "straight")
    assert [m["then"] for m in straights] == ["stop", "stop"]


def test_a_drive_before_a_turn_stops():
    compiled = compile_drive_program(
        run_with([{"type": "drive", "cm": 30}, {"type": "turn", "deg": 90}]))

    assert ops(compiled, "straight")[0]["then"] == "stop"


def test_a_drive_before_a_wait_stops():
    compiled = compile_drive_program(
        run_with([{"type": "drive", "cm": 30}, {"type": "wait", "ms": 500}]))

    assert ops(compiled, "straight")[0]["then"] == "stop"


def test_a_drive_before_an_arm_step_stops():
    compiled = compile_drive_program(run_with([
        {"type": "drive", "cm": 30},
        {"type": "armStep", "motor": "leftTask", "call": "run_angle",
         "angle": 90, "speed": 500}]))

    assert ops(compiled, "straight")[0]["then"] == "stop"


def test_the_last_straight_of_the_run_stops():
    compiled = compile_drive_program(run_with([{"type": "drive", "cm": 30}]))
    assert ops(compiled, "straight")[-1]["then"] == "stop"


def test_an_action_between_two_pieces_does_not_force_a_stop():
    """The plan's own example: db.straight(a, then=NONE); action(); straight(b)."""
    compiled = compile_drive_program(run_with([{"type": "drive", "cm": 40,
        "actions": [{"id": "a1", "at": {"cm": 25}}]}]))

    straights = ops(compiled, "straight")
    assert [m["then"] for m in straights] == ["none", "stop"]
    assert [m["op"] for m in compiled["moves"]] == ["straight", "action", "straight"]


# --- timed placement: trivial in a wait, refused on a moving step -----------

def test_a_wait_is_split_around_an_action_placed_in_time():
    compiled = compile_drive_program(run_with([{"type": "wait", "ms": 600,
        "actions": [{"id": "a1", "at": {"ms": 200}}]}]))

    assert [m["op"] for m in compiled["moves"]] == ["wait", "action", "wait"]
    waits = ops(compiled, "wait")
    assert [m["ms"] for m in waits] == [200, 400]


def test_a_negative_ms_in_a_wait_counts_back_from_its_own_end():
    compiled = compile_drive_program(run_with([{"type": "wait", "ms": 600,
        "actions": [{"id": "a1", "at": {"ms": -100}}]}]))

    waits = ops(compiled, "wait")
    assert [m["ms"] for m in waits] == [500, 100]


def test_two_actions_in_one_wait_split_it_into_three_pieces():
    compiled = compile_drive_program(run_with([{"type": "wait", "ms": 1000,
        "actions": [{"id": "w1", "at": {"ms": 200}},
                    {"id": "w2", "at": {"ms": -100}}]}]))

    assert action_ids(compiled) == ["w1", "w2"]
    assert [m["ms"] for m in ops(compiled, "wait")] == [200, 700, 100]


@pytest.mark.parametrize("step", [
    {"type": "drive", "cm": 40, "actions": [{"id": "a", "at": {"ms": 100}}]},
    {"type": "turn", "deg": 90, "actions": [{"id": "a", "at": {"ms": 100}}]},
    {"type": "toPoint", "x": 0, "y": -40, "actions": [{"id": "a", "at": {"ms": 100}}]},
    {"type": "toPose", "x": 0, "y": -40, "head": 90,
     "actions": [{"id": "a", "at": {"ms": 100}}]},
])
def test_a_timed_action_on_a_moving_step_is_refused(step):
    result = compile_drive_program(run_with([step]))

    assert not result["ok"]
    errors = [d for d in result["diagnostics"] if d["level"] == "error"]
    assert len(errors) == 1
    assert errors[0]["step"] == 0
    assert "not supported" in errors[0]["message"]

    # the geometry still happens -- only the marker is refused
    assert ops(result, "straight" if step["type"] != "turn" else "turn_to")


def test_a_cm_action_on_a_turn_fires_at_its_start():
    compiled = compile_drive_program(run_with([
        {"type": "turn", "deg": 90, "actions": [{"id": "a1", "at": {"cm": 0}}]}]))

    assert compiled["ok"], compiled["diagnostics"]
    assert [m["op"] for m in compiled["moves"]] == ["action", "turn_to"]


def test_a_nonzero_cm_action_on_a_turn_warns_but_still_fires_at_the_start():
    compiled = compile_drive_program(run_with([
        {"type": "turn", "deg": 90, "actions": [{"id": "a1", "at": {"cm": 5}}]}]))

    assert compiled["ok"]   # a warning, not an error
    warnings = [d for d in compiled["diagnostics"] if d["level"] == "warning"]
    assert len(warnings) == 1
    assert "next step" in warnings[0]["suggestion"]
    assert action_ids(compiled) == ["a1"]


# --- armStep -------------------------------------------------------------

def test_an_arm_step_becomes_one_arm_move():
    run = run_with([{"type": "armStep", "motor": "rightTask", "call": "run_angle",
                     "angle": 45, "speed": 300, "actions": [{"id": "lift"}]}])

    compiled = compile_drive_program(run)
    assert compiled["ok"], compiled["diagnostics"]
    assert [m["op"] for m in compiled["moves"]] == ["arm"]

    move = compiled["moves"][0]
    assert move["motor"] == "rightTask"
    assert move["call"] == "run_angle"
    assert move["angle"] == 45
    assert move["speed"] == 300
    assert move["wait"] is True

    # no marker mechanism left for an arm step -- see _emit_arm's own note
    assert "id" not in move


def test_an_arm_step_does_not_move_the_tracked_pose():
    run = run_with([{"type": "armStep", "motor": "leftTask", "call": "run_angle",
                     "angle": 90, "speed": 500}])

    compiled = compile_drive_program(run)
    assert compiled["end_pose"]["x"] == START["x"]
    assert compiled["end_pose"]["y"] == START["y"]


def test_an_explicit_do_overrides_an_arm_steps_own_motor_call():
    run = run_with([{"type": "armStep", "motor": "leftTask", "call": "run_angle",
                     "angle": 90, "speed": 500,
                     "actions": [{"id": "lift", "do": {"code": "core.leftTask.run(999)"}}]}])

    compiled = compile_drive_program(run)
    move = compiled["moves"][0]
    assert move["code"] == "core.leftTask.run(999)"
    assert "call" not in move   # the override replaces the whole command


# --- general shape -------------------------------------------------------

def test_an_empty_run_says_so():
    result = compile_drive_program(run_with([]))
    assert not result["ok"]
    assert result["moves"] == []


def test_an_unknown_step_is_a_diagnostic_not_an_exception():
    result = compile_drive_program(run_with([{"type": "teleport", "cm": 5}]))
    assert not result["ok"]
    assert "teleport" in result["diagnostics"][0]["message"]
    assert result["diagnostics"][0]["step"] == 0


def test_a_step_missing_its_number_is_a_diagnostic():
    result = compile_drive_program(run_with([{"type": "drive"}]))
    assert not result["ok"]
    assert result["diagnostics"][0]["step"] == 0


def test_an_action_carries_its_do_and_label_on_the_move_itself():
    run = run_with([{"type": "drive", "cm": 20, "actions": [
        {"id": "a1", "at": {"cm": 10}, "label": "Grab",
         "do": {"motor": "leftTask", "call": "run", "speed": 500}}]}])

    move = ops(compile_drive_program(run), "action")[0]
    assert move["id"] == "a1"
    assert move["label"] == "Grab"
    assert move["do"] == {"motor": "leftTask", "call": "run", "speed": 500}


def test_default_straight_speed_matches_the_placeholder_team_robot():
    """web/src/profiles.ts's TEAM_ROBOT.driveBase.straight_speed -- see
    DEFAULT_STRAIGHT_SPEED_MM_S's own docstring for why this is the
    fallback."""
    assert DEFAULT_STRAIGHT_SPEED_MM_S == 200.0
