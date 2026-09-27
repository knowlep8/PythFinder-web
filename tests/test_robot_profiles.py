"""Step 5.3's robot shape: a named profile, planning numbers nested under
"planning", DriveBase numbers alongside them and read by nothing here.

The browser always sends this shape now, but robot_from_description has to
go on reading the two shapes that came before it -- "fll_team" and a bare
object of planning numbers -- since old runs, and their own tests, still use
them. See docs/web-planner.md, step 5.3, and headless.py's own docstring.
"""

from pythfinder.headless import build_run, robot_from_description


TEMPLATE_RUN = {
    "version": 2,
    "name": "run_a",
    "steps_ms": 6,
    "robot": "fll_team",
    "start": {"x": -46, "y": -83, "head": 0},
    "steps": [
        {"type": "drive", "cm": 75},
        {"type": "wait", "ms": 600},
        {"type": "turn", "deg": 90},
        {"type": "drive", "cm": 30},
        {"type": "toPose", "x": -46, "y": -83, "head": 0},
    ],
}

# what the picker's "Team robot" profile sends, once a run has picked it --
# the same planning numbers robotConfig.py's FLL_ROBOT carries, plus a
# DriveBase group that only matters once 5.5/5.6 exist
TEAM_PROFILE = {
    "name": "Team robot",
    "planning": {
        "track_width_cm": 16,
        "max_velocity_cm_s": 64.3,
        "center_offset_cm": -3.5,
        "width_cm": 19,
        "length_cm": 14,
    },
    "driveBase": {
        "wheel_diameter_mm": 56,
        "axle_track_mm": 160,
        "straight_speed": 200,
        "straight_acceleration": 400,
        "turn_rate": 150,
        "turn_acceleration": 300,
        "use_gyro": True,
    },
}


def test_a_named_profile_plans_the_same_run_as_fll_team():
    """The team's own numbers, just nested under "planning" now.

    Step 5.7 wired the DriveBase numbers into module_text itself (core.
    configure(...) is written from them, and the "no DriveBase numbers of
    its own" warning goes away) -- so a profile's own file is no longer
    identical to "fll_team"'s the way it was before 5.5 existed. What this
    test is actually about -- the *planning* numbers under "planning" being
    read the same way whichever shape they arrive in -- is the path and
    timing, not the download, so that is what is compared now.
    """
    run = dict(TEMPLATE_RUN)
    run["robot"] = TEAM_PROFILE

    result = build_run(run)
    plain = build_run(TEMPLATE_RUN)

    assert result["ok"]
    assert result["total_ms"] == plain["total_ms"]
    assert result["poses"] == plain["poses"]


def test_driveBase_numbers_are_ignored_for_planning():
    """5.4/5.6's job, not this one -- changing them must change the planned
    path not at all, even though (since step 5.7) they do change the
    download's own core.configure(...) line."""
    tuned = dict(TEAM_PROFILE)
    tuned["driveBase"] = dict(TEAM_PROFILE["driveBase"],
                              wheel_diameter_mm = 62.4,
                              axle_track_mm = 158,
                              use_gyro = False)

    run = dict(TEMPLATE_RUN)
    run["robot"] = tuned

    result = build_run(run)
    untuned = build_run(dict(TEMPLATE_RUN, robot = TEAM_PROFILE))

    assert result["ok"]
    assert result["total_ms"] == untuned["total_ms"]
    assert result["poses"] == untuned["poses"]


def test_a_named_profile_with_no_planning_key_is_still_rejected():
    """A profile is not just any object -- it still needs the numbers."""
    run = dict(TEMPLATE_RUN)
    run["robot"] = {"name": "half-built profile", "driveBase": TEAM_PROFILE["driveBase"]}

    result = build_run(run)

    assert not result["ok"]
    assert "robot cannot be used" in result["diagnostics"][0]["message"]


def test_robot_from_description_reads_all_three_shapes_the_same_way():
    """"fll_team", a bare RobotNumbers object, and a named profile all have
    to describe the same robot for old runs to keep meaning what they meant.
    """
    team = robot_from_description("fll_team")
    bare = robot_from_description({"track_width_cm": 16,
                                   "max_velocity_cm_s": 64.3,
                                   "center_offset_cm": -3.5,
                                   "width_cm": 19,
                                   "length_cm": 14})
    named = robot_from_description(TEAM_PROFILE)

    for other in (bare, named):
        assert other.kinematics.track_width == team.kinematics.track_width
        assert other.REAL_MAX_VEL == team.REAL_MAX_VEL
        assert other.WIDTH_CM == team.WIDTH_CM
        assert other.LENGTH_CM == team.LENGTH_CM


def test_an_old_run_saying_fll_team_still_loads_as_the_team_robot():
    """Nothing about the migration to named profiles touches this string --
    the browser is what maps it to a picker selection (step 5.3); build_run
    only ever needed it to mean the built-in robot, which it still does.
    """
    result = build_run(TEMPLATE_RUN)

    assert result["ok"]
    assert result["version"] == 2
