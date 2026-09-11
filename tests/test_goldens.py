"""Check that the library still produces the committed hub exports.

These tests are the safety net for the phase 1 refactor in
docs/web-planner.md: they say nothing about whether the motion is *good*, only
that it has not changed. See tests/golden_runs.py for the runs themselves.
"""

import pytest

from golden_runs import GOLDEN_DIR, GOLDEN_RUNS


REGENERATE = "uv run python tests/regenerate_goldens.py"


def export(trajectory, name, steps, directory):
    """Write an export and hand back its text.

    Trajectory.generate() appends the .txt itself, so it is given a path
    without one.
    """
    trajectory.generate(str(directory / name), steps = steps)
    return (directory / "{0}.txt".format(name)).read_text()


@pytest.mark.parametrize("name", sorted(GOLDEN_RUNS))
def test_export_matches_golden(name, sim, tmp_path):
    build, steps = GOLDEN_RUNS[name]
    golden = GOLDEN_DIR / "{0}.txt".format(name)

    assert golden.exists(), (
        "no golden for '{0}' yet -- create it with:\n    {1} {0}"
        .format(name, REGENERATE))

    actual = export(build(sim), name, steps, tmp_path)
    expected = golden.read_text()

    if actual == expected:
        return

    # The export is one very long line, so a character index locates the change
    # far better than a diff of two enormous strings would.
    for i, (left, right) in enumerate(zip(actual, expected)):
        if left != right:
            context = 60
            pytest.fail(
                "'{0}' exports differently than tests/golden/{0}.txt, from "
                "character {1}:\n"
                "  now:    ...{2}...\n"
                "  golden: ...{3}...\n"
                "The robot would drive differently. If that is intended, "
                "regenerate with:\n    {4} {0}"
                .format(name, i,
                        actual[max(0, i - context):i + context],
                        expected[max(0, i - context):i + context],
                        REGENERATE))

    pytest.fail(
        "'{0}' export is {1} characters, golden is {2}. The run got {3}. "
        "If that is intended, regenerate with:\n    {4} {0}"
        .format(name, len(actual), len(expected),
                "longer" if len(actual) > len(expected) else "shorter",
                REGENERATE))


def test_build_is_deterministic(sim, tmp_path):
    """The same run built twice must export the same bytes.

    Without this, a golden failure could just be noise.
    """
    build, steps = GOLDEN_RUNS["template_run"]

    first = export(build(sim), "first", steps, tmp_path)
    second = export(build(sim), "second", steps, tmp_path)

    assert first == second


@pytest.mark.parametrize("asked_for, same_as", [
    ("to_point_tangent_head", "to_point"),
    ("to_pose_tangent_head", "to_pose"),
    ("to_pose_linear_head", "to_pose"),
])
def test_heading_modes_collapse_on_a_tank_drive(asked_for, same_as, sim,
                                                tmp_path):
    """The heading variants are one call on this robot.

    A tank drive cannot move sideways, so the segments force a tangent heading
    whatever was asked for. Worth asserting rather than leaving implied: it is
    why the web planner should not offer these as choices, and if a refactor
    lost the NON_HOLONOMIC check these paths would silently diverge.
    """
    variant, steps = GOLDEN_RUNS[asked_for]
    plain, _ = GOLDEN_RUNS[same_as]

    assert (export(variant(sim), asked_for, steps, tmp_path)
            == export(plain(sim), same_as, steps, tmp_path))


def test_consecutive_lines_merge(sim, tmp_path):
    """30cm then 45cm is one 75cm move, not two.

    The builder combines them, which is why the run template can add distance
    to a straight without paying for an extra accelerate-and-stop.
    """
    merged, steps = GOLDEN_RUNS["line_merged"]
    single, _ = GOLDEN_RUNS["line_forward"]

    assert (export(merged(sim), "merged", steps, tmp_path)
            == export(single(sim), "single", steps, tmp_path))


@pytest.mark.parametrize("name", sorted(GOLDEN_RUNS))
def test_builds_without_a_simulator(name):
    """Every run exports the same bytes with no simulator involved.

    This is what the web planner needs: no window, no preset, no pygame
    surfaces -- just a robot description. It takes no `sim` fixture at all, so
    a trajectory quietly depending on one would fail here rather than pass by
    accident.
    """
    build, steps = GOLDEN_RUNS[name]
    golden = GOLDEN_DIR / "{0}.txt".format(name)

    assert build(None).text(steps = steps) == golden.read_text(), (
        "'{0}' exports differently when built without a simulator".format(name))
