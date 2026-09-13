"""An action at the very start or end of a step must still fire.

The editor's "+ action while driving" button creates its action at `cm: 0` --
the moment the step begins -- because that is the only default that is right
for every step regardless of how long it is. That value was dropped silently:
markers are placed by an *open* interval test, so one landing exactly on a
segment's first or last state counts as outside it.

Nothing caught this. The fixtures in test_editor_shapes.py use `cm: 20`, and
every hand-written run in the suite picks a comfortable number in the middle,
so the one value a team member gets by clicking the button was the one value
never tested. On the field it looks like an arm that simply does not move.
"""

from pythfinder.headless import build_run


def build(step):
    return build_run({
        "version": 1,
        "name": "boundaries",
        "steps_ms": 6,
        "robot": "fll_team",
        "start": {"x": 0, "y": 0, "head": 0},
        "steps": [step],
    })


def drive_with_action(at):
    return build({"type": "drive", "cm": 40, "actions": [
        {"id": "a1", "at": at, "label": "Grab",
         "do": {"motor": "leftTask", "call": "run", "speed": 500}}]})


def dropped(result):
    return [d for d in result["diagnostics"] if "dropped" in d["message"]]


def test_an_action_at_the_start_of_a_step_fires():
    """What the editor's own button produces, and so the common case."""
    result = drive_with_action({"cm": 0})

    assert dropped(result) == [], "an action at 0cm was dropped"
    assert [m["id"] for m in result["markers"]] == ["a1"]

    # 1ms, not 0. Marker times come back one millisecond after the moment they
    # were asked for -- `ms: 0` gives 1 and `ms: 500` gives 501 -- and that
    # offset predates this work: the goldens the robot drove pin it. It is
    # asserted here rather than corrected, because changing it would move every
    # marker time in every file already driven.
    assert result["markers"][0]["time_ms"] == 1


def test_an_action_at_the_very_end_of_a_step_fires():
    result = drive_with_action({"cm": 40})

    assert dropped(result) == [], "an action at the end of the step was dropped"
    assert [m["id"] for m in result["markers"]] == ["a1"]


def test_an_action_at_zero_milliseconds_fires():
    result = drive_with_action({"ms": 0})

    assert dropped(result) == [], "an action at 0ms was dropped"
    assert [m["id"] for m in result["markers"]] == ["a1"]


def test_the_middle_of_a_step_still_works():
    """The case that always passed: it must keep passing."""
    result = drive_with_action({"cm": 20})

    assert dropped(result) == []
    assert [m["id"] for m in result["markers"]] == ["a1"]


def test_an_action_past_the_end_is_still_reported():
    """Widening the interval must not stop real mistakes being caught."""
    result = drive_with_action({"cm": 60})

    assert dropped(result), "an action beyond the step should still be reported"
    assert result["markers"] == []


def test_the_generated_file_binds_an_action_at_the_start():
    """A dropped marker is invisible until the robot does nothing, so check
    the code the hub actually runs, not just the marker list."""
    result = drive_with_action({"cm": 0})

    assert result["ok"], result["diagnostics"]
    assert "core.leftTask.run(500)" in result["module_text"]
    assert "_action_1(core)," in result["module_text"]
