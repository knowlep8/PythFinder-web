"""The shapes the editor sends must be the shapes build_run accepts.

The browser and the library agree by convention rather than by any shared
schema, so this checks the exact JSON the step list produces for its own
default blocks -- a "Move arm" step straight off the button, and a parallel
action added to a drive.

If the editor's defaults and build_run ever drift apart, a team member finds
out when their arm does nothing on the field. These are cheap; that is not.
"""

from pythfinder.headless import build_run


# exactly what runEditor.ts NEW_STEP.armStep produces
EDITOR_ARM_STEP = {
    "type": "armStep",
    "motor": "leftTask",
    "call": "run_angle",
    "angle": 90,
    "speed": 500,
    "actions": [{"id": "a1", "label": "Move arm"}],
}

# and what its "add action" produces on a drive
EDITOR_PARALLEL = {
    "type": "drive",
    "cm": 40,
    "actions": [{
        "id": "a2",
        "at": {"cm": 20},
        "label": "Grab on the way",
        "do": {"motor": "rightTask", "call": "run", "speed": 500},
    }],
}


def build(steps):
    return build_run({
        "version": 1,
        "name": "from_editor",
        "steps_ms": 6,
        "robot": "fll_team",
        "start": {"x": 0, "y": 0, "head": 0},
        "steps": steps,
    })


def test_the_editors_default_arm_step_builds_and_generates_code():
    result = build([{"type": "drive", "cm": 20}, dict(EDITOR_ARM_STEP)])

    assert result["ok"], result["diagnostics"]
    assert "core.leftTask.run_angle(500, 90, wait=True)" in result["module_text"]

    arm = result["steps"][1]
    assert arm["ends_ms"] > arm["starts_ms"], "it must occupy time on the timeline"


def test_the_editors_parallel_action_builds_and_generates_code():
    result = build([dict(EDITOR_PARALLEL)])

    assert result["ok"], result["diagnostics"]

    module = result["module_text"]
    assert "core.rightTask.run(500)" in module
    assert "wait=True" not in module, "a parallel action must never block"


def test_both_kinds_in_one_run_come_back_in_firing_order():
    result = build([
        dict(EDITOR_PARALLEL),
        dict(EDITOR_ARM_STEP, actions=[{"id": "a3", "label": "Lift"}]),
        {"type": "drive", "cm": -40},
    ])

    assert result["ok"], result["diagnostics"]
    assert [marker["id"] for marker in result["markers"]] == ["a2", "a3"]

    module = result["module_text"]
    assert module.index("_action_1(core),") < module.index("_action_2(core),")


def test_an_arm_step_with_no_action_still_drives():
    """A half-made step should not break the run while it is being typed."""
    result = build([{"type": "armStep", "motor": "leftTask",
                     "call": "run_angle", "angle": 90, "speed": 500}])

    assert result["ok"], result["diagnostics"]
    assert result["total_ms"] > 0
