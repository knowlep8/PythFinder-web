"""Step 3.3: a free-form code action, for what the motor picker cannot say.

The picker only offers the calls a parallel action may safely make --
`run` and `stop`, both instant. A team that wants to read a sensor, count
something, or run two motors from one action needs real Python, with `core`
in scope the way the generated file already gives it.

Two things this cannot check, because they need a hub: whether the code
actually does what it says, and whether Pybricks has the names it calls.
`compile()` only proves it parses -- see docs/web-planner.md, step 3.3.
"""

from pythfinder.headless import build_run


def build(steps):
    return build_run({
        "version": 1,
        "name": "code_action",
        "steps_ms": 6,
        "robot": "fll_team",
        "start": {"x": 0, "y": 0, "head": 0},
        "steps": steps,
    })


def with_code(code, at=None, ident="a1"):
    return {"type": "drive", "cm": 40, "actions": [
        {"id": ident, "at": at or {"cm": 0}, "label": "custom",
         "do": {"code": code}}]}


def test_the_code_becomes_the_action_body():
    result = build([with_code("core.leftTask.run(750)")])

    assert result["ok"], result["diagnostics"]
    assert "def _action_1(core):" in result["module_text"]
    assert "    core.leftTask.run(750)" in result["module_text"]


def test_a_code_action_carries_no_motor_guard():
    """The `if core.X is not None` guard is for the picker's own motor field.

    Code the team wrote can reach any motor, or none, so there is nothing
    for the generator to check before running it -- that is the team's own
    responsibility, same as anything else in the function body.
    """
    result = build([with_code("core.leftTask.run(500)\ncore.rightTask.run(-500)")])

    module = result["module_text"]
    assert "is not None" not in module.split("def _action_1")[1].split("def run")[0]


def test_blank_code_does_not_break_the_run():
    """A half-made action should not stop the run building, same as an
    unfinished motor picker (test_an_arm_step_with_no_action_still_drives)."""
    result = build([with_code("")])

    assert result["ok"], result["diagnostics"]
    assert "    pass        # nothing written yet" in result["module_text"]


def test_a_syntax_error_is_reported_on_its_own_step():
    result = build([
        {"type": "wait", "ms": 500},
        with_code("core.leftTask.run(500", at={"cm": 20}),
    ])

    assert not result["ok"]

    problems = [d for d in result["diagnostics"] if d["step"] == 1]
    assert problems, "the error must land on the step that has the bad code"
    assert problems[0]["level"] == "error"


def test_a_syntax_error_still_lets_the_rest_of_the_run_build():
    """One bad action should not blank the whole timeline while it is being
    typed -- only stop it from being downloaded."""
    result = build([with_code("def(")])

    assert not result["ok"]
    assert result["total_ms"] > 0
    assert result["steps"] != []


def test_wait_call_is_a_blocking_warning():
    result = build([with_code("wait(1000)")])

    assert result["ok"], "a warning must not block the run, only an error does"
    warnings = [d["message"] for d in result["diagnostics"] if d["level"] == "warning"]
    assert any("wait(" in message and "blocks the whole robot" in message
               for message in warnings)


def test_a_while_loop_is_a_blocking_warning():
    result = build([with_code("while True:\n    pass")])

    warnings = [d["message"] for d in result["diagnostics"] if d["level"] == "warning"]
    assert any("while loop" in message for message in warnings)


def test_run_angle_without_wait_false_is_a_blocking_warning():
    result = build([with_code("core.leftTask.run_angle(500, 90)")])

    warnings = [d["message"] for d in result["diagnostics"] if d["level"] == "warning"]

    # exact wording, not just a substring match: an earlier version of this
    # message read "calls run_angle) without" -- call[:-1] correctly dropped
    # the opening paren but the format string still appended the closing one
    # on its own, and "run_angle" in message stayed true either way. Found in
    # the browser, not by this test, because the assertion was loose enough
    # to pass both ways.
    assert any("calls run_angle() without wait=False" in message
               for message in warnings)


def test_run_angle_with_wait_false_is_not_a_warning():
    """The whole point of the warning is to lead someone here."""
    result = build([with_code("core.leftTask.run_angle(500, 90, wait=False)")])

    messages = [d["message"] for d in result["diagnostics"]]
    assert not any("run_angle" in message for message in messages)


def test_multi_line_code_keeps_its_own_indentation():
    code = "if core.leftTask is not None:\n    core.leftTask.run(500)\nelse:\n    pass"
    result = build([with_code(code)])

    assert result["ok"], result["diagnostics"]
    module = result["module_text"]

    assert "    if core.leftTask is not None:" in module
    assert "        core.leftTask.run(500)" in module
    assert "    else:" in module


def test_a_code_action_and_a_motor_action_fire_in_order_together():
    result = build([
        {"type": "drive", "cm": 40, "actions": [
            {"id": "motor", "at": {"cm": 0}, "label": "picker",
             "do": {"motor": "leftTask", "call": "run", "speed": 500}},
            {"id": "code", "at": {"cm": 20}, "label": "custom",
             "do": {"code": "core.rightTask.stop()"}},
        ]},
    ])

    assert result["ok"], result["diagnostics"]
    assert [marker["id"] for marker in result["markers"]] == ["motor", "code"]

    module = result["module_text"]
    assert module.index("_action_1(core),") < module.index("_action_2(core),")
    assert "core.leftTask.run(500)" in module
    assert "core.rightTask.stop()" in module
