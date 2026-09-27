"""Check the one-step hub export against the two-step one it replaces.

The team's flow has been: export a .txt from the simulator, then convert it
with the quick-start's tools/txt_to_py.py. Trajectory.hub_module() does both at
once, and has to produce the same file -- the bytes in it are what the robot
drives.

Two references are used, strongest first:

    1. tests/golden/hub/template_run.py, a copy of the traj_run_a.py actually
       running on the hub. Self-contained, so this always runs.
    2. the real tools/txt_to_py.py, run against each golden .txt, when the
       quick-start repo is checked out next to this one. Covers every run, and
       skips when it is not there.

Only the first line differs, which says which tool wrote the file.
"""

import importlib.util
import os
from pathlib import Path

import pytest

from pythfinder import Pose, TrajectoryBuilder
from pythfinder.Trajectory.robotConfig import FLL_ROBOT

from golden_runs import GOLDEN_DIR, GOLDEN_RUNS


HUB_DIR = GOLDEN_DIR / "hub"
TXT_TO_PY = Path(os.path.expanduser("~/pythfinder-EV3-quick-start/tools/txt_to_py.py"))


def body(module_text: str) -> str:
    """Everything but the opening docstring line."""
    return module_text.split("\n", 1)[1]


def load_txt_to_py():
    spec = importlib.util.spec_from_file_location("txt_to_py", TXT_TO_PY)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    return module


def test_matches_the_module_on_the_hub():
    """The real file, converted by the real tool, sitting on the real robot."""
    build, steps = GOLDEN_RUNS["template_run"]

    ours = build(None).hub_module("run_a", steps)
    theirs = (HUB_DIR / "template_run.py").read_text()

    assert body(ours) == body(theirs)


def test_reports_the_counts_the_hub_needs():
    build, steps = GOLDEN_RUNS["template_run"]
    text = build(None).hub_module("run_a", steps)

    assert "STEPS = 6" in text
    assert "MARKERS = (1764, 7942)" in text
    assert "COUNT = 2607" in text


def test_hub_module_code_elides_the_payload_but_keeps_everything_else():
    """Step 3.4's own "learning" view of this format -- moved here from
    test_python_view.py when step 5.7 switched build_run's own module_text
    over to the DriveBase file: hub_module_code is still a public method on
    Trajectory, and this is the only place its elision is checked now.
    """
    trajectory = (TrajectoryBuilder(Pose(x = 0, y = 0, head = 0), robot = FLL_ROBOT)
                 .inLineCM(40)
                 .addRelativeDisplacementMarker(0, "a1")
                 .build())

    actions = [{"id": "a1", "label": "Grab", "motor": "leftTask",
               "call": "run", "speed": 500}]

    module = trajectory.hub_module("run_a", 6, actions)
    code = trajectory.hub_module_code("run_a", 6, actions)

    # the structural "DATA = (...)" wrapper is deliberately kept -- only the
    # bytes literal inside it is elided
    assert "DATA = (\n" in code
    assert "b'" not in code, "the payload bytes themselves must not be there"
    assert "elided" in code
    assert "def _action_1(core):" in code
    assert "def run(core):" in code

    # everything that is not the payload must match the real file exactly --
    # this can never drift, because both come from the same _module_text.
    # rsplit rather than split: the closing "\n)\n" is the template's own,
    # and only the last occurrence is guaranteed to be it
    assert module.split("DATA = (")[0] == code.split("DATA = (")[0]
    assert module.rsplit("\n)\n", 1)[-1] == code.rsplit("\n)\n", 1)[-1], (
        "the action defs and run() must be identical")

    # a sanity check that the elision actually elided something
    assert len(code) < len(module) / 2


@pytest.mark.parametrize("name", sorted(GOLDEN_RUNS))
def test_matches_txt_to_py(name):
    if not TXT_TO_PY.exists():
        pytest.skip("the quick-start repo is not checked out at ~/pythfinder-EV3-quick-start")

    txt_to_py = load_txt_to_py()

    steps, markers, count, payload = txt_to_py.parse(
        str(GOLDEN_DIR / "{0}.txt".format(name)))
    theirs = txt_to_py.render(name, steps, markers, count, payload)

    build, run_steps = GOLDEN_RUNS[name]
    ours = build(None).hub_module(name, run_steps)

    assert body(ours) == body(theirs), (
        "'{0}' converts differently in one step than in two".format(name))
