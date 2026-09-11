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
