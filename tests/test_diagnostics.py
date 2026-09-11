"""Check that the builder reports its problems as data.

These used to be print() calls: the run was quietly changed and a note went to
a terminal. The web planner has no terminal, and needs to put the problem next
to the step that caused it -- so each one has to name its step and survive as an
object.
"""

import pytest

from pythfinder import Pose, TrajectoryBuilder
from pythfinder.Trajectory.diagnostics import Diagnostic
from pythfinder.Trajectory.field import FLL_FIELD
from pythfinder.Trajectory.robotConfig import FLL_ROBOT

from golden_runs import START


def builder(pose = START):
    """A quiet builder: these tests read the diagnostics rather than print them."""
    made = TrajectoryBuilder(pose, robot = FLL_ROBOT)
    made.print_diagnostics = False

    return made


def test_a_clean_run_says_nothing():
    trajectory = builder().inLineCM(30).turnToDeg(90).build()

    assert trajectory.diagnostics == []


def test_marker_past_the_end_of_its_move_names_the_step():
    """The case the team will hit constantly: a marker further in than the move goes."""
    # the reversing step is kept short on purpose: back up far enough and the
    # robot's corner leaves the mat, which would add a second diagnostic and
    # make this test about two things at once
    trajectory = (builder()
                  .inLineCM(20)
                  .inLineCM(-10)
                      # 80cm into a 10cm move -- there is no such point
                      .addRelativeDisplacementMarker(80, lambda: None)
                  .build())

    assert len(trajectory.diagnostics) == 1
    problem = trajectory.diagnostics[0]

    assert problem.level == Diagnostic.WARNING
    assert problem.step == 1                  # the second step, counted from 0
    assert "step 2" in str(problem)           # and reads as step 2 for a person
    assert problem.suggestion is not None

    # the marker was dropped, so the run itself still works
    assert trajectory.MARKERS == []
    assert trajectory.TIME > 0


def test_an_empty_run_is_an_error_rather_than_a_crash():
    trajectory = builder().build()

    assert trajectory.TIME == 0
    assert [d.level for d in trajectory.diagnostics] == [Diagnostic.ERROR]


def test_driving_off_the_mat_is_reported_with_where_and_when():
    """The FLL table is only 114.3cm deep, so 200cm forward leaves it."""
    trajectory = builder().inLineCM(200).build()

    off = [d for d in trajectory.diagnostics if "mat" in d.message or "table" in d.message]
    assert len(off) == 1

    assert off[0].level == Diagnostic.WARNING
    assert off[0].step == 0
    assert off[0].time_ms is not None
    assert 0 < off[0].time_ms <= trajectory.TIME


def test_a_run_along_the_mat_stays_on_it():
    """Same distance, but across the long side, which fits."""
    trajectory = (builder(Pose(x = 0, y = -90, head = 90))
                  .inLineCM(150)
                  .build())

    assert trajectory.diagnostics == []


def test_diagnostics_are_printed_by_default(capsys):
    """The desktop behaviour does not change: problems still appear on screen."""
    (TrajectoryBuilder(START, robot = FLL_ROBOT)
     .inLineCM(20)
        .addRelativeDisplacementMarker(80, lambda: None)
     .build())

    assert "step 1" in capsys.readouterr().out


def test_a_diagnostic_can_be_handed_to_the_web_planner():
    problem = Diagnostic.warning("something", step = 2, suggestion = "do this",
                                 time_ms = 40)

    assert problem.as_dict() == {"level": "warning",
                                 "message": "something",
                                 "step": 2,
                                 "suggestion": "do this",
                                 "time_ms": 40}
