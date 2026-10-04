"""The Trajectory class shadowed the Trajectory subpackage (docs/web-planner.md,
"Found along the way").

pythfinder/__init__.py does `from .Trajectory import *`, and among the names
that pulls up is the Trajectory *class* from trajectory.py -- forwarded
through trajectoryBuilder.py's own `import *` -- which overwrote the
`Trajectory` attribute this package's own import machinery had just set to
the Trajectory *subpackage*. `import a.b.c as x` resolves by walking
attributes, so anything of the form `import pythfinder.Trajectory.<module>`
walked into the class instead of the subpackage and failed. Nothing that
goes through `from pythfinder import X` or `from pythfinder.Trajectory.thing
import Y` ever noticed, which is why this had no test before.
"""

import pythfinder


def test_attribute_walk_import_reaches_the_subpackage():
    """This exact form failed before the fix:
    ImportError: cannot import name 'Segments' from 'Trajectory' (unknown location)
    """
    import pythfinder.Trajectory.Segments.Primitives.generic as generic

    assert hasattr(generic, "MotionState")


def test_pythfinder_dot_trajectory_is_the_subpackage():
    import types

    assert isinstance(pythfinder.Trajectory, types.ModuleType)


def test_the_trajectory_class_is_still_reachable_by_its_own_module():
    """The class did not go away -- it is just not at the top level any
    more, which nothing in this repo relied on (see the doc note)."""
    from pythfinder.Trajectory.trajectory import Trajectory

    assert isinstance(Trajectory, type)


def test_public_names_used_by_real_callers_still_work():
    """fll_run_template.py, fll_trajectory_example.py and the tests all
    import this way -- the one thing the fix was not allowed to break."""
    from pythfinder import Pose, TrajectoryBuilder

    assert TrajectoryBuilder(Pose(0, 0, 0)) is not None
