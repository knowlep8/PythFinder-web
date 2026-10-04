"""PythFinder: motion planning for FLL robots.

Importing this package no longer starts pygame.

The planning half -- poses, the builder, the exports -- is plain maths and
loads immediately. The interface half (the Simulator, its menus, the drawing)
is loaded the first time something asks for it:

    import pythfinder

    pythfinder.Pose(0, 0, 90)     # nothing but maths
    pythfinder.Simulator()        # pygame is imported and started here

That is what lets the web planner import this package in a browser, where
pygame does not exist and no window can be opened. Team scripts are unaffected:
they create a Simulator, which starts pygame exactly as before.

See docs/web-planner.md, step 1.7.
"""

from .Trajectory import *

# The star import above pulls in the Trajectory *class* (trajectory.py's,
# forwarded up through trajectoryBuilder.py's own `import *`) under the same
# name as the Trajectory *subpackage*, so it overwrites the subpackage
# reference that importing it a moment ago set on this module. Nothing here
# constructs that class by name -- TrajectoryBuilder.build() is how one gets
# made -- so put the subpackage back: that is what `import
# pythfinder.Trajectory.<anything>` needs to find by walking attributes, and
# what docs/web-planner.md's "Found along the way" bug was about. Fetched
# from sys.modules rather than `from . import Trajectory`, because that form
# resolves via getattr() first and would just hand back the class we are
# trying to undo.
import sys as _sys

Trajectory = _sys.modules[__name__ + ".Trajectory"]

del _sys


def __getattr__(name):
    """Find a name that is not part of the planning half.

    Two kinds of thing arrive here: a submodule (pythfinder.core,
    pythfinder.headless) and a name from the interface (Simulator, Constants).
    Submodules have to be handled first -- importing pythfinder.core asks the
    package for the attribute 'core', so answering that by importing core would
    call this function again, forever.
    """
    # dunder and private lookups are how tools inspect a module; answering them
    # must not drag in an interface
    if name.startswith("_"):
        raise AttributeError(
            "module {0!r} has no attribute {1!r}".format(__name__, name))

    from importlib import import_module

    try:
        module = import_module(".{0}".format(name), __name__)
    except ImportError:
        module = None       # not a submodule, so try the interface below

    if module is not None:
        globals()[name] = module
        return module

    # importing core starts pygame, loads the menu images and brings in
    # everything the simulator is made of
    core = import_module(".core", __name__)

    try:
        value = getattr(core, name)
    except AttributeError:
        raise AttributeError(
            "module {0!r} has no attribute {1!r}".format(__name__, name)) from None

    globals()[name] = value

    return value
