from .trajectoryBuilder import *
from .trajectoryGenerator import *

from .robotConfig import *
from .diagnostics import *
from .field import *


from .Kinematics import *
from .Segments import *
from .Markers import *
from .Control import *


# The follower animates in a Simulator, and the grapher draws with matplotlib,
# so both reach pythfinder.core and through it pygame. Importing them here
# would make the whole planning half of the library need an interface, so they
# are fetched on first use instead -- the same way Trajectory.follow() and
# .graph() import them only when called.
_INTERFACE_NAMES = {"TrajectoryFollower": ".trajectoryFollower",
                    "TrajectoryGrapher": ".trajectoryGrapher"}


def __getattr__(name):
    if name not in _INTERFACE_NAMES:
        raise AttributeError(
            "module {0!r} has no attribute {1!r}".format(__name__, name))

    from importlib import import_module

    value = getattr(import_module(_INTERFACE_NAMES[name], __name__), name)
    globals()[name] = value

    return value
