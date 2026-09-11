"""Shared test setup.

Two things have to happen before ``pythfinder`` is imported anywhere:

    1. SDL needs a video driver that does not open a window, because creating a
       Simulator calls pygame.display.set_mode().
    2. tests/ has to be on sys.path, so the test modules can import
       golden_runs.

Both are done at import time below, not in a fixture, since pytest imports
this file before it imports any test module.
"""

import os
import sys
from pathlib import Path

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
sys.path.insert(0, str(Path(__file__).parent))

import pygame                                                   # noqa: E402
import pytest                                                   # noqa: E402

import pythfinder                                               # noqa: E402


@pytest.fixture
def sim():
    """A fresh Simulator per test.

    Presets mutate the constants they are applied to, so runs are kept apart
    rather than sharing one simulator.
    """
    return pythfinder.Simulator()


@pytest.fixture(scope = "session", autouse = True)
def _shut_pygame_down():
    yield
    pygame.quit()
