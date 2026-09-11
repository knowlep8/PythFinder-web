# Web route planner — plan

A website where team members plan an FLL run on the BIOGLOW mat, tweak it,
attach attachment-motor actions, and download a Python file that goes straight
onto the SPIKE Prime hub.

Work through it top to bottom. Each step lists what to do and how we know it is
done. Tick the box when a step is merged.

---

## Where we are starting from

Facts measured on the current code (September 2026), so later steps can be
checked against them.

**The motion math is already portable.**
Segments, kinematics, constraints, feedforward and `TrajectoryBuilder` are plain
Python with no numpy. Building the `fll_run_template.py` run takes **38 ms** and
exporting it **10 ms** in CPython. The 1.6 s start-up is pygame and image
loading, none of which the web planner needs.

**Today's pipeline has three manual steps on two machines.**

```
fll_run_template.py  ->  Trajectory/TXT/run_a.txt  ->  tools/txt_to_py.py  ->  traj_run_a.py  ->  runs.py (hand-written marker functions)
```

Only the last file reaches the hub. The steps can get out of sync, and already
have (see step 0.1).

**What ties the math to the desktop simulator:**

| Coupling | Where | Size of fix |
|---|---|---|
| `import pygame` used only in one type hint (`pygame_vector_to_point`) | `Components/BetterClasses/mathEx.py:3,434` | tiny |
| `import pygame` for `get_ticks()` | `Trajectory/Control/Controllers/PIDController.py` | tiny |
| `import matplotlib` used only by `graph_motion_states` | `Trajectory/Segments/Primitives/generic.py:4,188` | tiny |
| Loads ~70 menu images at import, calls `screeninfo`, raises `FONT NOT FOUND` | `Components/Constants/constants.py`, `screenSize.py` | medium |
| Builder reads kinematics and constraints from `sim.presets` | `Trajectory/trajectoryBuilder.py:15-42` | small |
| Generator calls `sim.robot.to_motor_power` and `sim.constants.kinematics` | `Trajectory/trajectoryGenerator.py:47-53` | small |
| Errors are `print()`ed, e.g. markers silently discarded | `trajectoryBuilder.py:333,557-580` | small |

**Robot and field facts the web UI must reproduce** (from `constants.py`):

- Field is 200.5 × 114.3 cm, origin at the centre.
- **+x is UP the screen, +y is RIGHT**; heading 0 faces up; angles increase
  counter-clockwise.
- Robot is 19 cm wide by 14 cm long, with a 16 cm track width and a
  -3.5 cm centre-of-rotation offset (+x forward). Max velocity is 64.3 cm/s at
  `MAX_POWER` 100.
- In `Robot/fll_robot_team.png`, the front of the robot points at the LEFT edge
  of the image.
- Hub attachment motors are `core.leftTask` (port C) and `core.rightTask`
  (port D). Both are Pybricks `Motor`s and either may be `None`.

---

## Architecture decisions

Settled unless we find a reason to revisit them.

1. **One copy of the motion math, in Python.** The website runs the existing
   library in the browser with **Pyodide**. We do not port it to JavaScript: two
   copies drift, and the hub output must match the desktop tool exactly.
2. **Static site, no server.** No accounts, no hosting cost. Runs are saved in
   the browser and exported/imported as a `.json` run file.
3. **Draw with a normal HTML canvas**, not pygame-in-the-browser. The pygame
   menus are image-based and we want a better UI, not a port of the old one.
4. **The run is a list of steps, and that list is the source of truth.** The
   field view and a read-only "Python view" are both generated from it. We do
   not try to sync edits in both directions between blocks and text.
5. **The website writes the whole hub file,** including the marker functions.
   Kids never hand-match a tuple of functions to marker order again.
6. **The web code lives in this repo under `web/`**, so the site and the Python
   package always come from the same commit.

---

## Phase 0 — housekeeping

- [x] **0.1 Refresh the stale hub file.** `traj_run_a.py` (converted Sep 6 16:03)
  was older than `Trajectory/TXT/run_a.txt` (exported 18:33), and predated the
  then-uncommitted `.toPose(START_POSE)` in `fll_run_template.py`. The hub was
  running a 7.9 s run; the template described a 15.6 s one.
  - Re-ran `tools/txt_to_py.py Trajectory/TXT/run_a.txt` in the quick-start repo.
    `run_a.txt` was checked first and is byte-identical to a fresh export of the
    current template, so it did not need re-exporting.
  - Committed the pending edits to `fll_run_template.py` and
    `fll_trajectory_example.py`. The edit to
    `pythfinder/Images/Robot/fll_robot_team.png` is still uncommitted.
  - *Done:* `traj_run_a.py` has `COUNT = 2607`, 2607 states at 6 ms (15.6 s),
    15642 bytes on the hub.
- [x] **0.2 Start a branch.** `web-planner`, off `bioglow-field-build-script`
  rather than `main`: the season's field and the measured robot geometry live on
  that branch, and step 1.1 has to capture the real robot. `main` is 11 commits
  behind. Whether that branch reaches `main` is a separate decision.

---

## Phase 1 — a headless core

Goal: `build_run(run_description) -> result` works in a Python with **no pygame,
matplotlib or screeninfo installed**, and produces byte-identical output to
today.

- [x] **1.1 Capture golden outputs *before* changing anything.**
  - `pytest` added as a dev dependency.
  - `tests/golden_runs.py` holds 19 runs covering `inLineCM` (forward,
    backward, merged consecutive), `wait` (merged), `turnToDeg` (both
    directions and reversed), `toPoint`, `toPointTangentHead`, `toPose`,
    `toPoseTangentHead`, `toPoseLinearHead`, `reversed = True`, relative
    displacement and temporal markers including a negative one, absolute
    markers, a constraints marker, an interrupt marker, and the
    `fll_run_template.py` run at both 6 ms and 1 ms per state.
  - `tests/regenerate_goldens.py` rewrites them; `tests/test_goldens.py`
    compares byte-for-byte and points at the regenerate command when they
    differ.
  - *Done:* 24 tests pass. `tests/golden/template_run.txt` is **byte-identical
    to `Trajectory/TXT/run_a.txt` in the quick-start repo**, the file currently
    on the hub, so the reference set is anchored to the real robot.

  **Found while doing this:** on a tank drive the heading variants are not
  distinct. `PointSegment` and `PoseSegment` force `tangent = True` and
  `linear_head = False` whenever the chassis is `NON_HOLONOMIC`
  (`pointSegment.py:34`, `poseSegment.py:35`), because a robot that cannot move
  sideways must point along its line of travel. So `toPoint` ==
  `toPointTangentHead`, and `toPose` == `toPoseTangentHead` ==
  `toPoseLinearHead`. Only `reversed`, and the final turn `toPose` adds, change
  the motion. This is asserted in `test_heading_modes_collapse_on_a_tank_drive`
  and it changes what step 2.4 should offer.

- [x] **1.2 Remove the tiny imports.**
  - `mathEx.py`: `import pygame` gone. The only use was the type hint on
    `pygame_vector_to_point`, now quoted. The function stays where it is, since
    it is part of the public surface and its one caller
    (`Components/robot.py:219`) is pygame code anyway.
  - `PIDController.py`: `pygame.time.get_ticks()` replaced by
    `milliseconds_since_start()`, built on `time.monotonic()` and keeping the
    same meaning (whole ms since the program started). `time` is bound as
    `_time` because this module is star-imported.
  - `Segments/Primitives/generic.py`: matplotlib now imported inside
    `graph_motion_states`.
  - **Also needed, and not in the original list:** `trajectoryGrapher.py`
    imported matplotlib at the top, and `Trajectory/__init__.py` imports the
    grapher, so `import pythfinder` still required matplotlib. It now loads on
    demand through a module-level `mplt` that `_load_matplotlib()` fills in, so
    all ten call sites stay as they were.
  - *Done:* 24 tests pass, goldens unchanged to the byte. `import pythfinder`
    no longer loads matplotlib.

  Note the limit of this step: importing any submodule still imports
  `pythfinder/__init__.py`, which imports pygame and `core`, so the package as a
  whole still needs pygame. That is step 1.3's job, and only then can the
  headless import be tested end to end.

- [ ] **1.3 Split robot constants from UI constants.**
  - New `pythfinder/Trajectory/robotConfig.py` with a `RobotConfig` dataclass:
    `kinematics`, `constraints`, `real_max_velocity`, `max_power`. It adds
    `to_motor_power(velocity)` (moved from `Components/robot.py:103`).
  - Define the FLL robot as `FLL_ROBOT = RobotConfig(...)` next to it, using the
    measured numbers above. `constants.py` imports those values instead of
    defining them, so they are written down once.
  - Move the font check and all `pygame.image.load` calls so they only happen
    when a `Simulator` is created, not at import time.
  - *Done when:* goldens pass and the desktop simulator still opens, draws the
    field, and follows `fll_run_template.py`.

- [ ] **1.4 Builder and generator take a `RobotConfig`, not a `Simulator`.**
  - `TrajectoryBuilder(start_pose, robot=FLL_ROBOT)`. Keep the old
    `TrajectoryBuilder(sim, start_pose, preset)` signature working: it builds a
    `RobotConfig` from the preset and passes it through, so existing team
    scripts do not break.
  - `TrajectoryGenerator` takes the config. Split it into
    `wheel_speed_text(...) -> str` (pure) and the existing file-writing wrapper.
  - `Trajectory.follow()` and `graph()` still need a sim or matplotlib. Make
    `follow()` take `sim` as an argument when there is no stored one.
  - *Done when:* goldens pass using the new signature, and the old signature
    still works in `fll_run_template.py`.

- [ ] **1.5 One-step hub module.** Port `render()` and `parse()` from the
  quick-start's `tools/txt_to_py.py` into `pythfinder/Export/hubModule.py`,
  working from states directly instead of re-parsing the `.txt`.
  - *Done when:* for every golden run, the output equals running today's
    `txt_to_py.py` on the golden `.txt` (byte-for-byte).

- [ ] **1.6 Diagnostics as data.** Replace the `print()` warnings in the builder
  with a list of `Diagnostic(level, step_index, message, suggestion)` on the
  builder, and still print them on desktop. Cover at least:
  - a marker outside its segment, and so discarded;
  - an empty trajectory;
  - an int16 overflow in the export;
  - *(new)* the robot footprint leaving the mat.
  - *Done when:* a test builds a run with an out-of-range marker and gets one
    diagnostic that says which step it is on.

- [ ] **1.7 The run description and `build_run`.**
  - Define the JSON a run is saved as (see *Run file format* below).
  - `pythfinder/headless.py: build_run(description: dict) -> dict` returns
    `poses` (downsampled for drawing), `total_ms`, `markers` (with the action
    id each belongs to, in firing order), `diagnostics`, and `module_text`.
  - Marker order: pass each action's id as the marker's `fun`. After `build()`,
    read `final_markers`; it is already sorted by time, so its order *is* the
    hub order.
  - *Done when:* in a fresh venv with only this package installed with
    `--no-deps`, `python -c "from pythfinder.headless import build_run"` works,
    and the template run described as JSON gives the golden output.

- [ ] **1.8 Package it for the browser.** Build a pure-Python wheel
  (`uv build`). Pyodide will install it with `micropip.install(url, deps=False)`,
  so the existing pygame/matplotlib dependencies for desktop users stay as they
  are.
  - *Done when:* a scratch HTML page loads Pyodide, installs the wheel, runs
    `build_run` on the template run, and logs the same `COUNT` and `MARKERS`.

---

## Phase 2 — the website, minimum version

Goal: plan a run with drive/turn/wait/go-to steps, watch it, and download a
working hub file. No actions yet.

- [ ] **2.1 Scaffold `web/`.** Vite + TypeScript. Keep dependencies minimal; a
  small UI library such as Preact is fine if the step list gets fiddly.
  - GitHub Actions workflow builds the wheel and the site and deploys to
    GitHub Pages.
  - *Decision here:* public Pages site, or private (see *Open decisions*).
- [ ] **2.2 Python worker.** Run Pyodide in a Web Worker so the page never
  freezes. Message shape: `{run} -> {poses, markers, diagnostics, moduleText}`.
  Show a loading screen on first visit (~10 MB, cached afterwards). Rebuild with
  a short debounce whenever the run changes.
- [ ] **2.3 Field view.** Canvas with the BIOGLOW image at true scale and the
  robot sprite. Coordinate helpers carry the +x-up / +y-right / CCW convention
  over from `Components/robot.py:106-114`. Show the mouse position in field cm.
  Drag the robot to set the start pose; rotate with a handle or the arrow keys.
  - *Done when:* the start pose `(-46, -83, 0)` sits in the left launch area,
    exactly where the desktop simulator puts it.
- [ ] **2.4 Step list.** Add, edit, reorder, delete. Step types, mapped 1:1 to
  builder calls:
  - Drive (cm, forward/back) → `inLineCM`
  - Turn to heading (deg, reversed) → `turnToDeg`
  - Wait (ms) → `wait`
  - Go to point, with a "drive there backwards" tick → `toPoint`
  - Go to pose (a point plus a heading to finish on), with the same tick →
    `toPose`

  No heading-mode choice: step 1.1 established that the variants collapse on a
  tank drive, so offering them would be three buttons that do the same thing.
  `inSpline` is not offered either (it is a no-op today). Selecting a step
  highlights its part of the path.
- [ ] **2.5 Path and playback.** Draw the path from `poses`. Play/pause and a
  time scrubber animate the robot along it, with a readout of time, step and
  pose.
- [ ] **2.6 Problems panel.** Show `diagnostics` next to the step they refer
  to, in kid-friendly words. Paint the path red where the robot leaves the mat.
- [ ] **2.7 Download.** Run name field (must be a valid Python module name, so
  validate it). Download `run_<name>.py`. Show state count, run length and bytes
  on the hub.
  - *Done when:* a run planned on the website, downloaded and uploaded via
    code.pybricks.com, drives the same as the same run exported from
    `fll_run_template.py`.
- [ ] **2.8 Save and load.** Autosave to browser storage; export/import the run
  `.json`; a list of the team's runs.

---

## Phase 3 — attachment actions

Goal: kids attach motor actions to a step, and the downloaded file contains
working, correctly ordered marker functions.

- [ ] **3.1 Hub-side convention (quick-start repo).** The downloaded file
  should be self-contained:
  ```python
  # run_a.py  -- generated by the web planner, do not edit
  from trajectory import Trajectory

  STEPS = 6
  MARKERS = (1764, 7942)
  COUNT = 2607
  DATA = (...)

  def _action_1(core):      # step 1, 35 cm in: Left arm down
      core.leftTask.run(500)

  def _action_2(core):      # step 4, 1 ms before the end: Left arm up
      core.leftTask.run(-500)

  def run(core):
      t = Trajectory.fromValues(STEPS, MARKERS, COUNT, DATA)
      t.withMarkers((lambda: _action_1(core), lambda: _action_2(core)))
      t.follow(core)
  ```
  so that `runs.py` needs only `import run_a` and `run_a.run(core)`.
  - Add `Trajectory.fromValues(...)` to the hub's `trajectory.py`. A module
    cannot easily pass *itself* to `Trajectory(module)` on MicroPython, so
    **verify on the hub** before settling this.
  - Update the quick-start README and `runs.py` example.
  - *Done when:* a hand-written file in this shape runs on the hub with two
    actions firing in order.
- [ ] **3.2 Action blocks.** Attach to a step at "X cm in", "X ms in", or
  "X before the end" (negative values, as the builder already supports).
  Offered blocks only use non-blocking calls, because markers run inline on the
  hub's drive loop:
  - Run motor at speed → `run(speed)`
  - Turn motor by angle → `run_angle(speed, angle, wait=False)`
  - Run motor to angle → `run_target(speed, angle, wait=False)`
  - Stop / brake / hold motor

  Pick the motor from `leftTask` / `rightTask`, labelled with team-chosen names
  ("Left arm"). Each action appears as a marker dot on the path and fires
  visibly during playback.
- [ ] **3.3 Custom code action.** A CodeMirror 6 editor for a free-form action
  body, with `core` in scope. Check syntax in the worker with `compile()`; we
  cannot run it (Pybricks APIs are hub-only). Warn on obviously blocking calls
  (`wait(`, `run_angle(` without `wait=False`, `while`).
- [ ] **3.4 Python view.** Read-only panel showing the equivalent
  `TrajectoryBuilder` chain and the generated `run()`. It is for learning, and
  for pasting into the desktop tool.

---

## Phase 4 — polish

Order these after watching the kids use phase 3.

- [ ] **4.1** Drag waypoints on the field to edit Go-to steps directly.
- [ ] **4.2** Robot settings panel (track width, max velocity, centre offset,
  speed limits, sprite), behind a simple mentor toggle.
- [ ] **4.3** Launch-area presets and snapping for the start pose.
- [ ] **4.4** Overlay several runs at once, to check they do not collide with
  each other's missions.
- [ ] **4.5** Speed-limit (constraints marker) blocks for slow, careful sections.
- [ ] **4.6** Hub memory budget: total bytes across all runs in the selector.
- [ ] **4.7** *(stretch)* Send straight to the hub over Web Bluetooth, the way
  code.pybricks.com does.

---

## Run file format (draft — finalised in step 1.7)

```json
{
  "version": 1,
  "name": "run_a",
  "robot": "fll_team",
  "steps_ms": 6,
  "start": { "x": -46, "y": -83, "head": 0 },
  "steps": [
    { "type": "drive", "cm": 75,
      "actions": [ { "id": "a1", "at": { "cm": 35 },
                     "do": { "motor": "leftTask", "call": "run", "speed": 500 },
                     "label": "Left arm down" } ] },
    { "type": "wait", "ms": 600 },
    { "type": "turn", "deg": 90, "reversed": false },
    { "type": "drive", "cm": 30,
      "actions": [ { "id": "a2", "at": { "ms": -1 },
                     "do": { "motor": "leftTask", "call": "run", "speed": -500 },
                     "label": "Left arm up" } ] },
    { "type": "toPose", "x": -46, "y": -83, "head": 0, "heading": "fixed" }
  ]
}
```

`version` lets us migrate old saved runs when the format changes.

---

## Found along the way

Neither is caused by this work, and neither blocks it.

- [ ] **The joystick heading PID divides by zero.** `PIDController.calculate()`
  divides by the time since the previous call, in whole milliseconds. Two calls
  inside one millisecond means a `ZeroDivisionError`, and at the simulator's
  1000 FPS ceiling that is reachable: 199 of 200 back-to-back readings share a
  millisecond. It predates this work — `pygame.time.get_ticks()` behaved the
  same way, and step 1.2 kept the semantics deliberately. It only bites under
  joystick control, which is presumably why nobody has hit it. A guard that
  skips the derivative term when no time has passed would fix it; doing so
  changes simulator behaviour, so it wants a deliberate decision.
- **The editable install invents a bare `Trajectory` module.** Importing
  `pythfinder.Trajectory.Segments.Primitives.generic` directly fails with
  `cannot import name 'Segments' from 'Trajectory' (unknown location)`. It comes
  from the setuptools editable-install shim in `.venv`, whose `MAPPING` only
  covers the top-level `pythfinder` and delegates everything deeper: a clean
  interpreter shows no such module. Importing `pythfinder` first, as everything
  actually does, works fine. Worth re-checking in step 1.8, when the browser
  loads a real wheel rather than an editable install.

## Open decisions

Decide when we reach the step named. The recommendation is the default.

| Decision | Step | Recommendation |
|---|---|---|
| Public GitHub Pages site or private? The site includes the team robot photo and the field image derived from FIRST's PDF. | 2.1 | Private repo with Pages limited to the team, or an unlisted URL |
| Upstream the headless refactor to omegacoreFLL/PythFinder, or keep it in our fork? | end of 1 | Offer upstream once goldens prove nothing changed |
| One fixed team robot, or editable robot settings? | 4.2 | Fixed for the season; editable behind the mentor toggle |
| How `run()` gets the data on the hub (`fromValues` vs a module self-import) | 3.1 | Whichever works on the hub; test both |

## Risks

- **Silent behaviour change in the refactor.** Mitigated by the golden files in
  1.1, captured before any edit.
- **Pyodide first load on school Wi-Fi.** About 10 MB once, then cached. Test on
  the actual network before a practice session.
- **Hub heap.** Each run is roughly 6 bytes per exported state (about 16 KB for
  the template run). Several long runs in one program add up, hence 4.6.
- **Kids writing blocking custom actions.** Blocks cannot block; the code editor
  warns. The hub-side behaviour stays as documented in `fll_run_template.py`.
