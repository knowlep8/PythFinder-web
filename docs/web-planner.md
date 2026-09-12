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
2. **Served from our own container, and still a static bundle.** A Docker image
   (Vite build → nginx) runs on the team's VPS, or on a machine at home put
   online through a Cloudflare tunnel. Having a server is a deployment
   convenience, not a reason to move the maths onto it: the page stays static
   and all the Python still runs in the browser, so the planner keeps working
   at a venue once it is loaded.
   Pyodide and the wheel are **vendored into the image** rather than pulled from
   a CDN, so nothing external is needed at run time and a locked-down school
   network cannot break it. Runs are saved in the browser and
   exported/imported as a `.json` file, unless we add shared storage in 2.8.
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

- [x] **1.3 Split robot constants from UI constants.**
  - `pythfinder/Trajectory/robotConfig.py` holds `RobotConfig` (`kinematics`,
    `constraints`, `REAL_MAX_VEL`, `MAX_POWER`, plus the drawing dimensions) and
    `to_motor_power(velocity)`. It imports nothing but maths.
  - The team's measurements moved there, and `FLL_ROBOT` describes the robot.
    `constants.py` imports them back under their old names, and the FLL preset
    is **built from `FLL_ROBOT`** rather than repeating the numbers — so the
    unchanged goldens prove it reproduces the robot exactly.
  - The font check became `check_font_available()`, called from
    `Simulator.__init__`. Importing the library no longer needs the font; a
    missing font still raises `FONT NOT FOUND`, verified by faking its absence.
  - *Done:* 24 tests pass, goldens byte-identical. The simulator opens, reports
    the preset as 64.3 cm/s and 16 cm track, follows a run to the right pose and
    draws.

  **The image loads were deliberately left alone.** The step originally asked
  for them to be deferred as well. Measured first: 88 references across four
  files (66 in `mainMenu.py`), 130 load-and-scale calls, and 13 modules
  star-importing `constants.py` — so the names cannot be made lazy without
  rewriting every call site. It would buy about 1.5 s of desktop start-up and
  nothing for the browser, which cannot import `constants.py` at all
  (module-level `pygame.Color`, `pygame.transform`). Decided against; revisit
  only if the simulator ever feels slow to open.

  **Left for 1.4:** `to_motor_power` now exists both on `RobotConfig` and at
  `Components/robot.py:103`. The duplicate goes when the generator switches to
  taking a `RobotConfig`.

- [x] **1.4 Builder and generator take a `RobotConfig`, not a `Simulator`.**
  - `TrajectoryBuilder(START_POSE, robot = FLL_ROBOT)` builds with no interface
    at all. `TrajectoryBuilder(sim, start_pose, preset)` still works exactly as
    before — a `Pose` where the simulator usually goes is what tells them apart.
  - `TrajectoryGenerator` takes the robot, not the simulator, and each export
    now has a pure `wheel_speeds_text()` / `chassis_speeds_text()` half plus the
    file-writing wrapper. `Trajectory.text()` exposes it, which is what the
    browser will download instead of writing a file.
  - `Trajectory(states, markers, robot, sim = None)`. The follower and grapher
    are imported *inside* `follow()` and `graph()`, so importing this module no
    longer drags in `core` or matplotlib.
  - `follow()` now takes the simulator first — which is what README line 256 and
    both team scripts already passed, landing in `perfect` by luck. A bool in
    that position still means the old `follow(perfect, wait, steps)` call and
    shifts every argument along.
  - The duplicated `to_motor_power` is gone: `Components/robot.py` delegates to
    `RobotConfig`, so the exporter and the simulator cannot drift apart.
  - *Done:* 43 tests pass. Every golden run exports byte-identical text when
    built with no simulator, checked by `test_builds_without_a_simulator`. The
    real `fll_run_template.py --export` still writes exactly the bytes on the
    hub.

  **Behaviour that changed on purpose:** the robot is captured when the builder
  is created, instead of being read off `sim.constants` at export time. Building
  a second trajectory with a different preset can no longer change what an
  already-built trajectory exports.

  **Compatibility note:** `Trajectory(...)` itself is constructed differently
  now. Only the builder does that, and team scripts use the builder, but a
  script constructing one by hand would need updating.

- [x] **1.5 One-step hub module.**
  - `pythfinder/Export/hubModule.py` packs a run into the hub's format and
    renders the module text. `Trajectory.hub_module(name, steps)` returns it, so
    no `.txt` has to exist first and a browser can hand over a finished file.
  - Both exports are now built from one `wheel_speed_groups()` walk in the
    generator, so the `.txt` and the module cannot describe different motion.
  - The powers must be rounded to two decimals *before* being scaled by 100.
    That is what the `.txt` writes and therefore what the hub has always been
    fed; scaling the unrounded value shifts the occasional unit.
  - Tank only. A robot without exactly two drive wheels raises a clear error
    rather than writing a file the hub would misread — the format carries two
    motors per state.
  - *Done:* 64 tests pass. The generated module matches
    **the `traj_run_a.py` actually on the hub**, and matches `txt_to_py.py` run
    against all 19 golden exports (those cases skip if the quick-start repo is
    not checked out). A built wheel contains the new subpackage.

  **One deviation from "byte-for-byte":** the first line differs, because it
  says which tool wrote the file — ours does not claim to be `txt_to_py.py`.
  The tests compare everything after it, which is all the data the hub reads.

- [x] **1.6 Diagnostics as data.**
  - `Trajectory/diagnostics.py` holds `Diagnostic(level, message, step,
    suggestion, time_ms)`, with `as_dict()` for the browser. The builder
    collects them on `self.diagnostics` and hands them to the trajectory; it
    still prints them, so the desktop behaves as before. Set
    `builder.print_diagnostics = False` for quiet.
  - Covered: an action past the end of its step, an action counted back from
    the end to before the step starts, an action outside the whole run, an
    empty run, and the robot going off the mat.
  - Messages name the step and say what to do: *"an action 80cm into this step
    was dropped, because the step only goes as far as 10cm — try: put the
    action before 10cm, or make the step longer."*
  - **The mat check is new.** `Trajectory/field.py` holds `Field` and
    `FLL_FIELD`, and `constants.py` now takes the table size from there, so it
    is written down once. The builder rotates the robot's four corners along
    the path — every fifth state, since at full speed the robot covers about
    3mm in 5ms — and reports the worst excursion with the step and the time it
    happens. The simulator path measures against the preset's own image size,
    so the FTC field works too.
  - *Done:* 71 tests pass, goldens byte-identical, and the simulator still
    opens, follows and draws.

  **Fixed on the way:** building a run with no steps used to crash with an
  `IndexError`, because an empty trajectory has no last state to read a time
  from. It now reports an error diagnostic and hands back an empty trajectory.

  **Left as an exception:** an int16 overflow in the hub export still raises
  from `hubModule`, because there is no file to hand over — a diagnostic would
  imply the run was usable. Step 1.7 catches it and reports it as an error.

- [x] **1.7 The run description and `build_run`.**
  - `pythfinder/headless.py: build_run(run, pose_every_ms = 20)` takes the run
    as data and returns `ok`, `total_ms`, `poses` (thinned for drawing),
    `markers`, `diagnostics` and `module_text`. Nothing raises: a run a child
    typed wrong is ordinary, not exceptional, so every problem comes back as a
    diagnostic against its step.
  - Marker order works as planned. The action's id is handed over where the
    marker's function goes — never called, but it rides along on the marker.
    The builder sorts markers by time, so the order they come back in *is* the
    order the hub must bind them in.
  - **The structural problem is solved with a lazy `__init__`.** Importing
    `pythfinder` now loads only the planning half; `Simulator` and the rest of
    `core` arrive through a module `__getattr__` on first use. `pygame.init()`
    moved to `constants.py`, the first module that genuinely needs a live
    pygame — it loads images and reads the mouse cursor while being imported.
  - *Done:* 81 tests pass. In a plain system interpreter **with no pygame and
    no matplotlib installed**, `from pythfinder.headless import build_run`
    imports, builds the template run, and produces a hub module identical to
    the file on the robot, with `pygame` never appearing in `sys.modules`. The
    simulator still opens, follows and draws.

  **Two traps, each of which cost a test run here:**

  - A module `__getattr__` has to answer submodule names itself. Importing
    `pythfinder.core` asks the package for the attribute `core`, so answering
    that by importing `core` calls `__getattr__` again, until the stack runs
    out.
  - `pygame.init()` must run before `constants.py` is imported, not before
    `core.py` is. A module's import statements run first, and `core`'s first
    line imports `constants`, which reads the mouse cursor as it goes.

  **Step numbers are translated back.** Consecutive drives in one direction, and
  consecutive waits, are merged into a single segment by the builder, so segment
  numbers drift away from the step numbers a person wrote. `build_run` keeps a
  map and reports each diagnostic against the step as written.

  **The int16 overflow promised in 1.6** is caught here and reported as an error
  diagnostic, with `module_text` set to `None`.

- [x] **1.8 Package it for the browser.**
  - `uv build --wheel` produces the wheel; `micropip.install(wheel, deps = False)`
    installs it in Pyodide, so the pygame and matplotlib dependencies that
    desktop users need are simply skipped.
  - **The wheel had to be slimmed.** The published one is 13.5MB, of which the
    Python code is 0.08MB — 0.6%. The rest is menu images (57%), the
    documentation PDFs (26%) and screenshots (15%), none of which the planning
    half ever opens. `tools/slim_wheel.py` strips those and rewrites the
    RECORD, giving a **0.11MB** wheel for the browser. The PyPI wheel is
    untouched: the simulator still needs its images.
  - `tools/pyodide_check.html` is the page that proves it, and the instructions
    for re-running it are at the top of the file.
  - *Done:* in Chrome, Pyodide loaded in 3.6s, the wheel installed in 1.5s, and
    `build_run` returned `STEPS = 6`, `MARKERS = (1764, 7942)`, `COUNT = 2607` —
    **the same file the hub is running** — in 0.49s on Python 3.13.2. The mat
    diagnostic came through identically to the desktop.

  **Rebuilding a path took half a second** in the browser, against 38ms on the
  desktop: roughly ten times slower, as expected of WASM. Fast enough to rebuild
  as somebody edits, but step 2.2 should debounce rather than rebuild on every
  keystroke.

---

## Phase 2 — the website, minimum version

Goal: plan a run with drive/turn/wait/go-to steps, watch it, and download a
working hub file. No actions yet.

- [x] **2.1 Scaffold `web/`, and the container that serves it.** Vite +
  TypeScript. Keep dependencies minimal; a small UI library such as Preact is
  fine if the step list gets fiddly.
  - A multi-stage `Dockerfile`:
    1. python stage — `uv build --wheel`, then `tools/slim_wheel.py`, giving the
       0.11MB wheel;
    2. node stage — `npm ci && npm run build`;
    3. runtime — nginx serving the built site, the slim wheel, and a **vendored
       copy of Pyodide** (from the npm package or the release tarball). No CDN.
  - A `compose.yaml` next to it, so the host runs
    `git pull && docker compose up -d --build`. Rebuilding the image is the
    deploy; there is no registry unless we later want one.
  - Cache headers matter more than usual: the Pyodide runtime is the big
    download and never changes between releases, so serve it immutable and let
    the small bundle revalidate.
  - **HTTPS is not optional.** A service worker (2.9) and Web Bluetooth (4.7)
    both require a secure context. Cloudflare terminates TLS at its edge, so
    either host gets that for free: on the home machine a `cloudflared` tunnel
    means no inbound port is opened at all, and the tunnel can run as a second
    service in the same compose file.
  - **Let Cloudflare do the gatekeeping**, rather than building a login into the
    page. An Access policy in front of the hostname lets the team in by email
    and keeps everyone else out, and the same policy covers the storage API in
    2.8 if we build it.
  - Check that the edge actually caches the vendored Pyodide: the `.wasm` and
    data files are the bulk of the download, and a cache rule may be needed for
    them. That, not the container, decides how the first load feels from a
    kid's house.
  - *Decision here:* VPS or home-behind-a-tunnel — see *Open decisions*. The
    image is the same either way, so this can wait until the first deploy.
  - *Done:* the built page loads Pyodide from its own origin in 1.8s — against
    3.6s from a CDN in step 1.8 — unpacks the library in 0.1s, and builds the
    template run in 0.6s, reporting Python 3.13.2 and the same `STEPS = 6`,
    `MARKERS = (1764, 7942)`, `COUNT = 2607` the hub is running. Chrome's
    network log shows **8 requests, all to this host**: the page, the bundle,
    five Pyodide files and the wheel.

  **No package installer.** The plan said `micropip.install(wheel, deps=False)`.
  The page fetches the wheel and hands it to `pyodide.unpackArchive` instead: a
  wheel is a zip, this one has no dependencies, and micropip would itself have
  had to be served from the image. One fewer thing to vendor, and the same
  result.

  **The image builds, and the container serves it.** `docker compose up -d
  --build` produces it; the page then starts Pyodide from
  `http://127.0.0.1:8080/pyodide/` in 1.3s and builds the run, and Chrome's
  network log shows nine requests, **every one to the container**.

  Two faults only a real build could have found:

  - **nginx serves `.mjs` as `application/octet-stream`**, because its
    `mime.types` has no entry for it — and a browser refuses to run that as a
    module, while `curl` reports a cheerful 200 throughout. `nginx.conf` now
    names the type for `.mjs` alone; everything else the runtime is made of
    (`.wasm`, `.zip`, `.json`, `.js`) already had one, checked against the
    running container.
  - **`# syntax=docker/dockerfile:1` pulled a frontend image** from Docker Hub
    on every build, and was the first thing to fail when a credential helper
    was missing. Nothing here needs it, so it is gone: the three base images
    are now all the build fetches.

  The two Docker CLI traps that cost the first attempts — Homebrew's `docker`
  without the compose plugin, and a `credsStore` naming a helper that is not on
  the PATH — are written up in `web/README.md`, because anyone with Docker
  Desktop and Homebrew on the same Mac will meet them.

  **Left as it is for now:** a missing file answers with `index.html` rather
  than a 404, because `try_files` falls back to the app. That is right for a
  page with client-side state, but it means a typo'd asset path returns 200 and
  HTML. Worth tightening once there is a real asset list to protect.
- [x] **2.2 Python worker.**
  - `src/worker.ts` owns Pyodide. It says what it is doing while starting
    (`status`), says `ready` once with the Python version, and answers each
    `build` with a `built` or a `failed` carrying the same id. `build_run` is
    kept as a Python callable, so a build is one call rather than a re-import.
  - `src/planner.ts` is the page's side: `build(run)` for one specific run,
    answered with a promise, and `request(run)` for the run as it currently
    stands, answered through `onResult`. `request` waits 150ms for typing to
    stop and never lets builds pile up — a newer run replaces a waiting one,
    and an answer to a superseded request is dropped rather than drawn.
  - `src/types.ts` mirrors the JSON contract from 1.7. It is one half of an
    agreement with `pythfinder/headless.py`; that file's docstring is the
    other.
  - *Done:* in the container, the worker is ready in 4.0s from cold, builds the
    template run in 0.61s, and returns the same `STEPS = 6`,
    `MARKERS = (1764, 7942)`, `COUNT = 2607` as the hub. **Twenty edits as fast
    as the page can make them cause one build**, and its result is the newest
    edit's.

  **The no-stall claim is not yet measured.** The page watches frame gaps while
  building, which is the honest way to show the worker is doing its job — but
  Chrome does not paint a tab that is not on screen, so an automated run
  reports zero frames and the page says so rather than claiming a perfect
  score. Open `http://127.0.0.1:8080` by hand to see a real number.
- [x] **2.3 Field view.**
  - `src/field.ts` holds the convention and nothing else: the mat's size, the
    robot's, and the maps between field centimetres and canvas pixels, carried
    over from `Components/robot.py:106-114`.
  - `src/fieldView.ts` draws the mat and the robot, and lets the robot be
    dragged and turned with the arrow keys (shift for one degree). Every change
    asks the worker for a fresh run through the debounce from 2.2.
  - The mat image is 2172x1238, an aspect of 1.7544 against the table's 1.7542,
    so it can be drawn across the whole field rectangle without distorting.
  - *Done:* the start pose `(-46, -83, 0)` lands inside the red launch arc at
    the bottom left, nose up the field, exactly as the simulator draws it.
    Dragging to a measured point on screen read back **x 4.6, y -3.8** against
    4.5, -3.8 predicted from the mat's pixel extents; three taps of the right
    arrow gave 15°; and the run readout followed each change.

  **The heading convention is clockwise as drawn.** A turn from 0 (up the
  field) toward 90 (to the right) looks clockwise on the screen, even though it
  is a turn from +x toward +y. The canvas rotation is therefore `+head`, plus a
  quarter turn because the robot photo has its nose at the left edge — the same
  `head + 90` the simulator uses. `fll_run_template.py` described this as
  "counter-clockwise", which is true of the axes and misleading about the
  picture; its comment now says which way it looks.
- [x] **2.4 Step list.**
  - `src/runEditor.ts` lists the run as rows, one per builder call: Drive (cm,
    negative for backwards), Turn to (deg), Wait (ms), Go to point and Go to
    pose, the last three with a "backwards" tick. Add, reorder, delete, and
    type into any number.
  - No heading-mode choice and no splines, as decided in 1.1.
  - Selecting a step lights its share of the path. The path itself is drawn
    here rather than in 2.5, because there is nothing to light up without it;
    2.5 still owns playback and the scrubber.
  - *Done:* adding a Drive took the run from 15.6s to 17.7s and lit the new
    leg; moving it up merged it with the drive above (3.2s, and the row below
    reading "joined to the step above"); deleting it un-merged them and put the
    run back to 15.6s. The mat warning re-attributed itself to the right step
    at every stage.

  **The contract grew a `steps` list.** Nothing said *when* each step ran, so
  the page could not light one up. `build_run` now returns `starts_ms` and
  `ends_ms` per described step, and `TrajectoryBuilder.step_ends` became public
  to supply it. A step merged into the one before it reports its start and end
  as the same moment — there is no separate acceleration profile to point at —
  and the row says "joined to the step above" rather than claiming 0.0s.

  **Two things only using it revealed:**

  - **Selecting a turn showed nothing at all.** A turn on the spot has no
    stretch of ground to colour, and most turns are on the spot. It now marks
    the place instead, which is the same treatment merged steps needed.
  - **"Go to pose" ran off the edge of the panel.** Three numbers, a tick, a
    duration and three buttons do not fit one row, so its delete button could
    not be reached. The rows wrap now.

  **A rule for whoever touches the editor next:** the list is rebuilt for
  structural changes only — adding, deleting, reordering, selecting. Typing
  into a number must never rebuild it, or the box being typed into loses focus
  on every keystroke.
- [x] **2.5 Path and playback.**
  - `src/playback.ts` is the clock and nothing else: it walks a time from 0 to
    the length of the run, in real time, and says where it has got to. A run
    that takes 15 seconds on the mat takes 15 seconds here, which is the point
    when the team is trying to fit inside two and a half minutes.
  - Play, pause, a scrubber, and space to toggle. The readout shows the time,
    which step is running, and the pose at that moment; the step's row is
    outlined while it runs.
  - Part-way through a run the robot stands at the playhead and the start pose
    stays as an outline — it is still the thing you drag, and without it there
    is no telling where the run begins.
  - *Verified in the container:* at 7.1s of 15.6s the robot was drawn along the
    path with the start pose outlined, the button read ⏸, the slider had
    advanced, step 4's row was marked running, and the pose read
    `x 29.0 y -62.8 head 90.0°` — which the run's own geometry confirms
    (75cm up from x -46 is x 29, then a turn to 90 and 30cm along +y from
    y -83). Seeking to the end put the robot home at `x -46.0 y -83.0`, as
    `toPose(START)` requires.

  **A readout bug the end of the run exposed:** the heading there read
  `360.0°`, which is the same heading as 0 and looks like a mistake beside a
  start pose of `0°`. It is normalised now.

  **Honest limit on that last fix:** it is checked by arithmetic
  (`normaliseHead(360) === 0`) and by reading the call site, not by a
  screenshot. The browser stopped accepting synthetic clicks and key presses
  part-way through this step — in a fresh tab as well, after the same actions
  had worked minutes earlier — so the 0° readout and the space binding have not
  been seen working. Worth a glance next time the page is open by hand.
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
  - *Optional, and the one real gain from self-hosting:* a small storage API in
    the container — `GET/PUT /runs/<name>.json` against a mounted folder — so a
    run planned on one laptop opens on another, instead of being passed around
    as a file. It brings its own questions: who may overwrite whose run, and
    what backs the folder up. Keep browser storage as the fallback so the
    planner still works when the host is unreachable.
- [ ] **2.9 Work without the host.** A service worker caching the bundle, the
  Pyodide runtime and the wheel, so the planner still opens in a gym with no
  usable internet. This is what a hosted page has to earn back: everything now
  depends on reaching Cloudflare, and a competition venue is exactly where that
  fails. Needs the HTTPS from 2.1, since a service worker will not register
  without it.
  - **Never cache a login redirect.** With Access in front, an expired session
    answers a fetch with a redirect to a login page. A service worker that
    stores that as if it were the app will serve it back forever. Cache only
    same-origin 200s, and version the cache on each deploy.
  - *Done when:* the container is stopped, the page is reloaded, and a run can
    still be planned, checked and downloaded.

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
  code.pybricks.com does. Needs the secure context from 2.1.
- [ ] **4.8** Sharpen the offline story: check what actually survives a
  competition venue with no route to the host, and whether the service worker
  from 2.9 covers it.

---

## Run file format

As implemented in step 1.7 and understood by `build_run`.

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
    { "type": "toPose", "x": -46, "y": -83, "head": 0 }
  ]
}
```

- Step types: `drive` (cm), `wait` (ms), `turn` (deg), `toPoint` (x, y) and
  `toPose` (x, y, head). `turn`, `toPoint` and `toPose` take `reversed`. There
  is no heading-mode field — step 1.1 established those collapse on a tank
  drive.
- An action says **when** with `at`: `{"cm": 35}` into the step, or `{"ms": 40}`,
  and negative values count back from the end of the step.
- `do` and `label` are carried by the browser and written into the generated
  file in step 3; `build_run` only needs `id` and `at`.
- `robot` is `"fll_team"`, or an object of numbers for the settings panel:
  `track_width_cm`, `max_velocity_cm_s`, and optionally `center_offset_cm`,
  `max_power`, `width_cm`, `length_cm`.
- `version` lets us migrate old saved runs when the format changes.

---

## Found along the way

None of these are caused by this work, and none of them block it.

- [x] **The joystick heading PID divided by zero.** Fixed in `5e4dd06`: the gap
  is checked before dividing, and the previous derivative is held when no time
  has passed. Original report below.

  `PIDController.calculate()`
  divides by the time since the previous call, in whole milliseconds. Two calls
  inside one millisecond means a `ZeroDivisionError`, and at the simulator's
  1000 FPS ceiling that is reachable: 199 of 200 back-to-back readings share a
  millisecond. It predates this work — `pygame.time.get_ticks()` behaved the
  same way, and step 1.2 kept the semantics deliberately. It only bites under
  joystick control, which is presumably why nobody has hit it. A guard that
  skips the derivative term when no time has passed would fix it; doing so
  changes simulator behaviour, so it wants a deliberate decision.
- [ ] **The team's own run overhangs the table by 7mm.** The new mat check in
  1.6 reports it, and it is not a false alarm: turning on the spot at the launch
  area swings a corner 11.8cm from the middle of the robot — `sqrt(7² + 9.5²)` —
  while `x = -46` leaves only 11.15cm to the edge of a 114.3cm table. So during
  the final turn home, a corner of the robot sits 0.7cm past the edge of the
  mat. Worth checking against the real table, where the border wall may or may
  not be in the way; moving the start pose 1cm inwards would clear it.
- [ ] **`rotate_by` reflects as well as rotating.** Both
  `mathEx.Point.rotate_by` (`mathEx.py:55`) and the free `rotate_by`
  (`mathEx.py:274`) compute `y = x·sin − y·cos`, where a rotation needs
  `x·sin + y·cos`. As written they rotate *and* mirror across the x axis. The
  method also mutates the point it is called on and returns it, so a caller
  that expects a copy silently corrupts its input. Used by the holonomic
  field-centric joystick path in `core.py:188`, which the team's tank drive
  never reaches, and by nothing in the trajectory maths — the goldens are
  unaffected. Step 1.6 rotates its own corners rather than depend on it. Fixing
  it needs a look at whether any caller has been compensating for the mirror.
- [ ] **The swerve export is malformed.** In the wheel-speeds export, the swerve
  branch formats `(power, 2)` — a tuple — where it plainly meant
  `round(power, 2)`, so a swerve robot's file gets `(46.66, 2) 30.0` instead of
  `46.66 30.0`. Untouched by step 1.4, which carried the line across verbatim so
  the goldens could prove nothing changed. It affects nobody here: the team's
  robot is a tank drive, and this branch only runs for `SwerveKinematics`.
  Worth fixing before anyone uses the library for swerve, ideally with a golden
  run that covers it.
- [ ] **The `Trajectory` class shadows the `Trajectory` package.** Importing
  `pythfinder.Trajectory.Segments.Primitives.generic` directly fails with
  `cannot import name 'Segments' from 'Trajectory' (unknown location)`.

  Step 1.2 blamed the editable-install shim. **That was wrong**, and step 1.8
  disproved it: the same failure happens from a real installed wheel in a clean
  interpreter. The actual cause is a name collision. `pythfinder/__init__.py`
  does `from .Trajectory import *`, and among those names is the `Trajectory`
  *class* from `trajectory.py`, which overwrites the attribute pointing at the
  `Trajectory` *subpackage*. `pythfinder.Trajectory` is therefore a class, and
  `import a.b.c as x` — which resolves by walking attributes — walks into it and
  stops.

  Harmless for how the library is actually used: `from pythfinder import X` and
  `from pythfinder.Trajectory.thing import Y` both resolve through
  `sys.modules` and are unaffected, which is why every test and the browser
  build pass. Fixing it means renaming one of the two, or keeping the class out
  of the top-level namespace — worth doing before the package is published
  again, since it makes a normal-looking import fail for no visible reason.

## Open decisions

Decide when we reach the step named. The recommendation is the default.

| Decision | Step | Recommendation |
|---|---|---|
| Run the container on the VPS, or at home behind a Cloudflare tunnel? | 2.1 (first deploy) | Either works and the image is identical; Cloudflare answers "who can reach it" with an Access policy rather than with network membership, so the kids need nothing installed. Pick on where the box should live: the VPS is already up and maintained, the home machine costs nothing and opens no port. |
| Upstream the headless refactor to omegacoreFLL/PythFinder, or keep it in our fork? | end of 1 | Offer upstream once goldens prove nothing changed |
| One fixed team robot, or editable robot settings? | 4.2 | Fixed for the season; editable behind the mentor toggle |
| How `run()` gets the data on the hub (`fromValues` vs a module self-import) | 3.1 | Whichever works on the hub; test both |

## Risks

- **Silent behaviour change in the refactor.** Mitigated by the golden files in
  1.1, captured before any edit.
- **First load, and where it comes from.** Step 1.8 measured 3.6s for the
  Pyodide runtime and 1.5s to install the library, fetched from a CDN on a home
  connection. Served from our own container it should be no slower, and cannot
  be blocked by a school firewall. The library's wheel is 0.11MB once slimmed —
  13.5MB unslimmed, which is why `tools/slim_wheel.py` exists. Cached after the
  first visit either way.
- **The host has to be reachable, and has to stay up.** Self-hosting trades "a
  CDN might be blocked" for a longer chain that all has to work: the venue's
  internet, Cloudflare, the tunnel, and the container. A competition gym is
  where that chain is weakest. The service worker in 2.9 covers planning
  offline; anything kept only on the host — the shared runs in 2.8 — is not
  covered by it, and wants a backup.
- **An expired Access session looks like a broken app.** The team opens the
  planner and gets a login page, or worse, a half-working one if the service
  worker has cached around it. Worth deciding how long sessions last before the
  kids meet it mid-practice.
- **Hub heap.** Each run is roughly 6 bytes per exported state (about 16 KB for
  the template run). Several long runs in one program add up, hence 4.6.
- **Kids writing blocking custom actions.** Blocks cannot block; the code editor
  warns. The hub-side behaviour stays as documented in `fll_run_template.py`.
