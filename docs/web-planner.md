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
- [x] **2.6 Problems panel.**
  - Each problem now sits on the step that caused it, with its suggestion, and
    that row gets a red edge. A warning about step 4 belongs on step 4, not in
    a list at the bottom that nobody reads. Only run-wide problems — the ones
    with no step to sit on — stay in the panel below.
  - Anywhere the robot hangs off the mat is marked in red on the path.

  **The page works out the off-mat stretches itself**, rather than asking for
  them: the worker's warning says when the *worst* moment is, not how long the
  trouble lasts. The geometry is the same either way — four corners turned to
  face the way the robot is going — and it is computed once per build, because
  playback redraws sixty times a second.

  **Two rounds of guessing, settled by measuring.** The mark would not appear.
  I assumed the excursion was too brief to survive the 20ms thinning of the
  poses, and then that a thin line was hiding under the sprite. Reading the
  canvas pixels directly showed **zero** of the marker's colour, and running
  the page's own range-finding over real pose data showed it was working
  perfectly: two ranges, at t 14001-14401ms and t 14661-14961ms, each spanning
  **0.00cm** — the robot turning on the spot at the edge of the table. The
  marks were being drawn at the robot's own position, before the robot, and
  painted over.

  So they are drawn after the robot now, and ringed wide enough to clear it.
  The same pixel probe went from 0 to 1131 pixels of `#ff1744`, clustered on
  the launch area where the pirouette happens.

  **The lesson worth keeping:** a marker that lands where the robot stands is
  the normal case here, not the exception — turning on the spot at a table edge
  is how an FLL run usually misbehaves. Step 2.4 learned the same thing about
  selecting a turn.
- [x] **2.7 Download.**
  - A name box, a Download button, and a readout of what the run costs the hub.
    `src/download.ts` holds the three pieces: the name check, the size, and
    saving the file.
  - **The name is checked as a Python identifier**, because the hub imports the
    file by name — `import run_a`. A team member who types "Run 1" or "left
    side" would otherwise find that out on the hub, at a competition. Empty,
    leading digits, spaces and Python's own words are each refused with a
    reason, and the button is disabled until the name is usable.
  - The button is also disabled while the run has an error, rather than handing
    over a file that cannot drive.
  - *Verified in the container:* `run_a` enabled, reading "2607 states, 15.3KB
    on the hub"; `2fast`, `left side`, `import` and empty each disabled with
    their own message; `run_b` enabled again. The module's docstring follows
    the name box, and 2607 x 6 bytes is exactly the 15.3KB reported — the
    readout is the hub's memory cost, not the 61KB of Python source that
    carries it.

  **Done on the robot.** A run planned in the browser was downloaded, added to
  `runs.py`, and driven on the hub — so the whole chain holds end to end:
  plan in a page, build in Pyodide, download, upload at code.pybricks.com,
  drive.

  It dropped into the existing convention without waiting for 3.2: the file is
  data only for now, but it exposes exactly the four names `Trajectory(module)`
  already reads, so `import run_a` and `Trajectory(run_a)` is enough to drive
  it. What 3.2 adds is the attachment motor code and a `run()` of its own, not
  the ability to drive at all.

  **What that run did not prove.** It was planned with no actions —
  `MARKERS = ()` — because the page cannot add one until 3.2. So the browser's
  path reached the robot and drove, but **no marker from the planner has ever
  fired on the hub**: every marker test so far has been against the older
  hand-made export, or under CPython stubs. The first hub test of 3.2 must
  include an action of each kind, or that gap simply moves forward one step.
- [x] **2.8 Save and load.**
  - `src/store.ts` does three separate jobs: autosaving the run in progress,
    keeping a named list to pick between, and export/import as a `.json` file
    for carrying to another laptop.
  - Autosave happens on every change rather than on a button, because the
    change somebody loses is always the one they did not think to save.
  - Every read is defensive. Browser storage can be full, switched off, or
    holding something an older version of this page wrote, and none of those is
    a reason for a team member to lose their run: a file that is not a run is
    refused by name rather than loaded as rubbish.
  - *Verified in the container:* edits tracked into storage; Save listed the
    run and re-opening it restored the saved values over newer edits; Delete
    removed it; the export payload parsed back as a run, and `{"hello":1}`,
    `[1,2,3]` and `null` were all refused. Then the real test — **reload the
    page and the run comes back**: name, all five steps, the edited 137cm and
    the start pose.

  **A bug the first round-trip found:** autosave only ran inside `rebuild()`,
  which edits call — so a run that was opened and *left alone* was never
  written down. That is exactly the run somebody loses when they close the tab
  on their way out. The page now saves what is on screen at start-up too.

  **Seen in passing:** with the first step stretched to 137cm the run runs off
  the top of the table, and the off-mat marking from 2.6 drew a long red
  stretch with a 45.7cm warning on the step that caused it. Until then that
  path had only been exercised against a 0.7cm pirouette.
  - *Optional, and the one real gain from self-hosting:* a small storage API in
    the container — `GET/PUT /runs/<name>.json` against a mounted folder — so a
    run planned on one laptop opens on another, instead of being passed around
    as a file. It brings its own questions: who may overwrite whose run, and
    what backs the folder up. Keep browser storage as the fallback so the
    planner still works when the host is unreachable.
- [x] **2.9 Work without the host.**
  - `web/public/sw.js` keeps two caches: a shell for the page and its bundle,
    replaced on every deploy, and a heavy one for Pyodide, the wheel and the
    pictures — 12.7MB that does not change within a release. Bumping the
    worker's `VERSION` throws away the shell and keeps the heavy things, so a
    deploy costs kilobytes rather than another 13MB over school wifi.
  - Only same-origin, non-redirected 200s are stored, so an expired Cloudflare
    Access session cannot be filed away as if it were the app.
  - nginx serves `/sw.js` with `no-cache`: a stale worker keeps control of the
    page and would go on serving last week's planner after a deploy.
  - *Done:* **container stopped, page reloaded, and the planner still opened,
    started Python and built the full run** — 15.6s, five steps, the mat
    warning on step 5, and the download button live. `docker compose ps`
    showed `Exited (0)` and nothing was listening on the port.

  **The bug that would have wasted a competition:** caching only on fetch left
  the hashed bundle out. A worker takes control *after* the page load that
  starts it, so a first visit never passes its own `/assets/index-HASH.js`
  through the worker — the planner would cache 12.7MB of Pyodide and then
  refuse to open for want of 21KB of script. The worker now reads the page
  during install and precaches whatever it names.

  **And a test that lied:** a `fetch("/index.html", {cache: "no-store"})` from
  inside the page reported the host as reachable while the container was
  stopped. `no-store` governs the browser's HTTP cache, not the service worker,
  which answered from its own. A page controlled by a worker cannot tell
  whether its host is up; that has to be checked from outside.
  - *Done when:* the container is stopped, the page is reloaded, and a run can
    still be planned, checked and downloaded.

---

## Phase 3 — attachment actions

Goal: kids attach motor actions to a step, and the downloaded file contains
working, correctly ordered marker functions.

- [x] **3.1 Hub-side convention (quick-start repo).** *Driving on the hub, at
  the fourth attempt.* The downloaded file
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
  - `Trajectory.fromValues(steps, markers, count, data)` is in the hub's
    `trajectory.py`. It builds a `_Values` holder and goes through `read()`, so
    the two ways of building a trajectory share all their behaviour rather than
    growing apart. The self-reference problem is sidestepped rather than
    solved: the module passes its values, not itself.
  - `example_run.py` is a hand-written run in the new shape, four states of
    nonsense data so the shape is legible.
  - The quick-start README documents it, next to — not instead of — the
    existing two-file way, which is still the only way to drive a run until the
    planner generates the new one in 3.2.
  - **`runs.py` is left alone.** It has uncommitted changes of the team's in
    it, and switching it over buys nothing until 3.2 exists.

  *Checked in CPython, with Pybricks stubbed* (`trajectory.py` imported for
  real, a fake clock so a 15s run takes a moment):

  - a trajectory built from values is identical to one built from the module —
    same STEPS, COUNT, TRAJ_TIME, DATA and marker times;
  - driving the hub's own `traj_run_a`, both actions fired **in order**, at
    wheel commands 293 and 1323 against markers at 1764ms and 7942ms
    (÷ 6ms = 294 and 1323; the first fires just before that state's powers go
    out), 2606 commands for 2607 states, motors stopped and drive braked;
  - `example_run.run(core)` bound both actions and fired them in order, with
    the wheels getting the powers its data encodes.

  **The hub found something the stubs never could.** Running `example_run.py`
  on the robot died immediately:

      ImportError: no module named 'struct'

  Pybricks does not ship `struct`, and `trajectory.py` had imported
  `unpack_from` from it since the SPIKE port — so `traj_test` would have failed
  the same way. The whole data format was designed around
  `struct.unpack_from` indexing the bytes in place.

  The CPython harness could not have caught it: it imports the real
  `trajectory.py`, but CPython *has* `struct`, so that line always resolved.
  A stub stands in for the thing it replaces, never for the thing that is
  missing.

  Fixed in the quick-start repo: `ustruct` first (MicroPython's name for it),
  then `struct`, and failing both the three little-endian int16 are read
  straight out of the bytes by hand. The decoder is chosen once at import, not
  per iteration of the follow loop. Checked against the hub's own
  `traj_run_a` — all 2607 states decode identically either way, 888 of them
  carrying a negative, and the int16 extremes are right; with both modules
  hidden the full run still drives and both actions fire in order.

  **Then it found a second one:** `math_ex.py` failed on `import math`, taking
  `trajectory.py` with it. Pybricks provides `umath`, so that is tried first
  and plain `math` second — the same shape as `ustruct`.

  **Swept the rest rather than wait for a third.** Every other file in the
  quick-start imports only `pybricks` or something local; `math_ex.py` and
  `measure_max_velocity.py` were the only two reaching outside. So there is no
  third module of this class waiting, which is worth knowing before walking
  back to the robot.

  Re-checked with both `math` and `struct` hidden and `umath` standing in: the
  full run drives, 2606 wheel commands, both actions in order, and
  `example_run` moves its arm twice.

  **Fourth attempt: it drives.** The three things the stubs could not answer
  are answered, all at once, by watching it work:

  - MicroPython accepts the `@staticmethod` on `Trajectory`;
  - the `lambda` closures over `core` behave, so the actions reach the right
    robot;
  - the timing holds on real hardware — the arm moves twice, and the robot
    returns to where it started.

  The shape is settled, which is what 3.2 needs: it generates files in exactly
  this form.

  **Third attempt: it loaded cleanly and did nothing** — which was a fault in
  the example, not in the library.

  - `example_run.py` defines `run(core)` and nothing calls it. Imported from
    `runs.py` that is correct; run directly as the program, which is the
    obvious way to try it, it defines two functions and exits. It has a
    `__main__` guard now, with `Robot()` built inside the guard so importing it
    still costs nothing.
  - Its data was four states of nonsense — **24 milliseconds** of driving, with
    the two arm commands 6ms apart. Nothing anybody could see. The file was
    written to be readable and forgot it had a job to do on a robot. It now
    carries a real 4.3 second run: forward 25cm dropping the arm 8cm in, half a
    second still, then back to the start raising the arm at the end.

  The ports in `config.py` were already correct when that run sat silent, so
  the silence really was those two faults and not the wiring. (The port layout
  was remapped separately — wheels to E/A, task motors to B/F, sensors to C/D —
  read off the build.)

  **The pattern worth remembering:** every one of these was invisible here and
  obvious in seconds on the robot. Firmware that renames the standard library
  cannot be stubbed for — a stub reproduces what a module *does*, never what the
  platform *lacks*. And a test fixture small enough to read is often too small
  to prove anything: four states passed every check here and was unobservable
  there.
  **Two kinds of action block, decided with the team.** A run needs both, and
  they are different things rather than two settings of one thing:

  - **Parallel** — attached to a motion step, fires at a moment, the robot
    keeps driving. This is what a marker already is. Only motor calls that
    return at once may be offered: `run(speed)`, `stop()`.
  - **Sequential** — its own step. The robot stops, the arm runs *to
    completion*, and the next step begins after it. Only calls that end make
    sense here: `run_angle`, `run_target`, `run_until_stalled`.

  A step between two drives brings the robot to a standstill on its own —
  every segment decelerates to zero, which is why consecutive drives are
  merged — so a sequential step gets its stationary robot without any new
  motion maths.

  **How the waiting is done (approach A, chosen over splitting the run).** The
  arm step is a wait segment plus a marker that runs the motor with
  `wait=True`. The hub has no threading, so that blocking happens on the follow
  loop; `trajectory.py` now discounts time spent inside a marker from the path
  clock, so nothing is skipped. The alternative — splitting the run into
  several trajectories with the arm movements between them — is conceptually
  cleaner but changes the file shape that was just proved on the robot, and the
  contract `build_run` returns.

  **The cost of A, worth saying out loud:** the planner can only *estimate* how
  long an arm step takes, from its speed and angle, so the run length on screen
  is an estimate wherever a sequential step appears. The robot genuinely waits
  either way; only the prediction is approximate, and it should say so.

  **The Python half is done.** `armStep` becomes a wait segment sized by
  `angle / speed`, with its action at the moment the robot comes to rest;
  `hubModule` now emits `_action_N(core)` functions and a `run(core)` binding
  them in firing order, each guarded against an attachment that is not plugged
  in. 91 tests pass, seven of them written before the feature.

  **Two faults the tests-first order caught, both invisible otherwise:**

  - **Consecutive arm steps fired together.** They merge into one wait segment,
    so both actions landed on the same millisecond and were bound in whichever
    order they came out — the arm told to do two things at once, in the one
    step type whose entire purpose is that one follows the other. Each now
    starts where the previous finished.
  - **The merged second step reported zero length**, which tells a team member
    that a step the robot genuinely waits for is free. Merged *drives* still
    report no time of their own — two drives really are one acceleration
    profile — but merged arm steps take their own share.

  **The golden had to be split, not rewritten.** The template run has actions,
  so its module now carries code the hub fixture predates. The data is still
  compared against that fixture line for line — those are the numbers the robot
  drove — and the code section is asserted separately. (My first attempt at
  splitting cut the file at the first `def` or `from`, which truncated the
  generated module to its docstring, because the import sits directly under it.
  The constants are picked out by name now.)

- [x] **3.2 Action blocks.** *Both kinds working in the browser, generating
  real hub code — not yet driven on the robot.* Attach to a step at "X cm in", "X ms in", or
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

  **The browser half was being judged by a two-day-old library.** Both blocks
  render — a "Move arm" adder, and "+ action while driving" on motion steps but
  correctly not on a wait — yet the arm step came back marked with a problem:

      'armStep' is not something the robot knows how to do
      — try: use one of: drive, wait, turn, toPoint, toPose

  That list is `STEP_TYPES` *without* `armStep`: the page was running the
  library from before the feature existed. The container was innocent — it
  serves 113,519 bytes with `armStep` present, read out of the wheel rather
  than inferred — while the browser held 111,014 bytes from the day before.

  **The service worker had pinned it, and no reload could shift it.** The wheel
  sat in `planner-heavy`, which is cache-first and which no `VERSION` bump ever
  clears, because that cache was designed for things that do not change between
  releases: Pyodide, the mat, the robot. The wheel looked like one of them —
  fetched once at start-up, small next to 13MB — but it *is* PythFinder, so it
  changes whenever the Python half does. Worse, `fetch(url, {cache: "reload"})`
  returned the stale copy too: `cache: "reload"` governs the browser's HTTP
  cache, and a worker answers before the network is consulted at all. Short of
  clearing site data by hand, that browser could never have received a fix.

  Fixed by moving the wheel to the shell: replaced on every deploy, still
  cached for a gym with no host (110KB, against the 13MB that stays put). The
  fetch handler picks a cache by *name*, so the old entry would have gone on
  answering even under the new policy — `activate` deletes it, which is what
  actually frees a browser already holding one.

  **`activate` also re-fetches the wheel, for a gap the move did not close.**
  The shell is only discarded when `VERSION` changes, and `VERSION` lives in
  `sw.js` — so a deploy that changes *only* the Python half ships a new wheel
  while the worker stays byte-identical, `update()` finds nothing to install,
  and the browser keeps the library it had. Unlike everything else recorded
  here, that one is reasoned rather than witnessed: I thought I had caught it
  happening, and the reading turned out to be a probe racing a reload. The
  guard is kept because the argument holds on its own; the claim to have seen
  it is withdrawn.

  **The same shape as the `struct` failure, one level out.** There the harness
  had a module the hub lacked; here the tests had a library the browser lacked.
  Both times everything local passed while the only environment that matters
  ran something else, and both times the answer was to read what that
  environment actually had rather than trust that a build implies a delivery.

  **Then the fresh library dropped every action the button makes.** With the
  real wheel in place, a parallel action added by clicking "+ action while
  driving" was still discarded:

      an action 0.0cm into this step was dropped, because the step only
      goes as far as 75.0cm

  Not staleness this time. `addAction()` defaults to `at: { cm: 0 }` — the
  moment the step begins, the only default that is sensible whatever the step's
  length — and markers are placed by an **open** interval test, so one landing
  exactly on a segment's first or last state counts as outside it. Reproduced
  away from the browser: `0.5`, `1`, `20` and `74` cm all survive a 75cm step;
  `0` and `75` are dropped. The same open test rejects `ms: 0`, and `-40cm` on
  a 40cm step (counting back to the start).

  It is worse than a warning, because the file still downloads: the generated
  module carries `MARKERS = ()`, so the action reaches the robot as nothing at
  all. A team member sees an arm that does not move, with a working-looking
  file in their hand.

  **Two gates, not one** — a fix to either alone leaves half the cases broken:

  - `trajectoryBuilder.py:506`, on displacement, which `cm: 0` fails;
  - `generic.py:173`, on time, which `ms: 0` fails without ever reaching the
    first.

  The shared helper `in_open_interval` is the wrong place to fix it: its other
  callers are menu code (`presets.py`, `buttons.py`) with no stake in markers.
  The three callers of `time_in_segment_segm_time` are all marker placement,
  and `time_in_segment_traj_time` has none, so the blast radius is this
  feature.

  **Why the fixtures never caught it.** `test_editor_shapes.py` uses `cm: 20`,
  and every hand-written run in the suite picks a comfortable number in the
  middle. The one value a team member gets for free, by clicking the button,
  was the one value never tested — the same lesson as the four-state fixture in
  3.1, arriving from the other direction.

  **A wrong turn of my own, recorded because the reasoning was the fault.**
  My first attempt guarded the time gate with `if time < 0: return False`,
  reasoning that `normalize_segm_time` *wraps* negatives (`total_time + time
  - 1`) rather than clamping, so they had to be excluded. That deleted a
  marker the robot has actually driven: the goldens dropped from
  `MARKERS = (1764, 7942)` to `MARKERS = (1764,)`, because wrapping is not a
  hazard to defend against — it is precisely how "1ms before the end" is
  implemented, and `markers_relative` pins it with
  `.addRelativeTemporalMarker(-1, _action)`. Twelve tests went red, all of
  them earned. The gate now normalises first and tests what the callers will
  actually index with.

  **`binary_search` cannot return the last index**: it narrows with
  `while left + 1 < right` and returns `left`, so asking for the end of a 40cm
  drive answers with the state before it. Proved away from any trajectory — on
  `list(range(11))` it resolves 0, 1, 5 and 9 correctly and returns index 9
  for target 10. That needed an explicit last-state case.

  **But that was not why the exact end was refused, and I guessed three times
  before printing the number.** I blamed the interval, then a stale `.pyc`,
  then a shadowed import — checking each only well enough to move on. The
  actual value ends the argument at once:

      asked      : 40.0
      states[-1] : 39.99999997400722
      asked - last = 2.6e-08   ->  closed test: False

  A 40cm drive never quite reaches 40cm. The step's own length, the number the
  planner puts on screen, is 26 nanometres past where the profile stops, and
  `in_closed_interval` was right to reject it — my last-state branch sat behind
  a test that had already returned. The fix is a tolerance of 0.001cm, which
  must be coarser than the 3-decimal rounding markers get on the way in
  (`__process_relatives_into_absolutes_displacement`) while segment
  displacements carry full float error. The two sides are quantised
  differently by construction.

  The lesson is the one this project keeps teaching in new clothes: three
  plausible explanations cost more than one `repr()` of the value in dispute.

  With both gates and the tolerance in place: **101 tests pass**, the six new
  boundary cases among them, and the goldens and hub module are untouched.

  **Fixed: an action on a merged step fired in the step before it.** Found
  while measuring the boundary, pinned as failing tests first, then fixed:

      cm: 15 on the second of two 30cm drives  ->  1042ms
      cm: 30 on the second of two 30cm drives  ->  1584ms
      cm: 30 on a single 60cm drive            ->  1584ms

  The last two being identical was the whole bug. Consecutive drives merge into
  one profile, and a distance-based action on the *second* drive was measured
  from the start of that profile rather than from where its own step begins, so
  every action attached to it fired during the first — up to three seconds
  early, with no diagnostic at all.

  It was worse for a team than the drop it was found next to. A dropped action
  produces an arm that does not move, which is at least obviously broken; this
  produced an arm that moved confidently at the wrong place on the mat, from a
  file that looked correct.

  **The cause was one thing wearing three faces.** `segment_owner` records only
  the step that *created* a segment, so a merged step's marker was placed
  against a segment belonging to the step before it. Three symptoms, one fix:

  - a parallel action on a merged **drive** fired in the drive before;
  - three merged drives put all three actions on the same instant, and they
    came back `c, b, a` — the hub would have bound them backwards;
  - an **explicit** action on an arm step fired inside the previous arm step,
    because `into_wait` reached only the arm step's own implicit marker.

  `_at_within_step` now moves every action alike from "into this step" to "into
  this segment", accumulating centimetres across merged drives and milliseconds
  across merged waits, resetting wherever the builder starts a new segment.
  Negative offsets are resolved against the step's own length rather than left
  to the builder, which knows only the segment.

  Checked against an independent reference — two 30cm drives versus one 60cm
  drive, which must agree exactly:

      cm: 0   -> 1584 = 60cm@30      cm: -5  -> 2566 = 60cm@55
      cm: 15  -> 2125 = 60cm@45      cm: -30 -> 1584 = 60cm@30
      cm: 30  -> 3167 = 60cm@60      three drives -> 1, 1223, 1945 in order

  **One form is deliberately not covered**, pinned as xfail: a *time* offset
  into a merged drive. `cm` on a drive and `ms` on a wait are lengths the step
  itself declares; when the second of two merged drives "begins" in one
  acceleration profile is not knowable until the trajectory is built. The
  planner only ever creates `cm` actions, so it is unreachable today — the test
  is there for whoever adds a time field to the action row.

  **Arm steps back to back were mostly right**, which the team will lean on:
  arm down then arm up gives `MARKERS = (1702, 1882)`, two distinct
  `run_angle(..., wait=True)` calls bound in order — `into_wait` was already
  doing that job. What it did *not* cover was an **explicit** action on the
  same arm step: that went straight to the builder unoffset and fired at
  1711ms, inside the previous arm step. Only the implicit marker was ever
  corrected. Both go through `_at_within_step` now, and the same was true of
  two plain `wait` steps in a row. Worth knowing why the
  timeline still looks odd there: `_step_times` hands each merged arm step a
  notional slice, so the second reports `(2060, 2240)` while its marker fires
  at 1882 inside the shared wait segment. The markers are right; only the
  reported window differs, and `main.ts:122` will not highlight a step whose
  window is a single instant, so playback never points at the wrong one.

  **A turn covers no distance, so a cm-based action on one is meaningless.**
  I predicted its states would share a single displacement, making
  `left == right`. They do not, quite: an `AngularSegment` runs
  `19.999997353207014 -> 20.0`, a span of about 2.6e-6 from floating-point
  drift. So the interval admits only that sliver and every real cm value falls
  outside it, while *time*-based actions on the same turn work perfectly
  (`0ms` -> 1701, `-1ms` -> 3250, within a 1700–3250 span). The editor offers
  "+ action while driving" on turns, and the distance field is the only way it
  lets you place one — so that combination can never fire. That is a UI
  question, not a builder patch: a turn needs its action placed in time.

  **The marker offset is the library's own, and stays.** Marker times come
  back one millisecond after the moment asked for — `ms: 0` gives 1, `ms: 500`
  gives 501 — and `tests/golden/markers_relative.txt` opens with `1764 3708`,
  neither a number anyone would invent. The boundary test asserts that
  convention rather than a tidier one; correcting it would move every marker
  time in every file already driven.

  **The hub test that is still owed.** No planner-made marker has ever fired
  on the robot — the run driven in 3.1 had `MARKERS = ()`. `run_actions.py` in
  the repository root is generated for exactly that: 4.5 seconds, 755 states,
  `MARKERS = (1, 2086, 2266)`, one parallel action and a **back-to-back pair**
  of arm steps, which is the operation the team will reach for most.

      _action_1  core.leftTask.run(500)                        at 1ms
      _action_2  core.leftTask.run_angle(500,  90, wait=True)  at 2086ms
      _action_3  core.leftTask.run_angle(500, -90, wait=True)  at 2266ms

  Drive forward 30cm with the arm starting as it sets off; the arm then goes
  down 90° and back up 90° with the robot stationary; then back 30cm. Add it
  to `runs.py` and call `run_actions.run(core)`.

  Four things only the robot can answer, and each has a visible tell:

  - the parallel action fires **at the very start** — the arm should move as
    the robot sets off, not after. This is the `cm: 0` case that was silently
    dropped until the boundary fix, and it has never run on hardware;
  - the two arm steps happen **one after the other**, 180ms apart, not
    together. They share a wait segment, so this is the case `into_wait` and
    `_at_within_step` exist for;
  - the sequential actions **block the follow loop** and the clock discounts
    them, so the return leg should still be 30cm. If the compensation is wrong
    the robot comes back short or long — measure where it stops. Two blocking
    markers in a row is a harder test of that than one;
  - all three fire **in order**, once each.

- [x] **3.3 Custom code action.** A parallel action is now either kind, chosen
  from a small picker on its own row: **Run motor**, the existing picker, or
  **Custom code**, a CodeMirror 6 editor with `core` in scope, for whatever the
  picker cannot say — reading a sensor, counting something, moving two motors
  from one action.

  **Where the two checks the plan asked for actually live.** "Check syntax in
  the worker with `compile()`" turned out to need no new protocol at all:
  `build_run` already runs inside the worker on every change, and `compile()`
  is a plain builtin — Pyodide is CPython, so it behaves exactly as it would
  on the desktop. A `SyntaxError` becomes an error-level `Diagnostic` with
  Python's own message, on the step that has it; the existing `ok` gate that
  already disables the save button for any error did the rest with no new
  plumbing. The blocking-call check (`wait(`, `while`, and `run_angle(` /
  `run_target(` without `wait=False` — the plan named only `run_angle`, but
  `run_target` waits by default for the same reason, per `_call_for`) is
  substring matching, exactly as specified, and comes back as warnings, which
  the existing `.troubled` styling already shows without treating them as
  blocking.

  **The generated code carries no guard.** A picker action gets
  `if core.leftTask is not None:` because the picker names one motor the file
  can check before touching it. Code the team wrote may reach any motor,
  several, or none — there is nothing generic left to check, so it becomes
  the function body verbatim, indented under the def with its own relative
  indentation preserved. Blank code (an action mid-way through being typed)
  becomes `pass # nothing written yet`, the same allowance an unplugged motor
  picker already gets.

  **A message bug the Python test didn't catch, and the browser did.** An
  early version read *"this action's code calls run_angle) without
  wait=False"* — one paren, not two. `call[:-1]` correctly dropped the
  trailing `(` from `"run_angle("`, but the format string still appended a
  bare `)` on its own, and the unit test only asserted `"run_angle" in
  message`, which was true either way. Caught reading the actual diagnostic
  text in the browser, fixed, and the test tightened to the exact wording —
  the same shape as every other lesson this project keeps relearning: an
  assertion loose enough to pass proves less than it looks like.

  **A second bug the design caught before it shipped, not after.** CodeMirror's
  editing surface is a `contenteditable` div, not an `<input>`. The page's own
  Space-bar shortcut for play/pause checked only
  `event.target instanceof HTMLInputElement`, so a space typed while writing
  code would have fallen through and toggled playback instead of being typed.
  Found while designing the integration, before any code shipped; fixed by
  also checking `event.target.isContentEditable`, and confirmed in the browser
  by dispatching a real `keydown` at the focused code editor and checking the
  play icon never changed.

  **One CodeMirror instance per action, kept alive across `render()`.** The
  step list is fully rebuilt on every structural change — adding a step,
  reordering, even switching a *different* action's kind — and the project's
  own rule is that typing must never trigger one of those rebuilds. A fresh
  `EditorView` on every rebuild would have been fine for typing itself, but
  would reset anyone mid-edit in an action nothing structural touched: cursor
  gone, undo history gone, the moment someone elsewhere added a step. Instead
  each view is created once, keyed by the action's own id (ids already have to
  be stable — the hub binds by them), and `render()` re-parents its DOM node
  into whatever new row it built rather than discarding it; a `pruneCodeViews`
  pass destroys the ones whose action no longer exists. Proved in the browser,
  not just reasoned about: typing code, then adding an unrelated step
  elsewhere, left the *same* `.cm-content` DOM node in place with its text
  intact (`sameDomNode: true`).

  The update listener looks its action up by id when it fires, rather than
  closing over the index it was created with — a step can move, or an earlier
  action can be removed, after the view exists, and a captured index would
  then write to the wrong step.

  **Styling is the app's own palette, not CodeMirror's default.** A custom
  `EditorView.theme` sets background, text, caret and selection to match;
  token colours are CodeMirror's stock highlight style, left alone deliberately
  — hand-matching every Python token class to this palette was judged more
  than a first version needs, and the defaults read fine on the dark
  background as they are. `minimalSetup` rather than `basicSetup`: no line
  numbers or gutter for what is usually one to three lines, `EditorView.
  lineWrapping` so a long line wraps rather than scrolling, `indentWithTab`
  added explicitly (CodeMirror leaves Tab free by default, for pages that
  need it to move focus) since a code editor without a working Tab key would
  be its own kind of broken.

  Adds four packages — `codemirror`, `@codemirror/lang-python`,
  `@codemirror/view`, `@codemirror/commands` — and about 110KB gzipped to the
  bundle (21KB before). Checked against the 13MB Pyodide runtime it rides
  alongside, not worth trimming.
- [x] **3.4 Python view.** A collapsed `<details>` panel, "Python", below the
  download button: two read-only boxes, each with its own Copy button.

  **"The generated `run()`"** turned out to mean the download's own code, with
  its payload elided — not a rewrite. `hub_module_text`'s assembly was split
  into a shared `_module_text(heading, steps, markers, count, data_block,
  actions)` that both the real export and this view call, differing only in
  what `data_block` is: the real bytes, or one line saying how many states
  were elided. Everything else — docstring, `STEPS`/`MARKERS`/`COUNT`, every
  `_action_N`, `run()` itself — is the identical text, by construction, not by
  care taken to keep two things in sync. `tests/test_python_view.py` checks
  this directly: split both texts on `DATA = (`, and everything after must be
  character-for-character the same.

  **"The equivalent `TrajectoryBuilder` chain"** is new synthesis: one
  `.method(...)` line per described step, matching `fll_run_template.py`'s own
  idiom (`TrajectoryBuilder(sim, Pose(...), FLL_FIELD)` — the constructor form
  "team scripts have always used", not `build_run`'s sim-free one) closely
  enough to paste straight into that template's `build(sim)`, replacing "EDIT
  ME 2". The pose is written out in full rather than naming `START_POSE`, so
  pasting it is correct even when the file's own `START_POSE` is something
  else. Every action becomes `lambda: print("<label>")`, whatever kind it is —
  the template's own docstring says a marker only ever runs in the simulator,
  which has no motors to call, so the real code has nowhere to go here; it is
  the other panel.

  **The offset arithmetic is shared, not re-derived.** `chain_blocks` is built
  inside `build_run`'s own loop, using the exact `at` that `_at_within_step`
  just computed for the real marker — not a second pass re-deriving offsets
  from the step list, which would have been a second place for the 3.2 bug to
  reappear once real code exists to paste it into. Proved rather than assumed:
  `tests/test_python_view.py` takes the emitted chain, swaps its one
  desktop-only line for the sim-free constructor, `exec()`s it through the
  real `TrajectoryBuilder`, and checks the resulting trajectory's time and
  marker times against `build_run`'s own numbers for the identical steps — on
  the template run, on the merged-drive case 3.2 fixed, and on two arm steps
  in a row. Only the method-name table (`_chain_step_line`) is a second
  writing of anything, and it is the stable, rarely-touched half.

  A quoted label (`the "big" arm`) becomes a Python string literal, so it has
  to survive being one — escaped rather than trusted, and checked by actually
  running the result rather than inspecting the escaped text.

  Plain `<pre>`, not a second CodeMirror: the plan says *read-only*, and a
  syntax-highlighted editor component earns nothing back for text nobody
  types into. Framed to match 3.3's editor box so the two read as the same
  kind of thing. Copy tries `navigator.clipboard.writeText` and falls back to
  selecting the text when the Clipboard API is refused (no secure context, or
  simply declined) — still one paste away, not a dead button.

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
- `do` is either a motor command (`motor`, `call`, `speed`, `angle`) or, since
  step 3.3, free-form code: `"do": {"code": "core.leftTask.run(500)"}`. Code
  becomes the action's function body verbatim, with no motor guard; a motor
  command gets `if core.<motor> is not None:` first, because it names one
  motor the file can check before touching it.
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
