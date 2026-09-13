/**
 * Step 2.5: watching the run.
 *
 * The robot on the mat says where the run starts, the list says what it does,
 * and the slider walks through what that looks like. Playback is in real time,
 * so a run that takes 15 seconds on the mat takes 15 seconds here -- which is
 * the point, since the team is trying to fit inside two and a half minutes.
 */

import { FieldView } from "./fieldView";
import type { Waypoint } from "./fieldView";
import { Playback } from "./playback";
import { RunEditor } from "./runEditor";
import { createPlanner } from "./planner";
import { hubCost, nameProblem, saveModule } from "./download";
import {
  exportRun,
  forgetNamed,
  forgetRemote,
  importRun,
  listAllRemote,
  listRemote,
  listSaved,
  openRemote,
  recallOwner,
  recallWorking,
  rememberOwner,
  rememberWorking,
  saveNamed,
  saveRemote,
} from "./store";
import { normaliseHead } from "./field";
import type { FieldPose } from "./field";
import type { BuildResult, PathPose, Run, RunStep } from "./types";

/** The left launch area, as in fll_run_template.py. */
const START: FieldPose = { x: -46, y: -83, head: 0 };

/** The template run, as something to start from rather than an empty page. */
const FIRST_STEPS: RunStep[] = [
  { type: "drive", cm: 75 },
  { type: "wait", ms: 600 },
  { type: "turn", deg: 90 },
  { type: "drive", cm: 30 },
  { type: "toPose", x: -46, y: -83, head: 0 },
];

const canvas = document.getElementById("field") as HTMLCanvasElement;
const stepList = document.getElementById("steps") as HTMLElement;
const output = document.getElementById("log") as HTMLPreElement;

const poseReadout = document.getElementById("pose") as HTMLElement;
const mouseReadout = document.getElementById("mouse") as HTMLElement;
const runReadout = document.getElementById("run") as HTMLElement;
const stepReadout = document.getElementById("atstep") as HTMLElement;
const atPoseReadout = document.getElementById("atpose") as HTMLElement;

const playButton = document.getElementById("play") as HTMLButtonElement;
const scrub = document.getElementById("scrub") as HTMLInputElement;
const nowReadout = document.getElementById("now") as HTMLElement;
const totalReadout = document.getElementById("total") as HTMLElement;

const nameBox = document.getElementById("name") as HTMLInputElement;
const saveButton = document.getElementById("save") as HTMLButtonElement;
const costReadout = document.getElementById("cost") as HTMLElement;
const nameProblemLine = document.getElementById("nameproblem") as HTMLElement;

const codeView = document.getElementById("codeview") as HTMLElement;
const chainView = document.getElementById("chainview") as HTMLElement;
const copyCodeButton = document.getElementById("copycode") as HTMLButtonElement;
const copyChainButton = document.getElementById("copychain") as HTMLButtonElement;

const keepButton = document.getElementById("keep") as HTMLButtonElement;
const savedList = document.getElementById("saved") as HTMLSelectElement;
const forgetButton = document.getElementById("forget") as HTMLButtonElement;
const exportButton = document.getElementById("export") as HTMLButtonElement;
const importButton = document.getElementById("import") as HTMLButtonElement;
const importFile = document.getElementById("importfile") as HTMLInputElement;

const ownerBox = document.getElementById("owner") as HTMLInputElement;
const refreshAllButton = document.getElementById("refreshall") as HTMLButtonElement;
const allSavedList = document.getElementById("allsaved") as HTMLUListElement;

let pose: FieldPose = { ...START };
let latest: BuildResult | null = null;

function log(line: string) {
  output.textContent += "\n" + line;
  console.log("[planner] " + line);
}

function loadImage(source: string): Promise<HTMLImageElement> {
  return new Promise((resolve, reject) => {
    const image = new Image();

    image.onload = () => resolve(image);
    image.onerror = () => reject(new Error(`${source} did not load`));
    image.src = source;
  });
}

function showPose() {
  poseReadout.textContent = `x ${pose.x}  y ${pose.y}  head ${pose.head}°`;
}

function seconds(ms: number) {
  return `${(ms / 1000).toFixed(1)}s`;
}

/** Where the robot is at a moment in the run. */
function poseAt(ms: number): PathPose | null {
  if (latest === null || latest.poses.length === 0) {
    return null;
  }

  let found = latest.poses[0];

  for (const point of latest.poses) {
    if (point.t <= ms) {
      found = point;
    } else {
      break;
    }
  }

  return found;
}

/** Which step is running at a moment. The last match wins, so that a step
 *  merged into the one before it does not shadow the step actually moving. */
function stepAt(ms: number): number | null {
  if (latest === null) {
    return null;
  }

  let found: number | null = null;

  for (const step of latest.steps) {
    if (ms >= step.starts_ms && ms <= step.ends_ms && step.ends_ms > step.starts_ms) {
      found = step.index;
    }
  }

  return found;
}

async function main() {
  output.textContent = "";

  // Who's using this browser, remembered from last time.
  ownerBox.value = recallOwner();

  // Whatever was being worked on last time. A closed tab should not cost a
  // team member their afternoon.
  const restored = recallWorking();

  if (restored !== null) {
    pose = { ...restored.start };
    nameBox.value = restored.name;
  }

  const startingSteps = restored === null ? FIRST_STEPS : restored.steps;

  const [mat, robot] = await Promise.all([
    loadImage("/field/mat.png"),
    loadImage("/field/robot.png"),
  ]);

  const planner = createPlanner({
    onStatus: (text) => log(text + "..."),
    onResult: (result) => show(result),
    onError: (message) => log("planner error: " + message),
  });

  const view = new FieldView(canvas, { mat, robot }, pose, {
    onPoseChange: (moved) => {
      pose = moved;
      showPose();
      rebuild();
    },
    onHover: (point) => {
      mouseReadout.textContent =
        point === null ? "—" : `x ${point.x.toFixed(1)}  y ${point.y.toFixed(1)}`;
    },
    // step 4.1: dragging a Go-to step's own target on the field
    onWaypointChange: (index, point) => {
      editor.setStepPoint(index, point.x, point.y);
      rebuild();
    },
  });

  const editor = new RunEditor(stepList, startingSteps, {
    onChange: () => rebuild(),
    onSelect: () => {
      applyHighlight();
      applyWaypoints();
    },
  });

  const playback = new Playback({
    onTick: (ms, playing) => {
      playButton.textContent = playing ? "⏸" : "▶";
      scrub.value = String(ms);
      nowReadout.textContent = seconds(ms);

      const at = poseAt(ms);

      // at the very start the robot is simply where it was put, and is still
      // the thing you drag
      view.setPlayhead(ms === 0 || at === null ? null : at);

      // A run that turns a full circle ends on 360, which is the same heading
      // as 0 and reads like a mistake next to a start pose of 0.
      atPoseReadout.textContent =
        at === null
          ? "—"
          : `x ${at.x.toFixed(1)}  y ${at.y.toFixed(1)}  ` +
            `head ${normaliseHead(at.head).toFixed(1)}°`;

      const running = stepAt(ms);
      stepReadout.textContent = running === null ? "—" : `step ${running + 1}`;

      for (const row of stepList.querySelectorAll<HTMLElement>(".step")) {
        row.classList.toggle("running", Number(row.dataset.index) === running);
      }
    },
  });

  function currentRun(): Run {
    return {
      version: 1,
      // the name goes into the file's own docstring, so it follows the box
      name: nameProblem(nameBox.value) === null ? nameBox.value : "run",
      steps_ms: 6,
      robot: "fll_team",
      start: { x: pose.x, y: pose.y, head: pose.head },
      steps: editor.getSteps(),
    };
  }

  /**
   * Can this run be handed over, and what does it cost the hub?
   *
   * The name has to be a Python identifier: the hub imports the file by name,
   * and a team member who types "Run 1" would find that out at a competition
   * rather than here.
   */
  function showSaveState() {
    const problem = nameProblem(nameBox.value);
    const module = latest?.module_text ?? null;

    nameBox.classList.toggle("wrong", problem !== null);
    nameProblemLine.textContent = problem ?? "";
    nameProblemLine.hidden = problem === null;

    saveButton.disabled = problem !== null || module === null;

    if (module === null) {
      costReadout.textContent = latest === null ? "—" : "nothing to send yet";
      return;
    }

    const { states, bytes } = hubCost(module);

    costReadout.textContent =
      `${states} states, ${(bytes / 1024).toFixed(1)}KB on the hub`;
  }

  function rebuild() {
    const run = currentRun();

    // saved on every change rather than on a button, because the change a team
    // member loses is always the one they did not think to save
    rememberWorking(run);
    planner.request(run);

    // every step change reaches here -- typing an x into the step list same
    // as dragging its marker -- so this is the one place waypoints refresh
    // from, rather than a call at each place that can change one
    applyWaypoints();
  }

  /** Put a run on screen: its steps, where it starts, and its name. */
  function loadRun(run: Run) {
    pose = { ...run.start };
    nameBox.value = run.name;

    editor.setSteps(run.steps);
    view.setPose(pose);

    showPose();
    showSaveState();
    rebuild();
  }

  /**
   * The saved-run picker. Local storage renders straight away -- it is the
   * source of truth and never waits on a network. Anything saved under this
   * owner's name on another laptop, and not already known here, is merged in
   * once the shared store answers; a slow or unreachable host just means
   * that merge never arrives; the local list still works.
   */
  async function refreshSavedList() {
    const local = listSaved();
    const chosen = savedList.value;
    const known = new Set(local.map((entry) => entry.name));

    savedList.replaceChildren();

    const heading = document.createElement("option");
    heading.value = "";
    heading.textContent = known.size === 0 ? "— nothing saved —" : "— saved runs —";
    savedList.append(heading);

    for (const entry of local) {
      const option = document.createElement("option");
      option.value = entry.name;
      option.textContent = entry.name;
      savedList.append(option);
    }

    savedList.value = known.has(chosen) ? chosen : "";
    forgetButton.disabled = savedList.value === "";

    const owner = ownerBox.value.trim();

    if (owner === "") {
      return;
    }

    const remote = await listRemote(owner);
    const newlyKnown = remote.filter((entry) => !known.has(entry.name));

    if (newlyKnown.length === 0) {
      return;
    }

    heading.textContent = "— saved runs —";

    for (const entry of newlyKnown) {
      const option = document.createElement("option");
      option.value = entry.name;
      option.textContent = `${entry.name} (from another laptop)`;
      savedList.append(option);
    }
  }

  /**
   * The mentor view: every saved run, from every owner. On demand only, via
   * the Refresh button, not loaded at start-up -- it is a network call the
   * planner itself does not need, and 2.9's offline story means the page
   * must open and plan a run with no network reachable at all.
   */
  async function refreshAllSaved() {
    const runs = await listAllRemote();

    allSavedList.replaceChildren();

    if (runs.length === 0) {
      const empty = document.createElement("li");
      empty.className = "empty";
      empty.textContent = "— nothing saved yet, or the host is unreachable —";
      allSavedList.append(empty);
      return;
    }

    for (const entry of runs) {
      const item = document.createElement("li");
      const button = document.createElement("button");

      button.type = "button";
      button.textContent = `${entry.owner} — ${entry.name}`;
      button.title = `saved ${new Date(entry.savedAt).toLocaleString()}`;

      button.addEventListener("click", async () => {
        const run = await openRemote(entry.owner, entry.name);

        if (run === null) {
          log(`could not open "${entry.name}" (${entry.owner})`);
          return;
        }

        // Switching the owner box to match is what stops a mentor peeking
        // at Amy's run from accidentally re-saving it as their own.
        ownerBox.value = entry.owner;
        rememberOwner(entry.owner);
        loadRun(run);
        log(`opened "${entry.name}" (${entry.owner})`);
      });

      item.append(button);
      allSavedList.append(item);
    }
  }

  /**
   * Step 4.1: one draggable marker per Go-to step, read straight from the
   * step list rather than from a build result -- a build is an async
   * round-trip to the worker, and a dragged point has to track the pointer
   * with nothing in between.
   */
  function applyWaypoints() {
    const selected = editor.getSelected();

    const waypoints: Waypoint[] = editor
      .getSteps()
      .map((step, index) => ({ step, index }))
      .filter(
        (entry): entry is { step: RunStep & { type: "toPoint" | "toPose" }; index: number } =>
          entry.step.type === "toPoint" || entry.step.type === "toPose",
      )
      .map(({ step, index }) => ({
        index,
        point: { x: step.x ?? 0, y: step.y ?? 0 },
        selected: index === selected,
      }));

    view.setWaypoints(waypoints);
  }

  function applyHighlight() {
    const index = editor.getSelected();

    if (index === null || latest === null) {
      view.setHighlight(null);
      return;
    }

    const timing = latest.steps.find((step) => step.index === index);

    view.setHighlight(
      timing === undefined ? null : { from: timing.starts_ms, to: timing.ends_ms },
    );
  }

  function show(result: BuildResult) {
    latest = result;

    view.setPath(result.poses);
    editor.setTimes(result.steps);
    editor.setProblems(result.diagnostics);
    applyHighlight();

    scrub.max = String(result.total_ms);
    totalReadout.textContent = seconds(result.total_ms);
    playback.setDuration(result.total_ms);
    showSaveState();

    // step 3.4: read-only, so textContent is enough -- nothing here is typed into
    codeView.textContent = result.code_text ?? "nothing to run yet";
    chainView.textContent = result.builder_source ?? "nothing to run yet";
    copyCodeButton.disabled = result.code_text === null;
    copyChainButton.disabled = result.builder_source === null;

    const trouble = result.diagnostics.length;

    runReadout.textContent =
      seconds(result.total_ms) +
      (trouble > 0 ? `, ${trouble} problem${trouble > 1 ? "s" : ""}` : "");

    output.textContent = "";

    // Anything about a particular step now sits on that step. What is left is
    // trouble with the run as a whole, which has no row to sit on.
    const runWide = result.diagnostics.filter((problem) => problem.step === null);

    if (trouble === 0) {
      log("no problems");
    } else if (runWide.length === 0) {
      log("problems are marked on the steps that caused them");
    }

    for (const problem of runWide) {
      log(`${problem.level}: ${problem.message}`);

      if (problem.suggestion !== null) {
        log(`    try: ${problem.suggestion}`);
      }
    }
  }

  nameBox.addEventListener("input", () => {
    showSaveState();
    rebuild();
  });

  keepButton.addEventListener("click", () => {
    const problem = nameProblem(nameBox.value);

    if (problem !== null) {
      log(`cannot save: ${problem}`);
      return;
    }

    if (saveNamed(nameBox.value, currentRun())) {
      log(`saved "${nameBox.value}" in this browser`);
    } else {
      log("could not save: this browser is not letting the page store anything");
    }

    void refreshSavedList();
    savedList.value = nameBox.value;
    forgetButton.disabled = false;

    // Best-effort on top of the local save above, not instead of it -- see
    // store.ts. A team member who typed no name just keeps a local-only run.
    const owner = ownerBox.value.trim();

    if (owner === "") {
      return;
    }

    const name = nameBox.value;

    void saveRemote(owner, name, currentRun()).then((ok) => {
      log(
        ok
          ? `also saved "${name}" for ${owner}, so it opens on another laptop`
          : `could not reach the shared store -- "${name}" is only saved in this browser`,
      );
    });
  });

  savedList.addEventListener("change", () => {
    const name = savedList.value;

    forgetButton.disabled = name === "";

    if (name === "") {
      return;
    }

    const local = listSaved().find((entry) => entry.name === name);

    if (local !== undefined) {
      loadRun(local.run);
      log(`opened "${name}"`);
      return;
    }

    // Not known locally -- it must be one of listRemote()'s "from another
    // laptop" entries, so fetch its actual content before it can be opened.
    const owner = ownerBox.value.trim();

    if (owner === "") {
      return;
    }

    void openRemote(owner, name).then((run) => {
      if (run === null) {
        log(`could not open "${name}" from the shared store`);
        return;
      }

      loadRun(run);
      log(`opened "${name}" from another laptop`);
    });
  });

  forgetButton.addEventListener("click", () => {
    const going = savedList.value;

    if (going === "") {
      return;
    }

    forgetNamed(going);
    void refreshSavedList();
    log(`deleted "${going}" from this browser`);

    const owner = ownerBox.value.trim();

    if (owner !== "") {
      void forgetRemote(owner, going);
    }
  });

  exportButton.addEventListener("click", () => {
    exportRun(currentRun());
    log(`exported ${currentRun().name}.json`);
  });

  importButton.addEventListener("click", () => importFile.click());

  importFile.addEventListener("change", async () => {
    const file = importFile.files?.[0];

    if (file === undefined) {
      return;
    }

    try {
      loadRun(await importRun(file));
      log(`imported ${file.name}`);
    } catch (error) {
      log(String(error instanceof Error ? error.message : error));
    }

    // so the same file can be picked again after an edit
    importFile.value = "";
  });

  saveButton.addEventListener("click", () => {
    const module = latest?.module_text;

    if (!module) {
      return;
    }

    // an error means build_run could not produce a file worth driving
    if (!latest?.ok) {
      log("this run has a problem that has to be fixed before it can be saved");
      return;
    }

    saveModule(nameBox.value, module);
    log(`saved ${nameBox.value}.py — upload it at code.pybricks.com`);
  });

  /**
   * Copy a read-only view to the clipboard, step 3.4.
   *
   * The Clipboard API needs a secure context (https:, or 127.0.0.1 for
   * local work) and can be refused outright -- neither is this page's fault,
   * so the fallback is to select the text instead: still one paste away,
   * just not zero-click.
   */
  async function copyView(view: HTMLElement, label: string) {
    const text = view.textContent ?? "";

    try {
      await navigator.clipboard.writeText(text);
      log(`copied the ${label} to the clipboard`);
    } catch {
      const range = document.createRange();
      range.selectNodeContents(view);

      const selection = window.getSelection();
      selection?.removeAllRanges();
      selection?.addRange(range);

      log(`could not reach the clipboard -- selected the ${label} instead, copy it with Ctrl/Cmd+C`);
    }
  }

  copyCodeButton.addEventListener("click", () => copyView(codeView, "generated run()"));
  copyChainButton.addEventListener("click", () => copyView(chainView, "TrajectoryBuilder chain"));

  playButton.addEventListener("click", () => playback.toggle());

  scrub.addEventListener("input", () => {
    playback.pause();
    playback.seek(Number(scrub.value));
  });

  document.addEventListener("keydown", (event) => {
    // an <input>, or CodeMirror's contenteditable surface for a code action
    // (step 3.3) -- without the second check, a space typed into the code
    // editor falls through and toggles playback instead of being typed
    const typing =
      event.target instanceof HTMLInputElement ||
      (event.target instanceof HTMLElement && event.target.isContentEditable);

    if (event.code === "Space" && !typing) {
      event.preventDefault();
      playback.toggle();
    }
  });

  ownerBox.addEventListener("input", () => {
    rememberOwner(ownerBox.value);
    void refreshSavedList();
  });

  refreshAllButton.addEventListener("click", () => void refreshAllSaved());

  showPose();
  showSaveState();
  void refreshSavedList();
  applyWaypoints();

  // Save what is on screen straight away. Autosaving only on edit meant a run
  // that was opened and left alone was never written down, which is exactly
  // the run somebody loses when they close the tab.
  rememberWorking(currentRun());

  window.addEventListener("resize", () => view.resize());

  const { python, seconds: ready } = await planner.ready;
  log(`python ${python} ready in ${ready}s`);

  show((await planner.build(currentRun())).result);
}

/**
 * Keep working when the host cannot be reached.
 *
 * Registered after the page is up, so a worker that fails to install never
 * stops the planner loading. Needs HTTPS, or localhost -- see 2.1.
 */
function keepOffline() {
  if (!("serviceWorker" in navigator)) {
    return;
  }

  navigator.serviceWorker.register("/sw.js").catch((error) => {
    log("this page will not work offline: " + error);
  });
}

main()
  .then(keepOffline)
  .catch((error) => {
    log("failed: " + error);
    document.title = "FAIL";
    console.error(error);
  });

export {};
