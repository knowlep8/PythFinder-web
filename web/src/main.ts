/**
 * Step 2.5: watching the run.
 *
 * The robot on the mat says where the run starts, the list says what it does,
 * and the slider walks through what that looks like. Playback is in real time,
 * so a run that takes 15 seconds on the mat takes 15 seconds here -- which is
 * the point, since the team is trying to fit inside two and a half minutes.
 */

import { FieldView } from "./fieldView";
import { Playback } from "./playback";
import { RunEditor } from "./runEditor";
import { createPlanner } from "./planner";
import { hubCost, nameProblem, saveModule } from "./download";
import {
  exportRun,
  forgetNamed,
  importRun,
  listSaved,
  recallWorking,
  rememberWorking,
  saveNamed,
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

const keepButton = document.getElementById("keep") as HTMLButtonElement;
const savedList = document.getElementById("saved") as HTMLSelectElement;
const forgetButton = document.getElementById("forget") as HTMLButtonElement;
const exportButton = document.getElementById("export") as HTMLButtonElement;
const importButton = document.getElementById("import") as HTMLButtonElement;
const importFile = document.getElementById("importfile") as HTMLInputElement;

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
  });

  const editor = new RunEditor(stepList, startingSteps, {
    onChange: () => rebuild(),
    onSelect: () => applyHighlight(),
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

  function refreshSavedList() {
    const saved = listSaved();
    const chosen = savedList.value;

    savedList.replaceChildren();

    const heading = document.createElement("option");
    heading.value = "";
    heading.textContent = saved.length === 0 ? "— nothing saved —" : "— saved runs —";
    savedList.append(heading);

    for (const entry of saved) {
      const option = document.createElement("option");
      option.value = entry.name;
      option.textContent = entry.name;
      savedList.append(option);
    }

    savedList.value = saved.some((entry) => entry.name === chosen) ? chosen : "";
    forgetButton.disabled = savedList.value === "";
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

    refreshSavedList();
    savedList.value = nameBox.value;
    forgetButton.disabled = false;
  });

  savedList.addEventListener("change", () => {
    const chosen = listSaved().find((entry) => entry.name === savedList.value);

    forgetButton.disabled = savedList.value === "";

    if (chosen !== undefined) {
      loadRun(chosen.run);
      log(`opened "${chosen.name}"`);
    }
  });

  forgetButton.addEventListener("click", () => {
    const going = savedList.value;

    if (going === "") {
      return;
    }

    forgetNamed(going);
    refreshSavedList();
    log(`deleted "${going}" from this browser`);
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

  playButton.addEventListener("click", () => playback.toggle());

  scrub.addEventListener("input", () => {
    playback.pause();
    playback.seek(Number(scrub.value));
  });

  document.addEventListener("keydown", (event) => {
    const typing = event.target instanceof HTMLInputElement;

    if (event.code === "Space" && !typing) {
      event.preventDefault();
      playback.toggle();
    }
  });

  showPose();
  showSaveState();
  refreshSavedList();

  // Save what is on screen straight away. Autosaving only on edit meant a run
  // that was opened and left alone was never written down, which is exactly
  // the run somebody loses when they close the tab.
  rememberWorking(currentRun());

  window.addEventListener("resize", () => view.resize());

  const { python, seconds: ready } = await planner.ready;
  log(`python ${python} ready in ${ready}s`);

  show((await planner.build(currentRun())).result);
}

main().catch((error) => {
  log("failed: " + error);
  document.title = "FAIL";
  console.error(error);
});

export {};
