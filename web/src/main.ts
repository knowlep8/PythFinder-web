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

  const [mat, robot] = await Promise.all([
    loadImage("/field/mat.png"),
    loadImage("/field/robot.png"),
  ]);

  const planner = createPlanner({
    onStatus: (text) => log(text + "..."),
    onResult: (result) => show(result),
    onError: (message) => log("planner error: " + message),
  });

  const view = new FieldView(canvas, { mat, robot }, START, {
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

  const editor = new RunEditor(stepList, FIRST_STEPS, {
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
      name: "run_a",
      steps_ms: 6,
      robot: "fll_team",
      start: { x: pose.x, y: pose.y, head: pose.head },
      steps: editor.getSteps(),
    };
  }

  function rebuild() {
    planner.request(currentRun());
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
    applyHighlight();

    scrub.max = String(result.total_ms);
    totalReadout.textContent = seconds(result.total_ms);
    playback.setDuration(result.total_ms);

    const trouble = result.diagnostics.length;

    runReadout.textContent =
      seconds(result.total_ms) +
      (trouble > 0 ? `, ${trouble} problem${trouble > 1 ? "s" : ""}` : "");

    output.textContent = "";

    if (trouble === 0) {
      log("no problems");
      return;
    }

    for (const problem of result.diagnostics) {
      const where = problem.step === null ? "" : ` (step ${problem.step + 1})`;
      log(`${problem.level}${where}: ${problem.message}`);

      if (problem.suggestion !== null) {
        log(`    try: ${problem.suggestion}`);
      }
    }
  }

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
