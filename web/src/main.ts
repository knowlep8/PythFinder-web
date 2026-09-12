/**
 * Step 2.4: the run as a list of steps you can change.
 *
 * The robot on the mat says where the run starts; the list below says what it
 * does. Both feed the same description to the worker, and what comes back is
 * the path, how long each step takes, and anything wrong with it.
 *
 * Playback and the scrubber are 2.5; the problems panel proper is 2.6.
 */

import { FieldView } from "./fieldView";
import { RunEditor } from "./runEditor";
import { createPlanner } from "./planner";
import type { FieldPose } from "./field";
import type { BuildResult, Run, RunStep } from "./types";

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

    const trouble = result.diagnostics.length;

    runReadout.textContent =
      `${(result.total_ms / 1000).toFixed(1)}s` +
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

  showPose();
  window.addEventListener("resize", () => view.resize());

  const { python, seconds } = await planner.ready;
  log(`python ${python} ready in ${seconds}s`);

  show((await planner.build(currentRun())).result);
}

main().catch((error) => {
  log("failed: " + error);
  document.title = "FAIL";
  console.error(error);
});

export {};
