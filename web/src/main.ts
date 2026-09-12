/**
 * Step 2.3: the mat, and the robot standing on it.
 *
 * The robot can be dragged and turned, and every change asks the worker for a
 * fresh run -- which is what 2.2 built the debounce for. Drawing the path
 * itself is 2.5; for now the run shows up as its length and its problems.
 *
 * The page also checks its own arithmetic on start-up: the middle of the mat
 * has to land in the middle of the canvas, a round trip through the screen has
 * to come back where it started, and the launch-area pose has to sit where the
 * simulator puts it.
 */

import { FieldView } from "./fieldView";
import {
  FIELD,
  fieldToScreen,
  fitViewport,
  screenToField,
  type FieldPose,
} from "./field";
import { createPlanner } from "./planner";
import type { Run } from "./types";

/** The left launch area, as in fll_run_template.py. */
const START: FieldPose = { x: -46, y: -83, head: 0 };

const output = document.getElementById("log") as HTMLPreElement;
const poseReadout = document.getElementById("pose") as HTMLElement;
const mouseReadout = document.getElementById("mouse") as HTMLElement;
const runReadout = document.getElementById("run") as HTMLElement;
const canvas = document.getElementById("field") as HTMLCanvasElement;

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

/** The template run, starting wherever the robot currently stands. */
function runFrom(pose: FieldPose): Run {
  return {
    version: 1,
    name: "run_a",
    steps_ms: 6,
    robot: "fll_team",
    start: { x: pose.x, y: pose.y, head: pose.head },
    steps: [
      { type: "drive", cm: 75, actions: [{ id: "arm_down", at: { cm: 35 } }] },
      { type: "wait", ms: 600 },
      { type: "turn", deg: 90 },
      { type: "drive", cm: 30, actions: [{ id: "arm_up", at: { ms: -1 } }] },
      { type: "toPose", x: pose.x, y: pose.y, head: pose.head },
    ],
  };
}

function showPose(pose: FieldPose) {
  poseReadout.textContent = `x ${pose.x}  y ${pose.y}  head ${pose.head}°`;
}

/** Does the arithmetic agree with itself, and with the simulator? */
function checkTheMaths() {
  const view = fitViewport(1000, 570);
  const problems: string[] = [];

  const middle = fieldToScreen({ x: 0, y: 0 }, view);
  if (Math.round(middle.x) !== 500 || Math.round(middle.y) !== 285) {
    problems.push(`the middle of the mat landed at ${middle.x}, ${middle.y}`);
  }

  const back = screenToField(fieldToScreen(START, view), view);
  if (Math.abs(back.x - START.x) > 0.001 || Math.abs(back.y - START.y) > 0.001) {
    problems.push(`a round trip moved the pose to ${back.x}, ${back.y}`);
  }

  // the launch area is left of the middle and below it, as drawn
  const launch = fieldToScreen(START, view);
  if (!(launch.x < middle.x) || !(launch.y > middle.y)) {
    problems.push("the launch area is not down and to the left");
  }

  // +x is up the screen, +y is to the right
  if (!(fieldToScreen({ x: 10, y: 0 }, view).y < middle.y)) {
    problems.push("+x is not up the screen");
  }
  if (!(fieldToScreen({ x: 0, y: 10 }, view).x > middle.x)) {
    problems.push("+y is not to the right");
  }

  return problems;
}

async function main() {
  output.textContent = "";

  const problems = checkTheMaths();

  if (problems.length > 0) {
    problems.forEach((problem) => log("coordinates are wrong: " + problem));
    document.title = "FAIL";
  } else {
    log(
      `coordinates check out: ${FIELD.widthCm} x ${FIELD.heightCm}cm mat, ` +
        `+x up, +y right, origin in the middle`,
    );
  }

  const [mat, robot] = await Promise.all([
    loadImage("/field/mat.png"),
    loadImage("/field/robot.png"),
  ]);
  log(`mat ${mat.naturalWidth}x${mat.naturalHeight}, robot ${robot.naturalWidth}x${robot.naturalHeight}`);

  const planner = createPlanner({
    onStatus: (text) => log(text + "..."),
    onResult: (result) => {
      const trouble = result.diagnostics.length;

      runReadout.textContent =
        `${(result.total_ms / 1000).toFixed(1)}s` +
        (trouble > 0 ? `, ${trouble} problem${trouble > 1 ? "s" : ""}` : "");
    },
    onError: (message) => log("planner error: " + message),
  });

  const view = new FieldView(
    canvas,
    { mat, robot },
    START,
    {
      onPoseChange: (pose) => {
        showPose(pose);
        planner.request(runFrom(pose));
      },
      onHover: (point) => {
        mouseReadout.textContent =
          point === null ? "—" : `x ${point.x.toFixed(1)}  y ${point.y.toFixed(1)}`;
      },
    },
  );

  showPose(START);
  window.addEventListener("resize", () => view.resize());

  const { python, seconds } = await planner.ready;
  log(`python ${python} ready in ${seconds}s`);

  const { result } = await planner.build(runFrom(START));
  runReadout.textContent = `${(result.total_ms / 1000).toFixed(1)}s`;

  log(`the run from here takes ${result.total_ms}ms`);
  log(`problems: ${result.diagnostics.map((d) => d.message).join(" | ") || "none"}`);
  log("");
  log(
    problems.length === 0
      ? "RESULT: PASS - drag the robot, or turn it with the arrow keys"
      : "RESULT: FAIL - see above",
  );

  if (problems.length === 0) {
    document.title = "PASS";
  }
}

main().catch((error) => {
  log("RESULT: FAIL - " + error);
  document.title = "FAIL";
  console.error(error);
});

export {};
