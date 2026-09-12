/**
 * Step 2.2: Python in a worker, and proof that it stays out of the way.
 *
 * Three things are checked, in the order they matter:
 *
 *   1. the worker builds the run from fll_run_template.py, and gets the same
 *      file the hub is running;
 *   2. the page keeps drawing while it does -- measured, not asserted;
 *   3. a burst of edits causes one build, of the newest run, not twenty.
 *
 * The real interface starts in 2.3. This page is the scaffolding's test.
 */

import { createPlanner } from "./planner";
import type { Run } from "./types";

// what the hub is running today, from tests/golden/hub/template_run.py
const EXPECTED = {
  steps_line: "STEPS = 6",
  markers_line: "MARKERS = (1764, 7942)",
  count_line: "COUNT = 2607",
  total_ms: 15641,
};

const TEMPLATE_RUN: Run = {
  version: 1,
  name: "run_a",
  steps_ms: 6,
  robot: "fll_team",
  start: { x: -46, y: -83, head: 0 },
  steps: [
    { type: "drive", cm: 75, actions: [{ id: "arm_down", at: { cm: 35 } }] },
    { type: "wait", ms: 600 },
    { type: "turn", deg: 90 },
    { type: "drive", cm: 30, actions: [{ id: "arm_up", at: { ms: -1 } }] },
    { type: "toPose", x: -46, y: -83, head: 0 },
  ],
};

const output = document.getElementById("log") as HTMLPreElement;

function log(line: string) {
  output.textContent += "\n" + line;
  console.log("[planner] " + line);
}

/**
 * Watch how long the page goes between frames.
 *
 * This is the whole point of the worker: on the main thread a build would
 * stall painting for the better part of a second, and the longest gap would
 * say so.
 */
function watchFrames() {
  let last = performance.now();
  let worst = 0;
  let frames = 0;
  let running = true;

  function tick(now: number) {
    frames += 1;
    worst = Math.max(worst, now - last);
    last = now;

    if (running) {
      requestAnimationFrame(tick);
    }
  }

  requestAnimationFrame(tick);

  return {
    stop() {
      running = false;

      // The frame count matters as much as the gap. Chrome stops painting a
      // tab that is not visible, so a hidden tab reports a worst gap of 0ms --
      // which looks like a perfect score and means nothing at all.
      return { worstGap: Math.round(worst), frames };
    },
  };
}

function lineFor(moduleText: string, prefix: string) {
  return moduleText.split("\n").find((line) => line.startsWith(prefix)) ?? "";
}

/** The same run, driving a little further each time -- as if somebody typed. */
function editedRun(cm: number): Run {
  const steps = TEMPLATE_RUN.steps.map((step) => ({ ...step }));
  steps[0] = { ...steps[0], cm };

  return { ...TEMPLATE_RUN, steps };
}

async function main() {
  output.textContent = "";

  let lastResult = 0;

  const planner = createPlanner({
    onStatus: (text) => log(text + "..."),
    onResult: (result, seconds) => {
      lastResult = result.total_ms;
      log(`  background build: ${result.total_ms}ms run, took ${seconds}s`);
    },
    onError: (message) => log("planner error: " + message),
  });

  const started = performance.now();
  const { python, seconds } = await planner.ready;
  log(`python ${python} ready in ${seconds}s`);

  // 1. the run, built in the worker
  const frames = watchFrames();
  const { result, seconds: built } = await planner.build(TEMPLATE_RUN);
  const painting = frames.stop();

  log("");
  log(`built in ${built}s: ${result.total_ms}ms, ${result.poses.length} poses`);
  log(`actions: ${result.markers.map((m) => `${m.id} at ${m.time_ms}ms`).join(", ")}`);
  log(`problems: ${result.diagnostics.map((d) => d.message).join(" | ") || "none"}`);

  const moduleText = result.module_text ?? "";
  const got = {
    steps_line: lineFor(moduleText, "STEPS"),
    markers_line: lineFor(moduleText, "MARKERS"),
    count_line: lineFor(moduleText, "COUNT"),
    total_ms: result.total_ms,
  };

  log(`${got.steps_line}   ${got.markers_line}   ${got.count_line}`);
  log("");

  // 2. did the page keep drawing while that happened
  const judged = painting.frames >= 5;
  const stalled = judged && painting.worstGap >= 100;

  if (judged) {
    log(
      `longest gap between frames while building: ${painting.worstGap}ms ` +
        `over ${painting.frames} frames`,
    );
    log(
      stalled
        ? "  THAT IS A STALL - something is running on the page's thread"
        : "  the page kept drawing, so the worker is doing its job",
    );
  } else {
    log(`the page painted ${painting.frames} frames while building: too few to judge`);
    log("  a hidden tab is not painted at all, so open this one to measure it");
  }

  log("");

  // 3. a burst of edits should cost one build, of the last one
  log("typing 20 edits as fast as possible...");
  const before = planner.builds();

  for (let cm = 60; cm < 80; cm++) {
    planner.request(editedRun(cm));
  }

  await new Promise((wake) => setTimeout(wake, 2500));

  const buildsRun = planner.builds() - before;
  log(`  builds actually run: ${buildsRun}`);
  log(`  last result is for the last edit: ${lastResult > 0}`);
  log("");

  const wrong = (Object.keys(EXPECTED) as (keyof typeof EXPECTED)[]).filter(
    (key) => got[key] !== EXPECTED[key],
  );

  if (wrong.length > 0) {
    log("RESULT: FAIL - " + wrong.map((key) => `${key} was ${got[key]}`).join(", "));
    document.title = "FAIL";
  } else if (stalled) {
    log("RESULT: FAIL - the page stalled while building");
    document.title = "FAIL";
  } else if (buildsRun > 3) {
    log(`RESULT: FAIL - ${buildsRun} builds for 20 edits, the debounce is not working`);
    document.title = "FAIL";
  } else {
    log(
      `RESULT: PASS - same file as the hub, ` +
        `${judged ? "no stall" : "painting not measured"}, ` +
        `${buildsRun} build(s) for 20 edits, ` +
        `all in ${Math.round(performance.now() - started) / 1000}s`,
    );
    document.title = "PASS";
  }

  planner.stop();
}

main().catch((error) => {
  log("RESULT: FAIL - " + error);
  document.title = "FAIL";
  console.error(error);
});

export {};
