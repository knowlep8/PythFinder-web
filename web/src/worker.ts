/**
 * Python, kept off the page's thread.
 *
 * Building a run takes about 0.6s in the browser (step 1.8 measured it), which
 * is far too long to spend on the thread that is drawing the field and
 * answering clicks. So Pyodide lives here, and the page talks to it in
 * messages.
 *
 * The protocol is small: the worker says what it is doing while it starts,
 * says `ready` once, and then answers each `build` with a `built` or a
 * `failed` carrying the same id.
 */

import type { BuildResult, Run } from "./types";

export type ToWorker = { type: "build"; id: number; run: Run };

export type FromWorker =
  | { type: "status"; text: string }
  | { type: "ready"; python: string; seconds: number }
  | { type: "built"; id: number; result: BuildResult; seconds: number }
  | { type: "failed"; id: number; message: string }
  | { type: "broken"; message: string };

// Typed just enough to use, rather than pulling the webworker lib in beside
// the DOM one and having them argue about who owns `self`.
const ctx = self as unknown as {
  postMessage(message: FromWorker): void;
  onmessage: ((event: MessageEvent) => void) | null;
};

function say(message: FromWorker) {
  ctx.postMessage(message);
}

function seconds(from: number) {
  return Math.round(performance.now() - from) / 1000;
}

/** The Python side of build_run, held as a callable so each build is one call. */
let build: ((runJson: string) => string) | null = null;

async function start() {
  const began = performance.now();

  say({ type: "status", text: "starting Python" });

  // Served by our own container. The path is a variable so the bundler leaves
  // it alone -- these files are copied in, not imported from node_modules.
  const runtime = "/pyodide/pyodide.mjs";
  const { loadPyodide } = await import(/* @vite-ignore */ runtime);
  const pyodide = await loadPyodide({ indexURL: "/pyodide/" });

  say({ type: "status", text: "unpacking the planner" });

  const response = await fetch("/pythfinder.whl");

  if (!response.ok) {
    throw new Error(`/pythfinder.whl returned ${response.status}`);
  }

  // A wheel is a zip, and this one has no dependencies, so there is nothing
  // for a package installer to do.
  await pyodide.unpackArchive(await response.arrayBuffer(), "zip");

  say({ type: "status", text: "warming it up" });

  // Importing costs a moment; do it once, and keep the function itself.
  build = pyodide.runPython(`
import json

from pythfinder.headless import build_run


def _build(run_json):
    return json.dumps(build_run(json.loads(run_json)))


_build
`) as (runJson: string) => string;

  const python = pyodide.runPython("import sys; sys.version.split()[0]") as string;

  say({ type: "ready", python, seconds: seconds(began) });
}

const starting = start().catch((error) => {
  // remembered rather than thrown: every later build should say the same
  // thing, instead of failing in some new and mysterious way
  const message = String(error);
  say({ type: "broken", message });

  return message;
});

ctx.onmessage = async (event: MessageEvent) => {
  const message = event.data as ToWorker;

  if (message.type !== "build") {
    return;
  }

  const failure = await starting;

  if (build === null) {
    say({
      type: "failed",
      id: message.id,
      message: failure ?? "the planner never started",
    });
    return;
  }

  const began = performance.now();

  try {
    const answer = build(JSON.stringify(message.run));

    say({
      type: "built",
      id: message.id,
      result: JSON.parse(answer) as BuildResult,
      seconds: seconds(began),
    });
  } catch (error) {
    // build_run reports a bad run in its diagnostics rather than raising, so
    // anything landing here is a fault in the planner itself
    say({ type: "failed", id: message.id, message: String(error) });
  }
};
