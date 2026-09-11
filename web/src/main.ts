/**
 * Step 2.1: the scaffold, and proof that the container can do the job.
 *
 * Loads Pyodide from this same origin, unpacks the PythFinder wheel into its
 * filesystem, builds the run from fll_run_template.py, and checks the result
 * against the numbers in the file the hub is running today.
 *
 * The real interface arrives in 2.2 onwards; this page exists so the container
 * can be proved before anything is built on top of it.
 */

// what the hub is running today, from tests/golden/hub/template_run.py
const EXPECTED = {
  steps_line: "STEPS = 6",
  markers_line: "MARKERS = (1764, 7942)",
  count_line: "COUNT = 2607",
  total_ms: 15641,
};

const TEMPLATE_RUN = {
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

// not called `screen`: that is already a global in the DOM
const output = document.getElementById("log") as HTMLPreElement;

function log(line: string) {
  output.textContent += "\n" + line;
  console.log("[planner] " + line);
}

function seconds(from: number) {
  return ((performance.now() - from) / 1000).toFixed(1) + "s";
}

async function main() {
  output.textContent = "";

  // Loaded from our own origin, not a CDN. The path is held in a variable so
  // the bundler leaves it alone: these files are copied in by
  // scripts/copy-pyodide.mjs, not imported through node_modules.
  const runtime = "/pyodide/pyodide.mjs";

  let mark = performance.now();
  const { loadPyodide } = await import(/* @vite-ignore */ runtime);
  const pyodide = await loadPyodide({ indexURL: "/pyodide/" });
  log(`pyodide started from ${location.origin}/pyodide/ in ${seconds(mark)}`);

  // A wheel is a zip, and this one has no dependencies, so there is nothing
  // for a package installer to resolve: unpack it straight into the
  // filesystem, which saves serving micropip as well.
  mark = performance.now();
  const wheel = await fetch("/pythfinder.whl");

  if (!wheel.ok) {
    throw new Error(
      `the library is not being served: /pythfinder.whl returned ${wheel.status}`,
    );
  }

  const bytes = await wheel.arrayBuffer();
  await pyodide.unpackArchive(bytes, "zip");
  log(`library unpacked (${(bytes.byteLength / 1e6).toFixed(2)}MB) in ${seconds(mark)}`);

  mark = performance.now();
  pyodide.globals.set("run_json", JSON.stringify(TEMPLATE_RUN));

  const answer = await pyodide.runPythonAsync(`
import json, sys, time

from pythfinder.headless import build_run

started = time.time()
result = build_run(json.loads(run_json))
built_in = time.time() - started

lines = result["module_text"].splitlines()
def line_for(prefix):
    return next(line for line in lines if line.startswith(prefix))

json.dumps({
    "python": sys.version.split()[0],
    "ok": result["ok"],
    "total_ms": result["total_ms"],
    "poses": len(result["poses"]),
    "markers": [m["id"] + " at " + str(m["time_ms"]) + "ms" for m in result["markers"]],
    "diagnostics": [d["message"] for d in result["diagnostics"]],
    "steps_line": line_for("STEPS"),
    "markers_line": line_for("MARKERS"),
    "count_line": line_for("COUNT"),
    "build_seconds": round(built_in, 3),
})
`);

  const data = JSON.parse(answer);
  log(`run built in ${seconds(mark)}`);
  log("");
  log(`python ${data.python}`);
  log(`ok: ${data.ok}   total: ${data.total_ms}ms   poses: ${data.poses}`);
  log(`actions: ${data.markers.join(", ")}`);
  log(`problems: ${data.diagnostics.join(" | ") || "none"}`);
  log(`${data.steps_line}   ${data.markers_line}   ${data.count_line}`);
  log("");

  const wrong = (Object.keys(EXPECTED) as (keyof typeof EXPECTED)[]).filter(
    (key) => data[key] !== EXPECTED[key],
  );

  if (wrong.length === 0) {
    log("RESULT: PASS - this container produced the file the hub is running");
    document.title = "PASS";
  } else {
    log("RESULT: FAIL - " + wrong.map((key) => `${key} was ${data[key]}`).join(", "));
    document.title = "FAIL";
  }
}

main().catch((error) => {
  log("RESULT: FAIL - " + error);
  document.title = "FAIL";
  console.error(error);
});

// this file is loaded as a module, and only a dynamic import appears above, so
// say so explicitly for the type checker
export {};
