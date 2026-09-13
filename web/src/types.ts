/**
 * The shapes `build_run` speaks, from docs/web-planner.md.
 *
 * This is one half of a contract with pythfinder/headless.py; the other half
 * is that file's docstring. Keep them honest with each other.
 */

/** What a motor is told to do. Becomes a line of Python in the hub file. */
export interface MotorCommand {
  motor: "leftTask" | "rightTask";
  /** `run` and `stop` return at once; the rest finish and can be waited for */
  call: "run" | "stop" | "run_angle" | "run_target" | "run_until_stalled";
  speed?: number;
  angle?: number;
}

/**
 * Free-form Python for a parallel action, with `core` in scope.
 *
 * For whatever the motor picker cannot say -- reading a sensor, counting
 * something, moving two motors from one action. `build_run` checks it parses
 * with `compile()` and warns on obviously blocking calls; neither can prove
 * it runs, because that needs a hub. See docs/web-planner.md, step 3.3.
 */
export interface CodeCommand {
  code: string;
}

export type ActionBody = MotorCommand | CodeCommand;

export function isCode(body: ActionBody): body is CodeCommand {
  return "code" in body;
}

export interface RunAction {
  id: string;
  /**
   * When in the step it happens: a distance into it, or a time, with negative
   * counting back from the end.
   *
   * A sequential arm step leaves this out — it *is* the step, so it starts as
   * the robot comes to rest, and build_run works out when that is.
   */
  at?: { cm?: number; ms?: number };
  do?: ActionBody;
  label?: string;
}

export type StepType =
  | "drive"
  | "wait"
  | "turn"
  | "toPoint"
  | "toPose"
  /**
   * The robot stops, the arm runs to completion, the next step waits.
   *
   * Its motor command lives on the step itself rather than in `actions`,
   * because the step and the movement are the same thing.
   */
  | "armStep";

export interface RunStep {
  type: StepType;
  cm?: number;
  ms?: number;
  deg?: number;
  x?: number;
  y?: number;
  head?: number;
  reversed?: boolean;

  /** armStep only: which motor, and how it is told to move */
  motor?: MotorCommand["motor"];
  call?: MotorCommand["call"];
  speed?: number;
  angle?: number;

  actions?: RunAction[];
}

export interface RobotNumbers {
  track_width_cm: number;
  max_velocity_cm_s: number;
  center_offset_cm?: number;
  max_power?: number;
  width_cm?: number;
  length_cm?: number;
}

export interface Run {
  version: number;
  name: string;
  /** ms per exported state; 6 is what the hub uses */
  steps_ms: number;
  robot: "fll_team" | RobotNumbers;
  start: { x: number; y: number; head: number };
  steps: RunStep[];
}

/** One point along the path, thinned for drawing. */
export interface PathPose {
  t: number;
  x: number;
  y: number;
  head: number;
}

/** An action, with the moment it fires. In the order the hub binds them. */
export interface BuiltMarker {
  id: string;
  step: number | null;
  time_ms: number;
}

/** When one described step runs. A merged step starts and ends together. */
export interface BuiltStep {
  index: number;
  type: StepType | null;
  starts_ms: number;
  ends_ms: number;
}

export interface Diagnostic {
  level: "warning" | "error";
  message: string;
  step: number | null;
  suggestion: string | null;
  time_ms: number | null;
}

export interface BuildResult {
  version: number;
  name: string;
  ok: boolean;
  total_ms: number;
  steps: BuiltStep[];
  poses: PathPose[];
  markers: BuiltMarker[];
  diagnostics: Diagnostic[];
  /** the .py file to download, or null when there is nothing to drive */
  module_text: string | null;
  /** step 3.4: the same file, its DATA payload elided -- for reading, not saving */
  code_text: string | null;
  /** step 3.4: the equivalent TrajectoryBuilder chain, for the desktop tool */
  builder_source: string | null;
}
