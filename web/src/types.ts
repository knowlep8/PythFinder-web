/**
 * The shapes `build_run` speaks, from docs/web-planner.md.
 *
 * This is one half of a contract with pythfinder/headless.py; the other half
 * is that file's docstring. Keep them honest with each other.
 */

export interface RunAction {
  id: string;
  /** when in the step: distance into it, or a time. Negative counts back. */
  at: { cm?: number; ms?: number };
  /** carried for the generated file in phase 3; build_run ignores it */
  do?: unknown;
  label?: string;
}

export type StepType = "drive" | "wait" | "turn" | "toPoint" | "toPose";

export interface RunStep {
  type: StepType;
  cm?: number;
  ms?: number;
  deg?: number;
  x?: number;
  y?: number;
  head?: number;
  reversed?: boolean;
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
}
