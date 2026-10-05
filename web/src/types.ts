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
   * When in the step it happens: a distance into it, negative counting back
   * from the end.
   *
   * A sequential arm step leaves this out — it *is* the step, so it starts as
   * the robot comes to rest, and build_run works out when that is.
   *
   * cm-only since step 5.2 -- a DriveBase split has no "so many ms in" to
   * give it, and this editor never wrote one even before that step. A run
   * saved before 5.2 can still hold `{"ms": ...}` on disk, because nothing
   * about JSON stops it -- this type only says what the page itself ever
   * writes or reads back out. build_run flags that case with a diagnostic on
   * the step rather than silently reinterpreting it -- see
   * docs/web-planner.md, step 5.2.
   */
  at?: { cm?: number };
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
  /**
   * drive / toPoint / toPose only -- turning uses angular speed, not this.
   * Step 6.1: the speed for this step's whole straight part, in cm/s; absent
   * means the robot's own. Replaced 4.5's `speedLimits` from/to list, which
   * upgradeRun (store.ts) converts when an older run is opened.
   */
  speedLimit_cm_s?: number;
}

export interface RobotNumbers {
  track_width_cm: number;
  max_velocity_cm_s: number;
  center_offset_cm?: number;
  max_power?: number;
  width_cm?: number;
  length_cm?: number;
}

/**
 * Pybricks' own numbers for a `DriveBase` -- step 5.3. Units as Pybricks
 * takes them (mm, not cm), so they can go into the generated file unconverted
 * once 5.5/5.6 exist. `build_run` does not read these at all today: a run is
 * still planned from `planning` below, the way it always has been.
 */
export interface DriveBaseNumbers {
  wheel_diameter_mm: number;
  axle_track_mm: number;
  straight_speed: number;
  straight_acceleration: number;
  turn_rate: number;
  turn_acceleration: number;
  use_gyro: boolean;
}

/**
 * A named robot, step 5.3. Stored team-wide (`robots/<name>` in Firestore),
 * picked from the "Robot" picker in the run header, and copied whole into
 * whatever run picks it -- see `Run.robot`.
 */
export interface RobotProfile {
  name: string;
  planning: RobotNumbers;
  driveBase: DriveBaseNumbers;
}

/**
 * A named place to start from, step 5.3. Stored team-wide
 * (`starts/<name>` in Firestore), picked from the "Start" picker, which
 * moves the robot there on the field.
 */
export interface StartPosition {
  name: string;
  x: number;
  y: number;
  head: number;
}

export interface Run {
  version: number;
  name: string;
  /** ms per exported state; 6 is what the hub uses */
  steps_ms: number;
  /**
   * "fll_team" and a bare `RobotNumbers` are the pre-5.3 shapes -- still read
   * by `build_run`, and `"fll_team"` still means the built-in team profile.
   * A run saved by the page now always writes the full `RobotProfile` it was
   * planned with, name and numbers copied in whole rather than referenced, so
   * the run means the same thing on a laptop that has never seen that
   * profile. See step 5.3.
   */
  robot: "fll_team" | RobotNumbers | RobotProfile;
  /**
   * `name` (step 5.3) is which named start this came from -- kept even once
   * dragging the robot has moved it away from that start's own numbers, so
   * the run still says where it started from.
   */
  start: { x: number; y: number; head: number; name?: string };
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
  /**
   * The .py file to download, or null when there is nothing to drive.
   *
   * Step 5.7: this is the DriveBase file (driveModule.py, steps 5.4/5.5) --
   * straight(), turn_to(), wait() and the run's own actions, a few hundred
   * bytes of readable Python -- not the older recording of wheel powers
   * every few milliseconds. It elides nothing, so the Python view (3.4)
   * reads this same field rather than a second, elided copy of it.
   */
  module_text: string | null;
  /** step 3.4: the equivalent TrajectoryBuilder chain, for the desktop tool */
  builder_source: string | null;
}
