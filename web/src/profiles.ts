/**
 * Robot profiles and start positions -- step 5.3.
 *
 * The team shares one set of these, unlike the runs in store.ts: everybody
 * plans against the same robot and picks from the same launch areas, so
 * these live team-wide in Firestore (`robots/<name>`, `starts/<name>`),
 * not per-owner. Same open rules as runs, and the same defensive shape:
 * every remote call here is best-effort and fails closed to whatever is
 * already cached, so a flaky host is a reason a new profile does not reach
 * another laptop yet, never a reason the picker has nothing to offer.
 *
 * The built-in team robot and left launch start are not stored anywhere --
 * they are plain constants below, always in the list, so an empty or
 * unreachable store never leaves the pickers with nothing to pick. Nothing
 * stops either from being *edited*, though: saving a profile named "Team
 * robot" writes it to Firestore like any other, and from then on the cached
 * copy -- with, say, a properly measured wheel diameter -- is what the
 * pickers show instead of the placeholder below. What stays true is that the
 * name can never disappear from the list: there is always a "Team robot" and
 * a "Left launch" to pick.
 */

import { collection, doc, getDocs, setDoc } from "firebase/firestore";
import { db } from "./firebase";
import type { DriveBaseNumbers, RobotProfile, Run, StartPosition } from "./types";

/** A plain deep copy. Every shape in this file is JSON-shaped data with no
 *  methods or dates in it, so this is enough -- and it is what stops a
 *  picker's own copy of a profile (see profilePicker.ts) from sharing nested
 *  objects with a built-in constant or a cached one, which editing that copy
 *  would otherwise silently corrupt. */
export function clone<T>(value: T): T {
  return JSON.parse(JSON.stringify(value)) as T;
}

const ROBOTS_CACHE = "pythfinder.robots";
const STARTS_CACHE = "pythfinder.starts";

/**
 * The team's robot. Planning numbers are the ones measured in
 * `robotConfig.py`'s `FLL_ROBOT` -- kept honest with that file by hand, the
 * same relationship `types.ts`'s own docstring already asks for.
 *
 * `wheel_diameter_mm` and `axle_track_mm` are measured, on the base the kids
 * rebuilt on 2026-09-27, with the quick-start's `drivebase_test.py`: B drove
 * 1018mm for 1000 at 54.6, so 55.6; C then turned 379.6 for 360 and, at
 * 160.1, 360.2. They match `config.py` there on purpose -- a run whose
 * numbers differ makes the hub build a second DriveBase (step 5.6). Axle
 * track is its own field, not the planning track width, because it is tuned
 * to make turns land rather than measured with a ruler. The four speed/
 * acceleration numbers are still ordinary starting values, not measurements.
 */
export const TEAM_ROBOT: RobotProfile = {
  name: "Team robot",
  planning: {
    track_width_cm: 16,
    max_velocity_cm_s: 64.3,
    center_offset_cm: -3.5,
    width_cm: 19,
    length_cm: 14,
  },
  driveBase: {
    wheel_diameter_mm: 55.6, // measured, see above
    axle_track_mm: 160.1, // measured, see above
    straight_speed: 200,
    straight_acceleration: 400,
    turn_rate: 150,
    turn_acceleration: 300,
    use_gyro: true,
  },
};

/** The left launch area, as `fll_run_template.py`'s own `START_POSE`. */
export const LEFT_LAUNCH_START: StartPosition = { name: "Left launch", x: -46, y: -83, head: 0 };

const BUILTIN_ROBOTS: RobotProfile[] = [TEAM_ROBOT];
const BUILTIN_STARTS: StartPosition[] = [LEFT_LAUNCH_START];

/** Freshly-built DriveBase numbers for a profile that never had any -- an
 *  old run's bare `RobotNumbers`, which predates step 5.3. */
export function defaultDriveBase(): DriveBaseNumbers {
  return { ...TEAM_ROBOT.driveBase };
}

/**
 * Turn whatever a run's `robot` field holds into a full profile, so the
 * picker and the "changed since" check both always have one to work with.
 *
 * `"fll_team"` and a missing robot both mean the built-in team profile --
 * the same thing `robot_from_description` on the Python side already does,
 * kept honest with it by hand. A bare `RobotNumbers` object with no
 * `driveBase` at all is the pre-5.3 shape (the mentor-only settings panel
 * step 4.2 sketched, then dropped in favour of this step): wrapped here with
 * a placeholder name, since nothing that old ever had one, and the built-in
 * DriveBase numbers, since nothing that old ever had those either.
 */
export function profileFromRunRobot(robot: Run["robot"] | undefined): RobotProfile {
  if (robot === undefined || robot === "fll_team") {
    return clone(TEAM_ROBOT);
  }

  if ("driveBase" in robot) {
    return clone(robot);
  }

  return { name: "(imported robot)", planning: clone(robot), driveBase: defaultDriveBase() };
}

function read<T>(key: string): T[] {
  try {
    const held = localStorage.getItem(key);

    return held === null ? [] : (JSON.parse(held) as T[]);
  } catch {
    return [];
  }
}

function write<T>(key: string, list: T[]): void {
  try {
    localStorage.setItem(key, JSON.stringify(list));
  } catch {
    // out of room, or storage switched off -- the built-ins still work
  }
}

/** Built-ins first, then whatever is cached under a different name -- a
 *  cached entry with the *same* name as a built-in replaces it, which is how
 *  editing "Team robot" or "Left launch" is allowed without ever letting
 *  them be deleted: there is always an entry of that name, just possibly an
 *  edited one. */
function merged<T extends { name: string }>(builtins: T[], cached: T[]): T[] {
  const byName = new Map(builtins.map((item) => [item.name, item]));

  for (const item of cached) {
    byName.set(item.name, item);
  }

  return [...byName.values()];
}

/** Every known robot, cache and built-ins merged -- synchronous, for the
 *  picker to show something before any network call finishes. */
export function allRobots(): RobotProfile[] {
  return merged(BUILTIN_ROBOTS, read<RobotProfile>(ROBOTS_CACHE));
}

export function allStarts(): StartPosition[] {
  return merged(BUILTIN_STARTS, read<StartPosition>(STARTS_CACHE));
}

/** Save a profile: cached straight away, and best-effort to the shared
 *  store on top. Returns whether the remote write took -- the cache write
 *  never fails outright, only silently (see write() above). */
export async function saveRobot(profile: RobotProfile): Promise<boolean> {
  write(ROBOTS_CACHE, merged(read<RobotProfile>(ROBOTS_CACHE), [profile]));

  try {
    await setDoc(doc(db, "robots", profile.name), profile);
    return true;
  } catch {
    return false;
  }
}

export async function saveStart(start: StartPosition): Promise<boolean> {
  write(STARTS_CACHE, merged(read<StartPosition>(STARTS_CACHE), [start]));

  try {
    await setDoc(doc(db, "starts", start.name), start);
    return true;
  } catch {
    return false;
  }
}

/** Best-effort refresh from Firestore into the cache. Returns the merged
 *  list either way -- offline, that is just whatever was already cached. */
export async function refreshRobots(): Promise<RobotProfile[]> {
  try {
    const snapshot = await getDocs(collection(db, "robots"));
    const remote = snapshot.docs.map((entry) => entry.data() as RobotProfile);

    write(ROBOTS_CACHE, merged(read<RobotProfile>(ROBOTS_CACHE), remote));
  } catch {
    // offline, or the host is unreachable -- the cache stands as it is
  }

  return allRobots();
}

export async function refreshStarts(): Promise<StartPosition[]> {
  try {
    const snapshot = await getDocs(collection(db, "starts"));
    const remote = snapshot.docs.map((entry) => entry.data() as StartPosition);

    write(STARTS_CACHE, merged(read<StartPosition>(STARTS_CACHE), remote));
  } catch {
    // offline, or the host is unreachable -- the cache stands as it is
  }

  return allStarts();
}

/** Numbers, not identity: two profiles of the same name are "the same
 *  profile" only if every number still matches, which is what tells a run's
 *  own copy apart from a profile that has since been edited. Rounded, so a
 *  float that only differs in its last bit does not read as "changed". */
export function robotsMatch(a: RobotProfile, b: RobotProfile): boolean {
  return (
    a.name === b.name &&
    numbersMatch(a.planning, b.planning) &&
    numbersMatch(a.driveBase, b.driveBase)
  );
}

export function startsMatch(a: StartPosition, b: StartPosition): boolean {
  return a.name === b.name && round(a.x) === round(b.x) &&
    round(a.y) === round(b.y) && round(a.head) === round(b.head);
}

function numbersMatch(a: object, b: object): boolean {
  const left = a as Record<string, unknown>;
  const right = b as Record<string, unknown>;
  const keys = new Set([...Object.keys(left), ...Object.keys(right)]);

  for (const key of keys) {
    const leftValue = left[key];
    const rightValue = right[key];

    if (typeof leftValue === "number" && typeof rightValue === "number") {
      if (round(leftValue) !== round(rightValue)) {
        return false;
      }
    } else if (leftValue !== rightValue) {
      return false;
    }
  }

  return true;
}

function round(value: number): number {
  return Math.round(value * 1000) / 1000;
}
