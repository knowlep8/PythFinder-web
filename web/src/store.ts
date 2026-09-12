/**
 * Keeping runs between visits.
 *
 * Two separate jobs, deliberately:
 *
 *   autosave    the run being worked on, restored when the page reopens, so
 *               nobody loses an afternoon to a closed tab
 *   the list    runs saved under a name, to pick between
 *
 * Both live in this browser only. A run that has to travel to another laptop
 * goes as a .json file -- which is also the backup, until the shared storage
 * in the plan's 2.8 note exists.
 *
 * Every read is defensive. Browser storage can be full, switched off, or hold
 * something from an older version of this page, and none of those is a reason
 * for a team member to lose their run.
 */

import type { Run } from "./types";

const WORKING = "pythfinder.working";
const SAVED = "pythfinder.saved";

export interface SavedRun {
  name: string;
  savedAt: string;
  run: Run;
}

function read<T>(key: string, fallback: T): T {
  try {
    const held = localStorage.getItem(key);

    return held === null ? fallback : (JSON.parse(held) as T);
  } catch {
    return fallback;
  }
}

function write(key: string, value: unknown): boolean {
  try {
    localStorage.setItem(key, JSON.stringify(value));

    return true;
  } catch {
    // out of room, or storage switched off. The run is still on screen.
    return false;
  }
}

/** Looks enough like a run to load? Anything saved by an older page may not. */
export function looksLikeRun(value: unknown): value is Run {
  if (typeof value !== "object" || value === null) {
    return false;
  }

  const run = value as Partial<Run>;

  return (
    typeof run.name === "string" &&
    typeof run.start === "object" &&
    run.start !== null &&
    Array.isArray(run.steps)
  );
}

/** The run in progress, saved on every change. */
export function rememberWorking(run: Run): boolean {
  return write(WORKING, run);
}

export function recallWorking(): Run | null {
  const held = read<unknown>(WORKING, null);

  return looksLikeRun(held) ? held : null;
}

export function listSaved(): SavedRun[] {
  const held = read<unknown>(SAVED, []);

  if (!Array.isArray(held)) {
    return [];
  }

  return held.filter(
    (entry): entry is SavedRun =>
      typeof entry === "object" &&
      entry !== null &&
      typeof (entry as SavedRun).name === "string" &&
      looksLikeRun((entry as SavedRun).run),
  );
}

/** Save under a name, replacing any run already using it. */
export function saveNamed(name: string, run: Run): boolean {
  const kept = listSaved().filter((entry) => entry.name !== name);

  kept.push({ name, savedAt: new Date().toISOString(), run });
  kept.sort((a, b) => a.name.localeCompare(b.name));

  return write(SAVED, kept);
}

export function forgetNamed(name: string): boolean {
  return write(
    SAVED,
    listSaved().filter((entry) => entry.name !== name),
  );
}

/** The run as a file, for carrying to another laptop. */
export function exportRun(run: Run) {
  const file = new Blob([JSON.stringify(run, null, 2)], {
    type: "application/json",
  });
  const url = URL.createObjectURL(file);

  const link = document.createElement("a");
  link.href = url;
  link.download = `${run.name}.json`;
  document.body.append(link);
  link.click();
  link.remove();

  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

/** Read a run back from a file the team member picked. */
export async function importRun(file: File): Promise<Run> {
  const text = await file.text();

  let parsed: unknown;

  try {
    parsed = JSON.parse(text);
  } catch {
    throw new Error(`${file.name} is not a run file -- it is not even JSON`);
  }

  if (!looksLikeRun(parsed)) {
    throw new Error(`${file.name} does not look like a run`);
  }

  return parsed;
}
