/**
 * Keeping runs between visits.
 *
 * Three separate jobs, deliberately:
 *
 *   autosave    the run being worked on, restored when the page reopens, so
 *               nobody loses an afternoon to a closed tab
 *   the list    runs saved under a name, to pick between
 *   the shared store   the same, but reachable from another laptop -- see below
 *
 * Autosave and the list live in this browser only, and stay the source of
 * truth: a run that has to travel to another laptop can always go as a
 * .json file, same as before. The shared store is best-effort on top of
 * that, not instead of it -- every function below returns cleanly (false, an
 * empty list, or null) if the host cannot be reached, so a flaky connection
 * never stops the planner working, only stops a run from following you
 * between laptops until it is back.
 *
 * Every read is defensive. Browser storage can be full, switched off, or hold
 * something from an older version of this page, and none of those is a reason
 * for a team member to lose their run.
 */

import {
  collection,
  collectionGroup,
  deleteDoc,
  doc,
  getDoc,
  getDocs,
  setDoc,
} from "firebase/firestore";
import { db } from "./firebase";
import type { Run } from "./types";

const WORKING = "pythfinder.working";
const SAVED = "pythfinder.saved";
const OWNER = "pythfinder.owner";

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

/**
 * Who's using the page. Not a login -- there is no gating on this site at
 * all (see docs/web-planner.md) -- just the name someone typed, remembered
 * per browser so it does not have to be retyped every visit. It is what
 * keeps two people's saved runs apart in the shared store below.
 */
export function rememberOwner(owner: string): boolean {
  return write(OWNER, owner);
}

export function recallOwner(): string {
  return read<string>(OWNER, "");
}

export interface RemoteRunSummary {
  owner: string;
  name: string;
  savedAt: string;
}

/**
 * The shared store: Firestore, at runs/<owner>/items/<name>, opened directly
 * from the browser -- no backend of our own, since firestore.rules is the
 * only thing standing between a request and the data either way. Every call
 * here is best-effort: network trouble, or Firestore refusing a request, is
 * not a reason to interrupt anyone, only a reason the run stays local until
 * the host is reachable again.
 */

function runDoc(owner: string, name: string) {
  return doc(db, "runs", owner, "items", name);
}

interface StoredRun {
  owner: string;
  name: string;
  savedAt: string;
  run: Run;
}

function summaryOf(data: unknown): RemoteRunSummary | null {
  const stored = data as Partial<StoredRun> | undefined;

  if (
    stored === undefined ||
    typeof stored.owner !== "string" ||
    typeof stored.name !== "string" ||
    typeof stored.savedAt !== "string"
  ) {
    return null;
  }

  return { owner: stored.owner, name: stored.name, savedAt: stored.savedAt };
}

/** Save under this owner and name, replacing any run already using both. */
export async function saveRemote(owner: string, name: string, run: Run): Promise<boolean> {
  if (owner.trim() === "") {
    return false;
  }

  const stored: StoredRun = { owner, name, savedAt: new Date().toISOString(), run };

  try {
    await setDoc(runDoc(owner, name), stored);
    return true;
  } catch {
    return false;
  }
}

/** One owner's saved runs, name and when only -- cheap, for a picker list. */
export async function listRemote(owner: string): Promise<RemoteRunSummary[]> {
  if (owner.trim() === "") {
    return [];
  }

  try {
    const snapshot = await getDocs(collection(db, "runs", owner, "items"));

    return snapshot.docs
      .map((entry) => summaryOf(entry.data()))
      .filter((entry): entry is RemoteRunSummary => entry !== null);
  } catch {
    return [];
  }
}

/** Every saved run, from every owner -- the mentor view. */
export async function listAllRemote(): Promise<RemoteRunSummary[]> {
  try {
    const snapshot = await getDocs(collectionGroup(db, "items"));

    return snapshot.docs
      .map((entry) => summaryOf(entry.data()))
      .filter((entry): entry is RemoteRunSummary => entry !== null)
      .sort((a, b) => a.owner.localeCompare(b.owner) || a.name.localeCompare(b.name));
  } catch {
    return [];
  }
}

export async function openRemote(owner: string, name: string): Promise<Run | null> {
  try {
    const snapshot = await getDoc(runDoc(owner, name));

    if (!snapshot.exists()) {
      return null;
    }

    const stored = snapshot.data() as Partial<StoredRun>;

    return looksLikeRun(stored.run) ? stored.run : null;
  } catch {
    return null;
  }
}

export async function forgetRemote(owner: string, name: string): Promise<void> {
  try {
    await deleteDoc(runDoc(owner, name));
  } catch {
    // best-effort -- the local copy is already gone either way
  }
}
