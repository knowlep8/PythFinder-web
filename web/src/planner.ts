/**
 * The page's side of the worker.
 *
 * Two ways to ask for a run:
 *
 *   build(run)     one specific run, answered with a promise
 *   request(run)   the run as it currently stands, answered through onResult
 *
 * `request` is what the editor will call as somebody drags a waypoint or types
 * a number. It waits for them to stop (briefly), and never lets two builds
 * pile up: if a newer run arrives while one is being built, the older one is
 * forgotten rather than queued. Only the newest answer is ever delivered.
 */

import type { FromWorker, ToWorker } from "./worker";
import type { BuildResult, Run } from "./types";

export interface PlannerHandlers {
  /** progress while Python starts */
  onStatus?: (text: string) => void;
  /** a newer result for whatever was last requested */
  onResult?: (result: BuildResult, seconds: number) => void;
  /** the planner itself went wrong, not the run */
  onError?: (message: string) => void;
}

export interface Planner {
  /** resolves once Python is up */
  ready: Promise<{ python: string; seconds: number }>;
  /** build this exact run, and tell me about this one */
  build: (run: Run) => Promise<{ result: BuildResult; seconds: number }>;
  /** build the newest run shortly, and tell me through onResult */
  request: (run: Run) => void;
  /** how many builds the worker has actually run */
  builds: () => number;
  stop: () => void;
}

const SETTLE_MS = 150;

export function createPlanner(
  handlers: PlannerHandlers = {},
  settleMs: number = SETTLE_MS,
): Planner {
  const worker = new Worker(new URL("./worker.ts", import.meta.url), {
    type: "module",
  });

  let nextId = 1;
  let builds = 0;

  // the id of the newest request; answers to anything older are stale
  let newestId = 0;

  let waiting: ReturnType<typeof setTimeout> | null = null;
  let queued: Run | null = null;
  let inFlight = false;

  const answers = new Map<
    number,
    (answer: { result: BuildResult; seconds: number }) => void
  >();
  const failures = new Map<number, (error: Error) => void>();

  let announceReady: (value: { python: string; seconds: number }) => void;
  let announceBroken: (error: Error) => void;

  const ready = new Promise<{ python: string; seconds: number }>(
    (resolve, reject) => {
      announceReady = resolve;
      announceBroken = reject;
    },
  );

  // a rejected `ready` nobody is waiting on yet is still an unhandled
  // rejection; onError is how the page hears about it
  ready.catch(() => {});

  function send(run: Run, id: number) {
    inFlight = true;
    worker.postMessage({ type: "build", id, run } satisfies ToWorker);
  }

  function sendQueued() {
    if (queued === null || inFlight) {
      return;
    }

    const run = queued;
    queued = null;
    newestId = nextId++;

    send(run, newestId);
  }

  worker.onmessage = (event: MessageEvent) => {
    const message = event.data as FromWorker;

    switch (message.type) {
      case "status":
        handlers.onStatus?.(message.text);
        break;

      case "ready":
        announceReady({ python: message.python, seconds: message.seconds });
        break;

      case "broken":
        handlers.onError?.(message.message);
        announceBroken(new Error(message.message));
        break;

      case "built": {
        builds += 1;
        inFlight = false;

        const answer = answers.get(message.id);

        if (answer) {
          answers.delete(message.id);
          failures.delete(message.id);
          answer({ result: message.result, seconds: message.seconds });
        } else if (message.id === newestId) {
          // a background request, and still the newest: worth drawing
          handlers.onResult?.(message.result, message.seconds);
        }

        sendQueued();
        break;
      }

      case "failed": {
        inFlight = false;

        const failure = failures.get(message.id);

        if (failure) {
          answers.delete(message.id);
          failures.delete(message.id);
          failure(new Error(message.message));
        } else {
          handlers.onError?.(message.message);
        }

        sendQueued();
        break;
      }
    }
  };

  return {
    ready,

    build(run) {
      const id = nextId++;
      newestId = id;

      return new Promise((resolve, reject) => {
        answers.set(id, resolve);
        failures.set(id, reject);
        send(run, id);
      });
    },

    request(run) {
      queued = run;

      if (waiting !== null) {
        clearTimeout(waiting);
      }

      waiting = setTimeout(() => {
        waiting = null;
        sendQueued();
      }, settleMs);
    },

    builds: () => builds,

    stop() {
      if (waiting !== null) {
        clearTimeout(waiting);
      }
      worker.terminate();
    },
  };
}
