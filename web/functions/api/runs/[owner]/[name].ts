/**
 * GET/PUT/DELETE /api/runs/<owner>/<name> -- one saved run.
 *
 * `owner` and `name` are whatever the page sent; nothing here checks them
 * against anything, because there is nothing to check them against -- see
 * the "skip gating" decision in docs/web-planner.md. A PUT is still refused
 * if the body does not look like a run, same as importRun() already refuses
 * a file that is not one.
 */
import type { Env } from "../../../_shared/env";
import { looksLikeRun } from "../../../_shared/validateRun";
import { runKey } from "../../../_shared/keys";

interface Params {
  owner: string;
  name: string;
}

interface StoredRun {
  owner: string;
  name: string;
  savedAt: string;
  run: unknown;
}

function json(body: unknown, status = 200): Response {
  return new Response(JSON.stringify(body), {
    status,
    headers: { "content-type": "application/json" },
  });
}

export async function onRequestGet(context: {
  env: Env;
  params: Params;
}): Promise<Response> {
  const held = await context.env.RUNS.get(runKey(context.params.owner, context.params.name));

  if (held === null) {
    return json({ error: "not found" }, 404);
  }

  const stored = JSON.parse(held) as StoredRun;

  return json(stored.run);
}

export async function onRequestPut(context: {
  request: Request;
  env: Env;
  params: Params;
}): Promise<Response> {
  const owner = context.params.owner.trim();
  const name = context.params.name.trim();

  if (owner === "" || name === "") {
    return json({ error: "owner and name are both required" }, 400);
  }

  let run: unknown;

  try {
    run = await context.request.json();
  } catch {
    return json({ error: "body is not JSON" }, 400);
  }

  if (!looksLikeRun(run)) {
    return json({ error: "body does not look like a run" }, 400);
  }

  const savedAt = new Date().toISOString();
  const stored: StoredRun = { owner, name, savedAt, run };

  await context.env.RUNS.put(runKey(owner, name), JSON.stringify(stored), {
    metadata: { owner, name, savedAt },
  });

  return json({ savedAt });
}

export async function onRequestDelete(context: {
  env: Env;
  params: Params;
}): Promise<Response> {
  await context.env.RUNS.delete(runKey(context.params.owner, context.params.name));

  return json({ ok: true });
}
