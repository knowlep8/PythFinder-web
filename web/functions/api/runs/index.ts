/**
 * GET /api/runs?owner=<name> -- one person's saved runs, name and savedAt
 * only. Cheap: KV list() returns each key's metadata without a get per key,
 * and that metadata is all this needs.
 */
import type { Env, KVListKey } from "../../_shared/env";
import { ownerPrefix } from "../../_shared/keys";

interface RunMetadata {
  owner: string;
  name: string;
  savedAt: string;
}

function json(body: unknown, status = 200): Response {
  return new Response(JSON.stringify(body), {
    status,
    headers: { "content-type": "application/json" },
  });
}

function metadataOf(key: KVListKey): RunMetadata | null {
  const meta = key.metadata as Partial<RunMetadata> | undefined;

  if (
    meta === undefined ||
    typeof meta.owner !== "string" ||
    typeof meta.name !== "string" ||
    typeof meta.savedAt !== "string"
  ) {
    return null;
  }

  return { owner: meta.owner, name: meta.name, savedAt: meta.savedAt };
}

export async function onRequestGet(context: {
  request: Request;
  env: Env;
}): Promise<Response> {
  const owner = new URL(context.request.url).searchParams.get("owner")?.trim();

  if (owner === undefined || owner === "") {
    return json({ error: "owner is required" }, 400);
  }

  const listing = await context.env.RUNS.list({ prefix: ownerPrefix(owner) });
  const runs = listing.keys.map(metadataOf).filter((entry): entry is RunMetadata => entry !== null);

  return json({ runs });
}
