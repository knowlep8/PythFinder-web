/**
 * GET /api/runs/all -- every saved run, from every owner.
 *
 * The "mentor view" this whole design exists for. Deliberately not scoped or
 * gated any differently from the per-owner list: there is no login on this
 * site at all, so anyone who finds this URL sees the same thing a mentor
 * does. That is an accepted tradeoff, not an oversight -- see the "skip
 * gating" decision in docs/web-planner.md.
 */
import type { Env, KVListKey } from "../../_shared/env";

interface RunMetadata {
  owner: string;
  name: string;
  savedAt: string;
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

export async function onRequestGet(context: { env: Env }): Promise<Response> {
  const listing = await context.env.RUNS.list();

  const runs = listing.keys
    .map(metadataOf)
    .filter((entry): entry is RunMetadata => entry !== null)
    .sort((a, b) => a.owner.localeCompare(b.owner) || a.name.localeCompare(b.name));

  return new Response(JSON.stringify({ runs }), {
    headers: { "content-type": "application/json" },
  });
}
