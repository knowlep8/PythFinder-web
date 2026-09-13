/**
 * The KV surface these functions actually call -- four methods, hand-written
 * rather than adding @cloudflare/workers-types for them. Not type-checked by
 * the site's own tsconfig (its "include" is just src/); Cloudflare's own
 * build transpiles this without type-checking it either.
 */
export interface KVListKey {
  name: string;
  metadata?: unknown;
}

export interface KVNamespace {
  get(key: string): Promise<string | null>;
  put(key: string, value: string, options?: { metadata?: unknown }): Promise<void>;
  delete(key: string): Promise<void>;
  list(options?: { prefix?: string }): Promise<{
    keys: KVListKey[];
    list_complete: boolean;
    cursor?: string;
  }>;
}

export interface Env {
  RUNS: KVNamespace;
}
