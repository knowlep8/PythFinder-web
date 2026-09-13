/**
 * Every run is keyed by who saved it, not by anything authenticated -- there
 * is no login, so "owner" is just the name someone typed into the page. The
 * point is to stop Jake from absent-mindedly saving over Amy's "run_a", not
 * to keep anyone out.
 */
export function runKey(owner: string, name: string): string {
  return `runs/${encodeURIComponent(owner)}/${encodeURIComponent(name)}`;
}

export function ownerPrefix(owner: string): string {
  return `runs/${encodeURIComponent(owner)}/`;
}
