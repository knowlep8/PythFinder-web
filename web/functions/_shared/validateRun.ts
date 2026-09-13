/**
 * Mirrors src/store.ts's looksLikeRun, deliberately duplicated rather than
 * imported: Pages Functions only bundle modules from inside functions/, so
 * reaching into ../src would not build. Keep the two in sync by hand if the
 * run shape ever changes -- both are five lines, so that is not much to ask.
 */
export function looksLikeRun(value: unknown): boolean {
  if (typeof value !== "object" || value === null) {
    return false;
  }

  const run = value as Record<string, unknown>;

  return (
    typeof run.name === "string" &&
    typeof run.start === "object" &&
    run.start !== null &&
    Array.isArray(run.steps)
  );
}
