import { execSync } from "node:child_process";
import { defineConfig } from "vite";

/**
 * Which build this is, shown beside the page title: the commit and its date,
 * "+" if built with uncommitted changes. The page works offline (step 2.9),
 * so a laptop can quietly keep an old copy -- this is how to tell, by asking
 * a team member to read it out.
 *
 * Built from a checkout without git (the Docker image copies the source but
 * not .git) it says so, rather than failing the build.
 */
function plannerVersion(): string {
  try {
    const run = (command: string) =>
      execSync(command, { stdio: ["ignore", "pipe", "ignore"] }).toString().trim();

    const commit = run("git log -1 --format=%h");
    const date = run("git log -1 --format=%cs");
    const dirty = run("git status --porcelain") === "" ? "" : "+";

    return `${commit}${dirty} · ${date}`;
  } catch {
    return "unversioned build";
  }
}

export default defineConfig({
  define: {
    __PLANNER_VERSION__: JSON.stringify(plannerVersion()),
  },
  build: {
    // Pyodide needs a modern target anyway, and top-level await is used in
    // main.ts through the dynamic import of the vendored runtime
    target: "es2022",
    outDir: "dist",
    emptyOutDir: true,
  },
  server: {
    port: 5173,
  },
});
