/**
 * Copy the Pyodide runtime out of node_modules and into public/pyodide.
 *
 * The planner serves Pyodide from its own container rather than from a CDN:
 * nothing external is needed at run time, a school firewall cannot block it,
 * and the files arrive as fast as the rest of the page. Vite copies anything
 * in public/ into the build as-is, so this runs before dev and before build.
 *
 * Only the core is copied. The planner never calls loadPackage(), because the
 * library it needs is a dependency-free wheel it unpacks itself -- see
 * src/main.ts.
 */

import { copyFile, mkdir, stat } from "node:fs/promises";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const from = join(here, "..", "node_modules", "pyodide");
const into = join(here, "..", "public", "pyodide");

// what loadPyodide() actually reads
const WANTED = [
  "pyodide.mjs",
  "pyodide.asm.js",
  "pyodide.asm.wasm",
  "python_stdlib.zip",
  "pyodide-lock.json",
];

async function sizeOf(path) {
  return (await stat(path)).size;
}

async function main() {
  await mkdir(into, { recursive: true });

  let total = 0;

  for (const name of WANTED) {
    const source = join(from, name);

    try {
      await copyFile(source, join(into, name));
      total += await sizeOf(source);
    } catch (error) {
      if (error.code === "ENOENT") {
        throw new Error(
          `${name} is missing from node_modules/pyodide. ` +
            `Run npm install first, or check whether the pyodide package ` +
            `renamed its files.`,
        );
      }
      throw error;
    }
  }

  console.log(
    `pyodide: copied ${WANTED.length} files, ` +
      `${(total / 1e6).toFixed(1)}MB, into public/pyodide`,
  );
}

main().catch((error) => {
  console.error(error.message);
  process.exit(1);
});
