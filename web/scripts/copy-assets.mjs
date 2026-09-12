/**
 * Copy the mat and the robot into public/field.
 *
 * Both pictures live with the library, in pythfinder/Images. The wheel the
 * browser gets has them stripped out -- they are 57% of it, and the planning
 * code never opens them (step 1.8) -- so the page carries its own copies
 * instead.
 *
 * In the container the Dockerfile puts them in place before the build, so this
 * finds them already there and says so.
 */

import { copyFile, mkdir, stat } from "node:fs/promises";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const from = join(here, "..", "..", "pythfinder", "Images");
const into = join(here, "..", "public", "field");

const WANTED = [
  ["Field/FLL_table_BG.png", "mat.png"],
  ["Robot/fll_robot_team.png", "robot.png"],
];

async function exists(path) {
  try {
    await stat(path);
    return true;
  } catch {
    return false;
  }
}

async function main() {
  await mkdir(into, { recursive: true });

  const done = [];

  for (const [source, name] of WANTED) {
    const target = join(into, name);

    if (await exists(join(from, source))) {
      await copyFile(join(from, source), target);
      done.push(`${name} copied`);
      continue;
    }

    if (await exists(target)) {
      done.push(`${name} already in place`);
      continue;
    }

    throw new Error(
      `${source} is missing from ${from}, and there is no copy at ${target}. ` +
        `The page cannot draw the field without it.`,
    );
  }

  console.log("field: " + done.join(", "));
}

main().catch((error) => {
  console.error(error.message);
  process.exit(1);
});
