/**
 * Handing over the file the hub runs.
 *
 * The worker already returns the module text with every build, so there is
 * nothing to generate here: this is about giving it a sensible name, saying
 * how big it is, and getting it onto the laptop.
 */

/**
 * Is this a name Pybricks can import?
 *
 * The downloaded file is imported by name on the hub -- `import run_a` -- so
 * the name has to be a Python identifier. A team member typing "Run 1" or
 * "left side" would get a file that cannot be imported, and would find out
 * about it on the hub, at a competition.
 */
export function nameProblem(name: string): string | null {
  if (name.length === 0) {
    return "give the run a name";
  }

  if (/^[0-9]/.test(name)) {
    return "a name cannot start with a number";
  }

  if (!/^[A-Za-z_][A-Za-z0-9_]*$/.test(name)) {
    return "use letters, numbers and underscores only — no spaces";
  }

  // not exhaustive, but these are the ones a team would plausibly pick
  if (["import", "class", "def", "from", "return", "run", "trajectory"].includes(name)) {
    return `"${name}" is a word Python uses; pick another`;
  }

  return null;
}

/**
 * What the file costs the hub, read back out of the module itself.
 *
 * Step 5.7: there is no more COUNT of recorded states to read off -- the
 * DriveBase file is a short list of calls, not a payload of encoded numbers
 * -- so "moves" counts the lines that actually drive, turn, wait or fire
 * something in run(), the same way the old readout counted states: from the
 * file's own text, because there is no separate structured move list on
 * this side of the worker boundary.
 */
export function hubCost(moduleText: string): { moves: number; bytes: number } {
  const moves = moduleText.match(
    /^\s*(drive\.straight\(|core\.turn_to\(|wait\(\d|drive\.settings\(|_action_\d+\(core\)|core\.\w+Task\.\w+\()/gm,
  );

  return {
    moves: moves?.length ?? 0,
    bytes: new TextEncoder().encode(moduleText).length,
  };
}

/** Save the module as <name>.py. */
export function saveModule(name: string, moduleText: string) {
  const file = new Blob([moduleText], { type: "text/x-python" });
  const url = URL.createObjectURL(file);

  const link = document.createElement("a");
  link.href = url;
  link.download = `${name}.py`;
  document.body.append(link);
  link.click();
  link.remove();

  // let the browser start reading it before the handle goes
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
