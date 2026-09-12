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

/** What the file costs on the hub, read back out of the module itself. */
export function hubCost(moduleText: string): { states: number; bytes: number } {
  const count = moduleText.match(/^COUNT = (\d+)$/m);
  const states = count ? Number(count[1]) : 0;

  // three little-endian int16 per state: left, right, heading
  return { states, bytes: states * 6 };
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
