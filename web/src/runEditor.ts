/**
 * The run as a list of steps, and the buttons that change it.
 *
 * Each row is one call the builder will make -- drive, turn, wait, go to a
 * point, go to a pose -- so what a team member sees on screen and what comes
 * out in the file are the same list in the same order.
 *
 * There is deliberately no heading-mode choice. Step 1.1 established that
 * toPoint and toPointTangentHead, and all three toPose variants, are the same
 * call on a tank drive, so offering them would be several buttons that do one
 * thing. Splines are not offered either: inSpline does nothing today.
 *
 * One rule worth knowing before changing this: the list is only rebuilt for
 * structural changes -- adding, deleting, reordering, selecting. Typing in a
 * number must not rebuild it, or the box being typed into loses focus on every
 * keystroke.
 */

import type { BuiltStep, Diagnostic, RunStep, StepType } from "./types";

export interface RunEditorHandlers {
  onChange?: (steps: RunStep[]) => void;
  onSelect?: (index: number | null) => void;
}

interface FieldSpec {
  key: "cm" | "ms" | "deg" | "x" | "y" | "head";
  label: string;
  step?: number;
}

const SHAPES: Record<StepType, { title: string; fields: FieldSpec[]; reversible: boolean }> = {
  drive: {
    title: "Drive",
    fields: [{ key: "cm", label: "cm" }],
    reversible: false,
  },
  turn: {
    title: "Turn to",
    fields: [{ key: "deg", label: "°" }],
    reversible: true,
  },
  wait: {
    title: "Wait",
    fields: [{ key: "ms", label: "ms", step: 100 }],
    reversible: false,
  },
  toPoint: {
    title: "Go to point",
    fields: [
      { key: "x", label: "x" },
      { key: "y", label: "y" },
    ],
    reversible: true,
  },
  toPose: {
    title: "Go to pose",
    fields: [
      { key: "x", label: "x" },
      { key: "y", label: "y" },
      { key: "head", label: "°" },
    ],
    reversible: true,
  },
};

const NEW_STEP: Record<StepType, () => RunStep> = {
  drive: () => ({ type: "drive", cm: 30 }),
  turn: () => ({ type: "turn", deg: 90 }),
  wait: () => ({ type: "wait", ms: 500 }),
  toPoint: () => ({ type: "toPoint", x: 0, y: 0 }),
  toPose: () => ({ type: "toPose", x: 0, y: 0, head: 0 }),
};

export class RunEditor {
  private container: HTMLElement;
  private handlers: RunEditorHandlers;
  private steps: RunStep[];
  private times: BuiltStep[] = [];
  private problems: Diagnostic[] = [];
  private selected: number | null = null;

  constructor(
    container: HTMLElement,
    steps: RunStep[],
    handlers: RunEditorHandlers = {},
  ) {
    this.container = container;
    this.steps = steps.map((step) => ({ ...step }));
    this.handlers = handlers;

    this.render();
  }

  getSteps(): RunStep[] {
    return this.steps.map((step) => ({ ...step }));
  }

  /** Replace the whole run, as when one is loaded or imported. */
  setSteps(steps: RunStep[]) {
    this.steps = steps.map((step) => ({ ...step }));
    this.selected = null;

    this.render();
  }

  getSelected(): number | null {
    return this.selected;
  }

  /** How long each step took, once the worker has said. */
  setTimes(times: BuiltStep[]) {
    this.times = times;
    this.showTimes();
  }

  /** What went wrong, to be shown against the steps that caused it. */
  setProblems(problems: Diagnostic[]) {
    this.problems = problems;
    this.showProblems();
  }

  private changed() {
    this.handlers.onChange?.(this.getSteps());
  }

  private select(index: number | null) {
    this.selected = index;

    for (const row of this.container.querySelectorAll<HTMLElement>(".step")) {
      row.classList.toggle("selected", Number(row.dataset.index) === index);
    }

    this.handlers.onSelect?.(index);
  }

  private move(index: number, by: number) {
    const to = index + by;

    if (to < 0 || to >= this.steps.length) {
      return;
    }

    const [step] = this.steps.splice(index, 1);
    this.steps.splice(to, 0, step);

    this.selected = to;
    this.render();
    this.changed();
  }

  private remove(index: number) {
    this.steps.splice(index, 1);

    if (this.selected !== null && this.selected >= this.steps.length) {
      this.selected = this.steps.length > 0 ? this.steps.length - 1 : null;
    }

    this.render();
    this.changed();
  }

  private add(type: StepType) {
    this.steps.push(NEW_STEP[type]());
    this.selected = this.steps.length - 1;

    this.render();
    this.changed();
  }

  /** Only the durations, so this can run without disturbing what is focused. */
  private showTimes() {
    for (const row of this.container.querySelectorAll<HTMLElement>(".step")) {
      const index = Number(row.dataset.index);
      const timing = this.times.find((step) => step.index === index);
      const label = row.querySelector<HTMLElement>(".took");

      if (!label) {
        continue;
      }

      if (!timing) {
        label.textContent = "";
        continue;
      }

      const took = timing.ends_ms - timing.starts_ms;

      label.textContent =
        took > 0
          ? `${(took / 1000).toFixed(1)}s`
          : index > 0
            ? "joined to the step above"
            : "";
    }
  }

  render() {
    this.container.replaceChildren();

    this.steps.forEach((step, index) => {
      this.container.append(this.renderStep(step, index));
    });

    if (this.steps.length === 0) {
      const empty = document.createElement("p");
      empty.className = "empty";
      empty.textContent = "No steps yet. Add one below.";
      this.container.append(empty);
    }

    this.container.append(this.renderAdders());
    this.showTimes();
    this.showProblems();
  }

  /**
   * Put each problem on the step that caused it.
   *
   * A warning about step 4 belongs on step 4, not in a list at the bottom that
   * nobody reads. Run-wide problems have no step to sit on and stay in the
   * panel below.
   */
  private showProblems() {
    for (const row of this.container.querySelectorAll<HTMLElement>(".step")) {
      const index = Number(row.dataset.index);
      const mine = this.problems.filter((problem) => problem.step === index);

      row.querySelectorAll(".problem").forEach((old) => old.remove());
      row.classList.toggle("troubled", mine.length > 0);

      for (const problem of mine) {
        const line = document.createElement("p");

        line.className = `problem ${problem.level}`;
        line.textContent =
          problem.message +
          (problem.suggestion === null ? "" : ` — try: ${problem.suggestion}`);

        row.append(line);
      }
    }
  }

  private renderStep(step: RunStep, index: number): HTMLElement {
    const shape = SHAPES[step.type];

    const row = document.createElement("div");
    row.className = "step" + (index === this.selected ? " selected" : "");
    row.dataset.index = String(index);
    row.addEventListener("pointerdown", () => this.select(index));

    const number = document.createElement("span");
    number.className = "number";
    number.textContent = String(index + 1);
    row.append(number);

    const title = document.createElement("span");
    title.className = "title";
    title.textContent = shape.title;
    row.append(title);

    for (const field of shape.fields) {
      const box = document.createElement("input");
      box.type = "number";
      box.step = String(field.step ?? 1);
      box.value = String(step[field.key] ?? 0);
      box.title = field.label;

      // typing changes the run but must not redraw this list
      box.addEventListener("input", () => {
        const value = Number(box.value);

        if (Number.isFinite(value)) {
          // every key a FieldSpec can name is a number on RunStep, so this
          // stays checked rather than being cast away
          this.steps[index][field.key] = value;
          this.changed();
        }
      });

      const label = document.createElement("label");
      label.append(box, document.createTextNode(field.label));
      row.append(label);
    }

    if (shape.reversible) {
      const tick = document.createElement("input");
      tick.type = "checkbox";
      tick.checked = step.reversed === true;
      tick.addEventListener("change", () => {
        this.steps[index].reversed = tick.checked;
        this.changed();
      });

      const label = document.createElement("label");
      label.className = "reversed";
      label.append(tick, document.createTextNode("backwards"));
      row.append(label);
    }

    const took = document.createElement("span");
    took.className = "took";
    row.append(took);

    row.append(
      this.button("↑", "move this step earlier", () => this.move(index, -1)),
      this.button("↓", "move this step later", () => this.move(index, 1)),
      this.button("✕", "delete this step", () => this.remove(index)),
    );

    return row;
  }

  private renderAdders(): HTMLElement {
    const row = document.createElement("div");
    row.className = "adders";

    (Object.keys(SHAPES) as StepType[]).forEach((type) => {
      row.append(
        this.button(SHAPES[type].title, `add a ${SHAPES[type].title} step`, () =>
          this.add(type),
        ),
      );
    });

    return row;
  }

  private button(text: string, title: string, onClick: () => void): HTMLElement {
    const button = document.createElement("button");

    button.type = "button";
    button.textContent = text;
    button.title = title;
    button.addEventListener("click", (event) => {
      event.stopPropagation();
      onClick();
    });

    return button;
  }
}
