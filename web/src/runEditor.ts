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

import { indentWithTab } from "@codemirror/commands";
import { python } from "@codemirror/lang-python";
import { keymap, placeholder, EditorView } from "@codemirror/view";
import { minimalSetup } from "codemirror";

import type { ActionBody, BuiltStep, Diagnostic, RunStep, StepType } from "./types";
import { isCode } from "./types";

export interface RunEditorHandlers {
  onChange?: (steps: RunStep[]) => void;
  onSelect?: (index: number | null) => void;
}

interface FieldSpec {
  key: "cm" | "ms" | "deg" | "x" | "y" | "head" | "speed" | "angle";
  label: string;
  step?: number;
}

/** A picker: which motor, or which way to move it. */
interface ChoiceSpec {
  key: "motor" | "call";
  options: Array<[string, string]>;
}

/** Attachment motors, as named on the hub. */
const MOTORS: Array<[string, string]> = [
  ["leftTask", "left arm"],
  ["rightTask", "right arm"],
];

/**
 * The calls each kind of action may use, and why they differ.
 *
 * A sequential arm step waits for the motor, so it needs a call that ends.
 * A parallel action fires while the robot is driving, on the same loop that
 * drives it, so it must return at once -- anything that waits would stall the
 * path. That is the whole reason there are two kinds.
 */
const FINISHING_CALLS: Array<[string, string]> = [
  ["run_angle", "turn by"],
  ["run_target", "turn to"],
  ["run_until_stalled", "until it stops"],
];

const INSTANT_CALLS: Array<[string, string]> = [
  ["run", "start turning"],
  ["stop", "stop"],
];

/** A parallel action is either a motor from the picker, or code of its own. */
const ACTION_KINDS: Array<[string, string]> = [
  ["motor", "Run motor"],
  ["code", "Custom code"],
];

/**
 * Colours to match the page, not CodeMirror's own light default.
 *
 * Only the frame -- background, text, caret, selection. Token colours are
 * CodeMirror's stock `defaultHighlightStyle`, left alone: hand-matching every
 * token class to this palette is more than a first version needs, and stock
 * colours read fine on a dark background as they are.
 */
const codeTheme = EditorView.theme(
  {
    "&": {
      color: "#e6e6e6",
      backgroundColor: "#101214",
      border: "1px solid #333a42",
      borderRadius: "4px",
      fontSize: "13px",
    },
    ".cm-content": { caretColor: "#00e5ff", fontFamily: "inherit", padding: "6px 8px" },
    ".cm-cursor, .cm-dropCursor": { borderLeftColor: "#00e5ff" },
    "&.cm-focused": { outline: "1px solid #00e5ff" },
    ".cm-activeLine": { backgroundColor: "#17191c" },
    ".cm-selectionBackground, &.cm-focused .cm-selectionBackground": {
      backgroundColor: "#2a2e34",
    },
    ".cm-placeholder": { color: "#6b7280", fontStyle: "italic" },
  },
  { dark: true },
);

const SHAPES: Record<
  StepType,
  { title: string; fields: FieldSpec[]; choices?: ChoiceSpec[]; reversible: boolean }
> = {
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
  armStep: {
    title: "Move arm",
    choices: [
      { key: "motor", options: MOTORS },
      { key: "call", options: FINISHING_CALLS },
    ],
    fields: [
      { key: "angle", label: "°", step: 5 },
      { key: "speed", label: "°/s", step: 50 },
    ],
    reversible: false,
  },
};

const NEW_STEP: Record<StepType, () => RunStep> = {
  drive: () => ({ type: "drive", cm: 30 }),
  turn: () => ({ type: "turn", deg: 90 }),
  wait: () => ({ type: "wait", ms: 500 }),
  toPoint: () => ({ type: "toPoint", x: 0, y: 0 }),
  toPose: () => ({ type: "toPose", x: 0, y: 0, head: 0 }),
  armStep: () => ({
    type: "armStep",
    motor: "leftTask",
    call: "run_angle",
    angle: 90,
    speed: 500,
    actions: [{ id: newActionId(), label: "Move arm" }],
  }),
};

/** Actions are bound by id in the generated file, so every one needs its own. */
let actionCounter = 0;

function newActionId(): string {
  actionCounter += 1;

  return `a${Date.now().toString(36)}${actionCounter}`;
}

/** Steps a parallel action can ride along with: the ones that move. */
function canCarryActions(type: StepType): boolean {
  return type !== "armStep" && type !== "wait";
}

export class RunEditor {
  private container: HTMLElement;
  private handlers: RunEditorHandlers;
  private steps: RunStep[];
  private times: BuiltStep[] = [];
  private problems: Diagnostic[] = [];
  private selected: number | null = null;

  // One CodeMirror instance per code action, keyed by the action's own id
  // rather than its position. render() wipes and rebuilds the whole list on
  // every structural change, so without this a fresh EditorView would be
  // created every time -- dropping cursor position and undo history for
  // whoever was mid-edit, even in an action nothing structural touched. See
  // codeBox() and pruneCodeViews().
  private codeViews = new Map<string, EditorView>();

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
      const step = this.steps[index];

      if (took <= 0) {
        // a drive merged into the one above it: there is no separate profile
        label.textContent = index > 0 ? "joined to the step above" : "";
        label.classList.remove("estimate");
        continue;
      }

      // An arm step's length is worked out from the motor's speed and angle,
      // not measured. The robot waits for the motor itself, so the real time
      // may differ -- saying so is better than a confident wrong number.
      const guessed = step?.type === "armStep";

      label.textContent = `${guessed ? "about " : ""}${(took / 1000).toFixed(1)}s`;
      label.classList.toggle("estimate", guessed);
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
    this.pruneCodeViews();
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
    row.className =
      "step" +
      (index === this.selected ? " selected" : "") +
      (step.type === "armStep" ? " arm" : "");
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

    for (const choice of shape.choices ?? []) {
      const chosen = String(step[choice.key] ?? choice.options[0][0]);

      row.append(
        this.picker(choice.options, chosen, (picked) => {
          // Narrowed on the key rather than cast away: the two pickers hold
          // different sets of values, and this is what stops a motor name
          // being written into the call.
          if (choice.key === "motor") {
            this.steps[index].motor = picked as RunStep["motor"];
          } else {
            this.steps[index].call = picked as RunStep["call"];
          }

          this.changed();
        }),
      );
    }

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

    // Parallel actions hang under the move they ride along with. An arm step
    // is its own action, so it does not get any: offering one would be asking
    // the arm to move while the arm is moving.
    if (canCarryActions(step.type)) {
      (step.actions ?? []).forEach((_, which) => {
        row.append(this.renderAction(index, which));
      });

      const add = document.createElement("div");
      add.className = "addaction";
      add.append(
        this.button("+ action while driving", "do something during this move", () =>
          this.addAction(index),
        ),
      );
      row.append(add);
    }

    return row;
  }

  /** One thing that happens while this step is running. */
  private renderAction(index: number, which: number): HTMLElement {
    const action = (this.steps[index].actions ?? [])[which];
    const body: ActionBody = action.do ?? { motor: "leftTask", call: "run", speed: 500 };

    const line = document.createElement("div");
    line.className = "action";

    line.append(document.createTextNode("↳"));

    line.append(
      this.picker(ACTION_KINDS, isCode(body) ? "code" : "motor", (picked) => {
        this.setActionKind(index, which, picked as "motor" | "code");
      }),
    );

    if (isCode(body)) {
      line.append(document.createTextNode("at"));
    } else {
      line.append(
        this.picker(MOTORS, body.motor, (picked) => {
          this.setCommand(index, which, { motor: picked as "leftTask" | "rightTask" });
        }),
        this.picker(INSTANT_CALLS, body.call, (picked) => {
          this.setCommand(index, which, { call: picked as "run" | "stop" });
        }),
      );

      if (body.call !== "stop") {
        line.append(
          this.number(String(body.speed ?? 500), "°/s", 50, (value) => {
            this.setCommand(index, which, { speed: value });
          }),
        );
      }

      // when it happens, as a distance into the move
      line.append(document.createTextNode("at"));
    }

    line.append(
      this.number(String(action.at?.cm ?? 0), "cm in", 1, (value) => {
        const actions = this.steps[index].actions;

        if (actions) {
          actions[which] = { ...actions[which], at: { cm: value } };
          this.changed();
        }
      }),
    );

    line.append(
      this.button("✕", "remove this action", () => this.removeAction(index, which)),
    );

    if (isCode(body)) {
      line.append(this.codeBox(action.id, body.code));
    }

    return line;
  }

  private setCommand(index: number, which: number, change: Record<string, unknown>) {
    const actions = this.steps[index].actions;

    if (!actions) {
      return;
    }

    const existing = actions[which].do;
    const base = existing && !isCode(existing) ? existing : {
      motor: "leftTask" as const,
      call: "run" as const,
      speed: 500,
    };

    actions[which] = { ...actions[which], do: { ...base, ...change } };

    // the speed box appears and disappears with the call, which is structural
    if ("call" in change) {
      this.render();
    }

    this.changed();
  }

  /** Switch a parallel action between the motor picker and free-form code. */
  private setActionKind(index: number, which: number, kind: "motor" | "code") {
    const actions = this.steps[index].actions;

    if (!actions) {
      return;
    }

    const body: ActionBody =
      kind === "code" ? { code: "" } : { motor: "leftTask", call: "run", speed: 500 };

    actions[which] = { ...actions[which], do: body };

    this.render();
    this.changed();
  }

  /**
   * The CodeMirror editor for one code action, kept alive across render().
   *
   * render() wipes and rebuilds the whole step list on every structural
   * change -- adding a step, reordering, even switching a *different*
   * action's kind. A fresh EditorView every time would reset whoever was
   * mid-edit in an unrelated action: cursor gone, undo history gone. Instead
   * each action's view is created once, keyed by its own id, and its DOM
   * node is re-parented into whatever new row render() built around it.
   * pruneCodeViews() destroys the ones whose action no longer exists.
   *
   * The update listener looks the action up by id at the moment it fires,
   * rather than closing over `index`/`which` at creation time -- a step can
   * move (the ↑/↓ buttons) or an earlier action can be removed after this
   * view was created, and either would leave a captured index pointing at
   * the wrong step.
   */
  private codeBox(id: string, code: string): HTMLElement {
    const existing = this.codeViews.get(id);

    if (existing) {
      if (existing.state.doc.toString() !== code) {
        existing.dispatch({
          changes: { from: 0, to: existing.state.doc.length, insert: code },
        });
      }

      return existing.dom;
    }

    const view = new EditorView({
      doc: code,
      extensions: [
        minimalSetup,
        python(),
        codeTheme,
        EditorView.lineWrapping,
        placeholder("core.leftTask.run(500)"),
        keymap.of([indentWithTab]),
        EditorView.updateListener.of((update) => {
          if (!update.docChanged) {
            return;
          }

          const found = this.findAction(id);

          if (!found) {
            return;
          }

          const actions = this.steps[found.index].actions;

          if (actions) {
            actions[found.which] = {
              ...actions[found.which],
              do: { code: update.state.doc.toString() },
            };
          }

          this.changed();
        }),
      ],
    });

    this.codeViews.set(id, view);

    return view.dom;
  }

  /** Where an action lives right now, by its stable id. */
  private findAction(id: string): { index: number; which: number } | null {
    for (let index = 0; index < this.steps.length; index += 1) {
      const which = (this.steps[index].actions ?? []).findIndex((a) => a.id === id);

      if (which !== -1) {
        return { index, which };
      }
    }

    return null;
  }

  /** Destroy the CodeMirror instances for actions that no longer exist. */
  private pruneCodeViews() {
    const alive = new Set<string>();

    for (const step of this.steps) {
      for (const action of step.actions ?? []) {
        if (action.do && isCode(action.do)) {
          alive.add(action.id);
        }
      }
    }

    for (const [id, view] of this.codeViews) {
      if (!alive.has(id)) {
        view.destroy();
        this.codeViews.delete(id);
      }
    }
  }

  private addAction(index: number) {
    const step = this.steps[index];

    step.actions = [
      ...(step.actions ?? []),
      {
        id: newActionId(),
        at: { cm: 0 },
        label: "Action",
        do: { motor: "leftTask", call: "run", speed: 500 },
      },
    ];

    this.render();
    this.changed();
  }

  private removeAction(index: number, which: number) {
    const step = this.steps[index];
    step.actions = (step.actions ?? []).filter((_, at) => at !== which);

    this.render();
    this.changed();
  }

  private picker(
    options: Array<[string, string]>,
    chosen: string,
    onPick: (value: string) => void,
  ): HTMLElement {
    const select = document.createElement("select");

    for (const [value, label] of options) {
      const option = document.createElement("option");
      option.value = value;
      option.textContent = label;
      select.append(option);
    }

    select.value = chosen;
    select.addEventListener("change", () => onPick(select.value));

    return select;
  }

  private number(
    value: string,
    label: string,
    step: number,
    onType: (value: number) => void,
  ): HTMLElement {
    const box = document.createElement("input");

    box.type = "number";
    box.step = String(step);
    box.value = value;
    box.title = label;

    // typing changes the run but must not redraw this list
    box.addEventListener("input", () => {
      const typed = Number(box.value);

      if (Number.isFinite(typed)) {
        onType(typed);
      }
    });

    const wrapper = document.createElement("label");
    wrapper.append(box, document.createTextNode(label));

    return wrapper;
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
