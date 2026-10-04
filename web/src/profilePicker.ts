/**
 * A picker for a named, addable, editable thing -- step 5.3. Used twice, once
 * for robot profiles and once for start positions: same shape both times, a
 * `<select>` of names ending in "＋ New…", an "Edit" button for whatever is
 * chosen, and a small inline form under the picker rather than a dialog --
 * fields with their own units, Save and Cancel.
 *
 * This class knows nothing about robots or starts specifically -- `fields`
 * says which numbers (or the one checkbox) the form edits, through get/set
 * closures onto the item, so the same code drives both pickers.
 *
 * It also knows nothing about *why* a selection might not match what is on
 * screen -- that is main.ts's job (a run keeps its own copy, which can drift
 * from the shared one). This class just shows whatever note it is told to,
 * with whatever one-click fix it is given.
 */

export interface PickerField<T> {
  key: string;
  label: string;
  unit?: string;
  step?: number;
  kind?: "number" | "checkbox";
  /** groups fields under a small heading in the form, e.g. "DriveBase" */
  heading?: string;
  /** a short line under the field, for when what it does is not what its
   *  name suggests */
  note?: string;
  get: (item: T) => number | boolean;
  set: (item: T, value: number | boolean) => void;
}

export interface PickerOptions<T extends { name: string }> {
  title: string;
  fields: PickerField<T>[];
  /** cache + built-ins, synchronous -- so the picker has something to show
   *  before any network call finishes */
  list: () => T[];
  /** best-effort merge from the shared store; returns list() afterwards */
  refresh: () => Promise<T[]>;
  /** cache it and try the shared store; true only if the remote write took */
  persist: (item: T) => Promise<boolean>;
  /** a prefilled item for "＋ New…" -- the plan's own pre-fill rule (a new
   *  start from the robot's pose, a new robot from the one selected) lives
   *  in whoever constructs this, not here */
  makeNew: () => T;
  onPick: (item: T) => void;
  log: (message: string) => void;
}

export class ProfilePicker<T extends { name: string }> {
  private options: PickerOptions<T>;
  private select: HTMLSelectElement;
  private editButton: HTMLButtonElement;
  private form: HTMLElement;
  private note: HTMLElement;
  private items: T[];
  private currentName: string | undefined;
  // the run's own copy can name something that is not (or is no longer) in
  // the list at all -- an old run's unnamed RobotNumbers, migrated to a
  // placeholder name, is the case that actually happens; kept here so the
  // select can still show *something* selected rather than silently falling
  // back to blank
  private unlisted: T | null = null;

  constructor(container: HTMLElement, options: PickerOptions<T>) {
    this.options = options;
    this.items = options.list();

    const row = document.createElement("div");
    row.className = "picker";

    const label = document.createElement("span");
    label.className = "pickerlabel";
    label.textContent = options.title;
    row.append(label);

    this.select = document.createElement("select");
    this.select.addEventListener("change", () => this.onSelectChange());
    row.append(this.select);

    this.editButton = document.createElement("button");
    this.editButton.type = "button";
    this.editButton.textContent = "Edit";
    this.editButton.hidden = true;
    this.editButton.addEventListener("click", () => this.onEditClick());
    row.append(this.editButton);

    this.form = document.createElement("div");
    this.form.className = "pickerform";
    this.form.hidden = true;

    this.note = document.createElement("p");
    this.note.className = "pickernote";
    this.note.hidden = true;

    container.append(row, this.note, this.form);
    this.renderOptions();
  }

  /** What the run currently has picked, so the select reflects it and Edit
   *  knows what to edit. Call this after every rebuild -- picking a new
   *  start or dragging the robot both change it. */
  setSelected(item: T | undefined) {
    this.currentName = item?.name;
    this.unlisted = item && !this.items.some((known) => known.name === item.name) ? item : null;
    this.renderOptions();
    this.editButton.hidden = item === undefined;
  }

  /** "Differs from the current numbers", whether from a drag or from someone
   *  editing the shared profile elsewhere -- step 5.3 treats both the same
   *  way, since the fix is identical either way: copy the current numbers
   *  in. `onUse === null` when there is nothing to copy from (nothing
   *  selected at all). */
  showNote(text: string | null, onUse: (() => void) | null) {
    this.note.hidden = text === null;
    this.note.replaceChildren();

    if (text === null) {
      return;
    }

    this.note.append(document.createTextNode(text + " "));

    if (onUse !== null) {
      const button = document.createElement("button");
      button.type = "button";
      button.textContent = "use the current numbers";
      button.addEventListener("click", onUse);
      this.note.append(button);
    }
  }

  /** Pull in whatever the shared store now has, best-effort. */
  async refresh() {
    this.items = await this.options.refresh();
    this.renderOptions();
  }

  private renderOptions() {
    const previous = this.select.value;
    this.select.replaceChildren();

    const blank = document.createElement("option");
    blank.value = "";
    blank.textContent = `— pick a ${this.options.title.toLowerCase()} —`;
    this.select.append(blank);

    if (this.unlisted !== null) {
      const option = document.createElement("option");
      option.value = this.unlisted.name;
      option.textContent = `${this.unlisted.name} (not in the list)`;
      this.select.append(option);
    }

    for (const item of this.items) {
      const option = document.createElement("option");
      option.value = item.name;
      option.textContent = item.name;
      this.select.append(option);
    }

    const adder = document.createElement("option");
    adder.value = "__new__";
    adder.textContent = "＋ New…";
    this.select.append(adder);

    this.select.value = this.currentName ?? previous;
  }

  private findByName(name: string): T | undefined {
    if (this.unlisted?.name === name) {
      return this.unlisted;
    }

    return this.items.find((item) => item.name === name);
  }

  private onSelectChange() {
    const value = this.select.value;

    if (value === "__new__") {
      this.openForm("new", this.options.makeNew());
      // don't leave "＋ New…" showing as if it were a real selection
      this.select.value = this.currentName ?? "";
      return;
    }

    const picked = this.findByName(value);

    if (picked !== undefined) {
      this.options.onPick(picked);
    }
  }

  private onEditClick() {
    if (this.currentName === undefined) {
      return;
    }

    const current = this.findByName(this.currentName);

    if (current !== undefined) {
      this.openForm("edit", current);
    }
  }

  /** New starts pre-fill from the robot's current pose, new robots from the
   *  one selected -- both decided by whoever passes `base` in, not here. */
  private openForm(mode: "new" | "edit", base: T) {
    this.form.hidden = false;
    this.form.replaceChildren();

    const nameBox = document.createElement("input");
    nameBox.type = "text";
    nameBox.value = mode === "new" ? "" : base.name;
    nameBox.placeholder = "name";
    this.form.append(this.labelled("name", nameBox));

    const boxes = new Map<string, HTMLInputElement>();
    let lastHeading: string | undefined;

    for (const field of this.options.fields) {
      if (field.heading !== undefined && field.heading !== lastHeading) {
        lastHeading = field.heading;

        const heading = document.createElement("p");
        heading.className = "pickerheading";
        heading.textContent = field.heading;
        this.form.append(heading);
      }

      const box = document.createElement("input");

      if (field.kind === "checkbox") {
        box.type = "checkbox";
        box.checked = Boolean(field.get(base));
      } else {
        box.type = "number";
        box.step = String(field.step ?? 1);
        box.value = String(field.get(base));
      }

      boxes.set(field.key, box);

      const withUnit = field.unit === undefined ? field.label : `${field.label} (${field.unit})`;
      this.form.append(this.labelled(withUnit, box));

      if (field.note !== undefined) {
        const note = document.createElement("p");
        note.className = "pickerfieldnote";
        note.textContent = field.note;
        this.form.append(note);
      }
    }

    const save = document.createElement("button");
    save.type = "button";
    save.textContent = "Save";
    save.addEventListener("click", () => void this.saveForm(base, nameBox, boxes));

    const cancel = document.createElement("button");
    cancel.type = "button";
    cancel.textContent = "Cancel";
    cancel.addEventListener("click", () => this.closeForm());

    const buttons = document.createElement("div");
    buttons.className = "pickerformbuttons";
    buttons.append(save, cancel);
    this.form.append(buttons);
  }

  private labelled(text: string, box: HTMLInputElement): HTMLElement {
    const label = document.createElement("label");
    label.append(box, document.createTextNode(text));

    return label;
  }

  private async saveForm(
    base: T,
    nameBox: HTMLInputElement,
    boxes: Map<string, HTMLInputElement>,
  ) {
    const name = nameBox.value.trim();

    if (name === "") {
      this.options.log("give it a name before saving");
      return;
    }

    // a deep copy, not a patch onto `base` -- `base` may be the very object
    // main.ts is currently using for the run (the selected profile, or the
    // robot's live pose), and field.set below writes into nested objects
    // (planning/driveBase); mutating those in place would change what is on
    // screen before Save is even pressed
    const item = JSON.parse(JSON.stringify(base)) as T;
    item.name = name;

    for (const field of this.options.fields) {
      const box = boxes.get(field.key);

      if (box === undefined) {
        continue;
      }

      if (field.kind === "checkbox") {
        field.set(item, box.checked);
        continue;
      }

      const value = Number(box.value);

      if (Number.isFinite(value)) {
        field.set(item, value);
      }
    }

    const remoteOk = await this.options.persist(item);

    this.items = this.options.list();
    this.currentName = item.name;
    this.unlisted = null;
    this.editButton.hidden = false;
    this.renderOptions();
    this.closeForm();

    // picking up the saved item is what makes "New" and "Edit" both act as
    // "and now use this one" -- a mentor editing "Team robot" expects the
    // form they are looking at to be the numbers now in effect
    this.options.onPick(item);

    this.options.log(
      remoteOk
        ? `saved "${item.name}" for the team`
        : `could not reach the shared store -- "${item.name}" is only saved in this browser`,
    );
  }

  private closeForm() {
    this.form.hidden = true;
    this.form.replaceChildren();
  }
}
