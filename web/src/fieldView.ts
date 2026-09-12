/**
 * The mat, the robot, and dragging the robot around.
 *
 * Step 2.3: everything here is about where the run starts. The path and the
 * playback arrive in 2.5, and the step list in 2.4.
 */

import {
  FIELD,
  ROBOT,
  clampToField,
  fieldToScreen,
  fitViewport,
  insideRobot,
  normaliseHead,
  screenToField,
  spriteRotation,
  type FieldPoint,
  type FieldPose,
  type Viewport,
} from "./field";
import type { PathPose } from "./types";

export interface FieldViewHandlers {
  /** the start pose changed, because somebody dragged or turned the robot */
  onPoseChange?: (pose: FieldPose) => void;
  /** where the mouse is, in field cm, or null once it leaves */
  onHover?: (point: FieldPoint | null) => void;
}

const TURN_STEP_DEG = 5;
const FINE_TURN_STEP_DEG = 1;

export class FieldView {
  private canvas: HTMLCanvasElement;
  private context: CanvasRenderingContext2D;
  private mat: HTMLImageElement;
  private robot: HTMLImageElement;
  private handlers: FieldViewHandlers;

  private pose: FieldPose;
  private view: Viewport;
  private dragging = false;
  private path: PathPose[] = [];
  private highlight: { from: number; to: number } | null = null;

  constructor(
    canvas: HTMLCanvasElement,
    images: { mat: HTMLImageElement; robot: HTMLImageElement },
    startPose: FieldPose,
    handlers: FieldViewHandlers = {},
  ) {
    this.canvas = canvas;
    this.context = canvas.getContext("2d")!;
    this.mat = images.mat;
    this.robot = images.robot;
    this.handlers = handlers;
    this.pose = { ...startPose };
    this.view = fitViewport(canvas.width, canvas.height);

    canvas.tabIndex = 0; // so it can take the arrow keys
    canvas.addEventListener("pointerdown", this.onPointerDown);
    canvas.addEventListener("pointermove", this.onPointerMove);
    canvas.addEventListener("pointerup", this.onPointerUp);
    canvas.addEventListener("pointerleave", this.onPointerLeave);
    canvas.addEventListener("keydown", this.onKeyDown);

    this.resize();
  }

  getPose(): FieldPose {
    return { ...this.pose };
  }

  setPose(pose: FieldPose) {
    this.pose = { ...pose };
    this.draw();
  }

  /** The path the robot will take, as the worker last worked it out. */
  setPath(path: PathPose[]) {
    this.path = path;
    this.draw();
  }

  /** Light up one stretch of the path, in trajectory milliseconds. */
  setHighlight(range: { from: number; to: number } | null) {
    this.highlight = range;
    this.draw();
  }

  /** Match the canvas to the space it has been given, and to the screen's dots. */
  resize() {
    const box = this.canvas.getBoundingClientRect();
    const dots = window.devicePixelRatio || 1;

    this.canvas.width = Math.round(box.width * dots);
    this.canvas.height = Math.round(box.height * dots);

    this.view = fitViewport(this.canvas.width, this.canvas.height, 12 * dots);
    this.draw();
  }

  draw() {
    const { context, view } = this;

    context.clearRect(0, 0, this.canvas.width, this.canvas.height);

    const topLeft = fieldToScreen(
      { x: FIELD.heightCm / 2, y: -FIELD.widthCm / 2 },
      view,
    );

    context.drawImage(
      this.mat,
      topLeft.x,
      topLeft.y,
      FIELD.widthCm * view.scale,
      FIELD.heightCm * view.scale,
    );

    this.drawPath();
    this.drawRobot();
  }

  /** The whole path, with the selected step's share of it picked out. */
  private drawPath() {
    if (this.path.length < 2) {
      return;
    }

    const { context, view } = this;

    const stroke = (from: number, to: number, colour: string, width: number) => {
      context.strokeStyle = colour;
      context.lineWidth = width;
      context.lineJoin = "round";
      context.lineCap = "round";
      context.beginPath();

      for (let i = from; i <= to; i++) {
        const point = fieldToScreen(this.path[i], view);

        if (i === from) {
          context.moveTo(point.x, point.y);
        } else {
          context.lineTo(point.x, point.y);
        }
      }

      context.stroke();
    };

    const thin = Math.max(1.5, view.scale * 0.22);
    stroke(0, this.path.length - 1, "#5b2ea6", thin);

    if (this.highlight === null) {
      return;
    }

    const first = this.indexAt(this.highlight.from);
    const last = this.indexAt(this.highlight.to);

    if (last > first && this.spanCm(first, last) >= 1) {
      stroke(first, last, "#ff6d00", thin * 2.2);
      return;
    }

    // Nothing to light up along the ground: either a turn on the spot, which
    // is most of the turns in a run, or a step the builder merged into the one
    // before it. Mark where it happens instead, or selecting it shows nothing.
    const point = fieldToScreen(this.path[first], view);
    const radius = Math.max(4, thin * 2.5);

    context.fillStyle = "#ff6d00";
    context.beginPath();
    context.arc(point.x, point.y, radius, 0, Math.PI * 2);
    context.fill();

    context.strokeStyle = "#ffffff";
    context.lineWidth = Math.max(1, thin * 0.5);
    context.stroke();
  }

  /** How far the path gets from where the stretch started, in cm. */
  private spanCm(from: number, to: number): number {
    const start = this.path[from];
    let far = 0;

    for (let i = from; i <= to; i++) {
      far = Math.max(
        far,
        Math.hypot(this.path[i].x - start.x, this.path[i].y - start.y),
      );
    }

    return far;
  }

  /** The point on the path nearest a moment in the run. */
  private indexAt(ms: number): number {
    let best = 0;

    for (let i = 0; i < this.path.length; i++) {
      if (this.path[i].t <= ms) {
        best = i;
      } else {
        break;
      }
    }

    return best;
  }

  private drawRobot() {
    const { context, view } = this;
    const middle = fieldToScreen(this.pose, view);

    const length = ROBOT.lengthCm * view.scale;
    const width = ROBOT.widthCm * view.scale;

    context.save();
    context.translate(middle.x, middle.y);
    context.rotate(spriteRotation(this.pose.head));
    context.drawImage(this.robot, -length / 2, -width / 2, length, width);
    context.restore();

    // the nose, so which way it faces is obvious at a glance
    const nose = fieldToScreen(
      {
        x: this.pose.x + Math.cos((this.pose.head * Math.PI) / 180) * ROBOT.lengthCm,
        y: this.pose.y + Math.sin((this.pose.head * Math.PI) / 180) * ROBOT.lengthCm,
      },
      view,
    );

    context.strokeStyle = "#00e5ff";
    context.lineWidth = Math.max(2, view.scale * 0.3);
    context.beginPath();
    context.moveTo(middle.x, middle.y);
    context.lineTo(nose.x, nose.y);
    context.stroke();
  }

  private pointerField(event: PointerEvent): FieldPoint {
    const box = this.canvas.getBoundingClientRect();
    const dots = window.devicePixelRatio || 1;

    return screenToField(
      {
        x: (event.clientX - box.left) * dots,
        y: (event.clientY - box.top) * dots,
      },
      this.view,
    );
  }

  private onPointerDown = (event: PointerEvent) => {
    const point = this.pointerField(event);

    if (!insideRobot(point, this.pose)) {
      return;
    }

    this.dragging = true;
    this.canvas.setPointerCapture(event.pointerId);
    this.canvas.focus();
  };

  private onPointerMove = (event: PointerEvent) => {
    const point = this.pointerField(event);
    this.handlers.onHover?.(point);

    if (!this.dragging) {
      return;
    }

    // half the robot's diagonal, so it cannot be dragged off the mat entirely
    const reach = Math.hypot(ROBOT.lengthCm, ROBOT.widthCm) / 2;
    const { x, y } = clampToField(point, -reach);

    this.pose = { ...this.pose, x: round(x), y: round(y) };
    this.draw();
    this.handlers.onPoseChange?.(this.getPose());
  };

  private onPointerUp = (event: PointerEvent) => {
    if (this.dragging) {
      this.dragging = false;
      this.canvas.releasePointerCapture(event.pointerId);
    }
  };

  private onPointerLeave = () => {
    this.handlers.onHover?.(null);
  };

  private onKeyDown = (event: KeyboardEvent) => {
    const step = event.shiftKey ? FINE_TURN_STEP_DEG : TURN_STEP_DEG;
    let turn = 0;

    if (event.key === "ArrowLeft") {
      turn = -step; // anticlockwise as drawn
    } else if (event.key === "ArrowRight") {
      turn = step;
    } else {
      return;
    }

    event.preventDefault();

    this.pose = {
      ...this.pose,
      head: round(normaliseHead(this.pose.head + turn)),
    };

    this.draw();
    this.handlers.onPoseChange?.(this.getPose());
  };
}

function round(value: number): number {
  return Math.round(value * 10) / 10;
}
