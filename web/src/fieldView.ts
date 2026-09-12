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

    this.drawRobot();
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
