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
  robotCorners,
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
  /** step 4.1: a Go-to step's own target was dragged to a new field point */
  onWaypointChange?: (index: number, point: FieldPoint) => void;
}

/** A draggable target, step 4.1: one per toPoint/toPose step in the run. */
export interface Waypoint {
  /** which described step this is, so a drag can be written back to it */
  index: number;
  point: FieldPoint;
  /** true when the step list has this one selected -- drawn to match */
  selected: boolean;
}

const TURN_STEP_DEG = 5;
const FINE_TURN_STEP_DEG = 1;

/** How close a click has to land, in field cm, to grab a waypoint rather
 *  than pass through to whatever is under it. Generous: these are the
 *  smallest thing on the mat to aim for, and a finger is not a mouse. */
const WAYPOINT_HIT_CM = 6;
const WAYPOINT_RADIUS_CM = 2.2;

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
  private playhead: FieldPose | null = null;
  private offMat: Array<[number, number]> = [];

  // step 4.1
  private waypoints: Waypoint[] = [];
  private draggingWaypoint: number | null = null;

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
    this.offMat = this.findOffMat();
    this.draw();
  }

  /** Light up one stretch of the path, in trajectory milliseconds. */
  setHighlight(range: { from: number; to: number } | null) {
    this.highlight = range;
    this.draw();
  }

  /** Stand the robot part-way through the run, or back at the start pose. */
  setPlayhead(pose: FieldPose | null) {
    this.playhead = pose;
    this.draw();
  }

  /**
   * The Go-to steps' own targets, step 4.1. Fed from the step list, not
   * computed here -- the field draws them, the app owns what they mean.
   *
   * The app is expected to call this straight back from onWaypointChange
   * (through the step list, so both stay the single source of truth), which
   * happens synchronously within the same pointer event -- there is no async
   * gap here for a dragged point to go stale in.
   */
  setWaypoints(waypoints: Waypoint[]) {
    this.waypoints = waypoints;
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

    // After the robot, deliberately -- twice already logged in this project
    // as the mistake of drawing a marker before the thing that can cover it.
    // A Go-to step's own target commonly lands right where the robot starts
    // or ends (toPose back to the launch pose is the commonest last step), so
    // drawn first a waypoint would be invisible, and worse, unclickable,
    // exactly there.
    this.drawWaypoints();

    // The commonest way to hang off the mat is to turn on the spot at the
    // edge, which puts the mark exactly where the robot is standing.
    this.strokeOffMat(Math.max(1.5, this.view.scale * 0.22));
  }

  /** The path: the whole run, the selected step, and any trouble. */
  private drawPath() {
    if (this.path.length < 2) {
      return;
    }

    const thin = Math.max(1.5, this.view.scale * 0.22);

    this.strokeRange(0, this.path.length - 1, "#5b2ea6", thin);
    this.strokeHighlight(thin);
  }

  private strokeRange(from: number, to: number, colour: string, width: number) {
    const { context, view } = this;

    context.strokeStyle = colour;
    context.fillStyle = colour;
    context.lineWidth = width;
    context.lineJoin = "round";
    context.lineCap = "round";

    // a single point has no line to draw, and a zero-length stroke paints
    // nothing at all
    if (to <= from) {
      const point = fieldToScreen(this.path[from], view);

      context.beginPath();
      context.arc(point.x, point.y, width / 2, 0, Math.PI * 2);
      context.fill();

      return;
    }

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
  }

  private strokeHighlight(thin: number) {
    if (this.highlight === null) {
      return;
    }

    const { context, view } = this;
    const first = this.indexAt(this.highlight.from);
    const last = this.indexAt(this.highlight.to);

    if (last > first && this.spanCm(first, last) >= 1) {
      this.strokeRange(first, last, "#ff6d00", thin * 2.2);
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

  /** Paint the stretches where a corner of the robot is off the mat. */
  private strokeOffMat(thin: number) {
    const { context, view } = this;

    for (const [from, to] of this.offMat) {
      // Turning on the spot at the edge is the commonest way to hang off the
      // mat, and it covers no ground: as a line it is a couple of pixels wide,
      // hidden under the robot. Ring the spot instead.
      if (this.spanCm(from, to) < 1) {
        const point = fieldToScreen(this.path[from], view);

        // big enough to ring the robot rather than hide inside it: the robot
        // is about 19cm across, so this clears it
        const radius = (ROBOT.widthCm * 0.8) * view.scale;

        context.strokeStyle = "#ff1744";
        context.lineWidth = Math.max(2.5, thin * 1.2);
        context.beginPath();
        context.arc(point.x, point.y, radius, 0, Math.PI * 2);
        context.stroke();

        continue;
      }

      this.strokeRange(from, to, "#ff1744", thin * 1.3);
    }
  }

  /**
   * Which stretches of the path hang off the mat.
   *
   * Worked out here rather than asked for, because the worker's warning says
   * when the worst moment is, not how long the trouble lasts. The geometry is
   * the same either way: the four corners of the robot, turned to face the way
   * it is going.
   *
   * Done once per build. Playback redraws sixty times a second, and this walks
   * every pose.
   */
  private findOffMat(): Array<[number, number]> {
    const ranges: Array<[number, number]> = [];
    let from: number | null = null;

    for (let i = 0; i < this.path.length; i++) {
      const off = robotCorners(this.path[i]).some(
        (corner) =>
          Math.abs(corner.x) > FIELD.heightCm / 2 ||
          Math.abs(corner.y) > FIELD.widthCm / 2,
      );

      if (off && from === null) {
        from = i;
      } else if (!off && from !== null) {
        ranges.push([from, i - 1]);
        from = null;
      }
    }

    if (from !== null) {
      ranges.push([from, this.path.length - 1]);
    }

    return ranges;
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
    // Part-way through a run, the robot stands at the playhead and the start
    // pose is left as an outline -- it is still the thing you drag, and
    // without it there is no telling where the run begins.
    if (this.playhead === null) {
      this.drawRobotAt(this.pose);
      return;
    }

    this.drawStartOutline();
    this.drawRobotAt(this.playhead);
  }

  private drawRobotAt(pose: FieldPose) {
    const { context, view } = this;
    const middle = fieldToScreen(pose, view);

    const length = ROBOT.lengthCm * view.scale;
    const width = ROBOT.widthCm * view.scale;

    context.save();
    context.translate(middle.x, middle.y);
    context.rotate(spriteRotation(pose.head));
    context.drawImage(this.robot, -length / 2, -width / 2, length, width);
    context.restore();

    // the nose, so which way it faces is obvious at a glance
    const nose = fieldToScreen(
      {
        x: pose.x + Math.cos((pose.head * Math.PI) / 180) * ROBOT.lengthCm,
        y: pose.y + Math.sin((pose.head * Math.PI) / 180) * ROBOT.lengthCm,
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

  private drawStartOutline() {
    const { context, view } = this;
    const corners = robotCorners(this.pose).map((corner) =>
      fieldToScreen(corner, view),
    );

    // robotCorners gives front-left, front-right, back-left, back-right, so
    // walk them 0, 1, 3, 2 to go round the outside
    const around = [corners[0], corners[1], corners[3], corners[2]];

    context.strokeStyle = "#00e5ff80";
    context.lineWidth = Math.max(1, view.scale * 0.15);
    context.beginPath();
    around.forEach((corner, i) => {
      if (i === 0) {
        context.moveTo(corner.x, corner.y);
      } else {
        context.lineTo(corner.x, corner.y);
      }
    });
    context.closePath();
    context.stroke();
  }

  /** Step 4.1: one draggable handle per Go-to step, numbered to match the
   *  row it belongs to -- the same number the step list shows on the left. */
  private drawWaypoints() {
    const { context, view } = this;
    const radius = Math.max(4, WAYPOINT_RADIUS_CM * view.scale);

    for (const waypoint of this.waypoints) {
      const at = fieldToScreen(waypoint.point, view);
      const colour = waypoint.selected ? "#ff6d00" : "#00e5ff";

      context.fillStyle = colour;
      context.beginPath();
      context.arc(at.x, at.y, radius, 0, Math.PI * 2);
      context.fill();

      context.strokeStyle = "#101214";
      context.lineWidth = Math.max(1, view.scale * 0.12);
      context.stroke();

      context.fillStyle = "#101214";
      context.font = `${Math.max(9, radius * 1.1)}px ui-monospace, monospace`;
      context.textAlign = "center";
      context.textBaseline = "middle";
      context.fillText(String(waypoint.index + 1), at.x, at.y + 0.5);
    }
  }

  /** The nearest waypoint within grabbing distance of a click, or null. */
  private waypointAt(point: FieldPoint): number | null {
    let best: { index: number; distance: number } | null = null;

    for (const waypoint of this.waypoints) {
      const distance = Math.hypot(
        point.x - waypoint.point.x,
        point.y - waypoint.point.y,
      );

      if (distance <= WAYPOINT_HIT_CM && (best === null || distance < best.distance)) {
        best = { index: waypoint.index, distance };
      }
    }

    return best === null ? null : best.index;
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

    // Waypoints first: they are small, precise targets, and one landing
    // inside the robot's own footprint -- a toPose step back to the launch
    // pose is the commonest last step in a run -- must still be reachable.
    // Missing a waypoint by a few cm still finds the much bigger robot
    // underneath it, so checking robot-first would make that common case
    // unreachable rather than merely making the rare overlap ambiguous.
    const waypoint = this.waypointAt(point);

    if (waypoint !== null) {
      this.draggingWaypoint = waypoint;
      this.canvas.setPointerCapture(event.pointerId);
      this.canvas.focus();
      return;
    }

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

    if (this.draggingWaypoint !== null) {
      const { x, y } = clampToField(point);
      const moved = { x: round(x), y: round(y) };

      // held locally too, not just handed to the app, so the marker tracks
      // the pointer even if whatever is listening does not echo it straight
      // back -- the same reason onPoseChange below does not wait either
      this.waypoints = this.waypoints.map((w) =>
        w.index === this.draggingWaypoint ? { ...w, point: moved } : w,
      );
      this.draw();

      this.handlers.onWaypointChange?.(this.draggingWaypoint, moved);
      return;
    }

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

    if (this.draggingWaypoint !== null) {
      this.draggingWaypoint = null;
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
