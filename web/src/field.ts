/**
 * Where things are on the mat, and where that lands on the screen.
 *
 * The convention comes from the simulator (Components/robot.py:106-114) and is
 * easy to get backwards:
 *
 *     +x runs UP the screen, +y runs to the RIGHT, and the origin is the
 *     middle of the mat. A heading of 0 faces up, 90 faces right.
 *
 * So heading grows clockwise as drawn, which is why the canvas rotation below
 * is +head rather than -head.
 */

/** The FLL table: 200.5cm across (+y), 114.3cm deep (+x). */
export const FIELD = {
  widthCm: 200.5,
  heightCm: 114.3,
};

/** The team's robot, as measured on the build. */
export const ROBOT = {
  lengthCm: 14, // front to back, along the way it faces
  widthCm: 19, // side to side, across the wheels
};

export interface FieldPose {
  x: number;
  y: number;
  head: number;
}

export interface FieldPoint {
  x: number;
  y: number;
}

export interface ScreenPoint {
  x: number;
  y: number;
}

/** How the mat is laid onto a canvas: pixels per cm, and where the middle is. */
export interface Viewport {
  scale: number;
  middleX: number;
  middleY: number;
}

/** Fit the whole mat into a canvas, leaving a margin, keeping it square-on. */
export function fitViewport(
  canvasWidth: number,
  canvasHeight: number,
  marginPx = 12,
): Viewport {
  const usableWidth = Math.max(1, canvasWidth - marginPx * 2);
  const usableHeight = Math.max(1, canvasHeight - marginPx * 2);

  return {
    scale: Math.min(usableWidth / FIELD.widthCm, usableHeight / FIELD.heightCm),
    middleX: canvasWidth / 2,
    middleY: canvasHeight / 2,
  };
}

export function fieldToScreen(point: FieldPoint, view: Viewport): ScreenPoint {
  return {
    x: view.middleX + point.y * view.scale,
    y: view.middleY - point.x * view.scale,
  };
}

export function screenToField(point: ScreenPoint, view: Viewport): FieldPoint {
  return {
    x: (view.middleY - point.y) / view.scale,
    y: (point.x - view.middleX) / view.scale,
  };
}

/** The rotation to give a canvas so that drawing "up" means "the way it faces".
 *
 * The extra quarter turn is the sprite: the photo has the robot's front
 * pointing at the LEFT edge of the image, which is what the simulator's
 * scaling convention expects.
 */
export function spriteRotation(head: number): number {
  return ((head + 90) * Math.PI) / 180;
}

export function normaliseHead(head: number): number {
  const turned = head % 360;

  return turned < 0 ? turned + 360 : turned;
}

/** Keep a point on the mat, allowing for the robot hanging over its middle. */
export function clampToField(point: FieldPoint, marginCm = 0): FieldPoint {
  const halfX = FIELD.heightCm / 2 - marginCm;
  const halfY = FIELD.widthCm / 2 - marginCm;

  return {
    x: Math.min(halfX, Math.max(-halfX, point.x)),
    y: Math.min(halfY, Math.max(-halfY, point.y)),
  };
}

/** The four corners of the robot, in field cm. Useful for hit tests. */
export function robotCorners(pose: FieldPose): FieldPoint[] {
  const radians = (pose.head * Math.PI) / 180;
  const cos = Math.cos(radians);
  const sin = Math.sin(radians);

  const halfLength = ROBOT.lengthCm / 2;
  const halfWidth = ROBOT.widthCm / 2;

  const corners: FieldPoint[] = [];

  for (const forward of [halfLength, -halfLength]) {
    for (const left of [halfWidth, -halfWidth]) {
      corners.push({
        x: pose.x + forward * cos - left * sin,
        y: pose.y + forward * sin + left * cos,
      });
    }
  }

  return corners;
}

/** Is this point inside the robot? Tested in the robot's own frame. */
export function insideRobot(point: FieldPoint, pose: FieldPose): boolean {
  const radians = (pose.head * Math.PI) / 180;
  const dx = point.x - pose.x;
  const dy = point.y - pose.y;

  // rotate the offset back, so the robot is square again
  const forward = dx * Math.cos(radians) + dy * Math.sin(radians);
  const left = -dx * Math.sin(radians) + dy * Math.cos(radians);

  return (
    Math.abs(forward) <= ROBOT.lengthCm / 2 && Math.abs(left) <= ROBOT.widthCm / 2
  );
}
