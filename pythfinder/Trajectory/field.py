"""The mat the robot drives on.

Separate from constants.py, which cannot be imported without pygame, so that
the planner can tell whether a path runs off the table.

Axes, which are easy to get backwards: +x runs UP the screen and +y runs to the
RIGHT, with the origin in the middle of the mat. The FLL table is wider than it
is deep, so its 200.5cm is the +y extent and its 114.3cm the +x one.
"""

from pythfinder.Components.BetterClasses.mathEx import Point


class Field():
    def __init__(self,
                 width_cm: float,
                 height_cm: float,
                 name: str = ""):
        """
        Args:
            width_cm: across the screen, along +y.
            height_cm: up the screen, along +x.
            name: what to call it in a message.
        """

        self.WIDTH_CM = width_cm
        self.HEIGHT_CM = height_cm
        self.name = name

    def half_y(self) -> float:
        return self.WIDTH_CM / 2

    def half_x(self) -> float:
        return self.HEIGHT_CM / 2

    def outside_by(self, point: Point) -> float:
        """How far this point lies past the nearest edge, in cm. 0 if on the mat."""
        over_x = abs(point.x) - self.half_x()
        over_y = abs(point.y) - self.half_y()

        return max(0, over_x, over_y)

    def contains(self, point: Point) -> bool:
        return self.outside_by(point) == 0


# no FIRST document states the mat size, so it is measured off the 20cm
# reference grid in the official wireframe PDF. Re-derive with:
#   python tools/build_field_image.py --calibrate
FLL_FIELD = Field(width_cm = 200.5,
                  height_cm = 114.3,   # lands exactly on the familiar 45in
                  name = "FLL table")
