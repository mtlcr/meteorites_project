
FIELD_X_BOUNDS = (-1.0, 1.0)
FIELD_Y_BOUNDS = (-1.0, 1.0)


class Arena(object):
    """The bounds in which Aliens will fall during the simulation."""

    def __init__(self):
        """Initialize the Arena."""
        self.x_bounds = FIELD_X_BOUNDS
        self.y_bounds = FIELD_Y_BOUNDS

    def contains(self, point_xy):
        """Return True if point_xy is within this Arena, false otherwise."""
        return ((self.x_bounds[0] <= point_xy[0] <= self.x_bounds[1])
                and (self.y_bounds[0] <= point_xy[1] <= self.y_bounds[1]))

    def __repr__(self):
        """How the Arena is represented when printed out to the cli."""
        return f'(({self.x_bounds[0]}, {self.y_bounds[0]}), ({self.x_bounds[1]}, {self.y_bounds[1]}))'

    @property
    def bounds(self):
        """Return the bounds of the Arena."""
        return {'x': self.x_bounds,
                'y': self.y_bounds}
