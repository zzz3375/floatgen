"""Cycle orbit planner — phase-based waypoint generator.

Computes equally-spaced orbit waypoints around a centre point (e.g. wind
turbine) and orders them starting from the waypoint nearest to the drone's
current position.  This lets the drone join the orbit smoothly from its
hovering location instead of always starting at theta=0.

Typical usage from a ROS2 node::

    from cycle_planner import CyclePlanner

    planner = CyclePlanner(
        center_ned=(0.0, 0.0),
        radius=85.0,
        altitude_z=-95.0,
        num_waypoints=12,
    )
    # after takeoff, pass the drone's current NED position:
    planner.reorder_from(drone_ned)
    for wp in planner.waypoints:
        ...  # wp is (north, east, down)
"""

import math
from typing import List, Tuple

Waypoint = Tuple[float, float, float]


class CyclePlanner:
    """Generate orbit waypoints ordered by proximity to a reference position.

    Parameters
    ----------
    center_ned : tuple[float, float]
        (north, east) of the orbit centre in NED.
    radius : float
        Orbit radius in metres.
    altitude_z : float
        NED down-component (negative for above ground).
    num_waypoints : int
        Number of equally-spaced waypoints on the circle.
    """

    def __init__(
        self,
        center_ned: Tuple[float, float],
        radius: float,
        altitude_z: float,
        num_waypoints: int,
    ):
        self.center_ned = center_ned
        self.radius = radius
        self.altitude_z = altitude_z
        self.num_waypoints = num_waypoints
        self.waypoints: List[Waypoint] = []
        self._build(0)

    # ------------------------------------------------------------------
    # public API
    # ------------------------------------------------------------------

    def reorder_from(self, pos_ned: Tuple[float, float]) -> None:
        """Rebuild the waypoint list starting from the *next* point after the
        nearest one to *pos_ned*.

        Call this once after the drone reaches its hovering altitude.  The
        first waypoint in the resulting sequence is the orbit point
        immediately **after** the nearest one, so the drone flies forward
        into the orbit rather than backtracking to a point it has already
        passed.

        Parameters
        ----------
        pos_ned : tuple[float, float]
            (north, east) of the drone's current NED position.
        """
        k = self._nearest_index(pos_ned)
        self._build(k + 1)

    def nearest_index(self, pos_ned: Tuple[float, float]) -> int:
        """Return the index of the nearest waypoint to *pos_ned*.

        Useful for diagnostics or logging; ``reorder_from`` uses this
        internally.
        """
        return self._nearest_index(pos_ned)

    # ------------------------------------------------------------------
    # internals
    # ------------------------------------------------------------------

    def _nearest_index(self, pos_ned: Tuple[float, float]) -> int:
        delta_n = pos_ned[0] - self.center_ned[0]
        delta_e = pos_ned[1] - self.center_ned[1]
        phi = math.atan2(delta_e, delta_n)
        phi = phi % (2.0 * math.pi)
        k = round(phi * self.num_waypoints / (2.0 * math.pi)) % self.num_waypoints
        return k

    def _build(self, start_index: int) -> None:
        self.waypoints = []
        for i in range(self.num_waypoints):
            theta = 2.0 * math.pi * (i + start_index) / self.num_waypoints
            n = self.center_ned[0] + self.radius * math.cos(theta)
            e = self.center_ned[1] + self.radius * math.sin(theta)
            self.waypoints.append((n, e, self.altitude_z))
