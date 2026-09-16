# fusion/core_modules/__init__.py
"""
Modulární doménové komponenty pro jádro fúze:
- geo_math: geometrické, úhlové a kinematické funkce
- odometry: OdometryTracker
- constellation: ConstellationManager, AntennaState
- heading: HeadingManager, HeadingData
"""

from .geo_math import (
    norm_deg,
    angle_diff,
    circular_mean,
    geo_dist,
    compute_tangent_offset
)
from .odometry import (
    OdometryTracker
)
from .constellation import (
    AntennaState,
    ConstellationManager
)
from .heading import (
    HeadingData,
    HeadingManager
)
from .position import (
    PositionDeadReckoningTracker
)

__all__ = [
    "norm_deg",
    "angle_diff",
    "circular_mean",
    "geo_dist",
    "compute_tangent_offset",
    "OdometryTracker",
    "AntennaState",
    "ConstellationManager",
    "HeadingData",
    "HeadingManager",
    "PositionDeadReckoningTracker"
]
