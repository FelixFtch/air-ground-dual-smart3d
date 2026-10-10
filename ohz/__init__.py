"""People hazard shapes that prune the SMART tree.

The live shape is the ellipse/ellipsoid built in ``perception/hazards.py``.
A forward floor-shadow rectangle can replace that shape later without moving
RRT* or the local reaction zone. Both stay in ``planning/``.
"""
from perception.hazards import build_hazards, check_hazard_segment, validate_hazard

__all__ = ["build_hazards", "check_hazard_segment", "validate_hazard"]
