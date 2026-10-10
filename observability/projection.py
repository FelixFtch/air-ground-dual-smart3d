"""Uninflated body-height projection for display, not a navigation certificate."""
import math


def body_height_projection(cells,resolution,center_z,half_z,offset_z=0):
    lower=math.floor((center_z+offset_z-half_z)/resolution)
    upper=math.floor((center_z+offset_z+half_z)/resolution)
    blocked=set();free={}
    for key,cell in cells.items():
        if not lower<=key[2]<=upper:continue
        xy=key[:2]
        if cell['state']=='occupied':blocked.add(xy)
        elif cell['state']=='free':free[xy]=free.get(xy,0)+1
    return [list(k) for k,n in free.items() if n==upper-lower+1 and k not in blocked],[list(k) for k in blocked]


class BodyHeightProjection:
    """Incremental equivalent of ``body_height_projection`` for one fixed band.

    Initialize once after creating/restoring a MapCache. Call ``apply`` after
    each MapCache.apply, passing its current ``cells`` and the compact delta's
    ``changed_cells``. Refresh-only timestamp updates cannot change this
    uninflated geometry and need no work. Reinitialize for a replacement cache
    or a changed robot height/profile; this object does not own map identity.

    Only in-band states are retained. Any occupied z cell blocks its XY column;
    a column is free only when *every* band cell is explicitly free. Unknown
    cells remain unknown, including removed cells. Snapshot order is unspecified
    like the original blocked set; each snapshot returns independent lists.
    Caller owns synchronization with MapCache (no internal locks).
    """

    def __init__(self, cells, resolution, planar_z, half_z, offset_z=0):
        if (not all(math.isfinite(v) for v in (resolution, planar_z, half_z, offset_z))
                or resolution <= 0 or half_z < 0):
            raise ValueError("invalid_body_projection_band")
        self.lower = math.floor((planar_z+offset_z-half_z)/resolution)
        self.upper = math.floor((planar_z+offset_z+half_z)/resolution)
        self._height = self.upper-self.lower+1
        self._states = {}
        self._columns = {}
        self._free = set()
        self._blocked = set()
        self._free_snapshot = None
        self._blocked_snapshot = None
        for key, cell in cells.items():
            if self.lower <= key[2] <= self.upper:
                self._update(key, cell["state"])

    def _update(self, key, state):
        previous = self._states.get(key, "unknown")
        if state == previous:
            return
        if state not in ("free", "occupied", "unknown"):
            raise ValueError("invalid_projection_cell_state")
        xy = key[:2]
        was_free, was_blocked = xy in self._free, xy in self._blocked
        counts = self._columns.get(xy)
        if counts is None:
            counts = [0, 0]  # Explicit free and occupied z-cell counts.
            self._columns[xy] = counts
        if previous != "unknown":
            counts[previous == "occupied"] -= 1
        if state != "unknown":
            counts[state == "occupied"] += 1
            self._states[key] = state
        else:
            self._states.pop(key, None)
        if counts[1]:
            self._blocked.add(xy)
        else:
            self._blocked.discard(xy)
        if counts[0] == self._height and not counts[1]:
            self._free.add(xy)
        else:
            self._free.discard(xy)
        if not counts[0] and not counts[1]:
            self._columns.pop(xy, None)
        if was_free != (xy in self._free):
            self._free_snapshot = None
        if was_blocked != (xy in self._blocked):
            self._blocked_snapshot = None

    def apply(self, cells, changed_cells):
        """Update only reported keys, using final canonical cache state.

        Reading final cells also makes repeated keys/duplicate deltas idempotent;
        a removed key is unknown. Do not call this before MapCache.apply.
        """
        for change in changed_cells:
            key = tuple(change["key"])
            if self.lower <= key[2] <= self.upper:
                cell = cells.get(key)
                self._update(key, cell["state"] if cell is not None else "unknown")

    def immutable_snapshot(self):
        """Stable tuple coordinates for frequent display publication.

        Only changed XY membership invalidates a tuple. Earlier snapshots stay
        valid during subsequent apply calls, and consumers cannot mutate them.
        JSON encodes these tuples as the same coordinate arrays as list copies.
        """
        if self._free_snapshot is None:
            self._free_snapshot = tuple(self._free)
        if self._blocked_snapshot is None:
            self._blocked_snapshot = tuple(self._blocked)
        return self._free_snapshot, self._blocked_snapshot

    def snapshot(self):
        """Return independent mutable lists, preserving the original API."""
        free, blocked = self.immutable_snapshot()
        return [list(key) for key in free], [list(key) for key in blocked]
