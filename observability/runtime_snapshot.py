"""Incremental display geometry; never used as a collision authority."""
from observability.projection import BodyHeightProjection


class MapDisplayIndex:
    def __init__(self,cells,resolution,planar_z,half_z,offset_z=0):
        self.ground=BodyHeightProjection(cells,resolution,planar_z,half_z,offset_z)
        self.occupied={key for key,cell in cells.items() if cell['state']=='occupied'}
        self._occupied_snapshot=None

    def apply(self,cells,changed_cells):
        self.ground.apply(cells,changed_cells)
        transitions={'new_occupied':0,'cleared_occupied':0}
        for change in changed_cells:
            key=tuple(change['key']);cell=cells.get(key)
            if cell is not None and cell['state']=='occupied':
                if key not in self.occupied:
                    transitions['new_occupied']+=1
                    self._occupied_snapshot=None
                self.occupied.add(key)
            else:
                if key in self.occupied:
                    transitions['cleared_occupied']+=1
                    self._occupied_snapshot=None
                self.occupied.discard(key)
        return transitions

    def snapshot(self):
        """Immutable geometry snapshot; JSON wire arrays remain unchanged.

        Copy only tuple references when membership changes. Retained versions
        are never mutated by apply(), so JSON serialization may run after the
        caller releases its map lock without copying every coordinate list.
        """
        free,blocked=self.ground.immutable_snapshot()
        if self._occupied_snapshot is None:
            self._occupied_snapshot=tuple(self.occupied)
        return free,blocked,self._occupied_snapshot
