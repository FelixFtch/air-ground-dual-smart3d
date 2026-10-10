use std::collections::HashMap;

use crate::obstacles::rectangular_obstacle::StaticRectangularObstacle;
use crate::obstacles::RectangularObstacle;
use crate::rrt::state::RealVectorState;
use crate::rrt::validity_checker::ValidityChecker;
use num_traits::Float;

/// A spatial-indexed validity checker for axis-aligned rectangular obstacles.
///
/// Divides space into a uniform grid. Each obstacle is registered in all grid
/// cells its AABB covers. Queries only check obstacles in relevant cells.
///
/// **Mathematically equivalent** to `AnalyticValidityChecker` — identical
/// collision results. Zero heap allocation per query.
pub struct SpatialRectValidityChecker<F: Float, const N: usize> {
    obstacles: Vec<StaticRectangularObstacle<F, N>>,
    cell_size_inv: F, // 1.0 / cell_size, precomputed to avoid division
    grid: HashMap<[i32; N], Vec<usize>>,
}

impl<F: Float, const N: usize> SpatialRectValidityChecker<F, N> {
    pub fn new(obstacles: Vec<StaticRectangularObstacle<F, N>>, cell_size: F) -> Self {
        let cell_size_inv = F::one() / cell_size;
        let mut grid: HashMap<[i32; N], Vec<usize>> = HashMap::new();

        for (idx, obs) in obstacles.iter().enumerate() {
            let min_cells = Self::to_cell(obs.min_corner(), cell_size_inv);
            let max_cells = Self::to_cell(obs.max_corner(), cell_size_inv);
            Self::for_each_cell(&min_cells, &max_cells, |cell| {
                grid.entry(cell).or_default().push(idx);
            });
        }

        Self {
            obstacles,
            cell_size_inv,
            grid,
        }
    }

    #[inline(always)]
    fn coord_to_cell(v: F, inv: F) -> i32 {
        // Use to_f64 for the floor→i32 conversion (avoids needing ToPrimitive on F)
        (v.to_f64().unwrap() * inv.to_f64().unwrap()).floor() as i32
    }

    #[inline]
    fn to_cell(state: &RealVectorState<F, N>, inv: F) -> [i32; N] {
        std::array::from_fn(|i| Self::coord_to_cell(state[i], inv))
    }

    fn for_each_cell(min_cells: &[i32; N], max_cells: &[i32; N], mut cb: impl FnMut([i32; N])) {
        // For N=2 (the common case), this is just a nested loop over ~1-4 cells
        let mut cur: [i32; N] = *min_cells;
        loop {
            cb(cur);
            let mut d = N - 1;
            loop {
                cur[d] += 1;
                if cur[d] <= max_cells[d] {
                    break;
                }
                cur[d] = min_cells[d];
                if d == 0 {
                    return;
                }
                d -= 1;
            }
        }
    }
}

impl<F: Float, const N: usize> ValidityChecker<F, N> for SpatialRectValidityChecker<F, N> {
    fn is_state_valid(&self, state: &RealVectorState<F, N>) -> bool {
        let cell = Self::to_cell(state, self.cell_size_inv);
        if let Some(indices) = self.grid.get(&cell) {
            for &idx in indices {
                if self.obstacles[idx].contains(state) {
                    return false;
                }
            }
        }
        true
    }

    fn is_edge_valid(&self, start: &RealVectorState<F, N>, end: &RealVectorState<F, N>) -> bool {
        // Compute edge AABB in cell coords
        let min_cells: [i32; N] = std::array::from_fn(|i| {
            let lo = if start[i] < end[i] { start[i] } else { end[i] };
            Self::coord_to_cell(lo, self.cell_size_inv)
        });
        let max_cells: [i32; N] = std::array::from_fn(|i| {
            let hi = if start[i] > end[i] { start[i] } else { end[i] };
            Self::coord_to_cell(hi, self.cell_size_inv)
        });

        // Fast path: edge fits in a single cell (most common case, edge ~1m, cell 4m)
        if min_cells == max_cells {
            if let Some(indices) = self.grid.get(&min_cells) {
                for &idx in indices {
                    if self.obstacles[idx].intersects_edge(start, end) {
                        return false;
                    }
                }
            }
            return true;
        }

        // Multi-cell: iterate cells, check obstacles directly, allow duplicate checks
        // (checking a wall twice via Liang-Barsky costs ~5ns, allocating Vec costs ~100ns)
        let mut valid = true;
        Self::for_each_cell(&min_cells, &max_cells, |cell| {
            if !valid {
                return; // already found collision, skip remaining cells
            }
            if let Some(indices) = self.grid.get(&cell) {
                for &idx in indices {
                    if self.obstacles[idx].intersects_edge(start, end) {
                        valid = false;
                        return;
                    }
                }
            }
        });
        valid
    }
}
