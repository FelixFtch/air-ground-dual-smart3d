use crate::rrt::state::RealVectorState;
use kiddo::float::kdtree::Axis;
use std::{cmp::Ordering, collections::BinaryHeap};
use num_traits::Float;

/// A trait for a nearest neighbor data structure that supports nearest neighbors and radius queries.
/// Stores RealVectorStates and a usize index along with them.
pub trait NearestNeighbors<F: Float, const N: usize> {
    /// Constructs a new nearest neighbor data structure.
    /// The data structure is empty initially.
    fn new() -> Self;

    /// Adds a state to the data structure.
    ///
    /// Parameters:
    /// - `state`: The RealVectorState to add.
    /// - `item`: The index of the RealVectorState.
    fn add(&mut self, state: RealVectorState<F, N>, item: usize);

    /// Gets the nearest neighbor to the given RealVectorState.
    ///
    /// Parameters:
    /// - `state`: The RealVectorState to find the nearest neighbor to.
    ///
    /// Returns:
    /// The item/index of the nearest neighbor, if any.
    fn nearest_one(&self, state: &RealVectorState<F, N>) -> Option<usize> {
        let nearest_vec = self.nearest_k(state, 1);
        if nearest_vec.is_empty() {
            None
        } else {
            Some(nearest_vec[0])
        }
    }

    /// Gets the k nearest neighbors to the given RealVectorState.
    ///
    /// Parameters:
    /// - `state`: The RealVectorState to find the nearest neighbors to.
    /// - `k`: The number of neighbors to find.
    ///
    /// Returns:
    /// The items/indices of the k nearest neighbors.
    fn nearest_k(&self, state: &RealVectorState<F, N>, k: usize) -> Vec<usize>;

    /// Gets all RealVectorStates within a given radius of the given RealVectorState.
    ///
    /// Parameters:
    /// - `state`: The RealVectorState to find the neighbors of.
    /// - `radius`: The radius within which to find neighbors.
    ///
    /// Returns:
    /// The items/indices of the RealVectorStates within the radius.
    fn within_radius(&self, state: &RealVectorState<F, N>, radius: F) -> Vec<usize>;

    /// Gets all RealVectorStates within a given radius of the given RealVectorState in sorted order, nearest-first.
    ///
    /// Parameters:
    /// - `state`: The RealVectorState to find the neighbors of.
    /// - `radius`: The radius within which to find neighbors.
    ///
    /// Returns:
    /// The items/indices of the RealVectorStates within the radius.
    fn within_radius_sorted(&self, state: &RealVectorState<F, N>, radius: F) -> Vec<usize>;
}

/// A nearest neighbor data structure that uses a linear search to find the nearest neighbors.
/// This is useful for small datasets.
pub struct LinearNearestNeighbors<F: Float, const N: usize> {
    states: Vec<(RealVectorState<F, N>, usize)>,
}

impl<F: Float, const N: usize> NearestNeighbors<F, N> for LinearNearestNeighbors<F, N> {
    fn new() -> Self {
        Self { states: Vec::new() }
    }

    fn add(&mut self, state: RealVectorState<F, N>, item: usize) {
        self.states.push((state, item));
    }

    fn nearest_one(&self, state: &RealVectorState<F, N>) -> Option<usize> {
        let nearest = self.states.iter().min_by(|a, b| {
            state
                .euclidean_distance_squared(&a.0)
                .partial_cmp(&state.euclidean_distance_squared(&b.0))
                .unwrap()
        });
        nearest.map(|(_, i)| *i)
    }

    fn nearest_k(&self, state: &RealVectorState<F, N>, k: usize) -> Vec<usize> {
        let mut nearest = self
            .states
            .iter()
            .map(|(p, i)| (state.euclidean_distance_squared(&p), *i))
            .collect::<Vec<_>>();
        nearest.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        nearest.into_iter().take(k).map(|(_, i)| i).collect()
    }

    fn within_radius(&self, state: &RealVectorState<F, N>, radius: F) -> Vec<usize> {
        self.states
            .iter()
            .filter(|(p, _)| state.euclidean_distance_squared(&p) <= radius * radius)
            .map(|(_, i)| *i)
            .collect()
    }

    fn within_radius_sorted(&self, state: &RealVectorState<F, N>, radius: F) -> Vec<usize> {
        let mut within = self.within_radius(state, radius);
        within.sort_by(|&a, &b| {
            state
                .euclidean_distance_squared(&self.states[a].0)
                .partial_cmp(&state.euclidean_distance_squared(&self.states[b].0))
                .unwrap()
        });
        within
    }
}

// Fixed-capacity, cyclic-axis Kiddo leaves panic on repeated route goals or
// planar/collinear samples. Keep an exact adaptive-axis kd index instead: split
// only between distinct coordinates; coincident states retain every node ID in
// a dynamically sized leaf. Neither planner coordinates nor distances change.
const KD_LEAF_SIZE: usize = 32;
type Entry<F, const N: usize> = (RealVectorState<F, N>, usize);

enum ExactKdNode<F: Float, const N: usize> {
    Leaf { entries: Vec<Entry<F, N>>, bounds: [(F, F); N] },
    Branch { axis: usize, split: F, left: Box<Self>, right: Box<Self> },
}

impl<F: Float, const N: usize> ExactKdNode<F, N> {
    fn from_entries(mut entries: Vec<Entry<F, N>>) -> Self {
        let mut bounds = [(F::infinity(), F::neg_infinity()); N];
        for (state, _) in &entries {
            for (axis, bound) in bounds.iter_mut().enumerate() {
                bound.0 = bound.0.min(state[axis]);
                bound.1 = bound.1.max(state[axis]);
            }
        }
        let axis = (0..N).max_by(|&a, &b|
            (bounds[a].1 - bounds[a].0).partial_cmp(&(bounds[b].1 - bounds[b].0)).unwrap());
        if entries.len() <= KD_LEAF_SIZE || axis.is_none()
            || bounds[axis.unwrap()].0 == bounds[axis.unwrap()].1 {
            return Self::Leaf { entries, bounds };
        }
        let axis = axis.unwrap();
        entries.sort_unstable_by(|a, b| a.0[axis].partial_cmp(&b.0[axis]).unwrap()
            .then_with(|| a.1.cmp(&b.1)));
        // Find the closest legal median gap. A branch routes < split left and
        // >= split right, so equal coordinates must remain on the same side.
        let middle = entries.len() / 2;
        let pivot = (1..entries.len())
            .filter(|&i| entries[i - 1].0[axis] < entries[i].0[axis])
            .min_by_key(|&i| i.abs_diff(middle)).expect("nonzero spread has a gap");
        let right = entries.split_off(pivot);
        let split = right[0].0[axis];
        Self::Branch { axis, split, left: Box::new(Self::from_entries(entries)),
            right: Box::new(Self::from_entries(right)) }
    }

    fn add(&mut self, state: RealVectorState<F, N>, item: usize) {
        match self {
            Self::Branch { axis, split, left, right } => {
                if state[*axis] < *split { left.add(state, item); }
                else { right.add(state, item); }
            }
            Self::Leaf { entries, bounds } => {
                entries.push((state, item));
                for (axis, bound) in bounds.iter_mut().enumerate() {
                    bound.0 = bound.0.min(state[axis]);
                    bound.1 = bound.1.max(state[axis]);
                }
                if entries.len() > KD_LEAF_SIZE && bounds.iter().any(|&(lo, hi)| lo < hi) {
                    *self = Self::from_entries(std::mem::take(entries));
                }
            }
        }
    }

    fn nearest(&self, state: &RealVectorState<F, N>, k: usize, heap: &mut BinaryHeap<Neighbor<F>>) {
        match self {
            Self::Leaf { entries, .. } => {
                for (point, item) in entries {
                    let candidate = Neighbor { distance: state.euclidean_distance_squared(point), item: *item };
                    if heap.len() < k { heap.push(candidate); }
                    else if candidate < *heap.peek().unwrap() {
                        heap.pop(); heap.push(candidate);
                    }
                }
            }
            Self::Branch { axis, split, left, right } => {
                let delta = state[*axis] - *split;
                let (near, far) = if delta < F::zero() { (left, right) } else { (right, left) };
                near.nearest(state, k, heap);
                if heap.len() < k || delta * delta <= heap.peek().unwrap().distance {
                    far.nearest(state, k, heap);
                }
            }
        }
    }

    fn within(&self, state: &RealVectorState<F, N>, radius_squared: F, out: &mut Vec<Neighbor<F>>) {
        match self {
            Self::Leaf { entries, .. } => {
                for (point, item) in entries {
                    let distance = state.euclidean_distance_squared(point);
                    if distance <= radius_squared { out.push(Neighbor { distance, item: *item }); }
                }
            }
            Self::Branch { axis, split, left, right } => {
                let delta = state[*axis] - *split;
                let (near, far) = if delta < F::zero() { (left, right) } else { (right, left) };
                near.within(state, radius_squared, out);
                if delta * delta <= radius_squared { far.within(state, radius_squared, out); }
            }
        }
    }
}

#[derive(Clone, Copy)]
struct Neighbor<F: Float> { distance: F, item: usize }
impl<F: Float> PartialEq for Neighbor<F> {
    fn eq(&self, other: &Self) -> bool { self.distance == other.distance && self.item == other.item }
}
impl<F: Float> Eq for Neighbor<F> {}
impl<F: Float> PartialOrd for Neighbor<F> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) }
}
impl<F: Float> Ord for Neighbor<F> {
    fn cmp(&self, other: &Self) -> Ordering {
        self.distance.partial_cmp(&other.distance).unwrap().then_with(|| self.item.cmp(&other.item))
    }
}

pub struct KdTreeNearestNeighbors<F: Float + Axis, const N: usize> {
    root: ExactKdNode<F, N>,
    len: usize,
}

impl<F: Float + Axis, const N: usize> NearestNeighbors<F, N> for KdTreeNearestNeighbors<F, N> {
    fn new() -> Self {
        Self { root: ExactKdNode::from_entries(Vec::new()), len: 0 }
    }

    fn add(&mut self, state: RealVectorState<F, N>, item: usize) {
        assert!(state.values().iter().all(|v| Float::is_finite(*v)), "nearest-neighbor state must be finite");
        self.root.add(state, item);
        self.len += 1;
    }

    fn nearest_k(&self, state: &RealVectorState<F, N>, k: usize) -> Vec<usize> {
        let k = k.min(self.len);
        if k == 0 { return Vec::new(); }
        let mut heap = BinaryHeap::with_capacity(k);
        self.root.nearest(state, k, &mut heap);
        heap.into_sorted_vec().into_iter().map(|n| n.item).collect()
    }

    fn within_radius(&self, state: &RealVectorState<F, N>, radius: F) -> Vec<usize> {
        let mut found = Vec::new();
        self.root.within(state, radius * radius, &mut found);
        found.into_iter().map(|n| n.item).collect()
    }

    fn within_radius_sorted(&self, state: &RealVectorState<F, N>, radius: F) -> Vec<usize> {
        let mut found = Vec::new();
        self.root.within(state, radius * radius, &mut found);
        found.sort_unstable();
        found.into_iter().map(|n| n.item).collect()
    }
}

#[cfg(test)]
mod exact_kd_regressions {
    use super::*;

    fn compare<const N: usize>(points: Vec<[f32; N]>, queries: Vec<[f32; N]>) {
        let mut kd = KdTreeNearestNeighbors::<f32, N>::new();
        let mut linear = LinearNearestNeighbors::<f32, N>::new();
        for (id, p) in points.into_iter().enumerate() {
            let p = RealVectorState::new(p); kd.add(p, id); linear.add(p, id);
        }
        for q in queries {
            let q = RealVectorState::new(q);
            assert_eq!(kd.nearest_one(&q), linear.nearest_one(&q));
            for k in [0, 1, 5, 33, 1024] {
                assert_eq!(kd.nearest_k(&q, k), linear.nearest_k(&q, k));
            }
            for radius in [0., 0.5, 1., 4., 100.] {
                let mut actual = kd.within_radius(&q, radius); actual.sort_unstable();
                assert_eq!(actual, linear.within_radius(&q, radius));
                assert_eq!(kd.within_radius_sorted(&q, radius), linear.within_radius_sorted(&q, radius));
            }
        }
    }

    #[test]
    fn repeated_route_goals_retain_every_id_and_exact_queries() {
        let goals = [[-8., -8.], [8., -8.], [8., 8.], [-8., 8.]];
        let points = (0..512).map(|i| goals[i % 4]).collect();
        compare(points, vec![[0., 0.], [-8., -8.], [8., 7.5], [1., 2.]]);
    }

    #[test]
    fn collinear_and_coplanar_points_are_valid_in_three_dimensions() {
        let points = (0..512).map(|i| [(i % 32) as f32 * 0.25, (i / 32) as f32 * 0.25, 2.]).collect();
        compare(points, vec![[0., 0., 2.], [2.1, 1.2, 3.], [-3., 5., 1.]]);
        compare((0..512).map(|i| [0., i as f32 * 0.25, 2.]).collect(), vec![[0., 20., 2.], [1., 2., 3.]]);
    }

    #[test]
    fn empty_index_and_closed_radius_boundary() {
        compare::<2>(vec![], vec![[0., 0.]]);
        compare(vec![[1., 0.], [0., 1.], [-1., 0.], [0., -1.]], vec![[0., 0.]]);
    }

    #[test]
    fn mixed_incremental_index_matches_brute_force() {
        let mut seed = 0x12345678u32;
        let points = (0..1024).map(|i| {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            let x = ((seed >> 12) % 200) as f32 * 0.125;
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            let y = ((seed >> 12) % 200) as f32 * 0.125;
            if i % 5 == 0 { [8., 8.] } else { [x, y] }
        }).collect();
        compare(points, vec![[0., 0.], [8., 8.], [12.25, 6.375], [-1., -2.], [28., 28.]]);
    }
}
