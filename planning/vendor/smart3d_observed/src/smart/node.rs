use crate::rrt::rrt_star;
use crate::rrt::RealVectorState;
use num_traits::Float;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// A node in the RRT* tree.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Node<F: Float, const N: usize> {
    /// The state
    pub state: RealVectorState<F, N>,
    /// The index of the parent node (None if the node is the root).
    pub parent: Option<usize>,
    /// The index of the children nodes.
    pub children: Vec<usize>,
    /// The cost from the parent to this node.
    pub edge_cost: F,
    /// Cost from the root to this node.
    pub cumulative_cost: F,
    /// Tree ID
    pub tree_id: i32,
}

impl<F: Float, const N: usize> Node<F, N> {
    pub fn new(
        state: RealVectorState<F, N>,
        parent: Option<usize>,
        edge_cost: F,
        cumulative_cost: F,
        tree_id: i32,
    ) -> Self {
        Self {
            state,
            parent,
            children: Vec::new(),
            edge_cost,
            cumulative_cost,
            tree_id,
        }
    }

    pub fn add_child(&mut self, child: usize) {
        debug_assert!(!self.children.contains(&child));
        self.children.push(child);
    }

    pub fn remove_child(&mut self, child: usize) {
        if let Some(index) = self.children.iter().position(|&x| x == child) {
            self.children.remove(index);
        } else {
            #[cfg(debug_assertions)]
            {
                panic!("The child node {} does not exist on this node.", child);
            }
        }
    }

    pub fn state(&self) -> &RealVectorState<F, N> {
        &self.state
    }

    pub fn parent(&self) -> Option<usize> {
        self.parent
    }
}

pub fn convert_rrt_star_nodes_to_smart_nodes<F: Float, const N: usize>(
    rrt_star_nodes: &Vec<rrt_star::Node<F, N>>,
) -> Vec<Node<F, N>> {
    let mut children_map: HashMap<usize, Vec<usize>> = HashMap::new();

    // Build child relationships
    for (i, node) in rrt_star_nodes.iter().enumerate() {
        if let Some(parent_idx) = node.parent() {
            children_map.entry(parent_idx).or_default().push(i);
        }
    }

    // Construct new vector of nodes
    let mut smart_nodes = Vec::with_capacity(rrt_star_nodes.len());
    for (i, node) in rrt_star_nodes.iter().enumerate() {
        let edge_cost = if let Some(parent) = node.parent() {
            node.state()
                .euclidean_distance(rrt_star_nodes[parent].state())
        } else {
            F::infinity()
        };
        let mut smart_node = Node::new(
            node.state().clone(),
            node.parent(),
            edge_cost,
            node.cumulative_cost(),
            0,
        );
        if let Some(children) = children_map.get(&i) {
            for &child in children {
                smart_node.add_child(child);
            }
        }
        smart_nodes.push(smart_node);
    }

    smart_nodes
}
