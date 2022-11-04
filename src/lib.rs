use std::{
    collections::{BTreeMap, BTreeSet},
    ops::{AddAssign, Div, Rem},
};

use pyo3::{exceptions::PyValueError, prelude::*};

// /// Formats the sum of two numbers as string.
// #[pyfunction]
// fn sum_as_string(a: usize, b: usize) -> PyResult<String> {
//     Ok((a + b).to_string())
// }

// /// A Python module implemented in Rust.
// #[pymodule]
// fn a_star_rs(_py: Python, m: &PyModule) -> PyResult<()> {
//     m.add_function(wrap_pyfunction!(sum_as_string, m)?)?;
//     Ok(())
// }

/// Find a path between two points on a rectangular grid with the A*-Algorithm.
/// Returns a tuple with the list of indices for the path in order and a set of all places checked by the A*-Algorithm.
#[pyfunction]
pub fn get_path(
    width: usize,
    height: usize,
    individual_costs: Vec<f64>,
    start_idx: usize,
    exit_idx: usize,
    diagonal_allowed: bool,
) -> PyResult<(Vec<usize>, BTreeSet<usize>)> {
    if width * height != individual_costs.len() {
        return Err(PyValueError::new_err(
            "Width times height != number of costs",
        ));
    }
    if start_idx >= individual_costs.len() || exit_idx >= individual_costs.len() {
        return Err(pyo3::exceptions::PyIndexError::new_err(
            "Start index or exit index out of range",
        ));
    }
    let idx2pos = |i: usize| (i.rem(width), i.div(width));
    let pos2idx = |x: usize, y: usize| x + y * width;

    // Finding the path from exit to start inverts the parent-child relationship of the path from start to exit. Thus reconstructing the path can start at the original start and get the childs sequentially.
    let (exit_idx, start_idx) = (start_idx, exit_idx);

    // This heuristic is consistent as long as the true cost to get from one node to its children is always equal or greater than the difference between the heuristic costs of those nodes.
    // This is achieved by adding the supremum of the possible difference between the heuristic costs for a given move direction to the true move cost between the nodes.
    // https://en.wikipedia.org/wiki/Consistent_heuristic
    let (x_exit, y_exit) = idx2pos(exit_idx);
    let estimate_cost = |x: usize, y: usize| {
        let diffx = (x - x_exit) as f64;
        let diffy = (y - y_exit) as f64;
        (diffx * diffx + diffy * diffy).sqrt()
    };

    // Map from the index to the accumulated cost of a node
    let mut cum_costs: BTreeMap<usize, f64> = BTreeMap::new();
    // Map from the index of one node to the index of its parent
    let mut parents: BTreeMap<usize, usize> = BTreeMap::new();

    // Map of costs of neighbor nodes to their indices. A monotonically increasing insertion counter is included in the key as a tie breaker if two costs are equal as well as to make the path expansion greedy.
    let mut openlist = BTreeMap::new();
    openlist.insert(
        OpenlistKey {
            cost: 0.0,
            counter: 0,
        },
        start_idx,
    );
    // Counter to keep track of number of insertions into the openlist.
    let mut insertion_counter = 1;
    cum_costs.insert(start_idx, 0.0);
    // parents.insert(start_idx, start_idx); // The start has no parent, so its used as its own parent. This makes it possible to later identify the start.

    // Set of indices of all expanded nodes
    let mut closedlist = BTreeSet::new();

    // Try to find a shorter path as long as there are nodes to expand and we haven't found the exit.
    while let Some((_, current)) = openlist.pop() {
        if current == exit_idx {
            // Since start and exit were swapped, the path was constructed in reverse order. So the parents of all nodes on the reverse path actally point to the children of all nodes on the forward path.
            let mut path = Vec::with_capacity(width + height);
            path.push(current);
            let mut current = current;
            while let Some(next) = parents.get(&current) {
                path.push(*next);
                current = *next;
            }
            return Ok((path, closedlist));
        }
        closedlist.insert(current);
        let (x, y) = idx2pos(current);
        for dx in -1i32..2 {
            for dy in -1i32..2 {
                let is_diagonal_neighbor = dx * dy != 0;
                if !diagonal_allowed && is_diagonal_neighbor {
                    continue;
                }
                // Casting a negative value to usize (-1 in this case) wraps the value around according to twos complement. Adding this wrapped value to another usize leads to another twos complement wrap. The resulting value is the same as if first casting all arguments to signed values, doing the addition and then casting the result back to a unsigend value.
                let (x, y) = (x.wrapping_add(dx as usize), y.wrapping_add(dy as usize));
                // Skip if the neighbor is outside of the input grid. If x or y would have been -1 and thus outside of the grid at the little side of the indices range, it is now wrapped around to usize::MAX. Thus only one comparison against width and height to the bigger side is needed.
                if x >= width || y >= height {
                    continue;
                }
                let idx = pos2idx(x, y);
                // Skip neighbors if they were already expanded. This does not compromise the optimality of the solution as long as the heuristic is consistent.
                if closedlist.contains(&idx) {
                    continue;
                }
                // By adding 1 for a cardinal direction or sqrt(2) for a diagonal direction, the cost to get from the current to the child nodes is always greater or equal to the change in heuristic cost between them. This makes the heuristic consistent.
                let move_cost = individual_costs[idx]
                    + if is_diagonal_neighbor {
                        std::f64::consts::SQRT_2
                    } else {
                        1.0
                    };
                let cum_cost = cum_costs
                    .get(&current)
                    .expect("Every index in the openlist should also be in cum_costs")
                    + move_cost;
                // Check if a new or shorter way was found to this neighbor
                if cum_cost < *cum_costs.entry(idx).or_insert(f64::INFINITY) {
                    // A new or shorter way was found!
                    // Update the cumulative cost to get to this neighbor.
                    cum_costs.insert(idx, cum_cost);
                    // Remember the parent node through which this new shorter way leads
                    parents
                        .entry(idx)
                        .and_modify(|parent| *parent = current)
                        .or_insert(current);
                    // Insert the neighbor with a new key with the lower cumulative cost into the openlist. If there was a previous entry with higher cost for this neighbor in the openlist, the new node will get processed earlier because of the lower cost. Thus the entries with for this neighbor with higher costs will get removed lazily on the check against the closed list.
                    let combined_cost = cum_cost + estimate_cost(x, y);
                    openlist.insert(
                        OpenlistKey {
                            cost: combined_cost,
                            counter: insertion_counter,
                        },
                        idx,
                    );
                    insertion_counter.add_assign(1);
                }
            }
        }
    }
    // No path was found
    Ok((vec![usize::MAX], closedlist))
}

/// Struct containing all information for a position on the grid.
#[derive(Clone, Copy, PartialEq)]
struct OpenlistKey {
    // Combined cumulative and estimated cost
    cost: f64,
    // Insertion counter
    counter: u64,
}
impl Eq for OpenlistKey {}
// Lower cost compares lower, if costs are equal a higher counter compares lower.
impl PartialOrd for OpenlistKey {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        self.cost
            .partial_cmp(&other.cost)
            .map(|ord| ord.then(self.counter.cmp(&other.counter).reverse()))
    }
}
impl Ord for OpenlistKey {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.partial_cmp(&other)
            .expect("All costs should be in the totally ordered set of floats, thus excluding NaN.")
    }
}

trait BTreeMapExt<K: Ord, T> {
    fn pop(&mut self) -> Option<(K, T)>;
}
impl<K: Ord + Copy, T> BTreeMapExt<K, T> for BTreeMap<K, T> {
    fn pop(&mut self) -> Option<(K, T)> {
        let (k, _) = self.iter().next()?;
        let k = *k;
        self.remove_entry(&k)
    }
}

#[pymodule]
fn a_star_rs(_: Python<'_>, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(get_path, m)?)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use crate::get_path;

    #[test]
    fn test_get_path() {
        let width = 4;
        let height = 4;
        let costs = vec![1.0; width * height];
        let start_idx = 0;
        let exit_idx = costs.len() - 1;
        let (path, set) = get_path(width, height, costs, start_idx, exit_idx, true).unwrap();
        assert_ne!(path.len(), 1);
    }
}
