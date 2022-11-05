pub(crate) const WIDTH: usize = 1802;
pub(crate) const HEIGHT: usize = 1802;
pub(crate) const START_IDX: usize = 2;
pub(crate) const EXIT_IDX: usize = 3243599;
// Use a separate file and the include! macro to avoid parsing the huge array with rust-analyzer and other tools
pub(crate) const INDIVIDUAL_COSTS: [f64; 3247204] = include!("./individual_costs.txt");
