use a_star_rs::get_path;
use criterion::{criterion_group, criterion_main, BatchSize, Criterion};
use std::hint::black_box;

const WIDTH: usize = 1802;
const HEIGHT: usize = 1802;
const START_IDX: usize = 2;
const EXIT_IDX: usize = 3243599;
// Use a separate file and the include! macro to avoid parsing the huge array with rust-analyzer and other tools
static INDIVIDUAL_COSTS: [f64; 3247204] = include!("individual_costs.txt");

pub fn criterion_benchmark(c: &mut Criterion) {
    let individual_costs = INDIVIDUAL_COSTS.to_vec();
    let mut group = c.benchmark_group("flat_sampling");
    group.measurement_time(std::time::Duration::from_secs(10));
    group.bench_function("get_path_maze_small", move |b| {
        b.iter_batched(
            || individual_costs.clone(),
            |costs| {
                get_path(
                    black_box(WIDTH),
                    black_box(HEIGHT),
                    black_box(costs),
                    black_box(START_IDX),
                    black_box(EXIT_IDX),
                    black_box(true),
                )
            },
            BatchSize::LargeInput,
        )
    });
    group.finish();
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
