mod input;
use a_star_rs::get_path;
use criterion::{criterion_group, criterion_main, BatchSize, Criterion, SamplingMode};

fn criterion_benchmark(c: &mut Criterion) {
    let individual_costs = input::INDIVIDUAL_COSTS.to_vec();
    let mut group = c.benchmark_group("flat_sampling");
    group.sampling_mode(SamplingMode::Flat);
    group.bench_function("get_path_maze_small", move |b| {
        b.iter_batched(
            || individual_costs.clone(),
            |costs| {
                get_path(
                    input::WIDTH,
                    input::HEIGHT,
                    costs,
                    input::START_IDX,
                    input::EXIT_IDX,
                    true,
                )
            },
            BatchSize::LargeInput,
        )
    });
    group.measurement_time(std::time::Duration::from_secs(120));
    group.finish();
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
