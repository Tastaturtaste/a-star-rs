mod input;
use a_star_rs::get_path;
use criterion::{criterion_group, criterion_main, BatchSize, Criterion, SamplingMode, black_box};

fn criterion_benchmark(c: &mut Criterion) {
    let individual_costs = input::INDIVIDUAL_COSTS.to_vec();
    let mut group = c.benchmark_group("flat_sampling");
    // group.sampling_mode(SamplingMode::Flat);
    group.measurement_time(std::time::Duration::from_secs(10));
    group.bench_function("get_path_maze_small", move |b| {
        b.iter_batched(
            || individual_costs.clone(),
            |costs| {
                get_path(
                    black_box(input::WIDTH),
                    black_box(input::HEIGHT),
                    black_box(costs),
                    black_box(input::START_IDX),
                    black_box(input::EXIT_IDX),
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
