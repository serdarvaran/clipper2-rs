//! clipper2-rs against the upstream C++ library, operation by operation. The C++ side is timed
//! inside the shim (`clipper2_ref::bench`), on inputs converted once, so FFI copies are excluded.

#[path = "../tests/common/mod.rs"]
mod common;

use std::hint::black_box;
use std::time::Duration;

use clipper2_rs::{
    ClipType, Clipper64, EndType, FillRule, JoinType, Path64, Paths64, Point64, Rect64, ellipse,
    inflate_paths, minkowski_sum, rect_clip, simplify_paths, triangulate, union_self,
};
use common::{Rng, to_ref};
use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};

const BOOLEAN: i32 = 0;
const OFFSET: i32 = 1;
const RECT_CLIP: i32 = 2;
const MINKOWSKI: i32 = 3;
const TRIANGULATE: i32 = 4;
const SIMPLIFY: i32 = 5;

/// Jittered ellipses scattered over a square: overlapping outlines like sliced layers.
fn blobs(rng: &mut Rng, count: usize, points: usize, spread: i64) -> Paths64 {
    (0..count)
        .map(|_| {
            let center = Point64::new(
                rng.below(spread as usize) as i64,
                rng.below(spread as usize) as i64,
            );
            let radius = 50.0 + rng.below(450) as f64;
            ellipse(center, radius, radius * 0.7, points)
                .into_iter()
                .map(|p| {
                    Point64::new(
                        p.x + rng.below(21) as i64 - 10,
                        p.y + rng.below(21) as i64 - 10,
                    )
                })
                .collect()
        })
        .collect()
}

fn point_count(paths: &Paths64) -> usize {
    paths.iter().map(Vec::len).sum()
}

fn cpp(
    op: i32,
    a: i32,
    b: i32,
    d: f64,
    subjects: &Paths64,
    clips: &Paths64,
) -> impl Fn(u64) -> Duration + use<> {
    let (s, c) = (to_ref(subjects), to_ref(clips));
    move |iters| clipper2_ref::bench(op, a, b, d, &s, &c, iters)
}

fn boolean(c: &mut Criterion) {
    let mut group = c.benchmark_group("boolean");
    for (label, count, points, spread) in [("1k", 50, 20, 4_000), ("50k", 1_250, 40, 40_000)] {
        let mut rng = Rng::new(1);
        let (subjects, clips) = (
            blobs(&mut rng, count, points, spread),
            blobs(&mut rng, count, points, spread),
        );
        for ct in [
            ClipType::Intersection,
            ClipType::Union,
            ClipType::Difference,
            ClipType::Xor,
        ] {
            let id = format!("{ct:?}/{label}");
            group.bench_function(BenchmarkId::new("rust", &id), |b| {
                b.iter(|| {
                    let mut cl = Clipper64::new();
                    cl.add_subject(&subjects);
                    cl.add_clip(&clips);
                    black_box(cl.execute(ct, FillRule::NonZero).unwrap())
                })
            });
            let run = cpp(
                BOOLEAN,
                ct as i32,
                FillRule::NonZero as i32,
                0.0,
                &subjects,
                &clips,
            );
            group.bench_function(BenchmarkId::new("cpp", &id), |b| b.iter_custom(&run));
        }
    }
    group.finish();
}

fn offset(c: &mut Criterion) {
    let mut group = c.benchmark_group("offset");
    let subjects = blobs(&mut Rng::new(2), 250, 40, 40_000);
    for jt in [
        JoinType::Square,
        JoinType::Bevel,
        JoinType::Round,
        JoinType::Miter,
    ] {
        let id = format!("{jt:?}/10k");
        group.bench_function(BenchmarkId::new("rust", &id), |b| {
            b.iter(|| {
                black_box(inflate_paths(&subjects, 20.0, jt, EndType::Polygon, 2.0, 0.0).unwrap())
            })
        });
        let run = cpp(
            OFFSET,
            jt as i32,
            EndType::Polygon as i32,
            20.0,
            &subjects,
            &vec![],
        );
        group.bench_function(BenchmarkId::new("cpp", &id), |b| b.iter_custom(&run));
    }
    group.finish();
}

fn others(c: &mut Criterion) {
    let mut group = c.benchmark_group("other");

    let subjects = blobs(&mut Rng::new(3), 1_250, 40, 40_000);
    let rect = Rect64::new(10_000, 10_000, 30_000, 30_000);
    let rect_path = vec![vec![
        Point64::new(rect.left, rect.top),
        Point64::new(rect.right, rect.bottom),
    ]];
    group.bench_function(BenchmarkId::new("rust", "rect_clip/50k"), |b| {
        b.iter(|| black_box(rect_clip(&rect, &subjects)))
    });
    let run = cpp(RECT_CLIP, 0, 0, 0.0, &subjects, &rect_path);
    group.bench_function(BenchmarkId::new("cpp", "rect_clip/50k"), |b| {
        b.iter_custom(&run)
    });

    let path: Path64 = blobs(&mut Rng::new(4), 1, 2_000, 1_000).remove(0);
    let pattern = ellipse(Point64::new(0, 0), 10.0, 10.0, 16);
    group.bench_function(BenchmarkId::new("rust", "minkowski_sum/2k"), |b| {
        b.iter(|| black_box(minkowski_sum(&pattern, &path, true).unwrap()))
    });
    let run = cpp(
        MINKOWSKI,
        0,
        0,
        0.0,
        &vec![path.clone()],
        &vec![pattern.clone()],
    );
    group.bench_function(BenchmarkId::new("cpp", "minkowski_sum/2k"), |b| {
        b.iter_custom(&run)
    });

    let clean = union_self(&blobs(&mut Rng::new(5), 250, 40, 40_000), FillRule::NonZero).unwrap();
    for delaunay in [false, true] {
        let id = format!("triangulate/delaunay={delaunay}/{}", point_count(&clean));
        group.bench_function(BenchmarkId::new("rust", &id), |b| {
            b.iter(|| black_box(triangulate(&clean, delaunay).unwrap()))
        });
        let run = cpp(TRIANGULATE, delaunay as i32, 0, 0.0, &clean, &vec![]);
        group.bench_function(BenchmarkId::new("cpp", &id), |b| b.iter_custom(&run));
    }

    group.bench_function(BenchmarkId::new("rust", "simplify_paths/50k"), |b| {
        b.iter(|| black_box(simplify_paths(&subjects, 2.0, true)))
    });
    let run = cpp(SIMPLIFY, 0, 0, 2.0, &subjects, &vec![]);
    group.bench_function(BenchmarkId::new("cpp", "simplify_paths/50k"), |b| {
        b.iter_custom(&run)
    });

    group.finish();
}

fn config() -> Criterion {
    Criterion::default()
        .sample_size(20)
        .warm_up_time(Duration::from_secs(1))
        .measurement_time(Duration::from_secs(3))
}

criterion_group! { name = benches; config = config(); targets = boolean, offset, others }
criterion_main!(benches);
