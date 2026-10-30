mod common;

use clipper2_rs::{
    Path64, Point64, area_paths, minkowski_diff, minkowski_diff_d, minkowski_sum, minkowski_sum_d,
};
use common::*;

fn first(p: clipper2_rs::Paths64) -> Path64 {
    p.into_iter().next().unwrap_or_default()
}

#[test]
fn random_minkowski_matches_reference() {
    for shape in [Shape::SmallGrid, Shape::Medium, Shape::Star] {
        for seed in 0..cases() as u64 {
            let mut rng = Rng::new(seed * 179_424_673 + shape as u64);
            let pattern = first(random_paths(&mut rng, Shape::SmallGrid));
            let path = if seed % 2 == 0 {
                first(random_paths(&mut rng, shape))
            } else {
                first(random_open_paths(&mut rng, shape))
            };
            let closed = seed % 3 != 0;
            let input = format!(
                "closed={closed}\npattern={:?}\npath={:?}",
                to_ref(std::slice::from_ref(&pattern)),
                to_ref(std::slice::from_ref(&path))
            );
            let rp = to_ref(std::slice::from_ref(&pattern)).remove(0);
            let rq = to_ref(std::slice::from_ref(&path)).remove(0);
            assert_paths_eq(
                &format!("minkowski-sum-{shape:?}-{seed}"),
                &input,
                &minkowski_sum(&pattern, &path, closed).unwrap(),
                &clipper2_ref::minkowski(&rp, &rq, closed, false).unwrap(),
            );
            assert_paths_eq(
                &format!("minkowski-diff-{shape:?}-{seed}"),
                &input,
                &minkowski_diff(&pattern, &path, closed).unwrap(),
                &clipper2_ref::minkowski(&rp, &rq, closed, true).unwrap(),
            );
        }
    }
}

#[test]
fn random_minkowski_d_matches_reference() {
    for seed in 0..cases() as u64 {
        let mut rng = Rng::new(seed * 31 + 7);
        let dp = [0, 2, 3][rng.below(3)];
        let pattern = random_paths_d(&mut rng, Shape::SmallGrid, dp).remove(0);
        let path = random_paths_d(&mut rng, Shape::Medium, dp).remove(0);
        let closed = seed % 2 == 0;
        let rp = to_ref_d(std::slice::from_ref(&pattern)).remove(0);
        let rq = to_ref_d(std::slice::from_ref(&path)).remove(0);
        let input = format!("dp={dp} closed={closed}\n{rp:?}\n{rq:?}");
        assert_paths_d_eq(
            &format!("minkowski-sum-d-{seed}"),
            &input,
            &minkowski_sum_d(&pattern, &path, closed, dp).unwrap(),
            &clipper2_ref::minkowski_d(&rp, &rq, closed, dp, false).unwrap(),
        );
        assert_paths_d_eq(
            &format!("minkowski-diff-d-{seed}"),
            &input,
            &minkowski_diff_d(&pattern, &path, closed, dp).unwrap(),
            &clipper2_ref::minkowski_d(&rp, &rq, closed, dp, true).unwrap(),
        );
    }
}

fn sq(c: i64, h: i64) -> Path64 {
    vec![
        Point64::new(c - h, c - h),
        Point64::new(c + h, c - h),
        Point64::new(c + h, c + h),
        Point64::new(c - h, c + h),
    ]
}

#[test]
fn sum_of_two_centred_2x2_squares_is_4x4() {
    assert_eq!(
        area_paths(&minkowski_sum(&sq(0, 1), &sq(0, 1), true).unwrap()).abs(),
        16.0
    );
}

#[test]
fn degenerate_inputs_match_reference() {
    let cases: Vec<Path64> = vec![
        vec![],
        vec![Point64::new(3, 3)],
        vec![Point64::new(0, 0), Point64::new(5, 5)],
        vec![Point64::new(0, 0), Point64::new(5, 5), Point64::new(10, 10)],
    ];
    for (i, path) in cases.iter().enumerate() {
        for closed in [false, true] {
            let rp = to_ref(&[sq(0, 1)]).remove(0);
            let rq = to_ref(std::slice::from_ref(path)).remove(0);
            assert_paths_eq(
                &format!("minkowski-degenerate-{i}-{closed}"),
                "",
                &minkowski_sum(&sq(0, 1), path, closed).unwrap(),
                &clipper2_ref::minkowski(&rp, &rq, closed, false).unwrap(),
            );
            assert_paths_eq(
                &format!("minkowski-degenerate-pat-{i}-{closed}"),
                "",
                &minkowski_sum(path, &sq(0, 1), closed).unwrap(),
                &clipper2_ref::minkowski(&rq, &rp, closed, false).unwrap(),
            );
        }
    }
}
