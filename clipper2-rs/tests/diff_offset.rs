#![allow(clippy::too_many_arguments)]
mod common;

use std::time::{Duration, Instant};

use clipper2_ref::RefOffsetGroup;
use clipper2_rs::{
    ClipperError, ClipperOffset, EndType, JoinType, MAX_COORD, Paths64, Point64, area_paths,
    inflate_paths, inflate_paths_d,
};
use common::*;

const JOIN_TYPES: [JoinType; 4] = [
    JoinType::Square,
    JoinType::Bevel,
    JoinType::Round,
    JoinType::Miter,
];
const END_TYPES: [EndType; 5] = [
    EndType::Polygon,
    EndType::Joined,
    EndType::Butt,
    EndType::Square,
    EndType::Round,
];
const DELTAS: [f64; 6] = [-5.0, -0.5, 0.5, 1.0, 7.25, 1000.0];

fn groups_ref(groups: &[(Paths64, JoinType, EndType)]) -> Vec<RefOffsetGroup> {
    groups
        .iter()
        .map(|(p, jt, et)| RefOffsetGroup {
            paths: to_ref(p),
            join_type: *jt as u8,
            end_type: *et as u8,
        })
        .collect()
}

fn make(
    groups: &[(Paths64, JoinType, EndType)],
    miter: f64,
    arc: f64,
    pc: bool,
    rev: bool,
) -> ClipperOffset {
    let mut co = ClipperOffset::new(miter, arc, pc, rev);
    for (p, jt, et) in groups {
        co.add_paths(p, *jt, *et);
    }
    co
}

fn compare(
    case: &str,
    groups: &[(Paths64, JoinType, EndType)],
    delta: f64,
    miter: f64,
    arc: f64,
    pc: bool,
    rev: bool,
) {
    let input = format!(
        "delta={delta} miter={miter} arc={arc} pc={pc} rev={rev}\ngroups={:?}",
        groups_ref(groups)
    );
    let r = clipper2_ref::offset(delta, &groups_ref(groups), miter, arc, pc, rev).unwrap();
    let o = make(groups, miter, arc, pc, rev).execute(delta).unwrap();
    assert_paths_eq(case, &input, &o, &r);
    let rt = clipper2_ref::offset_tree(delta, &groups_ref(groups), miter, arc, pc, rev).unwrap();
    let ot = make(groups, miter, arc, pc, rev)
        .execute_tree(delta)
        .unwrap();
    assert_same(
        &format!("{case}-tree"),
        &input,
        &flatten_tree(&ot),
        &ref_tree_nodes(&rt),
    );
}

#[test]
fn random_offsets_match_reference() {
    for jt in JOIN_TYPES {
        for et in END_TYPES {
            for shape in Shape::ALL {
                for seed in 0..(cases() as u64 / 4).max(1) {
                    let mut rng = Rng::new(seed * 86_028_121 + jt as u64 * 7 + et as u64);
                    let paths = if et == EndType::Polygon {
                        random_paths(&mut rng, shape)
                    } else {
                        random_open_paths(&mut rng, shape)
                    };
                    let delta = DELTAS[rng.below(DELTAS.len())];
                    let miter = [2.0, 10.0][rng.below(2)];
                    let arc = [0.0, 0.25, 5.0][rng.below(3)];
                    let mut groups = vec![(paths, jt, et)];
                    if seed % 3 == 0 {
                        groups.push((
                            random_paths(&mut rng, shape),
                            JOIN_TYPES[rng.below(4)],
                            EndType::Polygon,
                        ));
                    }
                    let case = format!("offset-{jt:?}-{et:?}-{shape:?}-{seed}");
                    compare(
                        &case,
                        &groups,
                        delta,
                        miter,
                        arc,
                        seed & 1 == 1,
                        seed & 2 == 2,
                    );
                }
            }
        }
    }
}

#[test]
fn delta_callback_matches_reference() {
    for seed in 0..(cases() as u64 / 2).max(1) {
        let mut rng = Rng::new(seed * 3_571 + 11);
        let jt = JOIN_TYPES[rng.below(4)];
        let et = END_TYPES[rng.below(5)];
        let paths = if et == EndType::Polygon {
            random_paths(&mut rng, Shape::Medium)
        } else {
            random_open_paths(&mut rng, Shape::Medium)
        };
        let table: Vec<f64> = (0..1 + rng.below(5))
            .map(|_| rng.range_f64(-20.0, 40.0))
            .collect();
        let groups = vec![(paths, jt, et)];
        let r = clipper2_ref::offset_with_delta_table(
            &groups_ref(&groups),
            &table,
            2.0,
            0.0,
            false,
            false,
        )
        .unwrap();
        let mut calls = 0usize;
        let o = make(&groups, 2.0, 0.0, false, false)
            .execute_with_callback(&mut |_, _, _, _| {
                calls += 1;
                table[(calls - 1) % table.len()]
            })
            .unwrap();
        assert_paths_eq(
            &format!("delta-cb-{seed}"),
            &format!("{table:?}\n{:?}", groups_ref(&groups)),
            &o,
            &r,
        );
    }
}

#[test]
fn inflate_paths_d_matches_reference() {
    for seed in 0..(cases() as u64 / 2).max(1) {
        let mut rng = Rng::new(seed * 9_973 + 5);
        let precision = [0, 2, 4][rng.below(3)];
        let jt = JOIN_TYPES[rng.below(4)];
        let et = END_TYPES[rng.below(5)];
        let paths = random_paths_d(&mut rng, Shape::Medium, precision + 1);
        let delta = rng.range_f64(-3.0, 10.0);
        let r = clipper2_ref::inflate_paths_d(
            &to_ref_d(&paths),
            delta,
            jt as u8,
            et as u8,
            2.0,
            precision,
            0.0,
        )
        .unwrap();
        let o = inflate_paths_d(&paths, delta, jt, et, 2.0, precision, 0.0).unwrap();
        assert_paths_d_eq(
            &format!("inflate-d-{seed}"),
            &format!("{precision} {jt:?} {et:?} {delta}\n{:?}", to_ref_d(&paths)),
            &o,
            &r,
        );
    }
}

fn square2() -> Paths64 {
    vec![vec![
        Point64::new(0, 0),
        Point64::new(2, 0),
        Point64::new(2, 2),
        Point64::new(0, 2),
    ]]
}

#[test]
fn offset_of_2x2_square() {
    assert_eq!(
        area_paths(
            &inflate_paths(&square2(), 1.0, JoinType::Miter, EndType::Polygon, 2.0, 0.0).unwrap()
        )
        .abs(),
        16.0
    );
    // At delta 1 the integer arc is coarse (area 14 in C++ too); at delta 100 it is close to 12 + PI scaled.
    let big = vec![vec![
        Point64::new(0, 0),
        Point64::new(200, 0),
        Point64::new(200, 200),
        Point64::new(0, 200),
    ]];
    let round = area_paths(
        &inflate_paths(&big, 100.0, JoinType::Round, EndType::Polygon, 2.0, 0.0).unwrap(),
    )
    .abs();
    let exact = 40_000.0 + 4.0 * 200.0 * 100.0 + std::f64::consts::PI * 10_000.0;
    assert!((round - exact).abs() / exact < 0.001, "{round} vs {exact}");
    assert!(
        inflate_paths(
            &square2(),
            -1.0,
            JoinType::Miter,
            EndType::Polygon,
            2.0,
            0.0
        )
        .unwrap()
        .is_empty()
    );
}

#[test]
fn offset_edge_cases_match_reference() {
    let mut rng = Rng::new(99);
    let poly = random_paths(&mut rng, Shape::Star);
    for jt in JOIN_TYPES {
        compare(
            &format!("edge-zero-{jt:?}"),
            &[(poly.clone(), jt, EndType::Polygon)],
            0.0,
            2.0,
            0.0,
            false,
            false,
        );
        compare(
            &format!("edge-vanish-{jt:?}"),
            &[(square2(), jt, EndType::Polygon)],
            -5.0,
            2.0,
            0.0,
            false,
            false,
        );
        compare(
            &format!("edge-point-{jt:?}"),
            &[(vec![vec![Point64::new(3, 4)]], jt, EndType::Polygon)],
            6.0,
            2.0,
            0.0,
            false,
            false,
        );
    }
    for arc in [0.0, 0.001] {
        let start = Instant::now();
        compare(
            &format!("edge-arc-{arc}"),
            &[(poly.clone(), JoinType::Round, EndType::Polygon)],
            1000.0,
            2.0,
            arc,
            false,
            false,
        );
        assert!(start.elapsed() < Duration::from_secs(10));
    }
    let far = vec![vec![
        Point64::new(0, 0),
        Point64::new(MAX_COORD + 1, 0),
        Point64::new(0, 10),
    ]];
    assert_eq!(
        ClipperOffset::default()
            .tap(|c| c.add_paths(&far, JoinType::Miter, EndType::Polygon))
            .execute(1.0)
            .err(),
        Some(ClipperError::CoordinateOutOfRange)
    );
}

trait Tap: Sized {
    fn tap(mut self, f: impl FnOnce(&mut Self)) -> Self {
        f(&mut self);
        self
    }
}
impl Tap for ClipperOffset {}
