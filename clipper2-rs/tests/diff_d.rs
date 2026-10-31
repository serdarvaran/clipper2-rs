#![allow(clippy::too_many_arguments)]
mod common;

use clipper2_rs::{ClipType, ClipperD, ClipperError, FillRule, PathsD, PointD};
use common::*;

const CLIP_TYPES: [ClipType; 4] = [
    ClipType::Intersection,
    ClipType::Union,
    ClipType::Difference,
    ClipType::Xor,
];
const FILL_RULES: [FillRule; 4] = [
    FillRule::EvenOdd,
    FillRule::NonZero,
    FillRule::Positive,
    FillRule::Negative,
];
type RunResult = Result<
    (
        Option<clipper2_rs::SolutionD>,
        Option<(clipper2_rs::PolyTreeD, PathsD)>,
    ),
    ClipperError,
>;

const PRECISIONS: [i32; 5] = [-2, 0, 2, 4, 8];

fn compare_d(
    case: &str,
    ct: ClipType,
    fr: FillRule,
    precision: i32,
    pc: bool,
    rev: bool,
    subject: &PathsD,
    open: &PathsD,
    clip: &PathsD,
) {
    let input = format!(
        "ct={ct:?} fr={fr:?} precision={precision} pc={pc} rev={rev}\nsubject={:?}\nopen={:?}\nclip={:?}",
        to_ref_d(subject),
        to_ref_d(open),
        to_ref_d(clip)
    );
    let run = |tree: bool| -> RunResult {
        let mut c = ClipperD::new(precision)?;
        c.set_preserve_collinear(pc);
        c.set_reverse_solution(rev);
        c.add_subject(subject)?;
        c.add_open_subject(open)?;
        c.add_clip(clip)?;
        if tree {
            Ok((None, Some(c.execute_tree(ct, fr)?)))
        } else {
            Ok((Some(c.execute(ct, fr)?), None))
        }
    };
    let (rs, ro, rc) = (to_ref_d(subject), to_ref_d(open), to_ref_d(clip));
    match (
        run(false),
        clipper2_ref::boolean_d(ct as u8, fr as u8, precision, pc, rev, &rs, &ro, &rc),
    ) {
        (Ok((Some(o), _)), Ok(r)) => {
            assert_paths_d_eq(case, &input, &o.closed, &r.closed);
            assert_paths_d_eq(&format!("{case}-open"), &input, &o.open, &r.open);
        }
        (o, r) => assert_same(case, &input, &o.is_ok(), &r.is_ok()),
    }
    match (
        run(true),
        clipper2_ref::boolean_tree_d(ct as u8, fr as u8, precision, pc, rev, &rs, &ro, &rc),
    ) {
        (Ok((_, Some((tree, open_out)))), Ok(r)) => {
            assert_same(
                &format!("{case}-tree"),
                &input,
                &flatten_tree_d(&tree),
                &ref_tree_nodes_d(&r),
            );
            assert_paths_d_eq(&format!("{case}-tree-open"), &input, &open_out, &r.open);
        }
        (o, r) => assert_same(&format!("{case}-tree"), &input, &o.is_ok(), &r.is_ok()),
    }
}

#[test]
fn random_clipper_d_matches_reference() {
    for ct in CLIP_TYPES {
        for fr in FILL_RULES {
            for precision in PRECISIONS {
                for shape in [Shape::SmallGrid, Shape::Medium, Shape::Star] {
                    for seed in 0..(cases() as u64 / 4).max(1) {
                        let mut rng = Rng::new(
                            seed * 7_368_787
                                + ct as u64 * 5
                                + fr as u64
                                + (precision + 10) as u64 * 101,
                        );
                        let subject = random_paths_d(&mut rng, shape, precision.max(0) + 2);
                        let open = if seed % 2 == 0 {
                            vec![]
                        } else {
                            random_paths_d(&mut rng, shape, precision.max(0) + 2)
                        };
                        let clip = random_paths_d(&mut rng, shape, precision.max(0) + 2);
                        let case = format!("clipperd-{ct:?}-{fr:?}-p{precision}-{shape:?}-{seed}");
                        compare_d(
                            &case,
                            ct,
                            fr,
                            precision,
                            seed & 1 == 1,
                            seed & 2 == 2,
                            &subject,
                            &open,
                            &clip,
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn precision_and_scaled_range_are_checked() {
    assert_eq!(
        ClipperD::new(9).err(),
        Some(ClipperError::PrecisionOutOfRange)
    );
    assert_eq!(
        ClipperD::new(-9).err(),
        Some(ClipperError::PrecisionOutOfRange)
    );
    assert!(ClipperD::new(8).is_ok());
    let mut c = ClipperD::new(2).unwrap();
    let far = vec![vec![
        PointD::new(0.0, 0.0),
        PointD::new(1e17, 0.0),
        PointD::new(0.0, 1.0),
    ]];
    assert_eq!(
        c.add_subject(&far).err(),
        Some(ClipperError::CoordinateOutOfRange)
    );
    // Upstream's DoError throws only where the compiler reports C++ exceptions
    // (`__cpp_exceptions`); the MSVC build of the reference returns no error here.
    if cfg!(not(target_env = "msvc")) {
        assert!(clipper2_ref::boolean_d(2, 1, 2, true, false, &to_ref_d(&far), &[], &[]).is_err());
    }
}

#[test]
fn union_of_unit_squares_in_floating_point() {
    let sq = |x: f64, y: f64| {
        vec![
            PointD::new(x, y),
            PointD::new(x + 1.0, y),
            PointD::new(x + 1.0, y + 1.0),
            PointD::new(x, y + 1.0),
        ]
    };
    let mut c = ClipperD::default();
    c.add_subject(&vec![sq(0.0, 0.0)]).unwrap();
    c.add_clip(&vec![sq(0.5, 0.5)]).unwrap();
    let s = c.execute(ClipType::Union, FillRule::NonZero).unwrap();
    assert_eq!(s.closed.len(), 1);
    assert_eq!(clipper2_rs::area(&s.closed[0]).abs(), 1.75);
}

/// Upstream 2.0.1 recursed forever in `CheckSplitOwner` when disposed split records formed a
/// cycle (fixed in f97d65d); found by this suite (Xor, EvenOdd, precision -2, seed 460).
#[test]
fn split_owner_cycle_does_not_overflow() {
    let (ct, fr, precision, seed) = (ClipType::Xor, FillRule::EvenOdd, -2i32, 460u64);
    let mut rng =
        Rng::new(seed * 7_368_787 + ct as u64 * 5 + fr as u64 + (precision + 10) as u64 * 101);
    let subject = random_paths_d(&mut rng, Shape::Medium, 2);
    let clip = random_paths_d(&mut rng, Shape::Medium, 2);
    compare_d(
        "split-owner-cycle",
        ct,
        fr,
        precision,
        false,
        false,
        &subject,
        &vec![],
        &clip,
    );
}
