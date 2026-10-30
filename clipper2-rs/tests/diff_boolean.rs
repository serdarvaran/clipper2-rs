mod common;

use clipper2_rs::{ClipType, Clipper64, FillRule, Paths64, Point64};
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

fn compare(
    case: &str,
    ct: ClipType,
    fr: FillRule,
    pc: bool,
    rev: bool,
    subject: &Paths64,
    clip: &Paths64,
) {
    let mut c = Clipper64::new();
    c.set_preserve_collinear(pc);
    c.set_reverse_solution(rev);
    c.add_subject(subject);
    c.add_clip(clip);
    let ours = c.execute(ct, fr);
    let reference = clipper2_ref::boolean(
        ct as u8,
        fr as u8,
        pc,
        rev,
        &to_ref(subject),
        &[],
        &to_ref(clip),
    );
    let input = format!(
        "ct={ct:?} fr={fr:?} pc={pc} rev={rev}\nsubject={:?}\nclip={:?}",
        to_ref(subject),
        to_ref(clip)
    );
    match (ours, reference) {
        (Ok(o), Ok(r)) => assert_paths_eq(case, &input, &o.closed, &r.closed),
        (o, r) => assert_same(case, &input, &o.is_ok(), &r.is_ok()),
    }
}

#[test]
fn random_boolean_matches_reference() {
    for ct in CLIP_TYPES {
        for fr in FILL_RULES {
            for shape in Shape::ALL {
                for seed in 0..cases() as u64 {
                    let mut rng = Rng::new(seed * 7919 + ct as u64 * 31 + fr as u64);
                    let subject = random_paths(&mut rng, shape);
                    let clip = random_paths(&mut rng, shape);
                    let case = format!("boolean-{ct:?}-{fr:?}-{shape:?}-{seed}");
                    compare(&case, ct, fr, seed & 1 == 1, seed & 2 == 2, &subject, &clip);
                }
            }
        }
    }
}

fn p(pts: &[(i64, i64)]) -> Vec<Point64> {
    pts.iter().map(|&(x, y)| Point64::new(x, y)).collect()
}

fn all_ops(name: &str, subject: &Paths64, clip: &Paths64) {
    for ct in CLIP_TYPES {
        for fr in FILL_RULES {
            for pc in [false, true] {
                compare(
                    &format!("{name}-{ct:?}-{fr:?}-{pc}"),
                    ct,
                    fr,
                    pc,
                    false,
                    subject,
                    clip,
                );
            }
        }
    }
}

#[test]
fn degenerate_inputs_match_reference() {
    let square = vec![p(&[(0, 0), (10, 0), (10, 10), (0, 10)])];
    let cases: Vec<(&str, Paths64)> = vec![
        ("empty-path", vec![vec![]]),
        ("one-point", vec![p(&[(3, 3)])]),
        ("two-points", vec![p(&[(1, 1), (8, 8)])]),
        ("collinear", vec![p(&[(0, 0), (5, 5), (10, 10), (2, 2)])]),
        (
            "duplicates",
            vec![p(&[
                (0, 0),
                (0, 0),
                (10, 0),
                (10, 0),
                (10, 10),
                (0, 10),
                (0, 0),
            ])],
        ),
        ("flat", vec![p(&[(0, 5), (10, 5), (4, 5)])]),
    ];
    for (name, paths) in &cases {
        all_ops(&format!("degenerate-{name}-as-subject"), paths, &square);
        all_ops(&format!("degenerate-{name}-as-clip"), &square, paths);
    }
}

#[test]
fn coincident_and_touching_match_reference() {
    let a = vec![p(&[(0, 0), (10, 0), (10, 10), (0, 10)])];
    let cases: Vec<(&str, Paths64, Paths64)> = vec![
        ("self", a.clone(), a.clone()),
        (
            "vertex-touch",
            a.clone(),
            vec![p(&[(10, 10), (20, 10), (20, 20), (10, 20)])],
        ),
        (
            "shared-edge",
            a.clone(),
            vec![p(&[(10, 0), (20, 0), (20, 10), (10, 10)])],
        ),
        (
            "nested",
            a.clone(),
            vec![p(&[(2, 2), (8, 2), (8, 8), (2, 8)])],
        ),
        (
            "reversed-self",
            a.clone(),
            vec![p(&[(0, 10), (10, 10), (10, 0), (0, 0)])],
        ),
    ];
    for (name, s, c) in &cases {
        all_ops(&format!("coincident-{name}"), s, c);
    }
}
