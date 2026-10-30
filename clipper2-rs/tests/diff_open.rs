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

fn compare_open(
    case: &str,
    ct: ClipType,
    fr: FillRule,
    pc: bool,
    rev: bool,
    subject: &Paths64,
    open: &Paths64,
    clip: &Paths64,
) {
    let mut c = Clipper64::new();
    c.set_preserve_collinear(pc);
    c.set_reverse_solution(rev);
    c.add_subject(subject);
    c.add_open_subject(open);
    c.add_clip(clip);
    let ours = c.execute(ct, fr);
    let reference = clipper2_ref::boolean(
        ct as u8,
        fr as u8,
        pc,
        rev,
        &to_ref(subject),
        &to_ref(open),
        &to_ref(clip),
    );
    let input = format!(
        "ct={ct:?} fr={fr:?} pc={pc} rev={rev}\nsubject={:?}\nopen={:?}\nclip={:?}",
        to_ref(subject),
        to_ref(open),
        to_ref(clip)
    );
    match (ours, reference) {
        (Ok(o), Ok(r)) => {
            assert_paths_eq(case, &input, &o.closed, &r.closed);
            assert_paths_eq(&format!("{case}-open"), &input, &o.open, &r.open);
        }
        (o, r) => assert_same(case, &input, &o.is_ok(), &r.is_ok()),
    }
}

#[test]
fn random_open_paths_match_reference() {
    for ct in CLIP_TYPES {
        for fr in FILL_RULES {
            for shape in Shape::ALL {
                for seed in 0..cases() as u64 {
                    let mut rng = Rng::new(seed * 15_485_863 + ct as u64 * 13 + fr as u64);
                    let subject = if seed % 3 == 0 {
                        vec![]
                    } else {
                        random_paths(&mut rng, shape)
                    };
                    let open = random_open_paths(&mut rng, shape);
                    let clip = random_paths(&mut rng, shape);
                    let case = format!("open-{ct:?}-{fr:?}-{shape:?}-{seed}");
                    compare_open(
                        &case,
                        ct,
                        fr,
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

fn p(pts: &[(i64, i64)]) -> Vec<Point64> {
    pts.iter().map(|&(x, y)| Point64::new(x, y)).collect()
}

#[test]
fn open_path_edge_cases_match_reference() {
    let clip = vec![p(&[(2, 0), (8, 0), (8, 10), (2, 10)])];
    let cases: Vec<(&str, Paths64)> = vec![
        ("single-point", vec![p(&[(5, 5)])]),
        ("two-same-points", vec![p(&[(5, 5), (5, 5)])]),
        ("on-clip-edge", vec![p(&[(2, 0), (2, 10)])]),
        ("along-top-edge", vec![p(&[(0, 0), (10, 0)])]),
        ("crossing", vec![p(&[(0, 5), (10, 5)])]),
        (
            "zigzag",
            vec![p(&[(0, 0), (5, 10), (10, 0), (5, 5), (0, 10)])],
        ),
    ];
    for (name, open) in &cases {
        for ct in [
            ClipType::Intersection,
            ClipType::Difference,
            ClipType::Union,
        ] {
            for fr in FILL_RULES {
                compare_open(
                    &format!("open-edge-{name}-{ct:?}-{fr:?}"),
                    ct,
                    fr,
                    true,
                    false,
                    &vec![],
                    open,
                    &clip,
                );
            }
        }
    }
}

#[test]
fn horizontal_line_through_square_is_clipped_to_square() {
    let mut c = Clipper64::new();
    c.add_open_subject(&vec![p(&[(0, 5), (10, 5)])]);
    c.add_clip(&vec![p(&[(2, 0), (8, 0), (8, 10), (2, 10)])]);
    let s = c
        .execute(ClipType::Intersection, FillRule::NonZero)
        .unwrap();
    assert_eq!(s.open.len(), 1);
    let mut xs: Vec<i64> = s.open[0].iter().map(|p| p.x).collect();
    xs.sort();
    assert_eq!((xs, s.open[0][0].y), (vec![2, 8], 5));
}
