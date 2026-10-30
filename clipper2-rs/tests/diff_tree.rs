mod common;

use clipper2_rs::{ClipType, Clipper64, FillRule, PathType, Paths64, ReuseableDataContainer64};
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

fn compare_tree(
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
    let ours = c.execute_tree(ct, fr);
    let reference = clipper2_ref::boolean_tree(
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
        (Ok((tree, open_out)), Ok(r)) => {
            assert_same(case, &input, &flatten_tree(&tree), &ref_tree_nodes(&r));
            assert_paths_eq(&format!("{case}-open"), &input, &open_out, &r.open);
        }
        (o, r) => assert_same(case, &input, &o.is_ok(), &r.is_ok()),
    }
}

#[test]
fn random_tree_matches_reference() {
    for ct in CLIP_TYPES {
        for fr in FILL_RULES {
            for shape in Shape::ALL {
                for seed in 0..cases() as u64 {
                    let mut rng = Rng::new(seed * 104_729 + ct as u64 * 17 + fr as u64);
                    let subject = random_paths(&mut rng, shape);
                    let clip = random_paths(&mut rng, shape);
                    let case = format!("tree-{ct:?}-{fr:?}-{shape:?}-{seed}");
                    compare_tree(
                        &case,
                        ct,
                        fr,
                        seed & 1 == 1,
                        seed & 2 == 2,
                        &subject,
                        &vec![],
                        &clip,
                    );
                }
            }
        }
    }
}

#[test]
fn nested_squares_make_outer_hole_island() {
    let sq = |a: i64, b: i64| {
        vec![
            clipper2_rs::Point64::new(a, a),
            clipper2_rs::Point64::new(b, a),
            clipper2_rs::Point64::new(b, b),
            clipper2_rs::Point64::new(a, b),
        ]
    };
    let subject = vec![sq(0, 30), sq(5, 25), sq(10, 20)];
    let mut c = Clipper64::new();
    c.add_subject(&subject);
    let (tree, _) = c.execute_tree(ClipType::Union, FillRule::EvenOdd).unwrap();
    let shape: Vec<(i32, bool)> = flatten_tree(&tree)
        .into_iter()
        .map(|(_, p, h)| (p, h))
        .collect();
    assert_eq!(shape, vec![(-1, false), (0, true), (1, false)]);
    assert_eq!(tree.area(), 900.0 - 400.0 + 100.0);
}

#[test]
fn reuseable_data_gives_same_result_as_fresh_inputs() {
    let mut rng = Rng::new(42);
    for _ in 0..50 {
        let subject = random_paths(&mut rng, Shape::Medium);
        let clip = random_paths(&mut rng, Shape::Medium);
        let mut data = ReuseableDataContainer64::new();
        data.add_paths(&subject, PathType::Subject, false);
        data.add_paths(&clip, PathType::Clip, false);
        let mut reused = Clipper64::new();
        reused.add_reuseable_data(&data);
        let mut fresh = Clipper64::new();
        fresh.add_subject(&subject);
        fresh.add_clip(&clip);
        for ct in CLIP_TYPES {
            assert_eq!(
                reused.execute(ct, FillRule::NonZero).unwrap(),
                fresh.execute(ct, FillRule::NonZero).unwrap()
            );
        }
    }
}
