mod common;

use clipper2_rs::{
    Paths64, Point64, Rect64, RectD, area_paths, rect_clip, rect_clip_d, rect_clip_lines,
    rect_clip_lines_d,
};
use common::*;

fn random_rect(rng: &mut Rng, shape: Shape) -> Rect64 {
    let p = random_paths(rng, shape);
    let (a, b) = (p[0][0], p[0][1 % p[0].len()]);
    Rect64::new(a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y))
}

fn rr(r: &Rect64) -> (i64, i64, i64, i64) {
    (r.left, r.top, r.right, r.bottom)
}

#[test]
fn random_rect_clip_matches_reference() {
    for shape in Shape::ALL {
        for seed in 0..cases() as u64 * 4 {
            let mut rng = Rng::new(seed * 122_949_829 + shape as u64);
            let rect = random_rect(&mut rng, shape);
            let paths = random_paths(&mut rng, shape);
            let input = format!("rect={rect:?}\npaths={:?}", to_ref(&paths));
            assert_paths_eq(
                &format!("rectclip-{shape:?}-{seed}"),
                &input,
                &rect_clip(&rect, &paths),
                &clipper2_ref::rect_clip(rr(&rect), &to_ref(&paths)),
            );
            let lines = random_open_paths(&mut rng, shape);
            let input = format!("rect={rect:?}\nlines={:?}", to_ref(&lines));
            assert_paths_eq(
                &format!("rectclip-lines-{shape:?}-{seed}"),
                &input,
                &rect_clip_lines(&rect, &lines),
                &clipper2_ref::rect_clip_lines(rr(&rect), &to_ref(&lines)),
            );
        }
    }
}

#[test]
fn random_rect_clip_d_matches_reference() {
    for seed in 0..cases() as u64 {
        let mut rng = Rng::new(seed * 553 + 1);
        let precision = [0, 2, 3][rng.below(3)];
        let paths = random_paths_d(&mut rng, Shape::Medium, precision + 1);
        let (x0, x1) = (
            rng.range_f64(-5000.0, 5000.0),
            rng.range_f64(-5000.0, 5000.0),
        );
        let (y0, y1) = (
            rng.range_f64(-5000.0, 5000.0),
            rng.range_f64(-5000.0, 5000.0),
        );
        let rect = RectD::new(x0.min(x1), y0.min(y1), x0.max(x1), y0.max(y1));
        let r4 = (rect.left, rect.top, rect.right, rect.bottom);
        let input = format!("p={precision} rect={rect:?}\n{:?}", to_ref_d(&paths));
        let ours = rect_clip_d(&rect, &paths, precision).unwrap();
        let reference = clipper2_ref::rect_clip_d(r4, &to_ref_d(&paths), precision, false).unwrap();
        assert_paths_d_eq(&format!("rectclip-d-{seed}"), &input, &ours, &reference);
        let ours = rect_clip_lines_d(&rect, &paths, precision).unwrap();
        let reference = clipper2_ref::rect_clip_d(r4, &to_ref_d(&paths), precision, true).unwrap();
        assert_paths_d_eq(
            &format!("rectclip-lines-d-{seed}"),
            &input,
            &ours,
            &reference,
        );
    }
}

fn p(pts: &[(i64, i64)]) -> Vec<Point64> {
    pts.iter().map(|&(x, y)| Point64::new(x, y)).collect()
}

#[test]
fn rect_clip_basic_cases() {
    let sq = vec![p(&[(0, 0), (10, 0), (10, 10), (0, 10)])];
    assert_eq!(
        area_paths(&rect_clip(&Rect64::new(2, 2, 5, 5), &sq)).abs(),
        9.0
    );
    assert!(rect_clip(&Rect64::new(20, 20, 30, 30), &sq).is_empty());
    let inside = rect_clip(&Rect64::new(-5, -5, 50, 50), &sq);
    assert_eq!(area_paths(&inside).abs(), 100.0);
}

#[test]
fn degenerate_inputs_match_reference() {
    let rects = [
        Rect64::new(0, 0, 10, 10),
        Rect64::new(5, 5, 5, 10),
        Rect64::new(0, 0, 0, 0),
        Rect64::new(10, 10, 0, 0),
    ];
    let inputs: Vec<Paths64> = vec![
        vec![vec![]],
        vec![p(&[(3, 3)])],
        vec![p(&[(1, 1), (8, 8)])],
        vec![p(&[(0, 0), (5, 5), (10, 10), (2, 2)])],
        vec![p(&[(0, 0), (10, 0), (10, 10), (0, 10)])],
        vec![p(&[(-5, 5), (15, 5), (15, 6), (-5, 6)])],
        vec![p(&[(0, 0), (0, 0), (10, 0), (10, 10)])],
    ];
    for (ri, rect) in rects.iter().enumerate() {
        for (pi, paths) in inputs.iter().enumerate() {
            let input = format!("rect={rect:?}\n{:?}", to_ref(paths));
            assert_paths_eq(
                &format!("rect-degenerate-{ri}-{pi}"),
                &input,
                &rect_clip(rect, paths),
                &clipper2_ref::rect_clip(rr(rect), &to_ref(paths)),
            );
            assert_paths_eq(
                &format!("rect-lines-degenerate-{ri}-{pi}"),
                &input,
                &rect_clip_lines(rect, paths),
                &clipper2_ref::rect_clip_lines(rr(rect), &to_ref(paths)),
            );
        }
    }
}
