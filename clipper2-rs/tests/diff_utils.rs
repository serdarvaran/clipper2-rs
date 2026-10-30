mod common;

use clipper2_rs::*;
use common::*;

const EPS: [f64; 3] = [0.5, 2.0, 10.0];

#[test]
fn simplify_and_rdp_match_reference() {
    for shape in Shape::ALL {
        for seed in 0..cases() as u64 {
            let mut rng = Rng::new(seed * 982_451_653 + shape as u64);
            let paths = if seed % 2 == 0 {
                random_paths(&mut rng, shape)
            } else {
                random_open_paths(&mut rng, shape)
            };
            let eps = EPS[rng.below(3)] * if shape == Shape::Large { 1e12 } else { 1.0 };
            let closed = seed % 3 != 0;
            let input = format!("eps={eps} closed={closed}\n{:?}", to_ref(&paths));
            let (_, r) =
                clipper2_ref::util(0, eps, closed as i32, 0, 0, 0.0, 0.0, 0, &to_ref(&paths))
                    .unwrap();
            assert_paths_eq(
                &format!("simplify-{shape:?}-{seed}"),
                &input,
                &simplify_paths(&paths, eps, closed),
                &r,
            );
            let (_, r) = clipper2_ref::util(1, eps, 0, 0, 0, 0.0, 0.0, 0, &to_ref(&paths)).unwrap();
            assert_paths_eq(
                &format!("rdp-{shape:?}-{seed}"),
                &input,
                &ramer_douglas_peucker_paths(&paths, eps),
                &r,
            );
            let (_, r) =
                clipper2_ref::util(2, 0.0, (!closed) as i32, 0, 0, 0.0, 0.0, 0, &to_ref(&paths))
                    .unwrap();
            assert_paths_eq(
                &format!("trim-{shape:?}-{seed}"),
                &input,
                &[trim_collinear(&paths[0], !closed)],
                &r,
            );
            let (_, r) =
                clipper2_ref::util(6, 0.0, 0, 13, -7, 0.0, 0.0, 0, &to_ref(&paths)).unwrap();
            assert_paths_eq(
                &format!("translate-{shape:?}-{seed}"),
                &input,
                &translate_paths(&paths, 13, -7),
                &r,
            );
        }
    }
}

#[test]
fn floating_point_utilities_match_reference() {
    for seed in 0..cases() as u64 {
        let mut rng = Rng::new(seed * 7_919 + 3);
        let paths = random_paths_d(&mut rng, Shape::Medium, 2);
        let eps = EPS[rng.below(3)] / 10.0;
        let closed = seed % 2 == 0;
        let input = format!("eps={eps}\n{:?}", to_ref_d(&paths));
        let (_, r) = clipper2_ref::util_d(
            0,
            eps,
            closed as i32,
            0,
            0.0,
            0.0,
            0.0,
            0.0,
            0,
            &to_ref_d(&paths),
        )
        .unwrap();
        assert_paths_d_eq(
            &format!("simplify-d-{seed}"),
            &input,
            &simplify_paths(&paths, eps, closed),
            &r,
        );
        let (_, r) =
            clipper2_ref::util_d(1, eps, 0, 0, 0.0, 0.0, 0.0, 0.0, 0, &to_ref_d(&paths)).unwrap();
        assert_paths_d_eq(
            &format!("rdp-d-{seed}"),
            &input,
            &ramer_douglas_peucker_paths(&paths, eps),
            &r,
        );
        let precision = [0, 2, 4][rng.below(3)];
        let (_, r) = clipper2_ref::util_d(
            2,
            0.0,
            (!closed) as i32,
            precision,
            0.0,
            0.0,
            0.0,
            0.0,
            0,
            &to_ref_d(&paths),
        )
        .unwrap();
        assert_paths_d_eq(
            &format!("trim-d-{seed}"),
            &input,
            &[trim_collinear_d(&paths[0], precision, !closed).unwrap()],
            &r,
        );
        let (x, y) = (
            rng.range_f64(-10_000.0, 10_000.0),
            rng.range_f64(-10_000.0, 10_000.0),
        );
        let (ret, _) =
            clipper2_ref::util_d(3, 0.0, 0, 0, x, y, 0.0, 0.0, 0, &to_ref_d(&paths)).unwrap();
        assert_eq!(
            point_in_polygon(PointD::new(x, y), &paths[0]) as i32,
            ret,
            "pip-d-{seed}"
        );
    }
}

#[test]
fn point_in_polygon_and_ellipse_match_reference() {
    for shape in Shape::ALL {
        for seed in 0..cases() as u64 {
            let mut rng = Rng::new(seed * 15_485_867 + shape as u64);
            let paths = random_paths(&mut rng, shape);
            let r = if shape == Shape::SmallGrid {
                20
            } else {
                10_000
            };
            let pt = if seed % 3 == 0 {
                paths[0][rng.below(paths[0].len())]
            } else {
                Point64::new(rng.range_i64(-r, r), rng.range_i64(-r, r))
            };
            let (ret, _) =
                clipper2_ref::util(3, 0.0, 0, pt.x, pt.y, 0.0, 0.0, 0, &to_ref(&paths)).unwrap();
            assert_eq!(
                point_in_polygon(pt, &paths[0]) as i32,
                ret,
                "pip-{shape:?}-{seed}"
            );
        }
    }
    for seed in 0..cases() as u64 {
        let mut rng = Rng::new(seed + 17);
        let c = Point64::new(rng.range_i64(-1000, 1000), rng.range_i64(-1000, 1000));
        let (rx, ry) = (rng.range_f64(-1.0, 300.0), rng.range_f64(-1.0, 300.0));
        let steps = [0, 1, 2, 3, 17, 100][rng.below(6)];
        let (_, r) = clipper2_ref::util(4, 0.0, 0, c.x, c.y, rx, ry, steps, &[]).unwrap();
        assert_paths_eq(
            &format!("ellipse-{seed}"),
            &format!("{c:?} {rx} {ry} {steps}"),
            &[ellipse(c, rx, ry, steps as usize)],
            &r,
        );
        let cd = PointD::new(c.x as f64 / 3.0, c.y as f64 / 7.0);
        let (_, r) = clipper2_ref::util_d(4, 0.0, 0, 0, cd.x, cd.y, rx, ry, steps, &[]).unwrap();
        assert_paths_d_eq(
            &format!("ellipse-d-{seed}"),
            "",
            &[ellipse(cd, rx, ry, steps as usize)],
            &r,
        );
    }
}

#[test]
fn polytree_helpers_match_reference() {
    for seed in 0..cases() as u64 {
        let mut rng = Rng::new(seed * 5_003 + 9);
        let paths = [
            random_paths(&mut rng, Shape::Star),
            random_paths(&mut rng, Shape::Medium),
        ]
        .concat();
        let fr = [FillRule::EvenOdd, FillRule::NonZero][rng.below(2)];
        let mut c = Clipper64::new();
        c.add_subject(&paths);
        let (tree, _) = c.execute_tree(ClipType::Union, fr).unwrap();
        let (ret, r) =
            clipper2_ref::util(5, 0.0, fr as i32, 0, 0, 0.0, 0.0, 0, &to_ref(&paths)).unwrap();
        assert_paths_eq(
            &format!("polytree-to-paths-{seed}"),
            "",
            &poly_tree_to_paths(&tree),
            &r,
        );
        assert_eq!(
            check_polytree_fully_contains_children(&tree),
            ret == 1,
            "check-{seed}"
        );
    }
}

#[test]
fn boolean_convenience_functions_match_the_clipper() {
    let mut rng = Rng::new(4);
    for _ in 0..50 {
        let (s, c) = (
            random_paths(&mut rng, Shape::Medium),
            random_paths(&mut rng, Shape::Medium),
        );
        let run = |ct| {
            let mut k = Clipper64::new();
            k.add_subject(&s);
            k.add_clip(&c);
            k.execute(ct, FillRule::NonZero).unwrap().closed
        };
        assert_eq!(
            intersect(&s, &c, FillRule::NonZero).unwrap(),
            run(ClipType::Intersection)
        );
        assert_eq!(
            union(&s, &c, FillRule::NonZero).unwrap(),
            run(ClipType::Union)
        );
        assert_eq!(
            difference(&s, &c, FillRule::NonZero).unwrap(),
            run(ClipType::Difference)
        );
        assert_eq!(xor(&s, &c, FillRule::NonZero).unwrap(), run(ClipType::Xor));
        assert_eq!(
            boolean_op(ClipType::Union, FillRule::NonZero, &s, &c).unwrap(),
            run(ClipType::Union)
        );
        let tree = boolean_op_tree(ClipType::Union, FillRule::NonZero, &s, &c).unwrap();
        assert_eq!(poly_tree_to_paths(&tree).len(), tree.len());
    }
    let bowtie = vec![vec![
        Point64::new(0, 0),
        Point64::new(10, 10),
        Point64::new(10, 0),
        Point64::new(0, 10),
    ]];
    assert_eq!(union_self(&bowtie, FillRule::NonZero).unwrap().len(), 2);
}

#[test]
fn small_helpers() {
    let tri = vec![Point64::new(0, 0), Point64::new(3, 0), Point64::new(3, 4)];
    assert_eq!(distance(tri[0], tri[2]), 5.0);
    assert_eq!(length(&tri, false), 7.0);
    assert_eq!(length(&tri, true), 12.0);
    assert!(near_collinear(
        Point64::new(0, 0),
        Point64::new(50, 1),
        Point64::new(100, 0),
        0.01
    ));
    assert!(!near_collinear(
        Point64::new(0, 0),
        Point64::new(50, 50),
        Point64::new(100, 0),
        0.01
    ));
    let sq = vec![
        Point64::new(0, 0),
        Point64::new(5, 0),
        Point64::new(10, 0),
        Point64::new(10, 10),
        Point64::new(0, 10),
    ];
    assert_eq!(trim_collinear(&sq, false).len(), 4);
    assert_eq!(
        make_path(&[1, 2, 3, 4]).unwrap(),
        vec![Point64::new(1, 2), Point64::new(3, 4)]
    );
    assert_eq!(
        make_path(&[1, 2, 3]).err(),
        Some(ClipperError::OddCoordinateCount)
    );
    assert_eq!(
        make_path_d(&[0.5, 1.5]).unwrap(),
        vec![PointD::new(0.5, 1.5)]
    );
}

#[test]
fn degenerate_inputs_match_reference() {
    let p = |v: &[(i64, i64)]| {
        v.iter()
            .map(|&(x, y)| Point64::new(x, y))
            .collect::<Path64>()
    };
    let cases: Vec<Path64> = vec![
        vec![],
        p(&[(1, 1)]),
        p(&[(1, 1), (1, 1)]),
        p(&[(0, 0), (5, 5)]),
        p(&[(0, 0), (5, 5), (10, 10)]),
        p(&[(0, 0), (5, 5), (10, 10), (3, 3)]),
        p(&[(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)]),
    ];
    for (i, path) in cases.iter().enumerate() {
        for closed in [false, true] {
            let ps = vec![path.clone()];
            let (_, r) =
                clipper2_ref::util(0, 1.0, closed as i32, 0, 0, 0.0, 0.0, 0, &to_ref(&ps)).unwrap();
            assert_paths_eq(
                &format!("deg-simplify-{i}-{closed}"),
                "",
                &simplify_paths(&ps, 1.0, closed),
                &r,
            );
            let (_, r) = clipper2_ref::util(1, 1.0, 0, 0, 0, 0.0, 0.0, 0, &to_ref(&ps)).unwrap();
            assert_paths_eq(
                &format!("deg-rdp-{i}"),
                "",
                &ramer_douglas_peucker_paths(&ps, 1.0),
                &r,
            );
            let (_, r) =
                clipper2_ref::util(2, 0.0, (!closed) as i32, 0, 0, 0.0, 0.0, 0, &to_ref(&ps))
                    .unwrap();
            assert_paths_eq(
                &format!("deg-trim-{i}-{closed}"),
                "",
                &[trim_collinear(path, !closed)],
                &r,
            );
        }
    }
}
