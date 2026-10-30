mod common;

use clipper2_rs::{
    FillRule, Paths64, Point64, TriangulateError, area, triangulate, triangulate_d, union_self,
};
use common::*;

fn code(r: &Result<Paths64, TriangulateError>) -> u8 {
    match r {
        Ok(_) => 0,
        Err(TriangulateError::Fail) => 1,
        Err(TriangulateError::NoPolygons) => 2,
        Err(TriangulateError::PathsIntersect) => 3,
        Err(TriangulateError::CoordinateOutOfRange) => 4,
        Err(TriangulateError::Degenerate) => 5,
        Err(e) => panic!("unexpected error {e:?}"),
    }
}

fn compare(case: &str, paths: &Paths64, delaunay: bool) {
    let input = format!("delaunay={delaunay}\n{:?}", to_ref(paths));
    let ours = triangulate(paths, delaunay);
    // Degenerate means upstream's path walk would never end (it allocates without bound), so
    // the reference cannot be run on this input.
    if ours == Err(TriangulateError::Degenerate) {
        return;
    }
    let (rc, rp) = clipper2_ref::triangulate(&to_ref(paths), delaunay);
    // An exception thrown inside upstream (255) is reported as Fail.
    let rc = if rc == 255 { 1 } else { rc };
    assert_same(case, &input, &code(&ours), &rc);
    if let Ok(t) = ours {
        assert_paths_eq(case, &input, &t, &rp);
    }
}

#[test]
fn random_triangulation_matches_reference() {
    for shape in [Shape::SmallGrid, Shape::Medium, Shape::Star] {
        for seed in 0..cases() as u64 {
            let mut rng = Rng::new(seed * 413_158_523 + shape as u64);
            let raw = random_paths(&mut rng, shape);
            let clean = union_self(&raw, FillRule::NonZero).unwrap();
            compare(
                &format!("tri-clean-{shape:?}-{seed}"),
                &clean,
                seed % 2 == 0,
            );
            compare(&format!("tri-raw-{shape:?}-{seed}"), &raw, seed % 2 == 1);
        }
    }
}

#[test]
fn random_triangulation_d_matches_reference() {
    for seed in 0..cases() as u64 {
        let mut rng = Rng::new(seed * 17 + 2);
        let dp = [-1, 0, 2, 9][rng.below(4)];
        let paths = random_paths_d(&mut rng, Shape::Star, 2);
        let ours = triangulate_d(&paths, dp, seed % 2 == 0);
        if ours == Err(TriangulateError::Degenerate) {
            continue; // upstream would not terminate
        }
        let (rc, rp) = clipper2_ref::triangulate_d(&to_ref_d(&paths), dp, seed % 2 == 0);
        let input = format!("dp={dp}\n{:?}", to_ref_d(&paths));
        match ours {
            Ok(t) => {
                assert_same(&format!("tri-d-{seed}"), &input, &0u8, &rc);
                assert_paths_d_eq(&format!("tri-d-{seed}"), &input, &t, &rp);
            }
            Err(e) => assert_ne!(rc, 0, "tri-d-{seed}: ours {e:?}, reference succeeded"),
        }
    }
}

fn p(pts: &[(i64, i64)]) -> Vec<Point64> {
    pts.iter().map(|&(x, y)| Point64::new(x, y)).collect()
}

#[test]
fn square_and_holed_square() {
    let sq = vec![p(&[(0, 0), (10, 0), (10, 10), (0, 10)])];
    let t = triangulate(&sq, true).unwrap();
    assert_eq!(t.len(), 2);
    assert_eq!(t.iter().map(|x| area(x).abs()).sum::<f64>(), 100.0);
    let holed = vec![
        p(&[(0, 0), (30, 0), (30, 30), (0, 30)]),
        p(&[(10, 10), (10, 20), (20, 20), (20, 10)]),
    ];
    let t = triangulate(&holed, true).unwrap();
    assert_eq!(t.iter().map(|x| area(x).abs()).sum::<f64>(), 900.0 - 100.0);
}

#[test]
fn triangulate_rejects_like_reference() {
    compare("tri-empty", &vec![], true);
    compare("tri-empty-path", &vec![vec![]], true);
    compare(
        "tri-intersecting",
        &vec![
            p(&[(0, 0), (10, 0), (10, 10), (0, 10)]),
            p(&[(5, 5), (15, 5), (15, 15), (5, 15)]),
        ],
        true,
    );
    compare(
        "tri-bowtie",
        &vec![p(&[(0, 0), (10, 10), (10, 0), (0, 10)])],
        false,
    );
    compare("tri-collinear", &vec![p(&[(0, 0), (5, 5), (10, 10)])], true);
    compare("tri-tiny", &vec![p(&[(0, 0), (1, 0), (0, 1)])], true);
}

/// Upstream loops forever (allocating) on this self-intersecting input; found by seed 62.
#[test]
fn non_terminating_input_is_reported_as_degenerate() {
    let paths = vec![
        p(&[(14, 15), (18, 7), (1, 3), (8, 6), (0, 14)]),
        p(&[
            (8, 10),
            (12, 16),
            (0, 9),
            (13, 14),
            (19, 10),
            (17, 2),
            (6, 10),
            (0, 5),
            (19, 8),
            (11, 6),
            (12, 16),
        ]),
    ];
    assert_eq!(
        triangulate(&paths, false),
        Err(TriangulateError::Degenerate)
    );
}

/// Here the walk overshoots i0 inside the ascending bound and never leaves it, so a check at
/// the top of the outer loop alone does not stop it.
#[test]
fn walk_stuck_in_ascending_bound_is_reported_as_degenerate() {
    let paths = vec![
        p(&[(1, 3), (3, 0), (1, 3), (3, 3), (1, 1), (3, 3)]),
        p(&[(1, 0), (2, 1), (1, 3), (3, 1), (0, 3)]),
        p(&[
            (2, 4),
            (4, 4),
            (1, 1),
            (2, 1),
            (0, 2),
            (2, 3),
            (2, 2),
            (4, 4),
            (1, 3),
            (2, 2),
        ]),
    ];
    for delaunay in [false, true] {
        assert_eq!(
            triangulate(&paths, delaunay),
            Err(TriangulateError::Degenerate)
        );
    }
}

/// Tiny random polygons are mostly self-intersecting; about 4% of them never terminate upstream.
#[test]
fn tiny_self_intersecting_paths_match_reference() {
    let mut degenerate = 0;
    for seed in 0..cases() as u64 {
        let mut rng = Rng::new(seed * 7_919 + 5);
        let range = 4 + rng.below(40);
        let path_count = 1 + rng.below(3);
        let paths: Paths64 = (0..path_count)
            .map(|_| {
                let len = 3 + rng.below(14);
                (0..len)
                    .map(|_| Point64::new(rng.below(range) as i64, rng.below(range) as i64))
                    .collect()
            })
            .collect();
        for delaunay in [false, true] {
            if triangulate(&paths, delaunay) == Err(TriangulateError::Degenerate) {
                degenerate += 1;
            }
            compare(&format!("tri-tiny-{seed}-{delaunay}"), &paths, delaunay);
        }
    }
    assert!(degenerate > 0, "no degenerate walk exercised");
}

fn path(pts: &[(i64, i64)]) -> Paths64 {
    vec![pts.iter().map(|&(x, y)| Point64::new(x, y)).collect()]
}

/// Upstream #1055 (internal error) and #1056 (endless loop in `FixupEdgeIntersects`), both
/// caused by testing segments that share an end point for intersection.
#[test]
fn upstream_issue_paths_triangulate() {
    let issue_1055 = path(&[
        (11633393715823668, 6366352983210480),
        (14032328196566542, 6555359949515638),
        (14003250230676264, 5028765304975276),
        (15762468849501396, 5101460296319981),
        (16314950751571738, 10044719195429914),
        (14483037186691384, 10248265162321660),
        (13814243350057526, 7863869702662485),
        (11458925905425222, 8241883628441500),
        (12205067114667274, 10651552712182230),
        (12142356793566838, 9750353358707920),
        (13180182022039582, 9234473037762504),
        (13871076135814082, 9630059724866616),
        (16662324413960174, 13234316080439676),
        (16742777330712046, 14390491256863804),
        (15704952102239302, 14906371577809220),
        (15014057988464802, 14510784890705108),
        (12324797185103114, 11038221945230514),
        (12345804735781466, 11106066000441144),
        (9888713892791512, 11091526796554952),
        (10324883930311650, 7907486596870628),
        (9409339746297270, 4854375710136508),
        (11473464729830542, 4229120452027344),
    ]);
    let issue_1056 = path(&[
        (4956510274156402, 1949095339415813),
        (4770558614626881, 1728685063645521),
        (4773228611416619, 1726446939526986),
        (4775938071598222, 1724256755070940),
    ]);
    for (name, paths) in [("1055", &issue_1055), ("1056", &issue_1056)] {
        assert!(triangulate(paths, true).is_ok(), "issue {name}");
        compare(&format!("issue-{name}"), paths, true);
    }
}
