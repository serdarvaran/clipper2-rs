//! Triangulation, checked against its definition rather than bit for bit against the C++
//! library: upstream has withdrawn its triangulation, and its results are wrong on a part of
//! valid input. The reference still guards against regressions: wherever its result is
//! valid, ours must be valid too.
//!
//! A result for non-intersecting input is valid when (all in exact integer arithmetic)
//! - every triangle has positive orientation and non-zero area;
//! - every triangle lies inside the input region, no two triangles overlap, and their areas
//!   add up to the region's area, so together they cover it exactly;
//! - no vertex lies inside another triangle's edge (no T-junctions);
//! - with Delaunay, no vertex lies strictly inside the circumcircle of the triangle on the
//!   other side of an unconstrained edge.
//!
//! The boolean engine is not used to check coverage: its fixed-point sweep drops slivers less
//! than one unit wide, which a triangulation can legitimately contain.

mod common;

use std::collections::HashMap;

use clipper2_rs::{
    FillRule, MAX_COORD, MIN_COORD, Path64, Paths64, PathsD, Point64, TriangulateError,
    scale_paths, triangulate, triangulate_d, union_self,
};
use common::*;

/// Twice the signed area, exactly.
fn area2(path: &Path64) -> i128 {
    let n = path.len();
    (0..n)
        .map(|i| {
            let (a, b) = (path[i], path[(i + 1) % n]);
            a.x as i128 * b.y as i128 - b.x as i128 * a.y as i128
        })
        .sum()
}

fn area2_paths(paths: &Paths64) -> i128 {
    paths.iter().map(area2).sum()
}

/// `> 0` when `d` lies strictly inside the circumcircle of the counter-clockwise triangle
/// `a, b, c`.
fn in_circle(a: Point64, b: Point64, c: Point64, d: Point64) -> i128 {
    let row = |p: Point64| {
        let (x, y) = ((p.x - d.x) as i128, (p.y - d.y) as i128);
        (x, y, x * x + y * y)
    };
    let (ax, ay, a2) = row(a);
    let (bx, by, b2) = row(b);
    let (cx, cy, c2) = row(c);
    ax * (by * c2 - b2 * cy) - ay * (bx * c2 - b2 * cx) + a2 * (bx * cy - by * cx)
}

/// Whether segment `a`-`b` lies on input edges (one, or several collinear ones in a row).
fn is_constrained(paths: &Paths64, a: Point64, b: Point64) -> bool {
    let along = |p: Point64| {
        (p.x - a.x) as i128 * (b.x - a.x) as i128 + (p.y - a.y) as i128 * (b.y - a.y) as i128
    };
    let mut covered: Vec<(i128, i128)> = paths
        .iter()
        .flat_map(|p| (0..p.len()).map(move |i| (p[i], p[(i + 1) % p.len()])))
        .filter(|&(s, e)| cross(a, b, s) == 0 && cross(a, b, e) == 0)
        .map(|(s, e)| (along(s).min(along(e)), along(s).max(along(e))))
        .collect();
    covered.sort_unstable();
    let mut reach = 0;
    for (lo, hi) in covered {
        if lo <= reach {
            reach = reach.max(hi);
        }
    }
    reach >= along(b)
}

/// Whether the interiors of two counter-clockwise triangles are disjoint: some edge of one has
/// the other on its closed outer side.
fn interiors_disjoint(t: &Path64, u: &Path64) -> bool {
    let separates = |a: &Path64, b: &Path64| {
        (0..3).any(|i| b.iter().all(|&r| cross(a[i], a[(i + 1) % 3], r) <= 0))
    };
    separates(t, u) || separates(u, t)
}

/// Whether segment `a`-`b` meets the open interior of the counter-clockwise triangle `t`
/// (separating axes: the triangle's edges and the segment's line).
fn meets_interior(a: Point64, b: Point64, t: &Path64) -> bool {
    let edge_separates =
        (0..3).any(|i| cross(t[i], t[(i + 1) % 3], a) <= 0 && cross(t[i], t[(i + 1) % 3], b) <= 0);
    let sides: Vec<i128> = t.iter().map(|&p| cross(a, b, p).signum()).collect();
    let line_separates = sides.iter().all(|&s| s >= 0) || sides.iter().all(|&s| s <= 0);
    !edge_separates && !line_separates
}

/// The input's boundary: its edges split at the vertices on them, without pieces that run
/// along each other in opposite directions (a zero-width slit or spike bounds no area).
fn boundary(paths: &Paths64) -> Vec<(Point64, Point64)> {
    let vertices: Vec<Point64> = paths.iter().flatten().copied().collect();
    let mut pieces: HashMap<(Point64, Point64), i32> = HashMap::new();
    for p in paths {
        for i in 0..p.len() {
            let (a, b) = (p[i], p[(i + 1) % p.len()]);
            if a == b {
                continue;
            }
            let along = |q: Point64| {
                (q.x - a.x) as i128 * (b.x - a.x) as i128
                    + (q.y - a.y) as i128 * (b.y - a.y) as i128
            };
            let mut cuts: Vec<Point64> = vertices
                .iter()
                .copied()
                .filter(|&q| strictly_on(q, a, b))
                .chain([a, b])
                .collect();
            cuts.sort_unstable_by_key(|&q| along(q));
            cuts.dedup();
            for w in cuts.windows(2) {
                *pieces.entry((w[0], w[1])).or_default() += 1;
                *pieces.entry((w[1], w[0])).or_default() -= 1;
            }
        }
    }
    pieces
        .into_iter()
        .filter(|&(_, n)| n > 0)
        .map(|(e, _)| e)
        .collect()
}

/// Whether the counter-clockwise triangle `t` lies inside the region of the non-intersecting
/// `paths` (NonZero): no edge of their `boundary` meets its interior, and its centroid, which
/// then lies on no input edge, is inside.
fn inside_region(paths: &Paths64, boundary: &[(Point64, Point64)], t: &Path64) -> bool {
    if boundary.iter().any(|&(a, b)| meets_interior(a, b, t)) {
        return false;
    }
    // winding number at the centroid, in coordinates scaled by 3
    let (cx, cy) = (
        t.iter().map(|p| p.x as i128).sum::<i128>(),
        t.iter().map(|p| p.y as i128).sum::<i128>(),
    );
    let mut winding = 0;
    for path in paths {
        for i in 0..path.len() {
            let (a, b) = (path[i], path[(i + 1) % path.len()]);
            let (ax, ay, bx, by) = (
                3 * a.x as i128,
                3 * a.y as i128,
                3 * b.x as i128,
                3 * b.y as i128,
            );
            let side = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
            if ay <= cy && by > cy && side > 0 {
                winding += 1;
            } else if by <= cy && ay > cy && side < 0 {
                winding -= 1;
            }
        }
    }
    winding != 0
}

/// `Err` describes the first property `tris` breaks as a triangulation of `paths`.
fn check(paths: &Paths64, tris: &Paths64, delaunay: bool) -> Result<(), String> {
    for t in tris {
        if t.len() != 3 || area2(t) <= 0 {
            return Err(format!("bad triangle {t:?}"));
        }
    }
    let (want, got) = (area2_paths(paths).abs(), area2_paths(tris));
    if want != got {
        return Err(format!(
            "area: region {want}, triangles {got} (twice the area)"
        ));
    }
    let edges = boundary(paths);
    if let Some(t) = tris.iter().find(|t| !inside_region(paths, &edges, t)) {
        return Err(format!("outside: {t:?}"));
    }
    let mut by_x: Vec<&Path64> = tris.iter().collect();
    by_x.sort_unstable_by_key(|t| t.iter().map(|p| p.x).min());
    for (i, t) in by_x.iter().enumerate() {
        let right = t.iter().map(|p| p.x).max().unwrap();
        for u in by_x[i + 1..]
            .iter()
            .take_while(|u| u.iter().map(|p| p.x).min().unwrap() < right)
        {
            if !interiors_disjoint(t, u) {
                return Err(format!("overlap: {t:?} and {u:?}"));
            }
        }
    }
    // a vertex on another triangle's edge (a T-junction) leaves the mesh non-conforming
    let mut verts: Vec<Point64> = tris.iter().flatten().copied().collect();
    verts.sort_unstable_by_key(|p| (p.x, p.y));
    verts.dedup();
    for t in tris {
        for i in 0..3 {
            let (a, b) = (t[i], t[(i + 1) % 3]);
            let lo = verts.partition_point(|p| p.x < a.x.min(b.x));
            let hi = verts.partition_point(|p| p.x <= a.x.max(b.x));
            if let Some(p) = verts[lo..hi].iter().find(|&&p| {
                p != a
                    && p != b
                    && p.y >= a.y.min(b.y)
                    && p.y <= a.y.max(b.y)
                    && area2(&vec![a, b, p]) == 0
            }) {
                return Err(format!("T-junction: {p:?} on edge {a:?}-{b:?}"));
            }
        }
    }
    if delaunay {
        let mut opposite = HashMap::new();
        for t in tris {
            for i in 0..3 {
                opposite.insert((t[i], t[(i + 1) % 3]), t[(i + 2) % 3]);
            }
        }
        for (&(a, b), &c) in &opposite {
            if let Some(&d) = opposite.get(&(b, a)) {
                if in_circle(a, b, c, d) > 0 && !is_constrained(paths, a, b) {
                    return Err(format!(
                        "not Delaunay: edge {a:?}-{b:?}, {d:?} inside {c:?}"
                    ));
                }
            }
        }
    }
    Ok(())
}

fn cross(a: Point64, b: Point64, c: Point64) -> i128 {
    (b.x - a.x) as i128 * (c.y - a.y) as i128 - (b.y - a.y) as i128 * (c.x - a.x) as i128
}

/// Whether `p` lies on segment `s`-`e` without being one of its ends.
fn strictly_on(p: Point64, s: Point64, e: Point64) -> bool {
    p != s
        && p != e
        && cross(s, e, p) == 0
        && p.x >= s.x.min(e.x)
        && p.x <= s.x.max(e.x)
        && p.y >= s.y.min(e.y)
        && p.y <= s.y.max(e.y)
}

/// Exactly what `triangulate` accepts: paths that touch only at shared vertices (no crossing,
/// overlapping or vertex-on-edge contact, no edge used twice in one direction, no overlapping
/// corners) and whose union covers their summed area (holes inside their outer paths).
fn is_non_intersecting(paths: &Paths64) -> bool {
    let segs: Vec<_> = paths
        .iter()
        .flat_map(|p| (0..p.len()).map(move |i| (p[i], p[(i + 1) % p.len()])))
        .collect();
    for (i, &(a, b)) in segs.iter().enumerate() {
        for &(c, d) in &segs[i + 1..] {
            let (d1, d2) = (cross(a, b, c).signum(), cross(a, b, d).signum());
            let (d3, d4) = (cross(c, d, a).signum(), cross(c, d, b).signum());
            // the same edge twice in the same direction: the interiors overlap
            if (a, b) == (c, d)
                || (d1 * d2 < 0 && d3 * d4 < 0)
                || strictly_on(c, a, b)
                || strictly_on(d, a, b)
                || strictly_on(a, c, d)
                || strictly_on(b, c, d)
            {
                return false;
            }
        }
    }
    !corners_overlap(paths)
        && area2_paths(&union_self(paths, FillRule::NonZero).unwrap()) == area2_paths(paths).abs()
}

/// Whether the input's winding number varies by more than one around a vertex: going round
/// it counter-clockwise, the winding rises by one across each edge that leaves the vertex and
/// falls by one across each edge that arrives. Valid input only has 0 and 1, so a vertex
/// where paths overlap (2 next to 0) shows a range of two. The union's area alone misses such
/// overlaps when they are slivers, which the boolean engine drops.
fn corners_overlap(paths: &Paths64) -> bool {
    let mut jumps: HashMap<Point64, Vec<(Point64, i32)>> = HashMap::new();
    for p in paths {
        let n = p.len();
        for i in 0..n {
            let (a, b) = (p[i], p[(i + 1) % n]);
            if a != b {
                jumps.entry(a).or_default().push((dir(a, b), 1));
                jumps.entry(b).or_default().push((dir(b, a), -1));
            }
        }
    }
    jumps.values_mut().any(|around| {
        around.sort_by(|&(d, _), &(e, _)| angle_order(d, e));
        let (mut level, mut low, mut high) = (0, 0, 0);
        for group in around.chunk_by(|&(d, _), &(e, _)| angle_order(d, e).is_eq()) {
            level += group.iter().map(|j| j.1).sum::<i32>();
            (low, high) = (low.min(level), high.max(level));
        }
        high - low > 1
    })
}

fn dir(from: Point64, to: Point64) -> Point64 {
    Point64::new(to.x - from.x, to.y - from.y)
}

/// Orders directions by their angle counter-clockwise from (1, 0).
fn angle_order(a: Point64, b: Point64) -> std::cmp::Ordering {
    let half = |d: Point64| d.y < 0 || (d.y == 0 && d.x < 0);
    half(a)
        .cmp(&half(b))
        .then_with(|| 0.cmp(&cross(Point64::new(0, 0), a, b)))
}

/// The boolean engine's union of random paths, often with holes and with polygons touching
/// at vertices. Rounding the intersections can leave tiny crossings; see
/// [`is_non_intersecting`].
fn union_input(seed: u64, shape: Shape) -> Paths64 {
    let mut rng = Rng::new(seed * 413_158_523 + shape as u64);
    union_self(&random_paths(&mut rng, shape), FillRule::NonZero).unwrap()
}

#[test]
fn checker_rejects_broken_triangulations() {
    let sq = vec![vec![
        Point64::new(0, 0),
        Point64::new(10, 0),
        Point64::new(10, 10),
        Point64::new(0, 10),
    ]];
    let t = |pts: [(i64, i64); 3]| pts.iter().map(|&(x, y)| Point64::new(x, y)).collect();
    let good = vec![
        t([(0, 0), (10, 0), (10, 10)]),
        t([(0, 0), (10, 10), (0, 10)]),
    ];
    assert_eq!(check(&sq, &good, true), Ok(()));
    let missing = vec![t([(0, 0), (10, 0), (10, 10)])];
    assert!(check(&sq, &missing, false).is_err());
    let overlapping = vec![
        t([(0, 0), (10, 0), (10, 10)]),
        t([(0, 0), (10, 10), (0, 10)]),
        t([(0, 0), (5, 0), (5, 5)]),
    ];
    assert!(check(&sq, &overlapping, false).is_err());
    // same total area, but one triangle overlaps another and one sticks out
    let shifted = vec![
        t([(0, 0), (10, 0), (10, 10)]),
        t([(0, 0), (10, 10), (0, 10)]),
    ]
    .into_iter()
    .chain([t([(0, 0), (1, 0), (0, 1)])])
    .collect::<Vec<Path64>>();
    assert!(check(&sq, &shifted, false).is_err());
    let swapped = vec![
        t([(0, 0), (10, 0), (10, 10)]),
        t([(0, 0), (10, 0), (10, 10)]),
    ];
    assert!(
        check(&sq, &swapped, false)
            .unwrap_err()
            .starts_with("overlap")
    );
    let out = vec![
        t([(0, 0), (10, 0), (10, 10)]),
        t([(0, 0), (5, 10), (-5, 10)]),
    ];
    assert!(check(&sq, &out, false).unwrap_err().starts_with("outside"));
    // a sliver less than a unit wide that the boolean engine's union drops
    let sliver = vec![vec![
        Point64::new(-6877, 2730),
        Point64::new(-1682, -2908),
        Point64::new(-1983, -2581),
    ]];
    assert_eq!(check(&sliver, &sliver, true), Ok(()));
    let clockwise = vec![
        t([(0, 0), (10, 10), (10, 0)]),
        t([(0, 0), (10, 10), (0, 10)]),
    ];
    assert!(check(&sq, &clockwise, false).is_err());
    let fan = vec![
        t([(0, 0), (10, 0), (5, 5)]),
        t([(10, 0), (10, 10), (5, 5)]),
        t([(0, 0), (5, 5), (0, 10)]),
        t([(5, 5), (10, 10), (0, 10)]),
    ];
    assert_eq!(check(&sq, &fan, false), Ok(()));
    let t_junction = vec![
        t([(0, 0), (10, 0), (10, 10)]),
        t([(0, 0), (5, 5), (0, 10)]),
        t([(5, 5), (10, 10), (0, 10)]),
    ];
    assert!(check(&sq, &t_junction, false).is_err());
    // both diagonals of a flat kite give a valid triangulation; only the short one is Delaunay
    let kite = vec![vec![
        Point64::new(0, 0),
        Point64::new(10, -1),
        Point64::new(20, 0),
        Point64::new(10, 1),
    ]];
    let thin = vec![
        t([(0, 0), (10, -1), (10, 1)]),
        t([(10, -1), (20, 0), (10, 1)]),
    ];
    let fat = vec![
        t([(0, 0), (10, -1), (20, 0)]),
        t([(0, 0), (20, 0), (10, 1)]),
    ];
    assert_eq!(check(&kite, &thin, true), Ok(()));
    assert!(check(&kite, &fat, true).is_err());
    assert_eq!(check(&kite, &fat, false), Ok(()));
}

#[test]
fn input_classification() {
    let valid = [
        // a path in the notch of another, sharing the notch vertex
        paths(&[
            &[(7, 9), (6, 1), (12, 4), (18, 0)],
            &[(12, 4), (11, 1), (14, 0)],
        ]),
        // a hole touching the outer path's bottom
        paths(&[
            &[(16, 19), (3, 11), (19, 0)],
            &[(16, 19), (16, 16), (12, 15)],
        ]),
        // a hole sharing two vertices with the outer path
        paths(&[
            &[(0, 0), (10, 0), (10, 10), (0, 10)],
            &[(0, 0), (5, 1), (10, 0)],
        ]),
        // a path touching itself
        paths(&[&[(12, 7), (10, 4), (14, 5), (7, 2), (5, 1), (14, 5), (20, 1)]]),
    ];
    for input in &valid {
        assert!(is_non_intersecting(input), "{input:?}");
    }
    let overlapping = [
        // a triangle inside a square, sharing an edge's end points, in the same direction
        paths(&[
            &[(0, 0), (10, 0), (10, 10), (0, 10)],
            &[(0, 0), (10, 0), (5, 1)],
        ]),
        // a half-unit sliver covered by both paths, which the boolean engine's union drops
        paths(&[
            &[(16, 18), (13, 13), (11, 10), (10, 9), (20, 9)],
            &[(11, 10), (12, 11), (13, 13), (10, 11), (6, 9), (10, 9)],
        ]),
    ];
    for input in &overlapping {
        assert!(!is_non_intersecting(input), "{input:?}");
    }
}

/// Survey of how the triangulation behaves on union output, split by whether the input is
/// exactly non-intersecting. Prints a summary and the first failures on non-intersecting
/// input; run with `--ignored --nocapture`.
#[test]
#[ignore]
fn survey_union_input() {
    let mut counts: HashMap<String, usize> = HashMap::new();
    let mut shown = 0;
    for shape in [Shape::SmallGrid, Shape::Medium, Shape::Star] {
        for seed in 0..cases() as u64 {
            let paths = union_input(seed, shape);
            if paths.is_empty() {
                continue;
            }
            let exact = is_non_intersecting(&paths);
            let shape = format!("{shape:?}{}", if exact { "" } else { " (crossings)" });
            for delaunay in [false, true] {
                let shown_before = shown;
                if !exact {
                    shown = 15;
                }
                let outcome = match triangulate(&paths, delaunay) {
                    Ok(tris) => match check(&paths, &tris, delaunay) {
                        Ok(()) => "ok".to_string(),
                        Err(e) => {
                            if shown < 15 {
                                shown += 1;
                                println!("{shape} seed {seed} delaunay={delaunay}: {e}");
                            }
                            format!("invalid: {}", e.split(':').next().unwrap())
                        }
                    },
                    Err(e) => {
                        if shown < 15 {
                            shown += 1;
                            println!("{shape} seed {seed} delaunay={delaunay}: {e:?}");
                        }
                        format!("error: {e:?}")
                    }
                };
                if !exact {
                    shown = shown_before;
                }
                *counts.entry(format!("{shape} {outcome}")).or_default() += 1;
            }
        }
    }
    let mut counts: Vec<_> = counts.into_iter().collect();
    counts.sort();
    for (k, n) in counts {
        println!("{n:>7}  {k}");
    }
}

fn paths(coords: &[&[(i64, i64)]]) -> Paths64 {
    coords
        .iter()
        .map(|p| p.iter().map(|&(x, y)| Point64::new(x, y)).collect())
        .collect()
}

/// Checks `input` and its reversal (clockwise outer paths).
fn assert_valid(input: &Paths64) {
    let reversed: Paths64 = input
        .iter()
        .map(|p| p.iter().rev().copied().collect())
        .collect();
    for (name, input) in [("input", input), ("reversed", &reversed)] {
        for delaunay in [false, true] {
            let tris = triangulate(input, delaunay)
                .unwrap_or_else(|e| panic!("{name}, delaunay={delaunay}: {e:?}"));
            if let Err(e) = check(input, &tris, delaunay) {
                panic!("{name}, delaunay={delaunay}: {e}\ntriangles {tris:?}");
            }
        }
    }
}

/// The edge from an inner local minimum down to the swept region crossed a polygon edge whose
/// bounding box the visibility check ruled out.
#[test]
fn loose_edge_from_inner_minimum_does_not_cross_polygon_edges() {
    assert_valid(&paths(&[&[
        (-6721, -2381),
        (-6446, -2534),
        (-6591, -1671),
        (-6586, -1644),
        (-7591, -2603),
    ]]));
}

/// A fan closed with a horizontal edge across a vertex later in the same row (a T-junction).
#[test]
fn horizontal_edge_does_not_pass_through_row_vertex() {
    assert_valid(&paths(&[&[
        (6629, -1944),
        (6677, -1944),
        (6583, -1943),
        (6617, -1944),
        (7059, -3027),
    ]]));
}

/// A collinear vertex dropped from one path but used by another one that touches it.
#[test]
fn edge_is_split_at_vertex_shared_with_touching_path() {
    assert_valid(&paths(&[
        &[(-901, 6750), (-610, 6197), (-345, 7333)],
        &[(-600, 6197), (-610, 6197), (-641, 6197), (-3699, -6829)],
    ]));
    assert_valid(&paths(&[
        &[(20, 11), (14, 9), (15, 9), (14, 8)],
        &[(14, 8), (14, 9), (14, 10), (8, 6)],
    ]));
}

/// A triangular hole with a side of length 1 was dropped as "tiny", although its area is not
/// zero.
#[test]
fn small_triangular_hole_is_kept() {
    assert_valid(&paths(&[
        &[(4, 12), (1, 3), (13, 1)],
        &[(9, 4), (10, 4), (10, 3), (10, 2)],
    ]));
}

/// A path whose bottom vertex sits in the notch of another path: the merged vertex lost the
/// notch's inner local minimum.
#[test]
fn vertex_shared_by_notch_and_bottom_stays_inner_minimum() {
    assert_valid(&paths(&[
        &[(7, 9), (6, 1), (12, 4), (18, 0)],
        &[(12, 4), (11, 1), (14, 0)],
    ]));
}

/// A hole whose bottom vertex is the outer path's bottom vertex: below the merged vertex is
/// outside, although the hole on its own is an inner local minimum there.
#[test]
fn hole_touching_outer_bottom_is_not_inner_minimum() {
    assert_valid(&paths(&[
        &[(16, 19), (3, 11), (19, 0)],
        &[(16, 19), (16, 16), (12, 15)],
    ]));
}

/// A hole whose bottom vertex lies on an outer edge, where the outer path has a collinear
/// vertex that add_path drops: once the edge is split there, the vertex has an edge going
/// down and is no longer an inner local minimum.
#[test]
fn split_vertex_is_no_longer_inner_minimum() {
    assert_valid(&paths(&[
        &[(18, 7), (4, 18), (0, 14), (1, 13), (6, 8)],
        &[(1, 13), (10, 12), (12, 9)],
    ]));
}

/// Paths that touch themselves at a vertex, both lobes with an inner local minimum there.
#[test]
fn pinched_path_keeps_inner_minimum() {
    assert_valid(&paths(&[&[
        (9, 14),
        (12, 13),
        (5, 17),
        (8, 13),
        (9, 14),
        (11, 10),
        (12, 10),
    ]]));
    assert_valid(&paths(&[&[
        (12, 7),
        (10, 4),
        (14, 5),
        (7, 2),
        (5, 1),
        (14, 5),
        (20, 1),
    ]]));
}

/// A path that runs back along itself, next to a path that shares one of its edges: after
/// splitting at the collinear vertices, the same edge runs both ways (a zero-width slit).
#[test]
fn edges_running_both_ways_are_removed() {
    assert_valid(&paths(&[
        &[
            (13, 9),
            (15, 9),
            (16, 9),
            (16, 12),
            (14, 10),
            (15, 9),
            (13, 9),
            (11, 9),
            (2, 11),
            (2, 10),
            (0, 6),
        ],
        &[(11, 11), (11, 9), (13, 9)],
    ]));
}

/// Upstream fixes clockwise input by flipping its local minima, but a vertex that add_path
/// queues twice (after skipping a collinear one) flipped back; and it took the orientation
/// from the lowest local minimum, which can belong to a hole.
#[test]
fn clockwise_input_is_triangulated() {
    assert_valid(&paths(&[&[(0, 0), (10, 20), (15, 10), (20, 0)]]));
    assert_valid(&paths(&[&[
        (27, 3),
        (11, 12),
        (20, 26),
        (22, 23),
        (28, 14),
    ]]));
    assert_valid(&paths(&[
        &[(0, 0), (0, 50), (100, 50), (100, 0)],
        &[(40, 30), (60, 30), (50, 50)],
    ]));
}

#[test]
fn extreme_coordinates() {
    let out_of_range = paths(&[&[(i64::MIN, 0), (i64::MAX, 0), (0, i64::MAX)]]);
    assert_eq!(
        triangulate(&out_of_range, true),
        Err(TriangulateError::CoordinateOutOfRange)
    );
    // overlapping squares at the range limits: their total area does not fit in an i128
    let (lo, hi) = (MIN_COORD, MAX_COORD);
    let square = vec![
        Point64::new(lo, lo),
        Point64::new(hi, lo),
        Point64::new(hi, hi),
        Point64::new(lo, hi),
    ];
    let _ = triangulate(&vec![square; 5], false);
}

/// Crossing edges, even ones within a unit of each other's end points, are reported instead
/// of being "fixed" into a wrong result.
#[test]
fn crossing_edges_are_reported() {
    let crossing = paths(&[&[(8, 7), (6, 1), (6, 2), (8, 0)]]);
    for delaunay in [false, true] {
        assert_eq!(
            triangulate(&crossing, delaunay),
            Err(TriangulateError::PathsIntersect)
        );
    }
}

/// A path with a vertex on its own edge produced the same triangle twice. Whatever the sweep
/// does with such input, `triangulate` must not return a result that does not cover it
/// exactly.
#[test]
fn wrong_result_is_reported_as_failure() {
    let input = paths(&[&[(16, 20), (12, 15), (15, 16), (11, 12), (16, 17), (20, 1)]]);
    for delaunay in [false, true] {
        if let Ok(tris) = triangulate(&input, delaunay) {
            assert_eq!(
                check(&input, &tris, delaunay),
                Ok(()),
                "delaunay={delaunay}"
            );
        }
    }
}

/// Whether the input's boundary (see [`boundary`]) crosses itself.
fn has_crossing(paths: &Paths64) -> bool {
    let segs = boundary(paths);
    segs.iter().enumerate().any(|(i, &(a, b))| {
        segs[i + 1..].iter().any(|&(c, d)| {
            cross(a, b, c).signum() * cross(a, b, d).signum() < 0
                && cross(c, d, a).signum() * cross(c, d, b).signum() < 0
        })
    })
}

/// Whether the C++ library triangulates `paths` validly.
fn reference_valid(paths: &Paths64, delaunay: bool) -> bool {
    let (rc, tris) = clipper2_ref::triangulate(&to_ref(paths), delaunay);
    rc == 0 && check(paths, &from_ref(&tris), delaunay).is_ok()
}

/// Non-intersecting inputs seen, and those that upstream triangulates but `triangulate` fails
/// on: a known tail of degenerate input (paths running back along themselves or sharing
/// edges, see the README), bounded per shape.
#[derive(Default)]
struct Regressions {
    exact: usize,
    failed: Vec<String>,
}

impl Regressions {
    fn assert_at_most(&self, name: &str, per_mille: usize) {
        assert!(
            self.failed.len() * 1000 <= self.exact * per_mille,
            "{name}: upstream triangulates {} of {} non-intersecting inputs that ours fails on \
             (at most {per_mille}\u{2030} allowed); first:\n{}",
            self.failed.len(),
            self.exact,
            self.failed[0]
        );
    }
}

/// What every result must satisfy: exact on non-intersecting input; on other input, an error
/// for a self-crossing boundary and otherwise a cover of the input's winding number (its
/// area). An error where upstream succeeds is counted in `regressions`.
fn check_outcome(
    case: &str,
    paths: &Paths64,
    delaunay: bool,
    regressions: &mut Regressions,
) -> Result<Paths64, TriangulateError> {
    let ours = triangulate(paths, delaunay);
    let exact = is_non_intersecting(paths);
    regressions.exact += exact as usize;
    match &ours {
        Ok(tris) if exact => {
            if let Err(e) = check(paths, tris, delaunay) {
                panic!(
                    "{case}: {e}\ninput {:?}\ntriangles {:?}",
                    to_ref(paths),
                    to_ref(tris)
                );
            }
        }
        Ok(tris) => assert_eq!(
            area2_paths(tris),
            area2_paths(paths).abs(),
            "{case}: input {:?}",
            to_ref(paths)
        ),
        // the reference would never return on Degenerate
        Err(e) if exact && *e != TriangulateError::Degenerate => {
            if reference_valid(paths, delaunay) {
                regressions
                    .failed
                    .push(format!("{case}: {e:?}\ninput {:?}", to_ref(paths)));
            }
        }
        Err(_) => {}
    }
    // normally PathsIntersect; Degenerate when the path walk never ends, Fail when add_path
    // drops a path (for example one with a spike) and the result then does not cover it
    if has_crossing(paths) {
        assert!(
            ours.is_err(),
            "{case}: crossing boundary accepted\ninput {:?}",
            to_ref(paths)
        );
    }
    ours
}

/// Allowed regressions against upstream, in thousandths of the non-intersecting inputs:
/// only small-grid input (many coinciding vertices and edges) reaches the degenerate tail.
fn regression_bound(shape: Shape) -> usize {
    if shape == Shape::SmallGrid { 2 } else { 0 }
}

#[test]
fn random_union_input() {
    for shape in [Shape::SmallGrid, Shape::Medium, Shape::Star] {
        let mut regressions = Regressions::default();
        for seed in 0..cases() as u64 {
            let paths = union_input(seed, shape);
            // union output runs counter-clockwise; its reversal tests clockwise input
            let reversed: Paths64 = paths
                .iter()
                .map(|p| p.iter().rev().copied().collect())
                .collect();
            for delaunay in [false, true] {
                let case = format!("{shape:?}-{seed}-{delaunay}");
                let _ = check_outcome(&case, &paths, delaunay, &mut regressions);
                let _ = check_outcome(&(case + "-cw"), &reversed, delaunay, &mut regressions);
            }
        }
        regressions.assert_at_most(&format!("{shape:?}"), regression_bound(shape));
    }
}

/// Raw random paths: mostly self-intersecting.
#[test]
fn random_raw_input() {
    for shape in [Shape::SmallGrid, Shape::Medium, Shape::Star] {
        let mut regressions = Regressions::default();
        for seed in 0..cases() as u64 {
            let mut rng = Rng::new(seed * 413_158_523 + shape as u64);
            let paths = random_paths(&mut rng, shape);
            let case = format!("raw-{shape:?}-{seed}");
            let _ = check_outcome(&case, &paths, seed % 2 == 1, &mut regressions);
        }
        regressions.assert_at_most(&format!("raw {shape:?}"), regression_bound(shape));
    }
}

/// Tiny random polygons are mostly self-intersecting; on about 4% of them upstream's path
/// walk never ends.
#[test]
fn tiny_self_intersecting_paths() {
    let (mut degenerate, mut regressions) = (0, Regressions::default());
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
            let case = format!("tiny-{seed}-{delaunay}");
            let ours = check_outcome(&case, &paths, delaunay, &mut regressions);
            if ours == Err(TriangulateError::Degenerate) {
                degenerate += 1;
            }
        }
    }
    assert!(degenerate > 0, "no degenerate walk exercised");
    regressions.assert_at_most("tiny", regression_bound(Shape::SmallGrid));
}

#[test]
fn triangulate_d_scales_like_triangulate() {
    for seed in 0..50u64 {
        let mut rng = Rng::new(seed * 17 + 2);
        let dp = [-1, 0, 2, 9][rng.below(4)];
        let paths: PathsD = random_paths_d(&mut rng, Shape::Star, 2);
        let scale = 10f64.powi(dp.clamp(0, 8));
        let scaled = scale_paths(&paths, scale, scale).unwrap();
        let expected = triangulate(&scaled, true)
            .map(|t| scale_paths::<i64, f64>(&t, 1.0 / scale, 1.0 / scale).unwrap());
        assert_eq!(triangulate_d(&paths, dp, true), expected, "seed {seed}");
    }
}

#[test]
fn square_and_holed_square() {
    let sq = paths(&[&[(0, 0), (10, 0), (10, 10), (0, 10)]]);
    assert_eq!(triangulate(&sq, true).unwrap().len(), 2);
    assert_valid(&sq);
    assert_valid(&paths(&[
        &[(0, 0), (30, 0), (30, 30), (0, 30)],
        &[(10, 10), (10, 20), (20, 20), (20, 10)],
    ]));
}

#[test]
fn invalid_input_is_rejected() {
    for delaunay in [false, true] {
        for empty in [vec![], vec![vec![]]] {
            assert_eq!(
                triangulate(&empty, delaunay),
                Err(TriangulateError::NoPolygons)
            );
        }
        let collinear = paths(&[&[(0, 0), (5, 5), (10, 10)]]);
        assert_eq!(
            triangulate(&collinear, delaunay),
            Err(TriangulateError::NoPolygons)
        );
        for intersecting in [
            paths(&[
                &[(0, 0), (10, 0), (10, 10), (0, 10)],
                &[(5, 5), (15, 5), (15, 15), (5, 15)],
            ]),
            paths(&[&[(0, 0), (10, 10), (10, 0), (0, 10)]]),
        ] {
            assert_eq!(
                triangulate(&intersecting, delaunay),
                Err(TriangulateError::PathsIntersect)
            );
        }
    }
}

/// Upstream drops a triangle with a side of length 1 (and returns no triangles at all here).
#[test]
fn unit_triangle_is_triangulated() {
    let tiny = paths(&[&[(0, 0), (1, 0), (0, 1)]]);
    assert_eq!(triangulate(&tiny, true).unwrap().len(), 1);
    assert_valid(&tiny);
}

/// Upstream loops forever (allocating) on this self-intersecting input.
#[test]
fn non_terminating_input_is_reported_as_degenerate() {
    let input = paths(&[
        &[(14, 15), (18, 7), (1, 3), (8, 6), (0, 14)],
        &[
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
        ],
    ]);
    assert_eq!(
        triangulate(&input, false),
        Err(TriangulateError::Degenerate)
    );
}

/// Here the walk overshoots i0 inside the ascending bound and never leaves it, so a check at
/// the top of the outer loop alone does not stop it.
#[test]
fn walk_stuck_in_ascending_bound_is_reported_as_degenerate() {
    let input = paths(&[
        &[(1, 3), (3, 0), (1, 3), (3, 3), (1, 1), (3, 3)],
        &[(1, 0), (2, 1), (1, 3), (3, 1), (0, 3)],
        &[
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
        ],
    ]);
    for delaunay in [false, true] {
        assert_eq!(
            triangulate(&input, delaunay),
            Err(TriangulateError::Degenerate)
        );
    }
}

/// Upstream #1055 (internal error) and #1056 (endless loop in `FixupEdgeIntersects`), both
/// caused by testing segments that share an end point for intersection.
#[test]
fn upstream_issue_paths_triangulate() {
    let issue_1055 = paths(&[&[
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
    ]]);
    let issue_1056 = paths(&[&[
        (4956510274156402, 1949095339415813),
        (4770558614626881, 1728685063645521),
        (4773228611416619, 1726446939526986),
        (4775938071598222, 1724256755070940),
    ]]);
    for (name, input) in [("1055", &issue_1055), ("1056", &issue_1056)] {
        let tris = triangulate(input, true).unwrap_or_else(|e| panic!("issue {name}: {e:?}"));
        // coordinates near 2^54: the exact in-circle test would overflow i128, so only the
        // coverage is checked
        assert_eq!(check(input, &tris, false), Ok(()), "issue {name}");
    }
}
