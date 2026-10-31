//! Port of the upstream Clipper2 polytree gtests (`TestPolytreeHoles.cpp`,
//! `TestPolytreeIntersection.cpp`, `TestPolytreeUnion.cpp`); see `upstream.rs` for the rest.
//! Path literals are copied verbatim from the C++ sources.

mod common;

use clipper2_rs::{
    ClipType, Clipper64, FillRule, Path64, Paths64, Point64, PointInPolygonResult, PolyTree64,
    area, area_paths, check_polytree_fully_contains_children, is_positive, make_path,
    point_in_polygon, poly_tree_to_paths,
};
use common::{read_upstream_tests, upstream_tests_dir};

fn mp(coords: &[i64]) -> Path64 {
    make_path(coords).unwrap()
}

fn union_tree(subject: &Paths64, fr: FillRule) -> PolyTree64 {
    let mut c = Clipper64::new();
    c.add_subject(subject);
    c.execute_tree(ClipType::Union, fr).unwrap().0
}

fn children(tree: &PolyTree64, id: usize) -> &[usize] {
    &tree.node(id).children
}

// ---- TestPolytreeHoles.cpp ----

#[test]
fn test_polytree_holes1() {
    let cases = read_upstream_tests(&upstream_tests_dir().join("PolytreeHoleOwner.txt"));
    let case = &cases[0];
    let mut c = Clipper64::new();
    c.add_subject(&case.subjects);
    c.add_open_subject(&case.subjects_open);
    c.add_clip(&case.clips);
    // upstream ignores Execute's result
    let (solution, _open) = c
        .execute_tree(case.clip_type, case.fill_rule)
        .unwrap_or_default();
    assert!(check_polytree_fully_contains_children(&solution));
}

fn poly_path_contains_point(tree: &PolyTree64, id: usize, pt: Point64, counter: &mut i32) {
    let polygon = &tree.node(id).polygon;
    if !polygon.is_empty() && point_in_polygon(pt, polygon) != PointInPolygonResult::IsOutside {
        if tree.is_hole(id) {
            *counter -= 1;
        } else {
            *counter += 1;
        }
    }
    for &child in children(tree, id) {
        poly_path_contains_point(tree, child, pt, counter);
    }
}

fn polytree_contains_point(tree: &PolyTree64, pt: Point64) -> bool {
    let mut counter = 0;
    for &root in tree.roots() {
        poly_path_contains_point(tree, root, pt, &mut counter);
    }
    // ie 'pt' can't be inside more holes than outers
    assert!(counter >= 0, "{pt:?}: counter {counter}");
    counter != 0
}

fn get_poly_path_area(tree: &PolyTree64, id: usize, area_sum: &mut f64) {
    *area_sum += area(&tree.node(id).polygon);
    for &child in children(tree, id) {
        get_poly_path_area(tree, child, area_sum);
    }
}

fn get_polytree_area(tree: &PolyTree64) -> f64 {
    let mut result = 0.0;
    for &root in tree.roots() {
        get_poly_path_area(tree, root, &mut result);
    }
    result
}

#[test]
fn test_polytree_holes2() {
    let cases = read_upstream_tests(&upstream_tests_dir().join("PolytreeHoleOwner2.txt"));
    let case = &cases[0];
    let subject = &case.subjects;
    let points_of_interest_outside = [
        Point64::new(21887, 10420),
        Point64::new(21726, 10825),
        Point64::new(21662, 10845),
        Point64::new(21617, 10890),
    ];
    // confirm that each 'points_of_interest_outside' is outside every subject,
    for &poi in &points_of_interest_outside {
        let outside_subject_count = subject
            .iter()
            .filter(|p| point_in_polygon(poi, p) != PointInPolygonResult::IsOutside)
            .count();
        assert_eq!(outside_subject_count, 0, "{poi:?}");
    }
    let points_of_interest_inside = [
        Point64::new(21887, 10430),
        Point64::new(21843, 10520),
        Point64::new(21810, 10686),
        Point64::new(21900, 10461),
    ];
    // confirm that each 'points_of_interest_inside' is inside a subject,
    // and inside only one subject (to exclude possible subject holes)
    for &poi in &points_of_interest_inside {
        let inside_subject_count = subject
            .iter()
            .filter(|p| point_in_polygon(poi, p) != PointInPolygonResult::IsOutside)
            .count();
        assert_eq!(inside_subject_count, 1, "{poi:?}");
    }

    let mut c = Clipper64::new();
    c.add_subject(subject);
    c.add_open_subject(&case.subjects_open);
    c.add_clip(&case.clips);
    // upstream ignores Execute's result
    let (solution_tree, _open) = c
        .execute_tree(case.clip_type, FillRule::Negative)
        .unwrap_or_default();
    let solution_paths = poly_tree_to_paths(&solution_tree);
    assert!(!solution_paths.is_empty());
    let subject_area = -area_paths(subject); // negate (see fillrule)
    let solution_tree_area = get_polytree_area(&solution_tree);
    let solution_paths_area = area_paths(&solution_paths);
    // 1a. check solution_paths_area is smaller than subject_area
    assert!(solution_paths_area < subject_area);
    // 1b. but not too much smaller
    assert!(solution_paths_area > subject_area * 0.92);
    // 2. check solution_tree's area matches solution_paths' area
    assert!((solution_tree_area - solution_paths_area).abs() <= 0.0001);
    // 3. check that all children are inside their parents
    assert!(check_polytree_fully_contains_children(&solution_tree));
    // 4. confirm all 'point_of_interest_outside' are outside polytree
    for &poi in &points_of_interest_outside {
        assert!(!polytree_contains_point(&solution_tree, poi), "{poi:?}");
    }
    // 5. confirm all 'point_of_interest_inside' are inside polytree
    for &poi in &points_of_interest_inside {
        assert!(polytree_contains_point(&solution_tree, poi), "{poi:?}");
    }
}

#[test]
fn test_polytree_holes3() {
    let subject = vec![mp(&[
        1072, 501, 1072, 501, 1072, 539, 1072, 539, 1072, 539, 870, 539, 870, 539, 870, 539, 870,
        520, 894, 520, 898, 524, 911, 524, 915, 520, 915, 520, 936, 520, 940, 524, 953, 524, 957,
        520, 957, 520, 978, 520, 983, 524, 995, 524, 1000, 520, 1021, 520, 1025, 524, 1038, 524,
        1042, 520, 1038, 516, 1025, 516, 1021, 520, 1000, 520, 995, 516, 983, 516, 978, 520, 957,
        520, 953, 516, 940, 516, 936, 520, 915, 520, 911, 516, 898, 516, 894, 520, 870, 520, 870,
        516, 870, 501, 870, 501, 870, 501, 1072, 501,
    ])];
    let clip = vec![mp(&[870, 501, 971, 501, 971, 539, 870, 539])];
    let mut c = Clipper64::new();
    c.add_subject(&subject);
    c.add_clip(&clip);
    let (solution, _) = c
        .execute_tree(ClipType::Intersection, FillRule::NonZero)
        .unwrap();
    let roots = solution.roots();
    assert!(roots.len() == 1 && children(&solution, roots[0]).len() == 2);
}

#[test]
fn test_polytree_holes4() {
    // #618
    let subject = vec![
        mp(&[
            50, 500, 50, 300, 100, 300, 100, 350, 150, 350, 150, 250, 200, 250, 200, 450, 350, 450,
            350, 200, 400, 200, 400, 225, 450, 225, 450, 175, 400, 175, 400, 200, 350, 200, 350,
            175, 200, 175, 200, 250, 150, 250, 150, 200, 100, 200, 100, 300, 50, 300, 50, 125, 500,
            125, 500, 500,
        ]),
        mp(&[250, 425, 250, 375, 300, 375, 300, 425]),
    ];
    let solution = union_tree(&subject, FillRule::NonZero);
    // Polytree root
    //   +- Polygon with 3 holes.
    //      +- Hole with 1 nested polygon.
    //         +-Polygon
    //      +- Hole
    //      +- Hole
    let roots = solution.roots();
    assert!(roots.len() == 1 && children(&solution, roots[0]).len() == 3);
}

#[test]
fn test_polytree_holes5() {
    let subject = vec![mp(&[0, 30, 400, 30, 400, 100, 0, 100])];
    let clip = vec![
        mp(&[20, 30, 30, 30, 30, 150, 20, 150]),
        mp(&[
            200, 0, 300, 0, 300, 30, 280, 30, 280, 20, 220, 20, 220, 30, 200, 30,
        ]),
        mp(&[200, 50, 300, 50, 300, 80, 200, 80]),
    ];
    let mut c = Clipper64::new();
    c.add_subject(&subject);
    c.add_clip(&clip);
    let (tree, _) = c.execute_tree(ClipType::Xor, FillRule::NonZero).unwrap();
    // Polytree with 3 polygons.
    //   + -Polygon (2) contains 2 holes.
    let roots = tree.roots();
    assert!(roots.len() == 3 && children(&tree, roots[2]).len() == 2);
}

#[test]
fn test_polytree_holes6() {
    // #618
    let subject = vec![
        mp(&[150, 50, 200, 50, 200, 100, 150, 100]),
        mp(&[125, 100, 150, 100, 150, 150, 125, 150]),
        mp(&[225, 50, 300, 50, 300, 80, 225, 80]),
        mp(&[
            225, 100, 300, 100, 300, 150, 275, 150, 275, 175, 260, 175, 260, 250, 235, 250, 235,
            300, 275, 300, 275, 275, 300, 275, 300, 350, 225, 350,
        ]),
        mp(&[300, 150, 350, 150, 350, 175, 300, 175]),
    ];
    let clip = vec![
        mp(&[0, 0, 400, 0, 400, 50, 0, 50]),
        mp(&[0, 100, 400, 100, 400, 150, 0, 150]),
        mp(&[260, 175, 325, 175, 325, 275, 260, 275]),
    ];
    let mut c = Clipper64::new();
    c.add_subject(&subject);
    c.add_clip(&clip);
    let (tree, _) = c.execute_tree(ClipType::Xor, FillRule::NonZero).unwrap();
    // Polytree with 3 polygons.
    //   + -Polygon (2) contains 1 holes.
    let roots = tree.roots();
    assert!(roots.len() == 3 && children(&tree, roots[2]).len() == 1);
}

#[test]
fn test_polytree_holes7() {
    // #618
    let subject = vec![
        mp(&[
            0, 0, 100000, 0, 100000, 100000, 200000, 100000, 200000, 0, 300000, 0, 300000, 200000,
            0, 200000,
        ]),
        mp(&[0, 0, 0, -100000, 250000, -100000, 250000, 0]),
    ];
    let polytree = union_tree(&subject, FillRule::NonZero);
    let roots = polytree.roots();
    assert!(roots.len() == 1 && children(&polytree, roots[0]).len() == 1);
}

#[test]
fn test_polytree_holes8() {
    // #942
    let subject = vec![
        mp(&[1588700, -8717600, 1616200, -8474800, 1588700, -8474800]),
        mp(&[
            13583800, -15601600, 13582800, -15508500, 13555300, -15508500, 13555500, -15182200,
            13010900, -15185400,
        ]),
        mp(&[956700, -3092300, 1152600, 3147400, 25600, 3151700]),
        mp(&[
            22575900, -16604000, 31286800, -12171900, 31110200, 4882800, 30996200, 4826300,
            30414400, 5447400, 30260000, 5391500, 29662200, 5805400, 28844500, 5337900, 28435000,
            5789300, 27721400, 5026400, 22876300, 5034300, 21977700, 4414900, 21148000, 4654700,
            20917600, 4653400, 19334300, 12411000, -2591700, 12177200, 53200, 3151100, -2564300,
            12149800, 7819400, 4692400, 10116000, 5228600, 6975500, 3120100, 7379700, 3124700,
            11037900, 596200, 12257000, 2587800, 12257000, 596200, 15227300, 2352700, 18444400,
            1112100, 19961100, 5549400, 20173200, 5078600, 20330000, 5079300, 20970200, 4544300,
            20989600, 4563700, 19465500, 1112100, 21611600, 4182100, 22925100, 1112200, 22952700,
            1637200, 23059000, 1112200, 24908100, 4181200, 27070100, 3800600, 27238000, 3800700,
            28582200, 520300, 29367800, 1050100, 29291400, 179400, 29133700, 360700, 29056700,
            312600, 29121900, 332500, 29269900, 162300, 28941400, 213100, 27491300, -3041500,
            27588700, -2997800, 22104900, -16142800, 13010900, -15603000, 13555500, -15182200,
            13555300, -15508500, 13582800, -15508500, 13583100, -15154700, 1588700, -8822800,
            1588700, -8379900, 1588700, -8474800, 1616200, -8474800, 1003900, -630100, 1253300,
            -12284500, 12983400, -16239900,
        ]),
        mp(&[198200, 12149800, 1010600, 12149800, 1011500, 11859600]),
        mp(&[21996700, -7432000, 22096700, -7432000, 22096700, -7332000]),
    ];
    let solution = union_tree(&subject, FillRule::NonZero);
    let roots = solution.roots();
    assert!(
        roots.len() == 1
            && children(&solution, roots[0]).len() == 2
            && children(&solution, children(&solution, roots[0])[1]).len() == 1
    );
}

#[test]
fn test_polytree_holes9() {
    // #957
    let subject = vec![
        mp(&[
            77910, 46865, 78720, 46865, 78720, 48000, 77910, 48000, 77910, 46865,
        ]),
        mp(&[
            82780, 53015, 93600, 53015, 93600, 54335, 82780, 54335, 82780, 53015,
        ]),
        mp(&[
            82780, 48975, 84080, 48975, 84080, 53015, 82780, 53015, 82780, 48975,
        ]),
        mp(&[
            77910, 48000, 84080, 48000, 84080, 48975, 77910, 48975, 77910, 48000,
        ]),
        mp(&[
            89880, 40615, 90700, 40615, 90700, 46865, 89880, 46865, 89880, 40615,
        ]),
        mp(&[
            92700, 54335, 93600, 54335, 93600, 61420, 92700, 61420, 92700, 54335,
        ]),
        mp(&[
            78950, 47425, 84080, 47425, 84080, 47770, 78950, 47770, 78950, 47425,
        ]),
        mp(&[
            82780, 61420, 93600, 61420, 93600, 62435, 82780, 62435, 82780, 61420,
        ]),
        mp(&[
            101680, 63085, 100675, 63085, 100675, 47770, 100680, 47770, 100680, 40615, 101680,
            40615, 101680, 63085,
        ]),
        mp(&[
            76195, 39880, 89880, 39880, 89880, 41045, 76195, 41045, 76195, 39880,
        ]),
        mp(&[
            85490, 56145, 90520, 56145, 90520, 59235, 85490, 59235, 85490, 56145,
        ]),
        mp(&[
            89880, 39880, 101680, 39880, 101680, 40615, 89880, 40615, 89880, 39880,
        ]),
        mp(&[
            89880, 46865, 100680, 46865, 100680, 47770, 89880, 47770, 89880, 46865,
        ]),
        mp(&[
            82780, 54335, 83280, 54335, 83280, 61420, 82780, 61420, 82780, 54335,
        ]),
        mp(&[
            76195, 41045, 76855, 41045, 76855, 62665, 76195, 62665, 76195, 41045,
        ]),
        mp(&[
            76195, 62665, 100675, 62665, 100675, 63085, 76195, 63085, 76195, 62665,
        ]),
        mp(&[
            82780, 41045, 84080, 41045, 84080, 47425, 82780, 47425, 82780, 41045,
        ]),
    ];
    let solution = union_tree(&subject, FillRule::NonZero);
    //  Polytree with 1 polygon.
    //    + -Polygon(0) contains 2 holes.
    //    + -Hole(0) contains 1 nested polygon.
    //    + -Polygon(0) contains 1 hole.
    //    + -Hole(0) contains 1 nested polygon.
    let roots = solution.roots();
    assert!(
        roots.len() == 1
            && children(&solution, roots[0]).len() == 2
            && children(&solution, children(&solution, roots[0])[0]).len() == 1
    );
}

#[test]
fn test_polytree_holes10() {
    // #973
    let subject = vec![
        mp(&[0, 0, 79530, 0, 79530, 940, 0, 940, 0, 0]),
        mp(&[0, 33360, 79530, 33360, 79530, 34300, 0, 34300, 0, 33360]),
        mp(&[
            78470, 940, 79530, 940, 79530, 33360, 78470, 33360, 78470, 940,
        ]),
        mp(&[0, 940, 940, 940, 940, 33360, 0, 33360, 0, 940]),
        mp(&[
            29290, 940, 30350, 940, 30350, 33360, 29290, 33360, 29290, 940,
        ]),
    ];
    let solution = union_tree(&subject, FillRule::NonZero);
    //  Polytree with 1 polygon.
    //    + -Polygon(0) contains 2 holes.
    let roots = solution.roots();
    assert!(roots.len() == 1 && children(&solution, roots[0]).len() == 2);
}

// ---- TestPolytreeIntersection.cpp ----

#[test]
fn test_poly_tree_intersection() {
    let subject = vec![mp(&[0, 0, 0, 5, 5, 5, 5, 0])];
    let clip = vec![mp(&[1, 1, 1, 6, 6, 6, 6, 1])];
    let mut c = Clipper64::new();
    c.add_subject(&subject);
    c.add_clip(&clip);
    let fr = if is_positive(&subject[0]) {
        FillRule::Positive
    } else {
        FillRule::Negative
    };
    let (solution, open_paths) = c.execute_tree(ClipType::Intersection, fr).unwrap();
    assert_eq!(open_paths.len(), 0);
    assert_eq!(solution.roots().len(), 1);
    assert_eq!(solution.node(solution.roots()[0]).polygon.len(), 4);
}

#[test]
fn test_poly_tree_cyclic_splits() {
    // emptied OutRecs whose splits form a cycle used to recurse forever in CheckSplitOwner
    let clip = vec![mp(&[
        3, 0, 3, 5, 0, 3, 5, 3, 0, 5, 5, 3, 0, 3, 0, 0, 0, 3, 5, 3, 0, 0, 3, 3, 2, 3,
    ])];
    let mut c = Clipper64::new();
    c.set_preserve_collinear(false);
    c.add_clip(&clip);
    assert!(c.execute_tree(ClipType::Xor, FillRule::EvenOdd).is_ok());
}

// ---- TestPolytreeUnion.cpp ----

#[test]
fn test_polytree_union() {
    let subject = vec![mp(&[0, 0, 0, 5, 5, 5, 5, 0]), mp(&[1, 1, 1, 6, 6, 6, 6, 1])];
    let mut c = Clipper64::new();
    c.add_subject(&subject);
    let fr = if is_positive(&subject[0]) {
        FillRule::Positive
    } else {
        // because clipping ops normally return Positive solutions
        c.set_reverse_solution(true);
        FillRule::Negative
    };
    let (solution, open_paths) = c.execute_tree(ClipType::Union, fr).unwrap();
    assert_eq!(open_paths.len(), 0);
    assert_eq!(solution.roots().len(), 1);
    let polygon = &solution.node(solution.roots()[0]).polygon;
    assert_eq!(polygon.len(), 8);
    assert_eq!(is_positive(&subject[0]), is_positive(polygon));
}

#[test]
fn test_polytree_union2() {
    // #987
    let subject = vec![
        mp(&[534, 1024, 534, -800, 1026, -800, 1026, 1024]),
        mp(&[1, 1024, 8721, 1024, 8721, 1920, 1, 1920]),
        mp(&[30, 1024, 30, -800, 70, -800, 70, 1024]),
        mp(&[1, 1024, 1, -1024, 3841, -1024, 3841, 1024]),
        mp(&[3900, -1024, 6145, -1024, 6145, 1024, 3900, 1024]),
        mp(&[5884, 1024, 5662, 1024, 5662, -1024, 5884, -1024]),
        mp(&[534, 1024, 200, 1024, 200, -800, 534, -800]),
        mp(&[200, -800, 200, 1024, 70, 1024, 70, -800]),
        mp(&[1200, 1920, 1313, 1920, 1313, -800, 1200, -800]),
        mp(&[6045, -800, 6045, 1024, 5884, 1024, 5884, -800]),
    ];
    let solution = union_tree(&subject, FillRule::EvenOdd);
    assert_eq!(solution.roots().len(), 1);
    assert_eq!(children(&solution, solution.roots()[0]).len(), 1);
}

#[test]
fn test_polytree_union3() {
    let subject = vec![mp(&[
        -120927680, 590077597, -120919386, 590077307, -120919432, 590077309, -120919451, 590077309,
        -120919455, 590077310, -120099297, 590048669, -120928004, 590077608, -120902794, 590076728,
        -120919444, 590077309, -120919450, 590077309, -120919842, 590077323, -120922852, 590077428,
        -120902452, 590076716, -120902455, 590076716, -120912590, 590077070, 11914491, 249689797,
    ])];
    let mut c = Clipper64::new();
    c.add_subject(&subject);
    // upstream only checks that Execute returns; its result is not inspected
    let _ = c.execute_tree(ClipType::Union, FillRule::EvenOdd);
}
