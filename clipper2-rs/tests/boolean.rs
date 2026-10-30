mod common;

use common::*;

#[test]
fn generator_is_deterministic() {
    for shape in Shape::ALL {
        let a = random_paths(&mut Rng::new(1), shape);
        let b = random_paths(&mut Rng::new(1), shape);
        assert_eq!(a, b, "{shape:?}");
        assert!(!a.is_empty());
    }
    let grid = random_paths(&mut Rng::new(7), Shape::SmallGrid);
    assert!(
        grid.iter()
            .flatten()
            .all(|p| (0..=20).contains(&p.x) && (0..=20).contains(&p.y))
    );
}

#[test]
fn upstream_polygons_file_parses() {
    let cases = read_upstream_tests(&upstream_tests_dir().join("Polygons.txt"));
    assert_eq!(cases.len(), 195);
    let first = &cases[0];
    assert_eq!(first.subjects.len(), 1);
    assert_eq!((first.area, first.count), (9000, 1));
    assert_eq!(first.subjects[0][0], clipper2_rs::Point64::new(100, 10));
}

use clipper2_rs::{
    ClipType, Clipper64, ClipperError, FillRule, MAX_COORD, Paths64, Point64, area_paths,
};

fn sq(x0: i64, y0: i64, x1: i64, y1: i64) -> Paths64 {
    vec![vec![
        Point64::new(x0, y0),
        Point64::new(x1, y0),
        Point64::new(x1, y1),
        Point64::new(x0, y1),
    ]]
}

fn run(ct: ClipType, subject: Paths64, clip: Paths64, fr: FillRule) -> Paths64 {
    let mut c = Clipper64::new();
    c.add_subject(&subject);
    c.add_clip(&clip);
    c.execute(ct, fr).unwrap().closed
}

#[test]
fn union_area_is_7() {
    assert_eq!(
        area_paths(&run(
            ClipType::Union,
            sq(0, 0, 2, 2),
            sq(1, 1, 3, 3),
            FillRule::NonZero
        ))
        .abs(),
        7.0
    );
}

#[test]
fn intersection_area_is_1() {
    assert_eq!(
        area_paths(&run(
            ClipType::Intersection,
            sq(0, 0, 2, 2),
            sq(1, 1, 3, 3),
            FillRule::NonZero
        ))
        .abs(),
        1.0
    );
}

#[test]
fn difference_area_is_3() {
    assert_eq!(
        area_paths(&run(
            ClipType::Difference,
            sq(0, 0, 2, 2),
            sq(1, 1, 3, 3),
            FillRule::NonZero
        ))
        .abs(),
        3.0
    );
}

#[test]
fn xor_has_two_paths_area_6() {
    let s = run(
        ClipType::Xor,
        sq(0, 0, 2, 2),
        sq(1, 1, 3, 3),
        FillRule::NonZero,
    );
    assert_eq!(s.len(), 2);
    assert_eq!(
        s.iter().map(|p| clipper2_rs::area(p).abs()).sum::<f64>(),
        6.0
    );
}

#[test]
fn evenodd_cancels_doubled_square_nonzero_keeps_it() {
    let doubled: Paths64 = [sq(0, 0, 2, 2), sq(0, 0, 2, 2)].concat();
    assert!(run(ClipType::Union, doubled.clone(), vec![], FillRule::EvenOdd).is_empty());
    assert_eq!(
        area_paths(&run(ClipType::Union, doubled, vec![], FillRule::NonZero)).abs(),
        4.0
    );
}

#[test]
fn out_of_range_input_returns_error() {
    let mut c = Clipper64::new();
    c.add_subject(&sq(0, 0, MAX_COORD + 1, 2));
    assert_eq!(
        c.execute(ClipType::Union, FillRule::NonZero).err(),
        Some(ClipperError::CoordinateOutOfRange)
    );
}
