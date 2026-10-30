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
