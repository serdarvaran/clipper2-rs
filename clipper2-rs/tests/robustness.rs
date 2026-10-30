//! Inputs on which upstream has undefined behaviour or never terminates.
use clipper2_rs::{
    ClipperError, ClipperOffset, EndType, JoinType, Path64, PathD, Paths64, inflate_paths,
    make_path,
};

fn square() -> Path64 {
    make_path(&[0, 0, 100, 0, 100, 100, 0, 100]).unwrap()
}

#[test]
fn empty_paths_are_ignored_by_offset() {
    for end in [
        EndType::Polygon,
        EndType::Butt,
        EndType::Square,
        EndType::Round,
    ] {
        let with_empty: Paths64 = vec![vec![], square(), vec![]];
        let without: Paths64 = vec![square()];
        assert_eq!(
            inflate_paths(&with_empty, 5.0, JoinType::Round, end, 2.0, 0.0),
            inflate_paths(&without, 5.0, JoinType::Round, end, 2.0, 0.0),
            "{end:?}"
        );
        assert_eq!(
            inflate_paths(&vec![vec![]], 5.0, JoinType::Round, end, 2.0, 0.0),
            Ok(vec![])
        );
    }
}

#[test]
fn non_finite_delta_is_an_error() {
    for delta in [f64::INFINITY, f64::NEG_INFINITY, f64::NAN] {
        assert_eq!(
            inflate_paths(
                &vec![square()],
                delta,
                JoinType::Round,
                EndType::Polygon,
                2.0,
                0.0
            ),
            Err(ClipperError::InvalidDelta)
        );
    }
}

#[test]
fn non_finite_callback_delta_is_an_error() {
    let mut co = ClipperOffset::default();
    co.add_paths(&vec![square()], JoinType::Round, EndType::Polygon);
    let mut cb =
        |_: &Path64, _: &PathD, j: usize, _: usize| if j == 2 { f64::INFINITY } else { 5.0 };
    assert_eq!(
        co.execute_with_callback(&mut cb),
        Err(ClipperError::InvalidDelta)
    );
    let mut finite = |_: &Path64, _: &PathD, _: usize, _: usize| 5.0;
    assert!(
        co.execute_with_callback(&mut finite)
            .is_ok_and(|p| p.len() == 1)
    );
}
