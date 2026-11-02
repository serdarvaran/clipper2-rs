//! Input validation and geometric boundary cases.
use clipper2_rs::{
    ClipperError, ClipperOffset, EndType, FillRule, JoinType, MAX_COORD, Path64, PathD, Paths64,
    Point64, PointD, area_paths, ellipse, get_bounds_paths, inflate_paths, inflate_paths_d,
    make_path, minkowski_diff, minkowski_diff_d, minkowski_sum, minkowski_sum_d, scale_path,
    scale_paths, scale_paths_d_to_64, union_self_d,
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

#[test]
fn scale_rejects_non_finite_factors_even_for_empty_paths() {
    for scale in [0.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for path in [vec![], vec![PointD::new(1., 2.)]] {
            for (sx, sy) in [(scale, 1.), (1., scale)] {
                assert_eq!(
                    scale_path::<f64, i64>(&path, sx, sy),
                    Err(ClipperError::InvalidScale)
                );
                assert_eq!(
                    scale_paths::<f64, i64>(std::slice::from_ref(&path), sx, sy),
                    Err(ClipperError::InvalidScale)
                );
                assert_eq!(
                    scale_paths::<f64, f64>(&[], sx, sy),
                    Err(ClipperError::InvalidScale)
                );
            }
        }
    }
}

#[test]
fn scale_rejects_non_finite_coordinates() {
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for point in [PointD::new(value, 0.), PointD::new(0., value)] {
            assert_eq!(
                scale_path::<f64, i64>(&[point], 1., 1.),
                Err(ClipperError::CoordinateOutOfRange)
            );
            assert_eq!(
                scale_paths::<f64, f64>(&[vec![point]], 1., 1.),
                Err(ClipperError::CoordinateOutOfRange)
            );
        }
    }
    let invalid = vec![vec![
        PointD::new(f64::NAN, 0.),
        PointD::new(100., 0.),
        PointD::new(100., 100.),
        PointD::new(0., 100.),
    ]];
    assert_eq!(
        union_self_d(&invalid, FillRule::NonZero, 2),
        Err(ClipperError::CoordinateOutOfRange)
    );
}

#[test]
fn scale_checks_negative_factors_and_single_paths() {
    let huge = vec![PointD::new(-1e30, -1e30), PointD::new(1e30, 1e30)];
    for scale in [-1., 1.] {
        assert_eq!(
            scale_paths_d_to_64(&vec![huge.clone()], scale),
            Err(ClipperError::CoordinateOutOfRange)
        );
        assert_eq!(
            scale_path::<f64, i64>(&huge, scale, scale),
            Err(ClipperError::CoordinateOutOfRange)
        );
    }
}

#[test]
fn scale_checks_rounded_integer_limits() {
    for value in [MAX_COORD as f64, -(MAX_COORD as f64)] {
        assert_eq!(
            scale_path::<f64, i64>(&[PointD::new(value, 0.)], 1., 1.),
            Err(ClipperError::CoordinateOutOfRange)
        );
    }
    let below_limit = f64::from_bits((MAX_COORD as f64).to_bits() - 1);
    assert_eq!(
        scale_path::<f64, i64>(&[PointD::new(below_limit, -below_limit)], 1., 1.).unwrap(),
        vec![Point64::new(MAX_COORD - 255, -MAX_COORD + 255)]
    );
}

#[test]
fn scale_rejects_floating_point_overflow() {
    assert_eq!(
        scale_path::<f64, f64>(&[PointD::new(f64::MAX, 0.)], 2., 1.),
        Err(ClipperError::CoordinateOutOfRange)
    );
}

#[test]
fn scale_reflects_valid_coordinates() {
    assert_eq!(
        scale_path::<f64, i64>(&[PointD::new(-2., 3.), PointD::new(4., -5.)], -2., 3.).unwrap(),
        vec![Point64::new(4, 9), Point64::new(-8, -15)]
    );
    #[cfg(feature = "usingz")]
    assert_eq!(
        scale_path::<f64, i64>(&[PointD::new_z(1., 2., 73)], 2., -3.).unwrap(),
        vec![Point64::new_z(2, -6, 73)]
    );
}

#[test]
fn offset_group_order_does_not_change_negative_delta() {
    let point = vec![Point64::new(1000, 1000)];
    let square = square();
    for point_first in [true, false] {
        let mut co = ClipperOffset::default();
        let groups = if point_first {
            [&point, &square]
        } else {
            [&square, &point]
        };
        for path in groups {
            co.add_path(path, JoinType::Miter, EndType::Polygon);
        }
        assert_eq!(area_paths(&co.execute(-5.).unwrap()).abs(), 8200.);
        assert_eq!(area_paths(&co.execute(-5.).unwrap()).abs(), 8200.);
    }
}

#[test]
fn offset_single_point_uses_full_integer_distance() {
    for join in [JoinType::Miter, JoinType::Square, JoinType::Bevel] {
        let paths = inflate_paths(
            &vec![vec![Point64::new(0, 0)]],
            3e9,
            join,
            EndType::Square,
            2.,
            0.,
        )
        .unwrap();
        let bounds = get_bounds_paths(&paths);
        assert_eq!(
            (bounds.left, bounds.top, bounds.right, bounds.bottom),
            (-3_000_000_000, -3_000_000_000, 3_000_000_000, 3_000_000_000)
        );
    }
    let paths = inflate_paths_d(
        &vec![vec![PointD::new(0., 0.)]],
        30.,
        JoinType::Miter,
        EndType::Square,
        2.,
        8,
        0.,
    )
    .unwrap();
    let bounds = get_bounds_paths(&paths);
    assert_eq!(
        (bounds.left, bounds.top, bounds.right, bounds.bottom),
        (-30., -30., 30., 30.)
    );
}

#[test]
fn offset_out_of_range_result_is_an_error() {
    for join in [JoinType::Miter, JoinType::Round] {
        let mut co = ClipperOffset::default();
        co.add_path(&square(), join, EndType::Polygon);
        assert_eq!(co.execute(1e19), Err(ClipperError::CoordinateOutOfRange));
        assert_eq!(
            co.execute_tree(1e19).err(),
            Some(ClipperError::CoordinateOutOfRange)
        );
        let mut cb = |_: &Path64, _: &PathD, _: usize, _: usize| 1e19;
        assert_eq!(
            co.execute_with_callback(&mut cb),
            Err(ClipperError::CoordinateOutOfRange)
        );
        assert!(!co.execute(5.).unwrap().is_empty());
    }
    assert!(
        inflate_paths(
            &vec![square()],
            -100.,
            JoinType::Miter,
            EndType::Polygon,
            2.,
            0.
        )
        .unwrap()
        .is_empty()
    );
}

#[test]
fn offset_single_point_rejects_distance_and_coordinate_overflow() {
    for (point, delta) in [
        (Point64::new(0, 0), 1e19),
        (Point64::new(MAX_COORD, 0), 1024.),
    ] {
        let mut co = ClipperOffset::default();
        co.add_path(&vec![point], JoinType::Miter, EndType::Square);
        assert_eq!(co.execute(delta), Err(ClipperError::CoordinateOutOfRange));
        if point.x == 0 {
            assert!(!co.execute(1.).unwrap().is_empty());
        } else {
            assert!(co.execute(0.).is_ok());
        }
    }
}

#[test]
fn offset_zero_delta_validates_input_without_rounding_it() {
    let invalid = vec![vec![Point64::new(MAX_COORD + 1, 0)]];
    assert_eq!(
        inflate_paths(&invalid, 0., JoinType::Miter, EndType::Polygon, 2., 0.),
        Err(ClipperError::CoordinateOutOfRange)
    );
    let invalid = vec![vec![PointD::new(f64::NAN, 0.)]];
    assert_eq!(
        inflate_paths_d(&invalid, 0., JoinType::Miter, EndType::Polygon, 2., 2, 0.),
        Err(ClipperError::CoordinateOutOfRange)
    );
    let valid = vec![vec![PointD::new(0.12345, -0.54321)]];
    assert_eq!(
        inflate_paths_d(&valid, 0., JoinType::Miter, EndType::Polygon, 2., 2, 0.).unwrap(),
        valid
    );
}

#[test]
fn minkowski_checks_both_operands_before_arithmetic() {
    let invalid = make_path(&[i64::MAX, 0, i64::MAX, 10, i64::MAX - 10, 0]).unwrap();
    let empty = vec![];
    for (pattern, path) in [(&invalid, &invalid), (&invalid, &empty), (&empty, &invalid)] {
        for closed in [true, false] {
            assert_eq!(
                minkowski_sum(pattern, path, closed),
                Err(ClipperError::CoordinateOutOfRange)
            );
            assert_eq!(
                minkowski_diff(pattern, path, closed),
                Err(ClipperError::CoordinateOutOfRange)
            );
        }
    }
}

#[test]
fn minkowski_checks_decimal_precision() {
    let point = vec![PointD::new(1., 2.)];
    for precision in [-9, 9, 309, i32::MIN, i32::MAX] {
        assert_eq!(
            minkowski_sum_d(&point, &point, true, precision),
            Err(ClipperError::PrecisionOutOfRange)
        );
        assert_eq!(
            minkowski_diff_d(&point, &point, true, precision),
            Err(ClipperError::PrecisionOutOfRange)
        );
    }
}

#[test]
fn minkowski_checks_scaled_coordinates() {
    let valid = vec![PointD::new(1., 2.)];
    for value in [f64::NAN, f64::INFINITY, -1e30, 1e30] {
        let invalid = vec![PointD::new(value, 0.)];
        for (pattern, path) in [(&invalid, &valid), (&valid, &invalid)] {
            assert_eq!(
                minkowski_sum_d(pattern, path, true, 2),
                Err(ClipperError::CoordinateOutOfRange)
            );
            assert_eq!(
                minkowski_diff_d(pattern, path, true, 2),
                Err(ClipperError::CoordinateOutOfRange)
            );
        }
    }
}

#[test]
fn ellipse_rejects_non_finite_radii() {
    for radius in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for steps in [0, 4] {
            assert!(ellipse(Point64::new(0, 0), radius, 1., steps).is_empty());
            assert!(ellipse(Point64::new(0, 0), 1., radius, steps).is_empty());
        }
    }
}

#[test]
fn ellipse_rejects_unrepresentable_capacity() {
    for radius in [1e300, f64::MAX] {
        assert!(ellipse(Point64::new(0, 0), radius, radius, 0).is_empty());
    }
    assert!(ellipse(Point64::new(0, 0), 1., 1., usize::MAX).is_empty());
}

#[test]
fn ellipse_preserves_circle_fallback_and_small_radii() {
    assert_eq!(
        ellipse(Point64::new(0, 0), 2., -1., 4),
        vec![
            Point64::new(2, 0),
            Point64::new(0, 2),
            Point64::new(-2, 0),
            Point64::new(0, -2),
        ]
    );
    assert_eq!(
        ellipse(Point64::new(0, 0), 0.01, 0.01, 0),
        vec![Point64::new(0, 0)]
    );
}
