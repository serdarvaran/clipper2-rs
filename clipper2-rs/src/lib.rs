//! Pure Rust port of [Clipper2](https://github.com/AngusJohnson/Clipper2) 2.0.1 by Angus Johnson.
#![forbid(unsafe_code)]

#[allow(dead_code)] // TEMP: internal helpers are consumed by the engine tasks; removed in task 16
mod core;

pub use crate::core::{
    ClipType, ClipperError, Coord, FillRule, MAX_COORD, MAX_DECIMAL_PRECISION, MIN_COORD, Path,
    Path64, PathD, Paths, Paths64, PathsD, Point, Point64, PointD, PointInPolygonResult, Rect,
    Rect64, RectD, area, area_paths, check_precision, get_bounds, get_bounds_paths, is_positive,
    mid_point, point_in_polygon, scale_path, scale_paths, scale_paths_64_to_d, scale_paths_d_to_64,
    strip_duplicates, strip_duplicates_paths, strip_near_equal, strip_near_equal_paths,
    transform_path, transform_paths,
};
