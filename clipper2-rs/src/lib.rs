//! Pure Rust port of [Clipper2](https://github.com/AngusJohnson/Clipper2) 2.0.1 by Angus Johnson.
#![forbid(unsafe_code)]

mod clipper_d;
#[allow(dead_code)] // TEMP: internal helpers are consumed by later tasks; removed in task 16
mod core;
mod engine;
mod offset;
mod polytree;
mod rectclip;
mod utils;

pub use crate::core::{
    ClipType, ClipperError, Coord, FillRule, MAX_COORD, MAX_DECIMAL_PRECISION, MIN_COORD, Path,
    Path64, PathD, Paths, Paths64, PathsD, Point, Point64, PointD, PointInPolygonResult, Rect,
    Rect64, RectD, area, area_paths, check_precision, get_bounds, get_bounds_paths, is_positive,
    mid_point, point_in_polygon, scale_path, scale_paths, scale_paths_64_to_d, scale_paths_d_to_64,
    strip_duplicates, strip_duplicates_paths, strip_near_equal, strip_near_equal_paths,
    transform_path, transform_paths,
};
#[cfg(feature = "usingz")]
pub use clipper_d::ZCallbackD;
pub use clipper_d::{ClipperD, SolutionD};
#[cfg(feature = "usingz")]
pub use engine::ZCallback64;
pub use engine::{Clipper64, PathType, ReuseableDataContainer64, Solution};
pub use offset::{
    ClipperOffset, DeltaCallback64, EndType, JoinType, inflate_paths, inflate_paths_d,
};
pub use polytree::{PolyNode, PolyNode64, PolyNodeD, PolyTree, PolyTree64, PolyTreeD};
pub use rectclip::{
    RectClip64, RectClipLines64, rect_clip, rect_clip_d, rect_clip_lines, rect_clip_lines_d,
    rect_clip_lines_path, rect_clip_path,
};
pub use utils::{ellipse, ellipse_rect};
