//! Path utilities from `clipper.h`.

use std::f64::consts::PI;

use crate::core::{Coord, Path, Point, Rect};

/// An ellipse approximated by `steps` points (`steps <= 2` picks a count from the radii).
/// `radius_y <= 0` means a circle.
pub fn ellipse<T: Coord>(
    center: Point<T>,
    radius_x: f64,
    mut radius_y: f64,
    mut steps: usize,
) -> Path<T> {
    if radius_x <= 0.0 {
        return Vec::new();
    }
    if radius_y <= 0.0 {
        radius_y = radius_x;
    }
    if steps <= 2 {
        steps = (PI * ((radius_x + radius_y) / 2.0).sqrt()) as usize;
    }
    let si = (2.0 * PI / steps as f64).sin();
    let co = (2.0 * PI / steps as f64).cos();
    let (mut dx, mut dy) = (co, si);
    let cx = center.x.to_f64();
    let cy = center.y.to_f64();
    let mut result = Vec::with_capacity(steps);
    result.push(Point::from_f64(cx + radius_x, cy));
    for _ in 1..steps {
        result.push(Point::from_f64(cx + radius_x * dx, cy + radius_y * dy));
        let x = dx * co - dy * si;
        dy = dy * co + dx * si;
        dx = x;
    }
    result
}

/// [`ellipse`] inscribed in a rectangle.
pub fn ellipse_rect<T: Coord>(rect: &Rect<T>, steps: usize) -> Path<T> {
    ellipse(
        rect.mid_point(),
        rect.width().to_f64() * 0.5,
        rect.height().to_f64() * 0.5,
        steps,
    )
}
