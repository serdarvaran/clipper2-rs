# Changelog

## Unreleased

- Keep each offset group's distance sign local; adding a degenerate group no longer
  changes shrinking polygons into growing ones.
- Remove the 32-bit distance limit from non-round single-point offsets.
- Propagate final-union errors from offset and Minkowski operations instead of returning
  successful empty geometry; validate zero-distance offsets too.
- Reject non-finite coordinates and zero/non-finite scale factors. Check rounded integer
  coordinates for both single-path and multi-path scaling, including negative scales.
- Validate Minkowski operands before arithmetic and enforce floating-point precision limits.
- Return an empty ellipse for non-finite radii or an unallocatable point count.

## 0.2.0 (2026-11-01)

- Triangulation, which upstream has withdrawn, is maintained here and no longer follows upstream
  bit for bit:
  - every result is checked to cover the input exactly; `TriangulateError::Fail` is returned
    instead of a wrong result;
  - a boundary that crosses itself is an error, normally `TriangulateError::PathsIntersect`,
    also when edges cross within a unit of an end point (upstream moved the end point, which
    could give a wrong result);
  - fixed: an edge from an inner local minimum crossing a polygon edge; a fan closed across a
    vertex further along the same row (a T-junction); collinear vertices that a touching path
    shares; triangles with a side of length 1 being dropped; the inner local minimum of a vertex
    shared by several paths;
  - clockwise input: upstream's orientation fix flipped a local minimum back when it was queued
    twice, and took the orientation from the lowest minimum, which can belong to a hole; the
    total area's sign decides now;
  - coordinates outside `MIN_COORD..=MAX_COORD` are `TriangulateError::CoordinateOutOfRange`;
  - paths that run back along themselves or share an edge with another path (after splitting
    at collinear vertices) have the coinciding edges removed;
  - where the input winds around a point more than once (overlapping paths, outside the
    contract), the triangles cover it as often; disjoint outer paths of opposite orientation
    are `Fail` (upstream returned a single triangle);
  - known limitation: `Fail` on about 0.1% of degenerate small-grid input (paths running back
    along themselves or sharing edges) that upstream triangulates;
  - the checks cost time: the triangulation now takes 0.86 (without Delaunay) and 1.00 (with)
    of the C++ library's time, from 0.68 and 0.87.
- Upstream fixes made after 2.0.1 (now tracked from
  [PolyClipping/Clipper2](https://github.com/PolyClipping/Clipper2), commit `b496b45`):
  - `FixSelfIntersects` no longer special-cases adjacent micro self-intersections; it splits
    them like any other (upstream #1067). Some boolean, offset and Minkowski results change.
  - Triangulation ignores segments that share an end point when testing for intersections,
    instead of relying on an exact zero test (upstream #1055, #1056).
  - `CheckSplitOwner` uses upstream's own recursion guard (f97d65d), which replaces this
    crate's: an emptied split that was already visited is now skipped entirely. No longer a
    difference from upstream.
  - Upstream adopted the two scanline speed-ups this crate already had (PolyClipping #2, #3);
    `DoTopOfScanbeam` now also reuses the edge positions computed for an unchanged scanbeam.

## 0.1.0 (2026-10-31)

First release: a port of Clipper2 2.0.1 (upstream commit `21ebba0`).

- Covers `Clipper64` and `ClipperD`, `PolyTree`, `ClipperOffset`, `RectClip64` and
  `RectClipLines64`, the Minkowski functions, triangulation, the path utilities and the
  `usingz` feature.
- Results are checked bit for bit against the C++ library, and upstream's gtest suite is
  ported.
- Differences from upstream:
  - stable sorting of equal intersections;
  - a recursion guard in `CheckSplitOwner`;
  - `TriangulateError::Degenerate` instead of a non-terminating outline walk;
  - errors instead of undefined behaviour for out-of-range coordinates, non-finite offset
    deltas, empty paths in the offsetter, and an unprocessable intersection list.

  See the README for details.
- Faster than the C++ library on every benchmarked operation.
