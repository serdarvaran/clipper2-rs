# Changelog

## Unreleased

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
