# Changelog

## 0.1.0 (unreleased)

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
