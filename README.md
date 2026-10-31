# clipper2-rs

A pure Rust port of [Clipper2](https://github.com/PolyClipping/Clipper2) by Angus Johnson (release 2.0.1
plus the upstream fixes made since, commit `b496b45`):
polygon boolean operations (intersection, union, difference, XOR), offsetting, rectangle
clipping, Minkowski sums and differences, constrained Delaunay triangulation, and optional
Z values. It has no dependencies and no `unsafe` code (`#![forbid(unsafe_code)]`).

The port follows the C++ library function by function, and its results are checked bit for bit
against the C++ library. The checks run on thousands of random inputs for every operation.

```toml
[dependencies]
clipper2-rs = "0.1"
```

```rust
use clipper2_rs::{FillRule, intersect, make_path};

fn main() -> Result<(), clipper2_rs::ClipperError> {
    let subject = vec![make_path(&[100, 50, 10, 79, 65, 2, 65, 98, 10, 21])?];
    let clip = vec![make_path(&[98, 63, 4, 68, 77, 8, 52, 100, 19, 12])?];
    let solution = intersect(&subject, &clip, FillRule::NonZero)?;
    println!("{solution:?}");
    Ok(())
}
```

Runnable examples are in [`clipper2-rs/examples`](clipper2-rs/examples). Each one runs with
`cargo run --example <name>`:

| Example | Shows |
|---|---|
| `boolean` | booleans on `Paths64` and `PathsD`, and a `PolyTree64` |
| `offset` | inflating and shrinking polygons, and stroking an open path |
| `rectclip` | clipping polygons and polylines to a rectangle |
| `triangulate` | triangulating a polygon with a hole |

## What is covered

The whole public C++ API except the C/DLL export layer (`clipper.export.h`). That includes:
- `Clipper64` and `ClipperD`, with paths or a `PolyTree`;
- `ClipperOffset` and `inflate_paths`;
- `RectClip64` and `RectClipLines64`;
- the Minkowski functions;
- `triangulate`;
- the path utilities, such as simplification, Ramer–Douglas–Peucker and `trim_collinear`;
- the one-call boolean functions;
- reusable input data (`ReuseableDataContainer64`).

API names follow Rust conventions (`InflatePaths` → `inflate_paths`). Floating-point APIs scale
by a precision given in decimal places.

## How it is tested

- **Differential tests:** the `clipper2-ref` crate in this repository compiles the upstream C++
  sources and exposes them to Rust. Every operation runs on generated inputs, and the Rust and C++
  results must be equal: every coordinate, every path, and the same `PolyTree` shape. The inputs
  include coordinates near the range limits, collinear and coincident edges, open paths, all
  clip types, fill rules, join and end types, and precisions. `CLIPPER2_DIFF_CASES` sets the
  number of cases per test (300 by default; releases are checked with 20,000).
- **Upstream's own test suite:** all 49 gtest cases and their data files are ported in
  `tests/upstream*.rs`, with upstream's tolerances.
- Both suites also run with the `usingz` feature.

## Performance

These are criterion medians on an Apple Silicon Mac. The C++ library was built with clang `-O2`
and timed inside C++ on the same inputs, so FFI copies are not counted. A ratio below 1 means
clipper2-rs is faster.

| Operation | Input | Rust / C++ |
|---|---|---|
| Intersection, union, difference, XOR | 1,000 and 50,000 points | 0.74 – 0.89 |
| `inflate_paths` (all join types) | 10,000 points | 0.74 – 0.83 |
| `minkowski_sum` | 2,000 points | 0.93 |
| `rect_clip` | 50,000 points | 0.96 |
| `simplify_paths` | 50,000 points | 0.83 |
| `triangulate` (with and without Delaunay) | 9,842 points | 0.68, 0.87 |

Run the comparison yourself with `cargo bench -p clipper2-rs`.

## Differences from the C++ library

Results are identical to upstream except in these cases:

1. **Order of equal intersections.** Upstream sorts intersections with `std::sort`, which leaves
   equal elements in whatever order the C++ standard library happens to produce. Results can
   therefore differ between libc++, libstdc++ and MSVC. clipper2-rs sorts stably, so its results
   are the same on every platform. The same applies to the two sorts in the triangulation.
   One upstream test case (`Polygons.txt` #168) depends on this order: libc++ gives 101
   polygons, a stable sort gives 99, and the stored value is 106 ± 5. Only that count is exempt
   in the ported test.
2. **`triangulate` always terminates.** On some self-intersecting paths, upstream's outline walk
   never ends and keeps allocating memory. clipper2-rs stops that walk after one full lap and
   returns `TriangulateError::Degenerate`. See
   [PolyClipping/Clipper2#6](https://github.com/PolyClipping/Clipper2/issues/6).

3. **Invalid input gives an error instead of undefined behaviour.**
   - `Clipper64` and `ClipperOffset` reject integer coordinates outside
     `MIN_COORD..=MAX_COORD` with `ClipperError::CoordinateOutOfRange`. Upstream checks the range
     only when scaling floating-point input.
   - A NaN or infinite offset delta, including one returned by a delta callback, is
     `ClipperError::InvalidDelta`. Upstream loops or allocates without bound.
   - Empty paths given to the offsetter are skipped. Upstream reads out of bounds.
   - If the intersection list cannot be processed in order, `execute` returns
     `ClipperError::ExecuteFailed`. Upstream reads past the end of the list.
   - An internal error in the triangulation is `TriangulateError::Fail` on every platform.
     Upstream throws there only when `__cpp_exceptions` or `__EXCEPTIONS` is defined; built with
     MSVC it carries on into undefined behaviour.

Triangulation is marked as beta upstream.

## The `usingz` feature

`usingz` adds a user `z: i64` to every `Point`, together with Z callbacks on `Clipper64`,
`ClipperD` and `ClipperOffset`. These match upstream's `USINGZ` build. The feature is
**not additive**: it changes the layout of `Point` for every crate in the build that uses
clipper2-rs. Libraries should leave the choice to the final application.

## Repository layout

| Path | Contents |
|---|---|
| `clipper2-rs/` | the published crate |
| `clipper2-ref/` | test-only C++ reference: the upstream sources, built with `cc` |
| `upstream/Clipper2/` | upstream sources and test data (commit `b496b45`) |
| `scripts/memguard.sh` | runs a command and kills it when it exceeds a memory or time limit |

Use `scripts/memguard.sh` for large randomized runs. Upstream's triangulation can exhaust
memory on degenerate input (see 3 above), so a guard keeps such a run from taking down the
machine. For example:

```sh
CLIPPER2_DIFF_CASES=20000 scripts/memguard.sh 2500 1800 cargo test --release --workspace
```

The minimum supported Rust version is 1.85.

## License

clipper2-rs is a derivative work of Clipper2 and uses the same
[Boost Software License 1.0](LICENSE).

- Clipper2: © 2010–2025 Angus Johnson.
- The Rust port: © 2026 the clipper2-rs authors.
