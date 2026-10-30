//! Test-only bindings to the upstream C++ Clipper2 2.0.1 (built by `build.rs`).
//!
//! Every function mirrors one upstream API and takes Clipper2's own enum codes
//! (`ClipType::Intersection = 1`, `FillRule::NonZero = 1`, ...). The `z` module is the
//! `USINGZ` build, whose Z callback sets `z = e1bot.z + e1top.z + e2bot.z + e2top.z`.

#![allow(clippy::too_many_arguments)] // mirrors the upstream signatures

use std::ptr;

/// A 2D (`(x, y)`) or 3D (`(x, y, z)`) integer point as exchanged with the reference.
pub trait RefPoint: Copy {
    fn xyz(self) -> (i64, i64, i64);
    fn from_xyz(x: i64, y: i64, z: i64) -> Self;
}

impl RefPoint for (i64, i64) {
    fn xyz(self) -> (i64, i64, i64) {
        (self.0, self.1, 0)
    }
    fn from_xyz(x: i64, y: i64, _z: i64) -> Self {
        (x, y)
    }
}

impl RefPoint for (i64, i64, i64) {
    fn xyz(self) -> (i64, i64, i64) {
        self
    }
    fn from_xyz(x: i64, y: i64, z: i64) -> Self {
        (x, y, z)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Solution<P> {
    pub closed: Vec<Vec<P>>,
    pub open: Vec<Vec<P>>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Node<P> {
    pub path: Vec<P>,
    /// Index of the parent node in `Tree::nodes` (DFS pre-order), -1 at the top level.
    pub parent: i32,
    pub is_hole: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Tree<P> {
    pub nodes: Vec<Node<P>>,
    pub open: Vec<Vec<P>>,
}

#[derive(Clone, Debug)]
pub struct OffsetGroup<P> {
    pub paths: Vec<Vec<P>>,
    pub join_type: u8,
    pub end_type: u8,
}

pub type RefPath = Vec<(i64, i64)>;
pub type RefPaths = Vec<RefPath>;
pub type RefSolution = Solution<(i64, i64)>;
pub type RefNode = Node<(i64, i64)>;
pub type RefTree = Tree<(i64, i64)>;
pub type RefOffsetGroup = OffsetGroup<(i64, i64)>;

const FLAG_OPEN: u8 = 1;
const FLAG_HOLE: u8 = 2;

#[repr(C)]
struct In64 {
    xy: *const i64,
    z: *const i64,
    lens: *const u32,
    count: usize,
}

#[repr(C)]
struct RefView {
    xy: *const i64,
    xy_len: usize,
    xyd: *const f64,
    xyd_len: usize,
    z: *const i64,
    z_len: usize,
    lens: *const u32,
    parents: *const i32,
    flags: *const u8,
    path_count: usize,
}

#[repr(C)]
struct RefOut {
    _private: [u8; 0],
}

struct Flat {
    xy: Vec<i64>,
    z: Vec<i64>,
    lens: Vec<u32>,
}

impl Flat {
    fn new<P: RefPoint>(paths: &[Vec<P>]) -> Self {
        let mut flat = Flat {
            xy: Vec::new(),
            z: Vec::new(),
            lens: Vec::new(),
        };
        for path in paths {
            flat.lens.push(path.len() as u32);
            for p in path {
                let (x, y, z) = p.xyz();
                flat.xy.extend([x, y]);
                flat.z.push(z);
            }
        }
        flat
    }

    fn input(&self) -> In64 {
        In64 {
            xy: self.xy.as_ptr(),
            z: self.z.as_ptr(),
            lens: self.lens.as_ptr(),
            count: self.lens.len(),
        }
    }
}

struct Decoded<P> {
    paths: Vec<Vec<P>>,
    parents: Vec<i32>,
    flags: Vec<u8>,
}

fn slice<'a, T>(ptr: *const T, len: usize) -> &'a [T] {
    if len == 0 {
        &[]
    } else {
        unsafe { std::slice::from_raw_parts(ptr, len) }
    }
}

fn decode<P: RefPoint>(view: &RefView) -> Decoded<P> {
    let xy = slice(view.xy, view.xy_len);
    let z = slice(view.z, view.z_len);
    let lens = slice(view.lens, view.path_count);
    let mut paths = Vec::with_capacity(lens.len());
    let mut k = 0;
    for &len in lens {
        let mut path = Vec::with_capacity(len as usize);
        for _ in 0..len {
            path.push(P::from_xyz(
                xy[2 * k],
                xy[2 * k + 1],
                z.get(k).copied().unwrap_or(0),
            ));
            k += 1;
        }
        paths.push(path);
    }
    Decoded {
        paths,
        parents: slice(view.parents, view.path_count).to_vec(),
        flags: slice(view.flags, view.path_count).to_vec(),
    }
}

fn status(code: i32) -> Result<(), String> {
    match code {
        0 => Ok(()),
        1 => Err("Execute returned false".into()),
        _ => Err("C++ exception".into()),
    }
}

fn split_open<P>(d: Decoded<P>) -> Solution<P> {
    let mut s = Solution {
        closed: Vec::new(),
        open: Vec::new(),
    };
    for (path, flags) in d.paths.into_iter().zip(d.flags) {
        if flags & FLAG_OPEN != 0 {
            s.open.push(path)
        } else {
            s.closed.push(path)
        }
    }
    s
}

fn into_tree<P>(d: Decoded<P>) -> Tree<P> {
    let mut t = Tree {
        nodes: Vec::new(),
        open: Vec::new(),
    };
    for ((path, parent), flags) in d.paths.into_iter().zip(d.parents).zip(d.flags) {
        if flags & FLAG_OPEN != 0 {
            t.open.push(path);
        } else {
            t.nodes.push(Node {
                path,
                parent,
                is_hole: flags & FLAG_HOLE != 0,
            });
        }
    }
    t
}

macro_rules! reference_api {
    ($prefix:literal, $P:ty) => {
        use super::*;

        unsafe extern "C" {
            #[link_name = concat!($prefix, "boolean64")]
            fn boolean64(
                ct: u8,
                fr: u8,
                pc: i32,
                rev: i32,
                tree: i32,
                s: In64,
                so: In64,
                c: In64,
                out: *mut *mut RefOut,
            ) -> i32;
            #[link_name = concat!($prefix, "offset64")]
            fn offset64(
                delta: f64,
                miter: f64,
                arc: f64,
                pc: i32,
                rev: i32,
                tree: i32,
                paths: In64,
                path_group: *const u32,
                group_count: usize,
                jt: *const u8,
                et: *const u8,
                out: *mut *mut RefOut,
            ) -> i32;
            #[link_name = concat!($prefix, "rectclip64")]
            fn rectclip64(
                l: i64,
                t: i64,
                r: i64,
                b: i64,
                lines: i32,
                paths: In64,
                out: *mut *mut RefOut,
            ) -> i32;
            #[link_name = concat!($prefix, "triangulate64")]
            fn triangulate64(
                delaunay: i32,
                paths: In64,
                result: *mut i32,
                out: *mut *mut RefOut,
            ) -> i32;
            #[link_name = concat!($prefix, "view")]
            fn view(r: *const RefOut, v: *mut RefView);
            #[link_name = concat!($prefix, "free")]
            fn free(r: *mut RefOut);
        }

        fn take(code: i32, out: *mut RefOut) -> Result<Decoded<$P>, String> {
            if out.is_null() {
                status(code)?;
                return Err("no result".into());
            }
            let mut v = std::mem::MaybeUninit::<RefView>::uninit();
            // SAFETY: `out` is a live RefOut from the shim, released right after decoding.
            let decoded = unsafe {
                view(out, v.as_mut_ptr());
                let d = decode::<$P>(&v.assume_init());
                free(out);
                d
            };
            status(code).map(|_| decoded)
        }

        fn run_boolean(
            ct: u8,
            fr: u8,
            pc: bool,
            rev: bool,
            tree: bool,
            s: &[Vec<$P>],
            so: &[Vec<$P>],
            c: &[Vec<$P>],
        ) -> Result<Decoded<$P>, String> {
            let (s, so, c) = (Flat::new(s), Flat::new(so), Flat::new(c));
            let mut out = ptr::null_mut();
            // SAFETY: inputs outlive the call; the shim copies them.
            let code = unsafe {
                boolean64(
                    ct,
                    fr,
                    pc as i32,
                    rev as i32,
                    tree as i32,
                    s.input(),
                    so.input(),
                    c.input(),
                    &mut out,
                )
            };
            take(code, out)
        }

        /// `Clipper64::Execute` into closed and open paths.
        pub fn boolean(
            ct: u8,
            fr: u8,
            preserve_collinear: bool,
            reverse_solution: bool,
            subjects: &[Vec<$P>],
            open_subjects: &[Vec<$P>],
            clips: &[Vec<$P>],
        ) -> Result<Solution<$P>, String> {
            run_boolean(
                ct,
                fr,
                preserve_collinear,
                reverse_solution,
                false,
                subjects,
                open_subjects,
                clips,
            )
            .map(split_open)
        }

        /// `Clipper64::Execute` into a `PolyTree64` (flattened, DFS pre-order) and open paths.
        pub fn boolean_tree(
            ct: u8,
            fr: u8,
            preserve_collinear: bool,
            reverse_solution: bool,
            subjects: &[Vec<$P>],
            open_subjects: &[Vec<$P>],
            clips: &[Vec<$P>],
        ) -> Result<Tree<$P>, String> {
            run_boolean(
                ct,
                fr,
                preserve_collinear,
                reverse_solution,
                true,
                subjects,
                open_subjects,
                clips,
            )
            .map(into_tree)
        }

        fn run_offset(
            delta: f64,
            groups: &[OffsetGroup<$P>],
            miter: f64,
            arc: f64,
            pc: bool,
            rev: bool,
            tree: bool,
        ) -> Result<Decoded<$P>, String> {
            let mut all = Vec::new();
            let mut path_group = Vec::new();
            for (g, group) in groups.iter().enumerate() {
                all.extend(group.paths.iter().cloned());
                path_group.extend(std::iter::repeat_n(g as u32, group.paths.len()));
            }
            let flat = Flat::new(&all);
            let jt: Vec<u8> = groups.iter().map(|g| g.join_type).collect();
            let et: Vec<u8> = groups.iter().map(|g| g.end_type).collect();
            let mut out = ptr::null_mut();
            // SAFETY: inputs outlive the call; the shim copies them.
            let code = unsafe {
                offset64(
                    delta,
                    miter,
                    arc,
                    pc as i32,
                    rev as i32,
                    tree as i32,
                    flat.input(),
                    path_group.as_ptr(),
                    groups.len(),
                    jt.as_ptr(),
                    et.as_ptr(),
                    &mut out,
                )
            };
            take(code, out)
        }

        /// `ClipperOffset::Execute(delta, Paths64&)`, one `AddPaths` per group in order.
        pub fn offset(
            delta: f64,
            groups: &[OffsetGroup<$P>],
            miter_limit: f64,
            arc_tolerance: f64,
            preserve_collinear: bool,
            reverse_solution: bool,
        ) -> Result<Vec<Vec<$P>>, String> {
            run_offset(
                delta,
                groups,
                miter_limit,
                arc_tolerance,
                preserve_collinear,
                reverse_solution,
                false,
            )
            .map(|d| d.paths)
        }

        /// `ClipperOffset::Execute(delta, PolyTree64&)`.
        pub fn offset_tree(
            delta: f64,
            groups: &[OffsetGroup<$P>],
            miter_limit: f64,
            arc_tolerance: f64,
            preserve_collinear: bool,
            reverse_solution: bool,
        ) -> Result<Tree<$P>, String> {
            run_offset(
                delta,
                groups,
                miter_limit,
                arc_tolerance,
                preserve_collinear,
                reverse_solution,
                true,
            )
            .map(into_tree)
        }

        fn run_rect(rect: (i64, i64, i64, i64), lines: bool, paths: &[Vec<$P>]) -> Vec<Vec<$P>> {
            let flat = Flat::new(paths);
            let mut out = ptr::null_mut();
            // SAFETY: inputs outlive the call; the shim copies them.
            let code = unsafe {
                rectclip64(
                    rect.0,
                    rect.1,
                    rect.2,
                    rect.3,
                    lines as i32,
                    flat.input(),
                    &mut out,
                )
            };
            take(code, out).expect("RectClip threw").paths
        }

        /// `RectClip(rect, paths)` with `rect = (left, top, right, bottom)`.
        pub fn rect_clip(rect: (i64, i64, i64, i64), paths: &[Vec<$P>]) -> Vec<Vec<$P>> {
            run_rect(rect, false, paths)
        }

        /// `RectClipLines(rect, paths)`.
        pub fn rect_clip_lines(rect: (i64, i64, i64, i64), paths: &[Vec<$P>]) -> Vec<Vec<$P>> {
            run_rect(rect, true, paths)
        }

        /// `Triangulate(paths, solution, use_delaunay)`: the `TriangulateResult` code and the triangles.
        pub fn triangulate(paths: &[Vec<$P>], use_delaunay: bool) -> (u8, Vec<Vec<$P>>) {
            let flat = Flat::new(paths);
            let mut result = 0;
            let mut out = ptr::null_mut();
            // SAFETY: inputs outlive the call; the shim copies them.
            let code =
                unsafe { triangulate64(use_delaunay as i32, flat.input(), &mut result, &mut out) };
            (
                result as u8,
                take(code, out).expect("Triangulate threw").paths,
            )
        }
    };
}

mod plain {
    reference_api!("ref_", (i64, i64));
}

pub use plain::*;

/// The `USINGZ` build of the reference.
pub mod z {
    reference_api!("ref_z_", (i64, i64, i64));

    pub type RefPathZ = Vec<(i64, i64, i64)>;
    pub type RefPathsZ = Vec<RefPathZ>;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn shoelace(path: &RefPath) -> f64 {
        let n = path.len();
        let mut a = 0.0;
        for i in 0..n {
            let (x1, y1) = path[i];
            let (x2, y2) = path[(i + 1) % n];
            a += (x1 as f64) * (y2 as f64) - (x2 as f64) * (y1 as f64);
        }
        a / 2.0
    }

    fn square(x0: i64, y0: i64, x1: i64, y1: i64) -> RefPath {
        vec![(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    }

    #[test]
    fn union_of_overlapping_squares_has_area_7() {
        let out = boolean(
            2,
            1,
            false,
            false,
            &[square(0, 0, 2, 2)],
            &[],
            &[square(1, 1, 3, 3)],
        )
        .unwrap();
        assert_eq!(out.closed.len(), 1);
        assert_eq!(shoelace(&out.closed[0]).abs(), 7.0);
    }

    #[test]
    fn tree_marks_hole_under_outer() {
        let outer = square(0, 0, 10, 10);
        let hole = vec![(3, 3), (3, 7), (7, 7), (7, 3)];
        let t = boolean_tree(2, 0, false, false, &[outer, hole], &[], &[]).unwrap();
        let shape: Vec<(i32, bool)> = t.nodes.iter().map(|n| (n.parent, n.is_hole)).collect();
        assert_eq!(shape, vec![(-1, false), (0, true)]);
    }

    #[test]
    fn offset_square_by_one_with_miter_has_area_16() {
        let g = RefOffsetGroup {
            paths: vec![square(0, 0, 2, 2)],
            join_type: 3,
            end_type: 0,
        };
        let out = offset(1.0, &[g], 2.0, 0.0, false, false).unwrap();
        assert_eq!(shoelace(&out[0]).abs(), 16.0);
    }

    #[test]
    fn rect_clip_square_to_unit_rect_has_area_1() {
        let out = rect_clip((1, 1, 2, 2), &[square(0, 0, 10, 10)]);
        assert_eq!(out.len(), 1);
        assert_eq!(shoelace(&out[0]).abs(), 1.0);
    }

    #[test]
    fn triangulate_square_gives_two_triangles() {
        let (code, tris) = triangulate(&[square(0, 0, 10, 10)], true);
        assert_eq!(code, 0);
        assert_eq!(tris.len(), 2);
    }

    #[test]
    fn z_union_sets_intersection_z_by_rule() {
        let a: z::RefPathZ = vec![(0, 0, 1), (10, 0, 1), (10, 10, 1), (0, 10, 1)];
        let b: z::RefPathZ = vec![(5, 5, 10), (15, 5, 10), (15, 15, 10), (5, 15, 10)];
        let out = z::boolean(2, 1, false, false, &[a], &[], &[b]).unwrap();
        let path = &out.closed[0];
        let at = |x, y| path.iter().find(|p| p.0 == x && p.1 == y).map(|p| p.2);
        assert_eq!(at(10, 5), Some(22));
        assert_eq!(at(5, 10), Some(22));
        assert_eq!(at(0, 0), Some(1));
    }
}
