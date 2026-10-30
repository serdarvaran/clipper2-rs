//! Shared test support: a deterministic generator, conversions to the reference's tuple
//! form, an exact comparator that dumps failures, and the upstream test-data reader.
#![allow(dead_code)]

use std::fmt::Debug;
use std::path::{Path, PathBuf};

use clipper2_rs::{ClipType, FillRule, MAX_COORD, Path64, Paths64, PathsD, Point64, PointD};

/// xorshift64*: small, fast and identical on every platform.
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
    }

    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform in `lo..=hi`.
    pub fn range_i64(&mut self, lo: i64, hi: i64) -> i64 {
        let span = (hi as i128 - lo as i128 + 1) as u128;
        (lo as i128 + (self.next_u64() as u128 % span) as i128) as i64
    }

    pub fn range_f64(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64 * (hi - lo)
    }

    /// Uniform in `0..n`.
    pub fn below(&mut self, n: usize) -> usize {
        (self.next_u64() % n as u64) as usize
    }

    pub fn bool(&mut self) -> bool {
        self.next_u64() & 1 == 1
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Shape {
    /// Coordinates `0..=20`: many collinear, horizontal and coincident edges.
    SmallGrid,
    /// Coordinates `±10_000`, up to 60 points.
    Medium,
    /// Coordinates `±MAX_COORD / 2`.
    Large,
    /// A star with a reversed hole inside.
    Star,
}

impl Shape {
    pub const ALL: [Shape; 4] = [Shape::SmallGrid, Shape::Medium, Shape::Large, Shape::Star];

    fn coord_range(self) -> i64 {
        match self {
            Shape::SmallGrid => 20,
            Shape::Medium | Shape::Star => 10_000,
            Shape::Large => MAX_COORD / 2,
        }
    }
}

fn random_path(rng: &mut Rng, shape: Shape, min_pts: usize, max_pts: usize) -> Path64 {
    let n = min_pts + rng.below(max_pts - min_pts + 1);
    let r = shape.coord_range();
    let lo = if shape == Shape::SmallGrid { 0 } else { -r };
    (0..n)
        .map(|_| Point64::new(rng.range_i64(lo, r), rng.range_i64(lo, r)))
        .collect()
}

fn star(rng: &mut Rng) -> Paths64 {
    let spikes = 5 + rng.below(8);
    let cx = rng.range_i64(-1000, 1000) as f64;
    let cy = rng.range_i64(-1000, 1000) as f64;
    let outer = rng.range_f64(2000.0, 8000.0);
    let inner = outer * rng.range_f64(0.3, 0.7);
    let start = rng.range_f64(0.0, std::f64::consts::TAU);
    let mut s = Vec::with_capacity(spikes * 2);
    for i in 0..spikes * 2 {
        let a = start + i as f64 * std::f64::consts::PI / spikes as f64;
        let rad = if i % 2 == 0 { outer } else { inner };
        s.push(Point64::new(
            (cx + rad * a.cos()).round() as i64,
            (cy + rad * a.sin()).round() as i64,
        ));
    }
    let hole_r = inner * 0.5;
    let sides = 3 + rng.below(6);
    let hole = (0..sides)
        .rev()
        .map(|i| {
            let a = i as f64 * std::f64::consts::TAU / sides as f64;
            Point64::new(
                (cx + hole_r * a.cos()).round() as i64,
                (cy + hole_r * a.sin()).round() as i64,
            )
        })
        .collect();
    vec![s, hole]
}

/// Closed paths of the given shape.
pub fn random_paths(rng: &mut Rng, shape: Shape) -> Paths64 {
    match shape {
        Shape::Star => star(rng),
        Shape::SmallGrid => (0..1 + rng.below(3))
            .map(|_| random_path(rng, shape, 3, 12))
            .collect(),
        Shape::Medium => (0..1 + rng.below(3))
            .map(|_| random_path(rng, shape, 3, 60))
            .collect(),
        Shape::Large => (0..1 + rng.below(2))
            .map(|_| random_path(rng, shape, 3, 20))
            .collect(),
    }
}

/// Open paths (2–8 points) in the shape's coordinate range.
pub fn random_open_paths(rng: &mut Rng, shape: Shape) -> Paths64 {
    (0..1 + rng.below(3))
        .map(|_| random_path(rng, shape, 2, 8))
        .collect()
}

/// Closed paths with `precision` decimal places, inside the range `ClipperD` accepts.
pub fn random_paths_d(rng: &mut Rng, shape: Shape, precision: i32) -> PathsD {
    let scale = 10f64.powi(precision);
    let limit = (MAX_COORD as f64 / scale / 4.0).min(1e9) as i64;
    random_paths(rng, shape)
        .into_iter()
        .map(|p| {
            p.into_iter()
                .map(|pt| PointD::new((pt.x % limit) as f64 / scale, (pt.y % limit) as f64 / scale))
                .collect()
        })
        .collect()
}

pub type RefPath = clipper2_ref::RefPath;
pub type RefPaths = clipper2_ref::RefPaths;

pub fn to_ref(paths: &[Path64]) -> RefPaths {
    paths
        .iter()
        .map(|p| p.iter().map(|pt| (pt.x, pt.y)).collect())
        .collect()
}

pub fn from_ref(paths: &[RefPath]) -> Paths64 {
    paths
        .iter()
        .map(|p| p.iter().map(|&(x, y)| Point64::new(x, y)).collect())
        .collect()
}

/// Number of random cases per combination (`CLIPPER2_DIFF_CASES`, default 300).
pub fn cases() -> usize {
    std::env::var("CLIPPER2_DIFF_CASES")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(300)
}

/// Exact equality (path order, point order, values). On mismatch the case is written to
/// `target/clipper2-diff-failures/<case>.txt` and the test panics with that path.
pub fn assert_same<T: PartialEq + Debug>(case: &str, input: &str, ours: &T, reference: &T) {
    if ours == reference {
        return;
    }
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../target/clipper2-diff-failures");
    std::fs::create_dir_all(&dir).unwrap();
    let file = dir.join(format!("{}.txt", case.replace(['/', ' ', ':'], "_")));
    std::fs::write(
        &file,
        format!(
            "case: {case}\ninput:\n{input}\n\nours:\n{ours:#?}\n\nreference:\n{reference:#?}\n"
        ),
    )
    .unwrap();
    panic!(
        "{case}: result differs from the C++ reference, see {}",
        file.display()
    );
}

/// [`assert_same`] for integer paths.
pub fn assert_paths_eq(case: &str, input: &str, ours: &[Path64], reference: &[RefPath]) {
    assert_same(case, input, &to_ref(ours), &reference.to_vec());
}

/// Bit patterns of floating-point paths, so that `-0.0` and `0.0` differ.
pub fn bits_d(paths: &[Vec<(f64, f64)>]) -> Vec<Vec<(u64, u64)>> {
    paths
        .iter()
        .map(|p| p.iter().map(|&(x, y)| (x.to_bits(), y.to_bits())).collect())
        .collect()
}

pub fn upstream_tests_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../upstream/Clipper2-2.0.1/Tests")
}

/// One case of an upstream test-data file (the format `CPP/Utils/ClipFileLoad.cpp` reads).
#[derive(Clone, Debug)]
pub struct UpstreamCase {
    pub caption: String,
    pub clip_type: ClipType,
    pub fill_rule: FillRule,
    pub area: i64,
    pub count: i64,
    pub subjects: Paths64,
    pub subjects_open: Paths64,
    pub clips: Paths64,
}

/// `GetInt`: optional spaces, optional `-`, digits, optional spaces and one comma.
fn get_int(s: &[u8], i: &mut usize) -> Option<i64> {
    while *i < s.len() && s[*i] == b' ' {
        *i += 1;
    }
    if *i == s.len() {
        return None;
    }
    let neg = s[*i] == b'-';
    if neg {
        *i += 1;
    }
    let start = *i;
    let mut v: i64 = 0;
    while *i < s.len() && s[*i].is_ascii_digit() {
        v = v * 10 + (s[*i] - b'0') as i64;
        *i += 1;
    }
    if *i == start {
        return None;
    }
    while *i < s.len() && s[*i] == b' ' {
        *i += 1;
    }
    if *i < s.len() && s[*i] == b',' {
        *i += 1;
    }
    Some(if neg { -v } else { v })
}

fn get_path(line: &str) -> Option<Path64> {
    let s = line.trim_end_matches('\r').as_bytes();
    let mut i = 0;
    let mut p = Vec::new();
    while let Some(x) = get_int(s, &mut i) {
        let Some(y) = get_int(s, &mut i) else { break };
        p.push(Point64::new(x, y));
    }
    (!p.is_empty()).then_some(p)
}

/// All cases of an upstream data file, in order. Clip type and fill rule carry over from the
/// previous case when a case omits them, as `LoadTestNum`'s by-reference outputs do.
pub fn read_upstream_tests(file: &Path) -> Vec<UpstreamCase> {
    let text = std::fs::read_to_string(file).unwrap_or_else(|e| panic!("{}: {e}", file.display()));
    let lines: Vec<&str> = text.lines().collect();
    let mut cases = Vec::new();
    let (mut ct, mut fr) = (ClipType::Intersection, FillRule::EvenOdd);
    let mut i = 0;
    while i < lines.len() {
        if !lines[i].contains("CAPTION:") {
            i += 1;
            continue;
        }
        let mut case = UpstreamCase {
            caption: lines[i].trim().to_string(),
            clip_type: ct,
            fill_rule: fr,
            area: 0,
            count: 0,
            subjects: Vec::new(),
            subjects_open: Vec::new(),
            clips: Vec::new(),
        };
        i += 1;
        while i < lines.len() && !lines[i].contains("CAPTION:") {
            let line = lines[i];
            i += 1;
            let read_paths = |target: &mut Paths64, i: &mut usize| {
                while *i < lines.len() {
                    match get_path(lines[*i]) {
                        Some(p) => {
                            target.push(p);
                            *i += 1;
                        }
                        None => break,
                    }
                }
            };
            if line.contains("INTERSECTION") {
                case.clip_type = ClipType::Intersection;
            } else if line.contains("UNION") {
                case.clip_type = ClipType::Union;
            } else if line.contains("DIFFERENCE") {
                case.clip_type = ClipType::Difference;
            } else if line.contains("XOR") {
                case.clip_type = ClipType::Xor;
            } else if line.contains("EVENODD") {
                case.fill_rule = FillRule::EvenOdd;
            } else if line.contains("NONZERO") {
                case.fill_rule = FillRule::NonZero;
            } else if line.contains("POSITIVE") {
                case.fill_rule = FillRule::Positive;
            } else if line.contains("NEGATIVE") {
                case.fill_rule = FillRule::Negative;
            } else if line.contains("SOL_AREA") {
                let mut j = 10.min(line.len());
                case.area = get_int(line.as_bytes(), &mut j).unwrap_or(0);
            } else if line.contains("SOL_COUNT") {
                let mut j = 11.min(line.len());
                case.count = get_int(line.as_bytes(), &mut j).unwrap_or(0);
            } else if line.contains("SUBJECTS_OPEN") {
                read_paths(&mut case.subjects_open, &mut i);
            } else if line.contains("SUBJECTS") {
                read_paths(&mut case.subjects, &mut i);
            } else if line.contains("CLIPS") {
                read_paths(&mut case.clips, &mut i);
            }
        }
        ct = case.clip_type;
        fr = case.fill_rule;
        cases.push(case);
    }
    cases
}

/// `(path, parent, is_hole)` per node in DFS pre-order, parent as an index into this list
/// (-1 at the top level) — the reference's `RefTree` layout.
pub fn flatten_tree(tree: &clipper2_rs::PolyTree64) -> Vec<(RefPath, i32, bool)> {
    let order = tree.dfs();
    let mut pos = vec![0i32; tree.len()];
    for (k, &id) in order.iter().enumerate() {
        pos[id] = k as i32;
    }
    order
        .iter()
        .map(|&id| {
            let n = tree.node(id);
            (
                n.polygon.iter().map(|p| (p.x, p.y)).collect(),
                n.parent.map_or(-1, |p| pos[p]),
                tree.is_hole(id),
            )
        })
        .collect()
}

pub fn ref_tree_nodes(t: &clipper2_ref::RefTree) -> Vec<(RefPath, i32, bool)> {
    t.nodes
        .iter()
        .map(|n| (n.path.clone(), n.parent, n.is_hole))
        .collect()
}
