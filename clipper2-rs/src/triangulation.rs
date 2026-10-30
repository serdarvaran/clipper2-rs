//! Constrained (optionally Delaunay) triangulation of non-intersecting polygons
//! (`clipper.triangulation.cpp`, marked BETA upstream).

use std::fmt;

use crate::core::{
    Path64, Paths64, PathsD, Point64, cross_product_sign_t, distance_sqr, scale_paths, sqr,
};

type Idx = u32;
const NONE: Idx = u32::MAX;

/// Why [`triangulate`] produced no triangles.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum TriangulateError {
    /// Internal failure (upstream returns `fail` or throws).
    Fail,
    /// Fewer than three usable vertices.
    NoPolygons,
    /// The paths intersect each other or themselves.
    PathsIntersect,
    /// A floating-point coordinate is out of range after scaling.
    CoordinateOutOfRange,
    /// A path's outline walk never returns to its start (typically a self-intersecting path).
    /// Upstream loops forever, allocating without bound, on such input.
    Degenerate,
}

impl fmt::Display for TriangulateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Fail => "triangulation failed",
            Self::NoPolygons => "no polygons to triangulate",
            Self::PathsIntersect => "paths intersect",
            Self::CoordinateOutOfRange => "coordinate out of range",
            Self::Degenerate => "path outline walk does not terminate",
        })
    }
}

impl std::error::Error for TriangulateError {}

/// Upstream's `throw "oops"`: an inconsistent internal state.
#[derive(Debug)]
struct Oops;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EdgeKind {
    Loose,
    Ascend,
    Descend,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum IntersectKind {
    None,
    Collinear,
    Intersect,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EdgeContains {
    Neither,
    Left,
    Right,
}

struct Vertex2 {
    pt: Point64,
    edges: Vec<Idx>,
    inner_lm: bool,
}

#[derive(Clone, Copy)]
struct Edge {
    v_l: Idx,
    v_r: Idx,
    v_b: Idx,
    v_t: Idx,
    kind: EdgeKind,
    tri_a: Idx,
    tri_b: Idx,
    is_active: bool,
    next_e: Idx,
    prev_e: Idx,
}

#[inline]
fn left_turning(p1: Point64, p2: Point64, p3: Point64) -> bool {
    cross_product_sign_t(p1, p2, p3) < 0
}

#[inline]
fn right_turning(p1: Point64, p2: Point64, p3: Point64) -> bool {
    cross_product_sign_t(p1, p2, p3) > 0
}

fn get_angle(a: Point64, b: Point64, c: Point64) -> f64 {
    let abx = b.x.wrapping_sub(a.x) as f64;
    let aby = b.y.wrapping_sub(a.y) as f64;
    let bcx = b.x.wrapping_sub(c.x) as f64;
    let bcy = b.y.wrapping_sub(c.y) as f64;
    let dp = abx * bcx + aby * bcy;
    let cp = abx * bcy - aby * bcx;
    cp.atan2(dp)
}

fn in_circle_test(pt_a: Point64, pt_b: Point64, pt_c: Point64, pt_d: Point64) -> f64 {
    let m00 = pt_a.x.wrapping_sub(pt_d.x) as f64;
    let m01 = pt_a.y.wrapping_sub(pt_d.y) as f64;
    let m02 = sqr(m00) + sqr(m01);
    let m10 = pt_b.x.wrapping_sub(pt_d.x) as f64;
    let m11 = pt_b.y.wrapping_sub(pt_d.y) as f64;
    let m12 = sqr(m10) + sqr(m11);
    let m20 = pt_c.x.wrapping_sub(pt_d.x) as f64;
    let m21 = pt_c.y.wrapping_sub(pt_d.y) as f64;
    let m22 = sqr(m20) + sqr(m21);
    m00 * (m11 * m22 - m21 * m12) - m10 * (m01 * m22 - m21 * m02) + m20 * (m01 * m12 - m11 * m02)
}

fn shortest_dist_from_segment(pt: Point64, seg1: Point64, seg2: Point64) -> f64 {
    let dx = seg2.x.wrapping_sub(seg1.x) as f64;
    let dy = seg2.y.wrapping_sub(seg1.y) as f64;
    let ax = pt.x.wrapping_sub(seg1.x) as f64;
    let ay = pt.y.wrapping_sub(seg1.y) as f64;
    let q_num = ax * dx + ay * dy;
    if q_num < 0.0 {
        distance_sqr(pt, seg1)
    } else if q_num > sqr(dx) + sqr(dy) {
        distance_sqr(pt, seg2)
    } else {
        sqr(ax * dy - dx * ay) / (dx * dx + dy * dy)
    }
}

fn segs_intersect(s1a: Point64, s1b: Point64, s2a: Point64, s2b: Point64) -> IntersectKind {
    let dy1 = s1b.y.wrapping_sub(s1a.y) as f64;
    let dx1 = s1b.x.wrapping_sub(s1a.x) as f64;
    let dy2 = s2b.y.wrapping_sub(s2a.y) as f64;
    let dx2 = s2b.x.wrapping_sub(s2a.x) as f64;
    let cp = dy1 * dx2 - dy2 * dx1;
    if cp == 0.0 {
        return IntersectKind::Collinear;
    }
    let ax = s1a.x.wrapping_sub(s2a.x) as f64;
    let ay = s1a.y.wrapping_sub(s2a.y) as f64;
    let mut t = ax * dy2 - ay * dx2;
    if t == 0.0 {
        return IntersectKind::None;
    }
    if t > 0.0 {
        if cp < 0.0 || t >= cp {
            return IntersectKind::None;
        }
    } else if cp > 0.0 || t <= cp {
        return IntersectKind::None;
    }
    t = ax * dy1 - ay * dx1;
    if t == 0.0 {
        return IntersectKind::None;
    }
    if t > 0.0 {
        if cp > 0.0 && t < cp {
            return IntersectKind::Intersect;
        }
    } else if cp < 0.0 && t > cp {
        return IntersectKind::Intersect;
    }
    IntersectKind::None
}

fn find_loc_min_idx(path: &Path64, len: usize, idx: &mut usize) -> bool {
    if len < 3 {
        return false;
    }
    let i0 = *idx;
    let mut n = (*idx + 1) % len;
    while path[n].y <= path[*idx].y {
        *idx = n;
        n = (n + 1) % len;
        if *idx == i0 {
            return false; // completely horizontal
        }
    }
    while path[n].y >= path[*idx].y {
        *idx = n;
        n = (n + 1) % len;
    }
    true
}

#[inline]
fn prev_i(idx: usize, len: usize) -> usize {
    if idx == 0 { len - 1 } else { idx - 1 }
}

#[inline]
fn next_i(idx: usize, len: usize) -> usize {
    (idx + 1) % len
}

struct Delaunay {
    verts: Vec<Vertex2>,
    /// `allVertices`: creation order, later sorted.
    all_vertices: Vec<Idx>,
    edges: Vec<Edge>,
    /// `allEdges`: creation order, sorted once by left x.
    all_edges: Vec<Idx>,
    tris: Vec<[Idx; 3]>,
    pending_delaunay: Vec<Idx>,
    horz_edges: Vec<Idx>,
    loc_mins: Vec<Idx>,
    use_delaunay: bool,
    lowermost: Idx,
    first_active: Idx,
}

impl Delaunay {
    fn new(use_delaunay: bool) -> Self {
        Delaunay {
            verts: Vec::new(),
            all_vertices: Vec::new(),
            edges: Vec::new(),
            all_edges: Vec::new(),
            tris: Vec::new(),
            pending_delaunay: Vec::new(),
            horz_edges: Vec::new(),
            loc_mins: Vec::new(),
            use_delaunay,
            lowermost: NONE,
            first_active: NONE,
        }
    }

    #[inline]
    fn e(&self, i: Idx) -> &Edge {
        &self.edges[i as usize]
    }

    #[inline]
    fn em(&mut self, i: Idx) -> &mut Edge {
        &mut self.edges[i as usize]
    }

    #[inline]
    fn v(&self, i: Idx) -> &Vertex2 {
        &self.verts[i as usize]
    }

    #[inline]
    fn vm(&mut self, i: Idx) -> &mut Vertex2 {
        &mut self.verts[i as usize]
    }

    #[inline]
    fn pt(&self, v: Idx) -> Point64 {
        self.verts[v as usize].pt
    }

    fn new_vertex(&mut self, pt: Point64) -> Idx {
        let i = self.verts.len() as Idx;
        self.verts.push(Vertex2 {
            pt,
            edges: Vec::with_capacity(2),
            inner_lm: false,
        });
        self.all_vertices.push(i);
        i
    }

    #[inline]
    fn is_horizontal(&self, e: Idx) -> bool {
        self.pt(self.e(e).v_b).y == self.pt(self.e(e).v_t).y
    }

    fn edge_completed(&self, e: Idx) -> bool {
        let ed = self.e(e);
        if ed.tri_a == NONE {
            return false;
        }
        if ed.tri_b != NONE {
            return true;
        }
        ed.kind != EdgeKind::Loose
    }

    fn edge_contains(&self, e: Idx, v: Idx) -> EdgeContains {
        if self.e(e).v_l == v {
            EdgeContains::Left
        } else if self.e(e).v_r == v {
            EdgeContains::Right
        } else {
            EdgeContains::Neither
        }
    }

    fn get_loc_min_angle(&self, v: Idx) -> f64 {
        let edges = &self.v(v).edges;
        let (asc, des) = if self.e(edges[0]).kind == EdgeKind::Ascend {
            (0, 1)
        } else {
            (1, 0)
        };
        get_angle(
            self.pt(self.e(edges[des]).v_t),
            self.pt(v),
            self.pt(self.e(edges[asc]).v_t),
        )
    }

    fn remove_edge_from_vertex(&mut self, vert: Idx, edge: Idx) -> Result<(), Oops> {
        let edges = &mut self.vm(vert).edges;
        let pos = edges.iter().position(|&e| e == edge).ok_or(Oops)?;
        edges.remove(pos);
        Ok(())
    }

    fn find_linking_edge(&self, vert1: Idx, vert2: Idx, prefer_ascending: bool) -> Idx {
        let mut res = NONE;
        for &e in &self.v(vert1).edges {
            let ed = self.e(e);
            if ed.v_l == vert2 || ed.v_r == vert2 {
                if ed.kind == EdgeKind::Loose || ((ed.kind == EdgeKind::Ascend) == prefer_ascending)
                {
                    return e;
                }
                res = e;
            }
        }
        res
    }

    fn path_from_triangle(&self, tri: [Idx; 3]) -> Path64 {
        let e0 = self.e(tri[0]);
        let mut res = vec![self.pt(e0.v_l), self.pt(e0.v_r)];
        let e1 = self.e(tri[1]);
        if self.pt(e1.v_l) == res[0] || self.pt(e1.v_l) == res[1] {
            res.push(self.pt(e1.v_r));
        } else {
            res.push(self.pt(e1.v_l));
        }
        res
    }

    #[allow(clippy::needless_range_loop)] // indexes two parallel arrays, as upstream does
    fn force_legal(&mut self, edge: Idx) -> Result<(), Oops> {
        let ed = *self.e(edge);
        if ed.tri_a == NONE || ed.tri_b == NONE {
            return Ok(());
        }
        let mut vert_a = NONE;
        let mut vert_b = NONE;
        let mut edges_a = [NONE; 3];
        let mut edges_b = [NONE; 3];
        for i in 0..3 {
            let te = self.tris[ed.tri_a as usize][i];
            if te == edge {
                continue;
            }
            match self.edge_contains(te, ed.v_l) {
                EdgeContains::Left => {
                    edges_a[1] = te;
                    vert_a = self.e(te).v_r;
                }
                EdgeContains::Right => {
                    edges_a[1] = te;
                    vert_a = self.e(te).v_l;
                }
                EdgeContains::Neither => edges_b[1] = te,
            }
        }
        for i in 0..3 {
            let te = self.tris[ed.tri_b as usize][i];
            if te == edge {
                continue;
            }
            match self.edge_contains(te, ed.v_l) {
                EdgeContains::Left => {
                    edges_a[2] = te;
                    vert_b = self.e(te).v_r;
                }
                EdgeContains::Right => {
                    edges_a[2] = te;
                    vert_b = self.e(te).v_l;
                }
                EdgeContains::Neither => edges_b[2] = te,
            }
        }
        if vert_a == NONE || vert_b == NONE {
            return Err(Oops);
        }
        let (pa, pl, pr, pb) = (
            self.pt(vert_a),
            self.pt(ed.v_l),
            self.pt(ed.v_r),
            self.pt(vert_b),
        );
        if cross_product_sign_t(pa, pl, pr) == 0 {
            return Ok(());
        }
        let ict = in_circle_test(pa, pl, pr, pb);
        if ict == 0.0 || right_turning(pa, pl, pr) == (ict < 0.0) {
            return Ok(()); // on or outside the circle
        }
        self.em(edge).v_l = vert_a;
        self.em(edge).v_r = vert_b;
        let (tri_a, tri_b) = (ed.tri_a, ed.tri_b);
        self.tris[tri_a as usize][0] = edge;
        for i in 1..3 {
            let ea = edges_a[i];
            self.tris[tri_a as usize][i] = ea;
            if ea == NONE {
                return Err(Oops);
            }
            if self.e(ea).kind == EdgeKind::Loose {
                self.pending_delaunay.push(ea);
            }
            if self.e(ea).tri_a == tri_a || self.e(ea).tri_b == tri_a {
                continue;
            }
            if self.e(ea).tri_a == tri_b {
                self.em(ea).tri_a = tri_a;
            } else if self.e(ea).tri_b == tri_b {
                self.em(ea).tri_b = tri_a;
            } else {
                return Err(Oops);
            }
        }
        self.tris[tri_b as usize][0] = edge;
        for i in 1..3 {
            let eb = edges_b[i];
            self.tris[tri_b as usize][i] = eb;
            if eb == NONE {
                return Err(Oops);
            }
            if self.e(eb).kind == EdgeKind::Loose {
                self.pending_delaunay.push(eb);
            }
            if self.e(eb).tri_a == tri_b || self.e(eb).tri_b == tri_b {
                continue;
            }
            if self.e(eb).tri_a == tri_a {
                self.em(eb).tri_a = tri_b;
            } else if self.e(eb).tri_b == tri_a {
                self.em(eb).tri_b = tri_b;
            } else {
                return Err(Oops);
            }
        }
        Ok(())
    }

    fn create_edge(&mut self, v1: Idx, v2: Idx, k: EdgeKind) -> Idx {
        let (p1, p2) = (self.pt(v1), self.pt(v2));
        let (v_b, v_t) = if p1.y == p2.y {
            (v1, v2)
        } else if p1.y < p2.y {
            (v2, v1)
        } else {
            (v1, v2)
        };
        let (v_l, v_r) = if p1.x <= p2.x { (v1, v2) } else { (v2, v1) };
        let res = self.edges.len() as Idx;
        self.edges.push(Edge {
            v_l,
            v_r,
            v_b,
            v_t,
            kind: k,
            tri_a: NONE,
            tri_b: NONE,
            is_active: false,
            next_e: NONE,
            prev_e: NONE,
        });
        self.all_edges.push(res);
        self.vm(v1).edges.push(res);
        self.vm(v2).edges.push(res);
        if k == EdgeKind::Loose {
            self.pending_delaunay.push(res);
            self.add_edge_to_actives(res);
        }
        res
    }

    fn create_triangle(&mut self, e1: Idx, e2: Idx, e3: Idx) -> Result<Idx, Oops> {
        let res = self.tris.len() as Idx;
        self.tris.push([e1, e2, e3]);
        for e in [e1, e2, e3] {
            if self.e(e).tri_a != NONE {
                self.em(e).tri_b = res;
                self.remove_edge_from_actives(e)?;
            } else {
                self.em(e).tri_a = res;
                if self.e(e).kind != EdgeKind::Loose {
                    self.remove_edge_from_actives(e)?;
                }
            }
        }
        Ok(res)
    }

    fn remove_intersection(&mut self, e1: Idx, e2: Idx) -> Result<bool, Oops> {
        let (a, b) = (*self.e(e1), *self.e(e2));
        let mut v = a.v_l;
        let mut tmp_e = e2;
        let mut d = shortest_dist_from_segment(self.pt(a.v_l), self.pt(b.v_l), self.pt(b.v_r));
        let mut d2 = shortest_dist_from_segment(self.pt(a.v_r), self.pt(b.v_l), self.pt(b.v_r));
        if d2 < d {
            d = d2;
            v = a.v_r;
        }
        d2 = shortest_dist_from_segment(self.pt(b.v_l), self.pt(a.v_l), self.pt(a.v_r));
        if d2 < d {
            d = d2;
            tmp_e = e1;
            v = b.v_l;
        }
        d2 = shortest_dist_from_segment(self.pt(b.v_r), self.pt(a.v_l), self.pt(a.v_r));
        if d2 < d {
            d = d2;
            tmp_e = e1;
            v = b.v_r;
        }
        if d > 1.0 {
            return Ok(false); // not just a rounding intersection
        }
        let v2 = self.e(tmp_e).v_t;
        self.remove_edge_from_vertex(v2, tmp_e)?;
        if self.e(tmp_e).v_l == v2 {
            self.em(tmp_e).v_l = v;
        } else {
            self.em(tmp_e).v_r = v;
        }
        self.em(tmp_e).v_t = v;
        self.vm(v).edges.push(tmp_e);
        self.vm(v).inner_lm = false; // #47
        let vb = self.e(tmp_e).v_b;
        if self.v(vb).inner_lm && self.get_loc_min_angle(vb) <= 0.0 {
            self.vm(vb).inner_lm = false; // #44, 52
        }
        let kind = self.e(tmp_e).kind;
        self.create_edge(v, v2, kind);
        Ok(true)
    }

    fn fixup_edge_intersects(&mut self) -> Result<bool, Oops> {
        let mut i1 = 0;
        while i1 < self.all_edges.len() {
            let e1 = self.all_edges[i1];
            let mut i2 = i1 + 1;
            while i2 < self.all_edges.len() {
                let e2 = self.all_edges[i2];
                let (a, b) = (*self.e(e1), *self.e(e2));
                if self.pt(b.v_l).x >= self.pt(a.v_r).x {
                    break; // every later edge is too far right
                }
                if self.pt(b.v_t).y < self.pt(a.v_b).y
                    && self.pt(b.v_b).y > self.pt(a.v_t).y
                    && segs_intersect(
                        self.pt(b.v_l),
                        self.pt(b.v_r),
                        self.pt(a.v_l),
                        self.pt(a.v_r),
                    ) == IntersectKind::Intersect
                    && !self.remove_intersection(e2, e1)?
                {
                    return Ok(false);
                }
                i2 += 1;
            }
            i1 += 1;
        }
        Ok(true)
    }

    fn split_edge(&mut self, long_e: Idx, short_e: Idx) -> Result<(), Oops> {
        let old_t = self.e(long_e).v_t;
        let new_t = self.e(short_e).v_t;
        self.remove_edge_from_vertex(old_t, long_e)?;
        self.em(long_e).v_t = new_t;
        if self.e(long_e).v_l == old_t {
            self.em(long_e).v_l = new_t;
        } else {
            self.em(long_e).v_r = new_t;
        }
        self.vm(new_t).edges.push(long_e);
        let kind = self.e(long_e).kind;
        self.create_edge(new_t, old_t, kind);
        Ok(())
    }

    fn merge_dup_or_collinear_vertices(&mut self) -> Result<(), Oops> {
        if self.all_vertices.is_empty() {
            return Ok(());
        }
        let mut i1 = 0;
        for i2 in 1..self.all_vertices.len() {
            let (v1, v2) = (self.all_vertices[i1], self.all_vertices[i2]);
            if self.pt(v1) != self.pt(v2) {
                i1 = i2;
                continue;
            }
            if !self.v(v1).inner_lm || !self.v(v2).inner_lm {
                self.vm(v1).inner_lm = false;
            }
            let moved = std::mem::take(&mut self.vm(v2).edges);
            for &e in &moved {
                let ed = self.em(e);
                if ed.v_b == v2 {
                    ed.v_b = v1;
                } else {
                    ed.v_t = v1;
                }
                if ed.v_l == v2 {
                    ed.v_l = v1;
                } else {
                    ed.v_r = v1;
                }
            }
            self.vm(v1).edges.extend(moved);
            let mut ie = 0;
            while ie < self.v(v1).edges.len() {
                let e1 = self.v(v1).edges[ie];
                if self.is_horizontal(e1) || self.e(e1).v_b != v1 {
                    ie += 1;
                    continue;
                }
                let mut ie2 = ie + 1;
                while ie2 < self.v(v1).edges.len() {
                    let e2 = self.v(v1).edges[ie2];
                    let (a, b) = (*self.e(e1), *self.e(e2));
                    if b.v_b != v1
                        || self.pt(a.v_t).y == self.pt(b.v_t).y
                        || cross_product_sign_t(self.pt(a.v_t), self.pt(v1), self.pt(b.v_t)) != 0
                    {
                        ie2 += 1;
                        continue;
                    }
                    if self.pt(a.v_t).y < self.pt(b.v_t).y {
                        self.split_edge(e1, e2)?;
                    } else {
                        self.split_edge(e2, e1)?;
                    }
                    break; // only two edges can be collinear
                }
                ie += 1;
            }
        }
        Ok(())
    }

    fn create_inner_loc_min_loose_edge(&mut self, v_above: Idx) -> Idx {
        if self.first_active == NONE {
            return NONE;
        }
        let pa = self.pt(v_above);
        let (x_above, y_above) = (pa.x, pa.y);
        let mut e = self.first_active;
        let mut e_below = NONE;
        let mut best_d = -1.0;
        while e != NONE {
            let ed = *self.e(e);
            let (pl, pr) = (self.pt(ed.v_l), self.pt(ed.v_r));
            if pl.x <= x_above
                && pr.x >= x_above
                && self.pt(ed.v_b).y >= y_above
                && ed.v_b != v_above
                && ed.v_t != v_above
                && !left_turning(pl, pa, pr)
            {
                let d = shortest_dist_from_segment(pa, pl, pr);
                if e_below == NONE || d < best_d {
                    e_below = e;
                    best_d = d;
                }
            }
            e = ed.next_e;
        }
        if e_below == NONE {
            return NONE;
        }
        let eb = *self.e(e_below);
        let mut v_best = if self.pt(eb.v_t).y <= y_above {
            eb.v_b
        } else {
            eb.v_t
        };
        let mut x_best = self.pt(v_best).x;
        let mut y_best = self.pt(v_best).y;
        e = self.first_active;
        if x_best < x_above {
            while e != NONE {
                let ed = *self.e(e);
                if self.pt(ed.v_r).x > x_best
                    && self.pt(ed.v_l).x < x_above
                    && self.pt(ed.v_b).y > y_above
                    && self.pt(ed.v_t).y < y_best
                    && segs_intersect(self.pt(ed.v_b), self.pt(ed.v_t), self.pt(v_best), pa)
                        == IntersectKind::Intersect
                {
                    v_best = if self.pt(ed.v_t).y > y_above {
                        ed.v_t
                    } else {
                        ed.v_b
                    };
                    x_best = self.pt(v_best).x;
                    y_best = self.pt(v_best).y;
                }
                e = ed.next_e;
            }
        } else {
            while e != NONE {
                let ed = *self.e(e);
                if self.pt(ed.v_r).x < x_best
                    && self.pt(ed.v_l).x > x_above
                    && self.pt(ed.v_b).y > y_above
                    && self.pt(ed.v_t).y < y_best
                    && segs_intersect(self.pt(ed.v_b), self.pt(ed.v_t), self.pt(v_best), pa)
                        == IntersectKind::Intersect
                {
                    v_best = if self.pt(ed.v_t).y > y_above {
                        ed.v_t
                    } else {
                        ed.v_b
                    };
                    x_best = self.pt(v_best).x;
                    y_best = self.pt(v_best).y;
                }
                e = ed.next_e;
            }
        }
        self.create_edge(v_best, v_above, EdgeKind::Loose)
    }

    fn horizontal_between(&self, v1: Idx, v2: Idx) -> Idx {
        let (p1, p2) = (self.pt(v1), self.pt(v2));
        let y = p1.y;
        let (l, r) = if p1.x > p2.x {
            (p2.x, p1.x)
        } else {
            (p1.x, p2.x)
        };
        let mut res = self.first_active;
        while res != NONE {
            let ed = self.e(res);
            let (pl, pr) = (self.pt(ed.v_l), self.pt(ed.v_r));
            if pl.y == y && pr.y == y && pl.x >= l && pr.x <= r && (pl.x != l || pl.x != r) {
                break;
            }
            res = ed.next_e;
        }
        res
    }

    /// `DoTriangulateLeft`; upstream's tail recursion is a loop here.
    fn do_triangulate_left(
        &mut self,
        mut edge: Idx,
        mut pivot: Idx,
        min_y: i64,
    ) -> Result<(), Oops> {
        loop {
            let mut v_alt = NONE;
            let mut e_alt = NONE;
            let v = if self.e(edge).v_b == pivot {
                self.e(edge).v_t
            } else {
                self.e(edge).v_b
            };
            for k in 0..self.v(pivot).edges.len() {
                let e = self.v(pivot).edges[k];
                if e == edge || !self.e(e).is_active {
                    continue;
                }
                let vx = if self.e(e).v_t == pivot {
                    self.e(e).v_b
                } else {
                    self.e(e).v_t
                };
                if vx == v {
                    continue;
                }
                let (pv, pp, px) = (self.pt(v), self.pt(pivot), self.pt(vx));
                let cps = cross_product_sign_t(pv, pp, px);
                if cps == 0 {
                    if (pv.x > pp.x) == (pp.x > px.x) {
                        continue;
                    }
                } else if cps > 0 || (v_alt != NONE && !left_turning(px, pp, self.pt(v_alt))) {
                    continue;
                }
                v_alt = vx;
                e_alt = e;
            }
            if v_alt == NONE || self.pt(v_alt).y < min_y {
                return Ok(());
            }
            let (pa, pp) = (self.pt(v_alt), self.pt(pivot));
            if pa.y < pp.y {
                if self.e(e_alt).kind == EdgeKind::Ascend {
                    return Ok(());
                }
            } else if pa.y > pp.y && self.e(e_alt).kind == EdgeKind::Descend {
                return Ok(());
            }
            let mut e_x = self.find_linking_edge(v_alt, v, pa.y < self.pt(v).y);
            if e_x == NONE {
                if pa.y == self.pt(v).y
                    && self.pt(v).y == min_y
                    && self.horizontal_between(v_alt, v) != NONE
                {
                    return Ok(());
                }
                e_x = self.create_edge(v_alt, v, EdgeKind::Loose);
            }
            self.create_triangle(edge, e_alt, e_x)?;
            if self.edge_completed(e_x) {
                return Ok(());
            }
            edge = e_x;
            pivot = v_alt;
        }
    }

    /// `DoTriangulateRight`; upstream's tail recursion is a loop here.
    fn do_triangulate_right(
        &mut self,
        mut edge: Idx,
        mut pivot: Idx,
        min_y: i64,
    ) -> Result<(), Oops> {
        loop {
            let mut v_alt = NONE;
            let mut e_alt = NONE;
            let v = if self.e(edge).v_b == pivot {
                self.e(edge).v_t
            } else {
                self.e(edge).v_b
            };
            for k in 0..self.v(pivot).edges.len() {
                let e = self.v(pivot).edges[k];
                if e == edge || !self.e(e).is_active {
                    continue;
                }
                let vx = if self.e(e).v_t == pivot {
                    self.e(e).v_b
                } else {
                    self.e(e).v_t
                };
                if vx == v {
                    continue;
                }
                let (pv, pp, px) = (self.pt(v), self.pt(pivot), self.pt(vx));
                let cps = cross_product_sign_t(pv, pp, px);
                if cps == 0 {
                    if (pv.x > pp.x) == (pp.x > px.x) {
                        continue;
                    }
                } else if cps < 0 || (v_alt != NONE && !right_turning(px, pp, self.pt(v_alt))) {
                    continue;
                }
                v_alt = vx;
                e_alt = e;
            }
            if v_alt == NONE || self.pt(v_alt).y < min_y {
                return Ok(());
            }
            let (pa, pp) = (self.pt(v_alt), self.pt(pivot));
            if pa.y < pp.y {
                if self.e(e_alt).kind == EdgeKind::Descend {
                    return Ok(());
                }
            } else if pa.y > pp.y && self.e(e_alt).kind == EdgeKind::Ascend {
                return Ok(());
            }
            let mut e_x = self.find_linking_edge(v_alt, v, pa.y > self.pt(v).y);
            if e_x == NONE {
                if pa.y == self.pt(v).y
                    && self.pt(v).y == min_y
                    && self.horizontal_between(v_alt, v) != NONE
                {
                    return Ok(());
                }
                e_x = self.create_edge(v_alt, v, EdgeKind::Loose);
            }
            self.create_triangle(edge, e_x, e_alt)?;
            if self.edge_completed(e_x) {
                return Ok(());
            }
            edge = e_x;
            pivot = v_alt;
        }
    }

    fn add_edge_to_actives(&mut self, edge: Idx) {
        if self.e(edge).is_active {
            return;
        }
        let first = self.first_active;
        let ed = self.em(edge);
        ed.prev_e = NONE;
        ed.next_e = first;
        ed.is_active = true;
        if first != NONE {
            self.em(first).prev_e = edge;
        }
        self.first_active = edge;
    }

    fn remove_edge_from_actives(&mut self, edge: Idx) -> Result<(), Oops> {
        let (vb, vt) = (self.e(edge).v_b, self.e(edge).v_t);
        self.remove_edge_from_vertex(vb, edge)?;
        self.remove_edge_from_vertex(vt, edge)?;
        let (prev, next) = (self.e(edge).prev_e, self.e(edge).next_e);
        if next != NONE {
            self.em(next).prev_e = prev;
        }
        if prev != NONE {
            self.em(prev).next_e = next;
        }
        self.em(edge).is_active = false;
        if self.first_active == edge {
            self.first_active = next;
        }
        Ok(())
    }

    fn add_path(&mut self, path: &Path64) -> Result<(), TriangulateError> {
        let len = path.len();
        let mut i0 = 0;
        if !find_loc_min_idx(path, len, &mut i0) {
            return Ok(());
        }
        let mut i_prev = prev_i(i0, len);
        while path[i_prev] == path[i0] {
            i_prev = prev_i(i_prev, len);
        }
        let mut i_next = next_i(i0, len);
        let mut i = i0;
        while cross_product_sign_t(path[i_prev], path[i], path[i_next]) == 0 {
            find_loc_min_idx(path, len, &mut i);
            if i == i0 {
                return Ok(()); // entirely collinear
            }
            i_prev = prev_i(i, len);
            while path[i_prev] == path[i] {
                i_prev = prev_i(i_prev, len);
            }
            i_next = next_i(i, len);
        }
        let vert_cnt = self.all_vertices.len();
        let v0 = self.new_vertex(path[i]);
        if left_turning(path[i_prev], path[i], path[i_next]) {
            self.vm(v0).inner_lm = true;
        }
        let mut v_prev = v0;
        // A valid walk is one lap: `i` advances `len` times, from i0 back to i0. Upstream never
        // checks i0 inside the collinear skips, so on some self-intersecting paths it overshoots
        // and walks (adding vertices) forever.
        let mut steps = 0;
        macro_rules! advance {
            () => {
                i = i_next;
                steps += 1;
                if steps > len {
                    return Err(TriangulateError::Degenerate);
                }
            };
        }
        advance!();
        loop {
            self.loc_mins.push(v_prev);
            let pp = self.pt(v_prev);
            if self.lowermost == NONE
                || pp.y > self.pt(self.lowermost).y
                || (pp.y == self.pt(self.lowermost).y && pp.x < self.pt(self.lowermost).x)
            {
                self.lowermost = v_prev;
            }
            i_next = next_i(i, len);
            if cross_product_sign_t(self.pt(v_prev), path[i], path[i_next]) == 0 {
                advance!();
                continue;
            }
            while path[i].y <= self.pt(v_prev).y {
                let v = self.new_vertex(path[i]);
                self.create_edge(v_prev, v, EdgeKind::Ascend);
                v_prev = v;
                advance!();
                i_next = next_i(i, len);
                while cross_product_sign_t(self.pt(v_prev), path[i], path[i_next]) == 0 {
                    advance!();
                    i_next = next_i(i, len);
                }
            }
            let mut v_prev_prev = v_prev;
            while i != i0 && path[i].y >= self.pt(v_prev).y {
                let v = self.new_vertex(path[i]);
                self.create_edge(v, v_prev, EdgeKind::Descend);
                v_prev_prev = v_prev;
                v_prev = v;
                advance!();
                i_next = next_i(i, len);
                while cross_product_sign_t(self.pt(v_prev), path[i], path[i_next]) == 0 {
                    advance!();
                    i_next = next_i(i, len);
                }
            }
            if i == i0 {
                break;
            }
            if left_turning(self.pt(v_prev_prev), self.pt(v_prev), path[i]) {
                self.vm(v_prev).inner_lm = true;
            }
        }
        self.create_edge(v0, v_prev, EdgeKind::Descend);
        let added = self.all_vertices.len() - vert_cnt;
        let i = vert_cnt;
        let tiny = |s: &Self, a: usize, b: usize| {
            distance_sqr(s.pt(s.all_vertices[a]), s.pt(s.all_vertices[b])) <= 1.0
        };
        if added < 3
            || (added == 3
                && (tiny(self, i, i + 1) || tiny(self, i + 1, i + 2) || tiny(self, i + 2, i)))
        {
            for j in vert_cnt..self.all_vertices.len() {
                let v = self.all_vertices[j];
                self.vm(v).edges.clear(); // flag to ignore
            }
        }
        Ok(())
    }

    fn add_paths(&mut self, paths: &Paths64) -> Result<bool, TriangulateError> {
        let total: usize = paths.iter().map(|p| p.len()).sum();
        if total == 0 {
            return Ok(false);
        }
        for path in paths {
            self.add_path(path)?;
        }
        Ok(self.all_vertices.len() > 2)
    }

    fn execute(&mut self, paths: &Paths64) -> Result<Paths64, TriangulateError> {
        if !self.add_paths(paths)? {
            return Err(TriangulateError::NoPolygons);
        }
        self.run().map_err(|_| TriangulateError::Fail)?
    }

    fn run(&mut self) -> Result<Result<Paths64, TriangulateError>, Oops> {
        if self.v(self.lowermost).inner_lm {
            while let Some(lm) = self.loc_mins.pop() {
                let v = self.vm(lm);
                v.inner_lm = !v.inner_lm;
            }
            for e in &mut self.edges {
                e.kind = if e.kind == EdgeKind::Ascend {
                    EdgeKind::Descend
                } else {
                    EdgeKind::Ascend
                };
            }
        } else {
            self.loc_mins.clear();
        }

        let edges = &self.edges;
        let verts = &self.verts;
        self.all_edges.sort_by(|&a, &b| {
            let (xa, xb) = (
                verts[edges[a as usize].v_l as usize].pt.x,
                verts[edges[b as usize].v_l as usize].pt.x,
            );
            xa.cmp(&xb)
        });
        if !self.fixup_edge_intersects()? {
            return Ok(Err(TriangulateError::PathsIntersect));
        }

        let verts = &self.verts;
        self.all_vertices.sort_by(|&a, &b| {
            let (pa, pb) = (verts[a as usize].pt, verts[b as usize].pt);
            if pa.y == pb.y {
                pa.x.cmp(&pb.x)
            } else {
                pb.y.cmp(&pa.y)
            }
        });
        self.merge_dup_or_collinear_vertices()?;

        let mut curr_y = self.pt(self.all_vertices[0]).y;
        for vi in 0..self.all_vertices.len() {
            let v = self.all_vertices[vi];
            if self.v(v).edges.is_empty() {
                continue;
            }
            if self.pt(v).y != curr_y {
                while let Some(lm) = self.loc_mins.pop() {
                    let e = self.create_inner_loc_min_loose_edge(lm);
                    if e == NONE {
                        return Ok(Err(TriangulateError::Fail));
                    }
                    let vb = self.e(e).v_b;
                    if self.is_horizontal(e) {
                        if self.e(e).v_l == vb {
                            self.do_triangulate_left(e, vb, curr_y)?;
                        } else {
                            self.do_triangulate_right(e, vb, curr_y)?;
                        }
                    } else {
                        self.do_triangulate_left(e, vb, curr_y)?;
                        if !self.edge_completed(e) {
                            self.do_triangulate_right(e, vb, curr_y)?;
                        }
                    }
                    let (a, b) = (self.v(lm).edges[0], self.v(lm).edges[1]);
                    self.add_edge_to_actives(a);
                    self.add_edge_to_actives(b);
                }
                while let Some(e) = self.horz_edges.pop() {
                    if self.edge_completed(e) {
                        continue;
                    }
                    let vb = self.e(e).v_b;
                    if vb == self.e(e).v_l {
                        // #45
                        if self.e(e).kind == EdgeKind::Ascend {
                            self.do_triangulate_left(e, vb, curr_y)?;
                        }
                    } else if self.e(e).kind == EdgeKind::Descend {
                        self.do_triangulate_right(e, vb, curr_y)?;
                    }
                }
                curr_y = self.pt(v).y;
            }

            let mut i = self.v(v).edges.len() as isize - 1;
            while i >= 0 {
                if i as usize >= self.v(v).edges.len() {
                    i -= 1;
                    continue;
                }
                let e = self.v(v).edges[i as usize];
                i -= 1;
                if self.edge_completed(e) || self.e(e).kind == EdgeKind::Loose {
                    continue;
                }
                if v == self.e(e).v_b {
                    if self.is_horizontal(e) {
                        self.horz_edges.push(e);
                    }
                    if !self.v(v).inner_lm {
                        self.add_edge_to_actives(e);
                    }
                } else if self.is_horizontal(e) {
                    self.horz_edges.push(e);
                } else {
                    let vb = self.e(e).v_b;
                    let y = self.pt(v).y;
                    if self.e(e).kind == EdgeKind::Ascend {
                        self.do_triangulate_left(e, vb, y)?;
                    } else {
                        self.do_triangulate_right(e, vb, y)?;
                    }
                }
            }
            if self.v(v).inner_lm {
                self.loc_mins.push(v);
            }
        }
        while let Some(e) = self.horz_edges.pop() {
            if !self.edge_completed(e) && self.e(e).v_b == self.e(e).v_l {
                let vb = self.e(e).v_b;
                self.do_triangulate_left(e, vb, curr_y)?;
            }
        }

        if self.use_delaunay {
            while let Some(e) = self.pending_delaunay.pop() {
                self.force_legal(e)?;
            }
        }

        let mut res = Vec::with_capacity(self.tris.len());
        for t in 0..self.tris.len() {
            let mut p = self.path_from_triangle(self.tris[t]);
            let cps = cross_product_sign_t(p[0], p[1], p[2]);
            if cps == 0 {
                continue; // empty triangle
            }
            if cps < 0 {
                p.reverse();
            }
            res.push(p);
        }
        Ok(Ok(res))
    }
}

/// Triangulates non-intersecting polygons (holes allowed). With `use_delaunay` the result
/// is a constrained Delaunay triangulation.
pub fn triangulate(paths: &Paths64, use_delaunay: bool) -> Result<Paths64, TriangulateError> {
    Delaunay::new(use_delaunay).execute(paths)
}

/// Floating-point [`triangulate`]; coordinates are scaled by `10^decimal_places` (clamped to `0..=8`).
pub fn triangulate_d(
    paths: &PathsD,
    decimal_places: i32,
    use_delaunay: bool,
) -> Result<PathsD, TriangulateError> {
    let scale = if decimal_places <= 0 {
        1.0
    } else if decimal_places > 8 {
        10f64.powi(8)
    } else {
        10f64.powi(decimal_places)
    };
    let pp64: Paths64 =
        scale_paths(paths, scale, scale).map_err(|_| TriangulateError::CoordinateOutOfRange)?;
    let sol = triangulate(&pp64, use_delaunay)?;
    scale_paths(&sol, 1.0 / scale, 1.0 / scale).map_err(|_| TriangulateError::CoordinateOutOfRange)
}
