// C ABI over upstream Clipper2 2.0.1 for the clipper2-rs reference tests.
// Compiled twice by build.rs: plain (ref_* symbols) and with USINGZ, where the
// Clipper2Lib namespace is renamed to Clipper2LibZ (ref_z_* symbols).
// Enum codes are Clipper2's own (ClipType Intersection = 1, ...).
#include "clipper2/clipper.h"
#include "clipper2/clipper.minkowski.h"
#include "clipper2/clipper.offset.h"
#include "clipper2/clipper.rectclip.h"
#include "clipper2/clipper.triangulation.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

#ifdef USINGZ
#define FN(name) ref_z_##name
#else
#define FN(name) ref_##name
#endif

using namespace Clipper2Lib;

struct RefOut {
  std::vector<int64_t> xy;    // Point64 coordinates
  std::vector<double> xyd;    // PointD coordinates
  std::vector<int64_t> z;     // z per point (USINGZ only)
  std::vector<uint32_t> lens;
  std::vector<int32_t> parents;
  std::vector<uint8_t> flags; // bit 0 open, bit 1 hole
};

static const uint8_t FLAG_OPEN = 1;
static const uint8_t FLAG_HOLE = 2;

enum : int32_t { OK = 0, EXECUTE_FAILED = 1, EXCEPTION = 2 };

struct In64 { const int64_t* xy; const int64_t* z; const uint32_t* lens; size_t count; };
struct InD { const double* xy; const int64_t* z; const uint32_t* lens; size_t count; };

static Paths64 to_paths(const In64& in) {
  Paths64 paths(in.count);
  size_t k = 0;
  for (size_t i = 0; i < in.count; ++i) {
    paths[i].reserve(in.lens[i]);
    for (uint32_t j = 0; j < in.lens[i]; ++j, ++k) {
#ifdef USINGZ
      paths[i].emplace_back(in.xy[2 * k], in.xy[2 * k + 1], in.z ? in.z[k] : 0);
#else
      paths[i].emplace_back(in.xy[2 * k], in.xy[2 * k + 1]);
#endif
    }
  }
  return paths;
}

static PathsD to_paths(const InD& in) {
  PathsD paths(in.count);
  size_t k = 0;
  for (size_t i = 0; i < in.count; ++i) {
    paths[i].reserve(in.lens[i]);
    for (uint32_t j = 0; j < in.lens[i]; ++j, ++k) {
#ifdef USINGZ
      paths[i].emplace_back(in.xy[2 * k], in.xy[2 * k + 1], in.z ? in.z[k] : 0);
#else
      paths[i].emplace_back(in.xy[2 * k], in.xy[2 * k + 1]);
#endif
    }
  }
  return paths;
}

static void push_point(RefOut* r, const Point64& p) {
  r->xy.push_back(p.x);
  r->xy.push_back(p.y);
#ifdef USINGZ
  r->z.push_back(p.z);
#endif
}

static void push_point(RefOut* r, const PointD& p) {
  r->xyd.push_back(p.x);
  r->xyd.push_back(p.y);
#ifdef USINGZ
  r->z.push_back(p.z);
#endif
}

template <typename P>
static void push_path(RefOut* r, const std::vector<P>& path, int32_t parent, uint8_t flags) {
  r->lens.push_back(static_cast<uint32_t>(path.size()));
  r->parents.push_back(parent);
  r->flags.push_back(flags);
  for (const P& p : path) push_point(r, p);
}

template <typename P>
static void push_paths(RefOut* r, const std::vector<std::vector<P>>& paths, uint8_t flags) {
  for (const auto& path : paths) push_path(r, path, -1, flags);
}

template <typename Node>
static void push_node(RefOut* r, const Node& node, int32_t parent) {
  const int32_t index = static_cast<int32_t>(r->lens.size());
  push_path(r, node.Polygon(), parent, node.IsHole() ? FLAG_HOLE : 0);
  for (size_t i = 0; i < node.Count(); ++i) push_node(r, *node.Child(i), index);
}

template <typename Tree>
static void push_tree(RefOut* r, const Tree& tree) {
  for (size_t i = 0; i < tree.Count(); ++i) push_node(r, *tree.Child(i), -1);
}

#ifdef USINGZ
static void z_rule64(const Point64& e1bot, const Point64& e1top, const Point64& e2bot,
                     const Point64& e2top, Point64& pt) {
  pt.z = e1bot.z + 10 * e1top.z + 100 * e2bot.z + 1000 * e2top.z + 10000 * pt.z;
}
static void z_ruleD(const PointD& e1bot, const PointD& e1top, const PointD& e2bot,
                    const PointD& e2top, PointD& pt) {
  pt.z = e1bot.z + 10 * e1top.z + 100 * e2bot.z + 1000 * e2top.z + 10000 * pt.z;
}
#endif

extern "C" {

int32_t FN(boolean64)(uint8_t ct, uint8_t fr, int32_t preserve_collinear, int32_t reverse_solution,
                      int32_t tree, In64 subjects, In64 open_subjects, In64 clips, RefOut** out) {
  try {
    Clipper64 c;
    c.PreserveCollinear(preserve_collinear != 0);
    c.ReverseSolution(reverse_solution != 0);
#ifdef USINGZ
    c.SetZCallback(z_rule64);
#endif
    if (subjects.count) c.AddSubject(to_paths(subjects));
    if (open_subjects.count) c.AddOpenSubject(to_paths(open_subjects));
    if (clips.count) c.AddClip(to_paths(clips));
    RefOut* r = new RefOut();
    Paths64 open;
    bool ok;
    if (tree) {
      PolyTree64 pt;
      ok = c.Execute(static_cast<ClipType>(ct), static_cast<FillRule>(fr), pt, open);
      push_tree(r, pt);
    } else {
      Paths64 closed;
      ok = c.Execute(static_cast<ClipType>(ct), static_cast<FillRule>(fr), closed, open);
      push_paths(r, closed, 0);
    }
    push_paths(r, open, FLAG_OPEN);
    *out = r;
    return ok ? OK : EXECUTE_FAILED;
  } catch (...) {
    return EXCEPTION;
  }
}

int32_t FN(booleanD)(uint8_t ct, uint8_t fr, int32_t precision, int32_t preserve_collinear,
                     int32_t reverse_solution, int32_t tree, InD subjects, InD open_subjects, InD clips,
                     RefOut** out) {
  try {
    ClipperD c(precision);
    c.PreserveCollinear(preserve_collinear != 0);
    c.ReverseSolution(reverse_solution != 0);
#ifdef USINGZ
    c.SetZCallback(z_ruleD);
#endif
    if (subjects.count) c.AddSubject(to_paths(subjects));
    if (open_subjects.count) c.AddOpenSubject(to_paths(open_subjects));
    if (clips.count) c.AddClip(to_paths(clips));
    RefOut* r = new RefOut();
    PathsD open;
    bool ok;
    if (tree) {
      PolyTreeD pt;
      ok = c.Execute(static_cast<ClipType>(ct), static_cast<FillRule>(fr), pt, open);
      push_tree(r, pt);
    } else {
      PathsD closed;
      ok = c.Execute(static_cast<ClipType>(ct), static_cast<FillRule>(fr), closed, open);
      push_paths(r, closed, 0);
    }
    push_paths(r, open, FLAG_OPEN);
    *out = r;
    return ok ? OK : EXECUTE_FAILED;
  } catch (...) {
    return EXCEPTION;
  }
}

// tree: 0 = paths, 1 = PolyTree64, 2 = paths with a delta callback that returns
// deltas[call_index % delta_count] (calls counted from 0 per Execute).
int32_t FN(offset64)(double delta, double miter_limit, double arc_tolerance, int32_t preserve_collinear,
                     int32_t reverse_solution, int32_t tree, In64 paths, const uint32_t* path_group,
                     size_t group_count, const uint8_t* join_types, const uint8_t* end_types,
                     const double* deltas, size_t delta_count, RefOut** out) {
  try {
    ClipperOffset co(miter_limit, arc_tolerance, preserve_collinear != 0, reverse_solution != 0);
#ifdef USINGZ
    co.SetZCallback(z_rule64);
#endif
    const Paths64 all = to_paths(paths);
    for (size_t g = 0; g < group_count; ++g) {
      Paths64 group;
      for (size_t i = 0; i < all.size(); ++i)
        if (path_group[i] == g) group.push_back(all[i]);
      co.AddPaths(group, static_cast<JoinType>(join_types[g]), static_cast<EndType>(end_types[g]));
    }
    RefOut* r = new RefOut();
    if (tree == 1) {
      PolyTree64 pt;
      co.Execute(delta, pt);
      push_tree(r, pt);
    } else if (tree == 2) {
      size_t calls = 0;
      Paths64 solution;
      co.Execute([&](const Path64&, const PathD&, size_t, size_t) { return deltas[calls++ % delta_count]; },
                 solution);
      push_paths(r, solution, 0);
    } else {
      Paths64 solution;
      co.Execute(delta, solution);
      push_paths(r, solution, 0);
    }
    *out = r;
    return OK;
  } catch (...) {
    return EXCEPTION;
  }
}

int32_t FN(inflateD)(double delta, uint8_t jt, uint8_t et, double miter_limit, int32_t precision,
                     double arc_tolerance, InD paths, RefOut** out) {
  try {
    RefOut* r = new RefOut();
    push_paths(r, InflatePaths(to_paths(paths), delta, static_cast<JoinType>(jt), static_cast<EndType>(et),
                               miter_limit, precision, arc_tolerance), 0);
    *out = r;
    return OK;
  } catch (...) {
    return EXCEPTION;
  }
}

int32_t FN(rectclip64)(int64_t left, int64_t top, int64_t right, int64_t bottom, int32_t lines,
                       In64 paths, RefOut** out) {
  try {
    const Rect64 rect(left, top, right, bottom);
    RefOut* r = new RefOut();
    push_paths(r, lines ? RectClipLines(rect, to_paths(paths)) : RectClip(rect, to_paths(paths)), 0);
    *out = r;
    return OK;
  } catch (...) {
    return EXCEPTION;
  }
}

int32_t FN(rectclipD)(double left, double top, double right, double bottom, int32_t lines,
                      int32_t precision, InD paths, RefOut** out) {
  try {
    const RectD rect(left, top, right, bottom);
    RefOut* r = new RefOut();
    push_paths(r, lines ? RectClipLines(rect, to_paths(paths), precision)
                        : RectClip(rect, to_paths(paths), precision), 0);
    *out = r;
    return OK;
  } catch (...) {
    return EXCEPTION;
  }
}

int32_t FN(minkowski64)(In64 pattern, In64 path, int32_t is_closed, int32_t diff, RefOut** out) {
  try {
    const Path64 pat = to_paths(pattern).at(0), pth = to_paths(path).at(0);
    RefOut* r = new RefOut();
    push_paths(r, diff ? MinkowskiDiff(pat, pth, is_closed != 0) : MinkowskiSum(pat, pth, is_closed != 0), 0);
    *out = r;
    return OK;
  } catch (...) {
    return EXCEPTION;
  }
}

int32_t FN(minkowskiD)(InD pattern, InD path, int32_t is_closed, int32_t decimal_places, int32_t diff,
                       RefOut** out) {
  try {
    const PathD pat = to_paths(pattern).at(0), pth = to_paths(path).at(0);
    RefOut* r = new RefOut();
    push_paths(r, diff ? MinkowskiDiff(pat, pth, is_closed != 0, decimal_places)
                       : MinkowskiSum(pat, pth, is_closed != 0, decimal_places), 0);
    *out = r;
    return OK;
  } catch (...) {
    return EXCEPTION;
  }
}

// Utilities from clipper.h, selected by `op` (see clipper2-ref/src/lib.rs).
int32_t FN(util64)(int32_t op, double eps, int32_t flag, int64_t x, int64_t y, double rx, double ry,
                   int32_t steps, In64 paths, int32_t* ret, RefOut** out) {
  try {
    const Paths64 pp = to_paths(paths);
    RefOut* r = new RefOut();
    switch (op) {
      case 0: push_paths(r, SimplifyPaths(pp, eps, flag != 0), 0); break;
      case 1: push_paths(r, RamerDouglasPeucker(pp, eps), 0); break;
      case 2: push_paths(r, Paths64{TrimCollinear(pp.at(0), flag != 0)}, 0); break;
      case 3: *ret = static_cast<int32_t>(PointInPolygon(Point64(x, y), pp.at(0))); break;
      case 4: push_paths(r, Paths64{Ellipse(Point64(x, y), rx, ry, static_cast<size_t>(steps))}, 0); break;
      case 5: {
        Clipper64 c;
        c.AddSubject(pp);
        PolyTree64 tree;
        c.Execute(ClipType::Union, static_cast<FillRule>(flag), tree);
        *ret = CheckPolytreeFullyContainsChildren(tree) ? 1 : 0;
        push_paths(r, PolyTreeToPaths64(tree), 0);
        break;
      }
      case 6: push_paths(r, TranslatePaths(pp, x, y), 0); break;
      default: delete r; return EXCEPTION;
    }
    *out = r;
    return OK;
  } catch (...) {
    return EXCEPTION;
  }
}

int32_t FN(utilD)(int32_t op, double eps, int32_t flag, int32_t precision, double x, double y, double rx,
                  double ry, int32_t steps, InD paths, int32_t* ret, RefOut** out) {
  try {
    const PathsD pp = to_paths(paths);
    RefOut* r = new RefOut();
    switch (op) {
      case 0: push_paths(r, SimplifyPaths(pp, eps, flag != 0), 0); break;
      case 1: push_paths(r, RamerDouglasPeucker(pp, eps), 0); break;
      case 2: push_paths(r, PathsD{TrimCollinear(pp.at(0), precision, flag != 0)}, 0); break;
      case 3: *ret = static_cast<int32_t>(PointInPolygon(PointD(x, y), pp.at(0))); break;
      case 4: push_paths(r, PathsD{Ellipse(PointD(x, y), rx, ry, static_cast<size_t>(steps))}, 0); break;
      default: delete r; return EXCEPTION;
    }
    *out = r;
    return OK;
  } catch (...) {
    return EXCEPTION;
  }
}

// A C++ exception from inside Triangulate (it throws "oops" on inconsistent state) is reported
// as result 255.
int32_t FN(triangulate64)(int32_t use_delaunay, In64 paths, int32_t* result, RefOut** out) {
  try {
    Paths64 solution;
    try {
      *result = static_cast<int32_t>(Triangulate(to_paths(paths), solution, use_delaunay != 0));
    } catch (...) {
      *result = 255;
    }
    RefOut* r = new RefOut();
    push_paths(r, solution, 0);
    *out = r;
    return OK;
  } catch (...) {
    return EXCEPTION;
  }
}

int32_t FN(triangulateD)(int32_t use_delaunay, int32_t dec_places, InD paths, int32_t* result, RefOut** out) {
  try {
    PathsD solution;
    try {
      *result = static_cast<int32_t>(Triangulate(to_paths(paths), dec_places, solution, use_delaunay != 0));
    } catch (...) {
      *result = 255;
    }
    RefOut* r = new RefOut();
    push_paths(r, solution, 0);
    *out = r;
    return OK;
  } catch (...) {
    return EXCEPTION;
  }
}

struct RefView {
  const int64_t* xy; size_t xy_len;
  const double* xyd; size_t xyd_len;
  const int64_t* z; size_t z_len;
  const uint32_t* lens; const int32_t* parents; const uint8_t* flags; size_t path_count;
};

void FN(view)(const RefOut* r, RefView* v) {
  v->xy = r->xy.data(); v->xy_len = r->xy.size();
  v->xyd = r->xyd.data(); v->xyd_len = r->xyd.size();
  v->z = r->z.data(); v->z_len = r->z.size();
  v->lens = r->lens.data(); v->parents = r->parents.data(); v->flags = r->flags.data();
  v->path_count = r->lens.size();
}

void FN(free)(RefOut* r) { delete r; }

// Wall time in ns of `iters` runs of one operation on inputs converted once, so the benches
// measure the library and not the FFI copies. 0 boolean (a clip type, b fill rule),
// 1 InflatePaths (a join, b end, d delta), 2 RectClip (rect = clips[0][0], clips[0][1]),
// 3 MinkowskiSum(clips[0], subjects[0], closed), 4 Triangulate (a delaunay), 5 SimplifyPaths (d).
uint64_t FN(bench64)(int32_t op, int32_t a, int32_t b, double d, In64 subjects, In64 clips,
                     uint64_t iters) {
  const Paths64 s = to_paths(subjects), c = to_paths(clips);
  size_t sink = 0;
  const auto start = std::chrono::steady_clock::now();
  for (uint64_t k = 0; k < iters; ++k) {
    Paths64 r;
    switch (op) {
      case 0: {
        Clipper64 cl;
        cl.AddSubject(s);
        cl.AddClip(c);
        cl.Execute(static_cast<ClipType>(a), static_cast<FillRule>(b), r);
        break;
      }
      case 1: r = InflatePaths(s, d, static_cast<JoinType>(a), static_cast<EndType>(b)); break;
      case 2: r = RectClip(Rect64(c[0][0].x, c[0][0].y, c[0][1].x, c[0][1].y), s); break;
      case 3: r = MinkowskiSum(c[0], s[0], true); break;
      case 4: Triangulate(s, r, a != 0); break;
      case 5: r = SimplifyPaths(s, d, true); break;
    }
    sink += r.size();
  }
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - start).count();
  static volatile size_t keep;
  keep = sink;
  return static_cast<uint64_t>(ns);
}

}  // extern "C"
