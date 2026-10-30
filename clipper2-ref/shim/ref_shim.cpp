// C ABI over upstream Clipper2 2.0.1 for the clipper2-rs reference tests.
// Compiled twice by build.rs: plain (ref_* symbols) and with USINGZ, where the
// Clipper2Lib namespace is renamed to Clipper2LibZ (ref_z_* symbols).
// Enum codes are Clipper2's own (ClipType Intersection = 1, ...).
#include "clipper2/clipper.h"
#include "clipper2/clipper.minkowski.h"
#include "clipper2/clipper.offset.h"
#include "clipper2/clipper.rectclip.h"
#include "clipper2/clipper.triangulation.h"

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
  pt.z = e1bot.z + e1top.z + e2bot.z + e2top.z;
}
static void z_ruleD(const PointD& e1bot, const PointD& e1top, const PointD& e2bot,
                    const PointD& e2top, PointD& pt) {
  pt.z = e1bot.z + e1top.z + e2bot.z + e2top.z;
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

int32_t FN(offset64)(double delta, double miter_limit, double arc_tolerance, int32_t preserve_collinear,
                     int32_t reverse_solution, int32_t tree, In64 paths, const uint32_t* path_group,
                     size_t group_count, const uint8_t* join_types, const uint8_t* end_types,
                     RefOut** out) {
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
    if (tree) {
      PolyTree64 pt;
      co.Execute(delta, pt);
      push_tree(r, pt);
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

int32_t FN(triangulate64)(int32_t use_delaunay, In64 paths, int32_t* result, RefOut** out) {
  try {
    Paths64 solution;
    *result = static_cast<int32_t>(Triangulate(to_paths(paths), solution, use_delaunay != 0));
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

}  // extern "C"
