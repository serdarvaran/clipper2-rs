//! Engine records (`clipper.engine.h`). The C++ raw-pointer links become `u32` indices into
//! per-kind arenas owned by [`super::ClipperBase`]; `NONE` plays the role of `nullptr`.

use crate::core::{Path64, Point64, Rect64};

pub(crate) type Idx = u32;
pub(crate) const NONE: Idx = u32::MAX;

/// Which input a path belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PathType {
    Subject,
    Clip,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum JoinWith {
    NoJoin,
    Left,
    Right,
}

pub(crate) const VF_OPEN_START: u8 = 1;
pub(crate) const VF_OPEN_END: u8 = 2;
pub(crate) const VF_LOCAL_MAX: u8 = 4;
pub(crate) const VF_LOCAL_MIN: u8 = 8;

#[derive(Clone, Copy, Debug)]
pub(crate) struct Vertex {
    pub pt: Point64,
    pub next: Idx,
    pub prev: Idx,
    pub flags: u8,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct LocalMinima {
    pub vertex: Idx,
    pub polytype: PathType,
    pub is_open: bool,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct OutPt {
    pub pt: Point64,
    pub next: Idx,
    pub prev: Idx,
    pub outrec: Idx,
    pub horz: Idx,
}

#[derive(Clone, Debug)]
pub(crate) struct OutRec {
    pub idx: usize,
    pub owner: Idx,
    pub front_edge: Idx,
    pub back_edge: Idx,
    pub pts: Idx,
    pub polypath: Idx,
    pub splits: Option<Vec<Idx>>,
    pub recursive_split: Idx,
    pub bounds: Rect64,
    pub path: Path64,
    pub is_open: bool,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct Active {
    pub bot: Point64,
    pub top: Point64,
    pub curr_x: i64,
    pub dx: f64,
    pub wind_dx: i32,
    pub wind_cnt: i32,
    pub wind_cnt2: i32,
    pub outrec: Idx,
    pub prev_in_ael: Idx,
    pub next_in_ael: Idx,
    pub prev_in_sel: Idx,
    pub next_in_sel: Idx,
    pub jump: Idx,
    pub vertex_top: Idx,
    pub local_min: Idx,
    pub is_left_bound: bool,
    pub join_with: JoinWith,
}

impl Default for Active {
    fn default() -> Self {
        Active {
            bot: Point64::default(),
            top: Point64::default(),
            curr_x: 0,
            dx: 0.0,
            wind_dx: 1,
            wind_cnt: 0,
            wind_cnt2: 0,
            outrec: NONE,
            prev_in_ael: NONE,
            next_in_ael: NONE,
            prev_in_sel: NONE,
            next_in_sel: NONE,
            jump: NONE,
            vertex_top: NONE,
            local_min: NONE,
            is_left_bound: false,
            join_with: JoinWith::NoJoin,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct IntersectNode {
    pub pt: Point64,
    pub edge1: Idx,
    pub edge2: Idx,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct HorzSegment {
    pub left_op: Idx,
    pub right_op: Idx,
    pub left_to_right: bool,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct HorzJoin {
    pub op1: Idx,
    pub op2: Idx,
}
