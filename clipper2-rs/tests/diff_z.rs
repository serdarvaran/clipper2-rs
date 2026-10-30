#![cfg(feature = "usingz")]
mod common;

use clipper2_ref::z;
use clipper2_rs::{ClipType, Clipper64, ClipperD, FillRule, Paths64, PathsD, Point64, PointD};
use common::*;

const CLIP_TYPES: [ClipType; 4] = [
    ClipType::Intersection,
    ClipType::Union,
    ClipType::Difference,
    ClipType::Xor,
];
const FILL_RULES: [FillRule; 4] = [
    FillRule::EvenOdd,
    FillRule::NonZero,
    FillRule::Positive,
    FillRule::Negative,
];

fn with_z(rng: &mut Rng, paths: Paths64) -> Paths64 {
    paths
        .into_iter()
        .map(|p| {
            p.into_iter()
                .map(|pt| Point64::new_z(pt.x, pt.y, rng.range_i64(1, 100)))
                .collect()
        })
        .collect()
}

fn to_ref_z(paths: &[clipper2_rs::Path64]) -> Vec<z::RefPathZ> {
    paths
        .iter()
        .map(|p| p.iter().map(|pt| (pt.x, pt.y, pt.z)).collect())
        .collect()
}

fn to_ref_dz(paths: &[clipper2_rs::PathD]) -> Vec<Vec<(u64, u64, i64)>> {
    paths
        .iter()
        .map(|p| {
            p.iter()
                .map(|pt| (pt.x.to_bits(), pt.y.to_bits(), pt.z))
                .collect()
        })
        .collect()
}

fn ref_dz(paths: &[z::RefPathDZ]) -> Vec<Vec<(u64, u64, i64)>> {
    paths
        .iter()
        .map(|p| {
            p.iter()
                .map(|&(x, y, z)| (x.to_bits(), y.to_bits(), z))
                .collect()
        })
        .collect()
}

fn rule64() -> clipper2_rs::ZCallback64 {
    Box::new(
        |e1b: Point64, e1t: Point64, e2b: Point64, e2t: Point64, pt: &mut Point64| {
            pt.z = e1b.z + 10 * e1t.z + 100 * e2b.z + 1000 * e2t.z + 10000 * pt.z
        },
    )
}

#[test]
fn random_z_boolean_matches_reference() {
    for ct in CLIP_TYPES {
        for fr in FILL_RULES {
            for shape in Shape::ALL {
                for seed in 0..cases() as u64 {
                    let mut rng = Rng::new(seed * 32_452_843 + ct as u64 * 3 + fr as u64);
                    let subject = random_paths(&mut rng, shape);
                    let subject = with_z(&mut rng, subject);
                    let open = if seed % 2 == 0 {
                        vec![]
                    } else {
                        random_open_paths(&mut rng, shape)
                    };
                    let open = with_z(&mut rng, open);
                    let clip = random_paths(&mut rng, shape);
                    let clip = with_z(&mut rng, clip);
                    let mut c = Clipper64::new();
                    c.set_z_callback(Some(rule64()));
                    c.add_subject(&subject);
                    c.add_open_subject(&open);
                    c.add_clip(&clip);
                    let case = format!("z-{ct:?}-{fr:?}-{shape:?}-{seed}");
                    let input = format!(
                        "{ct:?} {fr:?}\nsubject={:?}\nopen={:?}\nclip={:?}",
                        to_ref_z(&subject),
                        to_ref_z(&open),
                        to_ref_z(&clip)
                    );
                    let ours = c.execute(ct, fr);
                    let reference = z::boolean(
                        ct as u8,
                        fr as u8,
                        true,
                        false,
                        &to_ref_z(&subject),
                        &to_ref_z(&open),
                        &to_ref_z(&clip),
                    );
                    match (ours, reference) {
                        (Ok(o), Ok(r)) => {
                            assert_same(&case, &input, &to_ref_z(&o.closed), &r.closed);
                            assert_same(
                                &format!("{case}-open"),
                                &input,
                                &to_ref_z(&o.open),
                                &r.open,
                            );
                        }
                        (o, r) => assert_same(&case, &input, &o.is_ok(), &r.is_ok()),
                    }
                }
            }
        }
    }
}

#[test]
fn random_z_clipper_d_matches_reference() {
    for ct in CLIP_TYPES {
        for fr in FILL_RULES {
            for seed in 0..cases() as u64 {
                let mut rng = Rng::new(seed * 49_979_687 + ct as u64 * 3 + fr as u64);
                let zd = |rng: &mut Rng, p: PathsD| -> PathsD {
                    p.into_iter()
                        .map(|path| {
                            path.into_iter()
                                .map(|pt| PointD::new_z(pt.x, pt.y, rng.range_i64(1, 100)))
                                .collect()
                        })
                        .collect()
                };
                let subject = random_paths_d(&mut rng, Shape::Medium, 2);
                let subject = zd(&mut rng, subject);
                let clip = random_paths_d(&mut rng, Shape::Medium, 2);
                let clip = zd(&mut rng, clip);
                let mut c = ClipperD::new(2).unwrap();
                c.set_z_callback(Some(Box::new(
                    |e1b: PointD, e1t: PointD, e2b: PointD, e2t: PointD, pt: &mut PointD| {
                        pt.z = e1b.z + 10 * e1t.z + 100 * e2b.z + 1000 * e2t.z + 10000 * pt.z
                    },
                )));
                c.add_subject(&subject).unwrap();
                c.add_clip(&clip).unwrap();
                let o = c.execute(ct, fr).unwrap();
                let rs: Vec<z::RefPathDZ> = subject
                    .iter()
                    .map(|p| p.iter().map(|q| (q.x, q.y, q.z)).collect())
                    .collect();
                let rc: Vec<z::RefPathDZ> = clip
                    .iter()
                    .map(|p| p.iter().map(|q| (q.x, q.y, q.z)).collect())
                    .collect();
                let r = z::boolean_d(ct as u8, fr as u8, 2, true, false, &rs, &[], &rc).unwrap();
                assert_same(
                    &format!("zd-{ct:?}-{fr:?}-{seed}"),
                    &format!("{rs:?}\n{rc:?}"),
                    &to_ref_dz(&o.closed),
                    &ref_dz(&r.closed),
                );
            }
        }
    }
}

#[test]
fn input_z_is_kept_and_crossings_use_the_callback() {
    let a = vec![vec![
        Point64::new_z(0, 0, 1),
        Point64::new_z(10, 0, 1),
        Point64::new_z(10, 10, 1),
        Point64::new_z(0, 10, 1),
    ]];
    let b = vec![vec![
        Point64::new_z(5, 5, 10),
        Point64::new_z(15, 5, 10),
        Point64::new_z(15, 15, 10),
        Point64::new_z(5, 15, 10),
    ]];
    let mut c = Clipper64::new();
    c.set_z_callback(Some(rule64()));
    c.add_subject(&a);
    c.add_clip(&b);
    let s = c.execute(ClipType::Union, FillRule::NonZero).unwrap();
    let z_at = |x, y| {
        s.closed[0]
            .iter()
            .find(|p| p.x == x && p.y == y)
            .map(|p| p.z)
    };
    assert_eq!(
        (z_at(0, 0), z_at(15, 15), z_at(10, 5), z_at(5, 10)),
        (Some(1), Some(10), Some(11_011), Some(11_011))
    );
}

#[test]
fn random_z_offset_matches_reference() {
    use clipper2_rs::{ClipperOffset, EndType, JoinType};
    let jts = [
        JoinType::Square,
        JoinType::Bevel,
        JoinType::Round,
        JoinType::Miter,
    ];
    let ets = [
        EndType::Polygon,
        EndType::Joined,
        EndType::Butt,
        EndType::Square,
        EndType::Round,
    ];
    for seed in 0..cases() as u64 {
        let mut rng = Rng::new(seed * 67_867_967 + 1);
        let (jt, et) = (jts[rng.below(4)], ets[rng.below(5)]);
        let paths = if et == EndType::Polygon {
            random_paths(&mut rng, Shape::Medium)
        } else {
            random_open_paths(&mut rng, Shape::Medium)
        };
        let paths = with_z(&mut rng, paths);
        let delta = rng.range_f64(-30.0, 60.0);
        let mut co = ClipperOffset::new(2.0, 0.0, false, false);
        co.set_z_callback(Some(rule64()));
        co.add_paths(&paths, jt, et);
        let o = co.execute(delta).unwrap();
        let g = clipper2_ref::OffsetGroup {
            paths: to_ref_z(&paths),
            join_type: jt as u8,
            end_type: et as u8,
        };
        let r = z::offset(delta, &[g], 2.0, 0.0, false, false).unwrap();
        assert_same(
            &format!("z-offset-{seed}"),
            &format!("{jt:?} {et:?} {delta}\n{:?}", to_ref_z(&paths)),
            &to_ref_z(&o),
            &r,
        );
    }
}
