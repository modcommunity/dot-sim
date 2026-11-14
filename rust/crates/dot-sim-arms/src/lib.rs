//! The view arms' elbow clearance scan: dot-player-char's `DotViewArmsClear.scan`, spelled in the
//! engine's order and precision so it agrees with the script to the bit. The view arms run it per
//! arm per frame (eleven turns of the elbow pole, each an elbow solve and a depth test against the
//! held weapon's box); it is presentation only and never feeds the simulation.

pub use dot_sim_math::Vec3;
use dot_sim_math::{clampf, maxf, minf};

/// Turns tried, degrees; ties go to the earlier (smaller) one (`DotViewArmsClear.ANGLES`).
pub const ANGLES: [f64; 11] = [
    0.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0, 60.0, -60.0, 80.0, -80.0,
];
/// The share of the forearm, from the hand end, left out of the depth (`HAND_FREE`).
pub const HAND_FREE: f64 = 0.35;
/// How far each frame's turn eases toward the best angle (`EASE`).
pub const EASE: f64 = 0.35;

/// A `Transform3D`: basis rows and origin.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Xform {
    pub rows: [Vec3; 3],
    pub origin: Vec3,
}

impl Xform {
    /// `Transform3D * Vector3`: each basis row dotted with the point, plus the origin.
    #[inline]
    pub fn xform(&self, v: Vec3) -> Vec3 {
        Vec3::new(
            self.rows[0].dot(v) + self.origin.x,
            self.rows[1].dot(v) + self.origin.y,
            self.rows[2].dot(v) + self.origin.z,
        )
    }
}

/// An `AABB`: position and size.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Aabb {
    pub position: Vec3,
    pub size: Vec3,
}

impl Aabb {
    /// `AABB.end`.
    #[inline]
    pub fn end(&self) -> Vec3 {
        self.position + self.size
    }

    /// `AABB.has_point`: inside or on a face.
    #[inline]
    pub fn has_point(&self, p: Vec3) -> bool {
        let (o, s) = (self.position, self.size);
        !(p.x < o.x
            || p.y < o.y
            || p.z < o.z
            || p.x > o.x + s.x
            || p.y > o.y + s.y
            || p.z > o.z + s.z)
    }

    /// Whether `AABB.intersects_segment` finds a hit (the script only asks hit or null).
    pub fn intersects_segment(&self, from: Vec3, to: Vec3) -> bool {
        let (mut min, mut max) = (0.0f32, 1.0f32);
        let f = [from.x, from.y, from.z];
        let t = [to.x, to.y, to.z];
        let p = [self.position.x, self.position.y, self.position.z];
        let s = [self.size.x, self.size.y, self.size.z];
        for i in 0..3 {
            let (sf, st, begin) = (f[i], t[i], p[i]);
            let end = begin + s[i];
            let (cmin, cmax);
            if sf < st {
                if sf > end || st < begin {
                    return false;
                }
                let len = st - sf;
                cmin = if sf < begin { (begin - sf) / len } else { 0.0 };
                cmax = if st > end { (end - sf) / len } else { 1.0 };
            } else {
                if st > end || sf < begin {
                    return false;
                }
                let len = st - sf;
                cmin = if sf > end { (end - sf) / len } else { 0.0 };
                cmax = if st < begin { (begin - sf) / len } else { 1.0 };
            }
            if cmin > min {
                min = cmin;
            }
            if cmax < max {
                max = cmax;
            }
            if max < min {
                return false;
            }
        }
        true
    }
}

/// `Vector3.rotated(axis, angle)`: `Basis(axis, angle)` (its `set_axis_angle`) applied to `v`.
pub fn rotated(v: Vec3, axis: Vec3, angle: f32) -> Vec3 {
    let sq = Vec3::new(axis.x * axis.x, axis.y * axis.y, axis.z * axis.z);
    let cosine = angle.cos();
    let r00 = sq.x + cosine * (1.0 - sq.x);
    let r11 = sq.y + cosine * (1.0 - sq.y);
    let r22 = sq.z + cosine * (1.0 - sq.z);
    let sine = angle.sin();
    let t = 1.0 - cosine;
    let (xyzt, zyxs) = (axis.x * axis.y * t, axis.z * sine);
    let (r01, r10) = (xyzt - zyxs, xyzt + zyxs);
    let (xyzt, zyxs) = (axis.x * axis.z * t, axis.y * sine);
    let (r02, r20) = (xyzt + zyxs, xyzt - zyxs);
    let (xyzt, zyxs) = (axis.y * axis.z * t, axis.x * sine);
    let (r12, r21) = (xyzt - zyxs, xyzt + zyxs);
    Vec3::new(
        Vec3::new(r00, r01, r02).dot(v),
        Vec3::new(r10, r11, r12).dot(v),
        Vec3::new(r20, r21, r22).dot(v),
    )
}

/// `Vector3.lerp(to, weight)`: the weight rounded to `f32`, `from + (to - from) * weight` per component.
#[inline]
pub fn lerp(from: Vec3, to: Vec3, weight: f64) -> Vec3 {
    let w = weight as f32;
    Vec3::new(
        from.x + (to.x - from.x) * w,
        from.y + (to.y - from.y) * w,
        from.z + (to.z - from.z) * w,
    )
}

/// The script's `deg_to_rad`, in `f64`.
#[inline]
pub fn deg_to_rad(d: f64) -> f64 {
    d * (std::f64::consts::PI / 180.0)
}

/// `DotViewArmsClear.elbow_for`: where the two-bone solve puts the elbow for this target and pole.
/// `la`, `lb` are the upper arm and forearm (a `Vector2`'s `f32`s).
pub fn elbow_for(shoulder: Vec3, target: Vec3, la: f32, lb: f32, pole: Vec3) -> Vec3 {
    let (la, lb) = (la as f64, lb as f64);
    let to_target = target - shoulder;
    if la < 0.0001 || lb < 0.0001 || (to_target.length() as f64) < 0.0001 {
        return shoulder;
    }
    let reach = clampf(
        to_target.length() as f64,
        (la - lb).abs() + 0.001,
        la + lb - 0.001,
    );
    let dir = to_target.normalized();
    let along = (la * la - lb * lb + reach * reach) / (2.0 * reach);
    let side = pole - dir.scale(pole.dot(dir) as f64);
    let side = if (side.length_squared() as f64) > 0.000001 {
        side.normalized()
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    shoulder + dir.scale(along) + side.scale(maxf(la * la - along * along, 0.0).sqrt())
}

/// `DotViewArmsClear.depth_in`: how far `q` is inside `bounds` from its nearest face, or 0 outside.
#[inline]
pub fn depth_in(bounds: &Aabb, q: Vec3) -> f64 {
    if !bounds.has_point(q) {
        return 0.0;
    }
    let low = q - bounds.position;
    let high = bounds.end() - q;
    minf(
        minf(
            minf(low.x as f64, high.x as f64),
            minf(low.y as f64, high.y as f64),
        ),
        minf(low.z as f64, high.z as f64),
    )
}

/// `DotViewArmsClear.arm_depth`: how deep the upper arm and the forearm (all but its hand end) go into `bounds`.
pub fn arm_depth(
    to_anchor: &Xform,
    bounds: &Aabb,
    shoulder: Vec3,
    elbow: Vec3,
    target: Vec3,
) -> f64 {
    let s = to_anchor.xform(shoulder);
    let e = to_anchor.xform(elbow);
    let h = to_anchor.xform(lerp(target, elbow, HAND_FREE));
    if !bounds.intersects_segment(s, e) && !bounds.intersects_segment(e, h) {
        return 0.0;
    }
    let mut deepest = 0.0;
    for i in 0..9 {
        let t = i as f64 / 8.0;
        deepest = maxf(
            deepest,
            maxf(
                depth_in(bounds, lerp(s, e, t)),
                depth_in(bounds, lerp(e, h, t)),
            ),
        );
    }
    deepest
}

/// What [`scan`] found: the turned pole, the eased turn (degrees) and the turned arm's depth.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Clear {
    pub turned: Vec3,
    pub turn: f64,
    pub depth: f64,
}

/// `DotViewArmsClear.scan`: the eased turn of `pole` about the unit `dir` toward the least-buried of [`ANGLES`].
#[allow(clippy::too_many_arguments)]
pub fn scan(
    shoulder: Vec3,
    target: Vec3,
    la: f32,
    lb: f32,
    pole: Vec3,
    dir: Vec3,
    to_anchor: &Xform,
    bounds: &Aabb,
    up: Vec3,
    prev_turn: f64,
) -> Clear {
    let mut best = 0.0;
    let mut best_score = f64::INFINITY;
    for angle in ANGLES {
        let elbow = elbow_for(
            shoulder,
            target,
            la,
            lb,
            rotated(pole, dir, deg_to_rad(angle) as f32),
        );
        let raised = if (elbow - shoulder).dot(up) > 0.0 {
            0.05
        } else {
            0.0
        };
        let score =
            arm_depth(to_anchor, bounds, shoulder, elbow, target) + angle.abs() * 0.00005 + raised;
        if score < best_score - 0.002 {
            best_score = score;
            best = angle;
        }
    }
    let turn = prev_turn + (best - prev_turn) * EASE;
    let turned = rotated(pole, dir, deg_to_rad(turn) as f32);
    let depth = arm_depth(
        to_anchor,
        bounds,
        shoulder,
        elbow_for(shoulder, target, la, lb, turned),
        target,
    );
    Clear {
        turned,
        turn,
        depth,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn arm_case() -> (Vec3, Vec3, Vec3, Vec3, Xform, Aabb, Vec3) {
        let shoulder = Vec3::new(0.2, 1.4, 0.0);
        let target = Vec3::new(0.05, 1.2, -0.45);
        let dir = (target - shoulder).normalized();
        let pole = Vec3::new(1.0, -1.2, 0.5);
        let ident = Xform {
            rows: [
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ],
            origin: Vec3::default(),
        };
        let bounds = Aabb {
            position: Vec3::new(0.0, 1.15, -0.6),
            size: Vec3::new(0.25, 0.2, 0.5),
        };
        (
            shoulder,
            target,
            dir,
            pole,
            ident,
            bounds,
            Vec3::new(0.0, 1.0, 0.0),
        )
    }

    #[test]
    fn a_quarter_turn_about_y_takes_x_to_minus_z() {
        let r = rotated(
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            std::f32::consts::FRAC_PI_2,
        );
        assert!(r.x.abs() < 1e-6 && (r.z + 1.0).abs() < 1e-6 && r.y == 0.0);
    }

    #[test]
    fn segment_tests_match_the_box() {
        let b = Aabb {
            position: Vec3::new(0.0, 0.0, 0.0),
            size: Vec3::new(1.0, 1.0, 1.0),
        };
        assert!(b.intersects_segment(Vec3::new(-1.0, 0.5, 0.5), Vec3::new(2.0, 0.5, 0.5)));
        assert!(b.intersects_segment(Vec3::new(2.0, 0.5, 0.5), Vec3::new(-1.0, 0.5, 0.5)));
        assert!(!b.intersects_segment(Vec3::new(-1.0, 2.0, 0.5), Vec3::new(2.0, 2.0, 0.5)));
        assert!(b.has_point(Vec3::new(1.0, 1.0, 1.0)));
        assert!(!b.has_point(Vec3::new(1.0, 1.0, 1.01)));
        assert_eq!(depth_in(&b, Vec3::new(0.25, 0.5, 0.5)), 0.25);
    }

    #[test]
    fn elbow_keeps_both_bone_lengths() {
        let (shoulder, target, _, pole, ..) = arm_case();
        let e = elbow_for(shoulder, target, 0.3, 0.28, pole);
        assert!(((e - shoulder).length() - 0.3).abs() < 1e-4);
        assert!(((target - e).length() - 0.28).abs() < 1e-4);
        assert_eq!(elbow_for(shoulder, shoulder, 0.3, 0.28, pole), shoulder);
    }

    #[test]
    fn a_buried_arm_turns_out_and_eases() {
        let (shoulder, target, dir, pole, ident, bounds, up) = arm_case();
        let first = scan(
            shoulder, target, 0.3, 0.28, pole, dir, &ident, &bounds, up, 0.0,
        );
        let again = scan(
            shoulder, target, 0.3, 0.28, pole, dir, &ident, &bounds, up, first.turn,
        );
        // Eased: each frame moves 35% of the remaining way toward the same best angle.
        let best = first.turn / EASE;
        assert!(ANGLES.contains(&best) || first.turn == 0.0);
        assert_eq!(again.turn, first.turn + (best - first.turn) * EASE);
        assert!(first.depth >= 0.0);
    }

    #[test]
    fn a_clear_arm_keeps_its_pole() {
        let (shoulder, target, dir, pole, ident, _, up) = arm_case();
        let far = Aabb {
            position: Vec3::new(5.0, 5.0, 5.0),
            size: Vec3::new(0.1, 0.1, 0.1),
        };
        let c = scan(
            shoulder, target, 0.3, 0.28, pole, dir, &ident, &far, up, 0.0,
        );
        assert_eq!(c.turn, 0.0);
        assert_eq!(c.depth, 0.0);
        assert_eq!(c.turned, rotated(pole, dir, 0.0));
    }
}
