//! A humanoid figure's pose layers after the clip: dot-player-char's `DotHumanoidAim.apply_plan`,
//! `DotHumanoidIk.two_bone_under` and `DotHumanoidCarryPose.apply`, spelled operation for operation.
//!
//! The skeleton stays the engine's: this crate reads and writes bone poses through [`Rig`]. Every
//! vector, quaternion and transform operation is [`geom`]'s (`f32`, engine order); every scalar the
//! script keeps in a `float` is an `f64` here, rounded to `f32` where the script hands it to the engine.

pub mod geom;

pub use dot_sim_math::Vec3;
use dot_sim_math::{clampf, maxf};
pub use geom::{Basis, Quat, Xform};
use geom::{lerp3, lerpf, smoothstep};

/// `Vector3.RIGHT`.
const RIGHT: Vec3 = Vec3::new(1.0, 0.0, 0.0);
/// `Vector3.UP`.
const UP: Vec3 = Vec3::new(0.0, 1.0, 0.0);

/// A skeleton's local bone poses, as they are now.
pub trait Rig {
    /// `get_bone_pose`.
    fn pose(&self, bone: i32) -> Xform;
    /// `get_bone_pose_rotation`.
    fn rotation(&self, bone: i32) -> Quat;
    /// `get_bone_parent`.
    fn parent(&self, bone: i32) -> i32;
    /// `set_bone_pose_rotation`.
    fn set_rotation(&mut self, bone: i32, q: Quat);
}

/// `DotHumanoidIk.chain_pose`: identity times each bone's pose, root first.
pub fn chain_pose<R: Rig>(rig: &R, chain: &[i32]) -> Xform {
    let mut out = Xform::IDENTITY;
    for &b in chain {
        out = out * rig.pose(b);
    }
    out
}

/// `DotHumanoidIk.global_pose`: [`chain_pose`] over the bones from the root to `bone`.
pub fn global_pose<R: Rig>(rig: &R, bone: i32) -> Xform {
    let mut chain = Vec::with_capacity(16);
    let mut b = bone;
    while b >= 0 {
        chain.push(b);
        b = rig.parent(b);
    }
    chain.reverse();
    chain_pose(rig, &chain)
}

/// `DotHumanoidIk.chain_rotation`.
pub fn chain_rotation<R: Rig>(rig: &R, bone: i32) -> Quat {
    let mut q = Quat::IDENTITY;
    let mut b = bone;
    while b >= 0 {
        q = rig.rotation(b) * q;
        b = rig.parent(b);
    }
    q
}

/// `DotHumanoidAim.apply_plan`: `plan` is the script's flat plan, four numbers a bone (index,
/// parent, share, arm as 0 or 1).
pub fn aim<R: Rig>(
    rig: &mut R,
    plan: &[f64],
    to_figure: &Xform,
    pitch: f64,
    sprint_blend: f64,
    sprint_drop: f64,
) {
    let mut axis = Vec3::default();
    if (to_figure.basis.determinant() as f64).abs() > 1e-9 {
        axis = to_figure.basis.inverse().xform(RIGHT).normalized();
    }
    if !(axis.x.is_finite() && axis.y.is_finite() && axis.z.is_finite())
        || (axis.length_squared() as f64) < 0.5
    {
        return;
    }
    let n = plan.len() / 4;
    let mut parents = [Quat::IDENTITY; 32];
    let mut heap = Vec::new();
    let parents: &mut [Quat] = if n <= 32 {
        &mut parents[..n]
    } else {
        heap.resize(n, Quat::IDENTITY);
        &mut heap
    };
    for (k, p) in parents.iter_mut().enumerate() {
        *p = chain_rotation(rig, plan[k * 4 + 1] as i32);
    }
    for (k, &p) in parents.iter().enumerate() {
        let i = plan[k * 4] as i32;
        let share = plan[k * 4 + 2];
        let drop = if plan[k * 4 + 3] != 0.0 {
            -sprint_drop * sprint_blend
        } else {
            0.0
        };
        let turn = Quat::from_axis_angle(axis, (pitch * share + drop) as f32);
        let q = (p.inverse() * turn * p * rig.rotation(i)).normalized();
        rig.set_rotation(i, q);
    }
}

/// `DotHumanoidIk.two_bone_under`: turns `bones` (upper, lower, end) toward `target` (skeleton
/// space) with the elbow toward `pole_dir`, under the upper bone's parent pose `upper_global_parent`.
pub fn two_bone_under<R: Rig>(
    rig: &mut R,
    bones: [i32; 3],
    upper_global_parent: &Xform,
    target: Vec3,
    pole_dir: Vec3,
    weight: f64,
) {
    let upper = *upper_global_parent * rig.pose(bones[0]);
    let lower = if rig.parent(bones[1]) == bones[0] {
        upper * rig.pose(bones[1])
    } else {
        global_pose(rig, bones[1])
    };
    let wrist = if rig.parent(bones[2]) == bones[1] {
        lower * rig.pose(bones[2])
    } else {
        global_pose(rig, bones[2])
    };
    let a = lower.origin - upper.origin;
    let b = wrist.origin - lower.origin;
    let la = a.length() as f64;
    let lb = b.length() as f64;
    let to_target = target - upper.origin;
    if la < 0.0001 || lb < 0.0001 || (to_target.length() as f64) < 0.0001 {
        return;
    }
    let reach = clampf(
        to_target.length() as f64,
        (la - lb).abs() + 0.001,
        la + lb - 0.001,
    );
    let dir = to_target.normalized();
    let along = (la * la - lb * lb + reach * reach) / (2.0 * reach);
    let height = maxf(la * la - along * along, 0.0).sqrt();
    let pd = pole_dir.dot(dir);
    let side = pole_dir - Vec3::new(dir.x * pd, dir.y * pd, dir.z * pd);
    let side = if side.length_squared() as f64 > 0.000001 {
        side.normalized()
    } else {
        UP
    };
    let elbow = upper.origin + dir.scale(along) + side.scale(height);
    let hand_goal = upper.origin + dir.scale(reach);
    let q1 = Quat::from_arc(a.normalized(), (elbow - upper.origin).normalized());
    let q2 = Quat::from_arc(q1.xform(b).normalized(), (hand_goal - elbow).normalized());
    let upper_rot = (q1 * upper.basis.rotation_quaternion()).normalized();
    let lower_rot = (q2 * q1 * lower.basis.rotation_quaternion()).normalized();
    let upper_parent = upper_global_parent.basis.rotation_quaternion();
    let upper_local = (upper_parent.inverse() * upper_rot).normalized();
    let lower_local = (upper_rot.inverse() * lower_rot).normalized();
    let w = clampf(weight, 0.0, 1.0) as f32;
    let r0 = rig.rotation(bones[0]).slerp(upper_local, w);
    rig.set_rotation(bones[0], r0);
    let r1 = rig.rotation(bones[1]).slerp(lower_local, w);
    rig.set_rotation(bones[1], r1);
}

/// `DotHumanoidIk.two_bone_to` for a limb whose end is the point `end_local` in the lower bone's
/// frame; `upper_chain` and `lower_chain` are `chain_to` the upper and the lower bone.
pub fn two_bone_to<R: Rig>(
    rig: &mut R,
    upper_chain: &[i32],
    lower_chain: &[i32],
    end_local: Vec3,
    target: Vec3,
    pole_dir: Vec3,
    weight: f64,
) {
    let (Some(&upper), Some(&lower)) = (upper_chain.last(), lower_chain.last()) else {
        return;
    };
    let top = chain_pose(rig, upper_chain);
    let mid = chain_pose(rig, lower_chain);
    let a = mid.origin - top.origin;
    let b = mid.xform(end_local) - mid.origin;
    let la = a.length() as f64;
    let lb = b.length() as f64;
    let to_target = target - top.origin;
    if la < 0.0001 || lb < 0.0001 || (to_target.length() as f64) < 0.0001 {
        return;
    }
    let reach = clampf(
        to_target.length() as f64,
        (la - lb).abs() + 0.001,
        la + lb - 0.001,
    );
    let dir = to_target.normalized();
    let along = (la * la - lb * lb + reach * reach) / (2.0 * reach);
    let pd = pole_dir.dot(dir);
    let side = pole_dir - Vec3::new(dir.x * pd, dir.y * pd, dir.z * pd);
    let side = if side.length_squared() as f64 > 0.000001 {
        side.normalized()
    } else {
        UP
    };
    let height = maxf(la * la - along * along, 0.0).sqrt();
    let knee = top.origin + dir.scale(along) + side.scale(height);
    let q1 = Quat::from_arc(a.normalized(), (knee - top.origin).normalized());
    let q2 = Quat::from_arc(
        q1.xform(b).normalized(),
        (top.origin + dir.scale(reach) - knee).normalized(),
    );
    let upper_rot = (q1 * top.basis.rotation_quaternion()).normalized();
    let lower_rot = (q2 * q1 * mid.basis.rotation_quaternion()).normalized();
    let parent = chain_pose(rig, &upper_chain[..upper_chain.len() - 1])
        .basis
        .rotation_quaternion();
    let w = clampf(weight, 0.0, 1.0) as f32;
    let r0 = rig
        .rotation(upper)
        .slerp((parent.inverse() * upper_rot).normalized(), w);
    rig.set_rotation(upper, r0);
    let r1 = rig
        .rotation(lower)
        .slerp((upper_rot.inverse() * lower_rot).normalized(), w);
    rig.set_rotation(lower, r1);
}

/// One leg of [`on_floor`]: the ankle as a point in the lower leg's frame, and `chain_to` the
/// upper and the lower leg (the last bone of each is that bone).
#[derive(Clone, Copy, Debug)]
pub struct FloorLeg<'a> {
    pub end_local: Vec3,
    pub upper_chain: &'a [i32],
    pub lower_chain: &'a [i32],
}

/// `DotHumanoidLegs.on_floor` past its checks: each leg whose ankle `to_figure` puts under
/// `floor_y[k] + raised` is bent by [`two_bone_to`] until it is back on it. `saved` gets each
/// bent bone with its rotation from before the bend (the script keeps the first per bone).
pub fn on_floor<R: Rig>(
    rig: &mut R,
    legs: &[FloorLeg],
    to_figure: &Xform,
    floor_y: &[f32],
    raised: f64,
    saved: &mut Vec<(i32, Quat)>,
) {
    let to_skel = to_figure.affine_inverse();
    for (leg, &floor) in legs.iter().zip(floor_y) {
        let (Some(&upper), Some(&lower)) = (leg.upper_chain.last(), leg.lower_chain.last()) else {
            continue;
        };
        let knee = chain_pose(rig, leg.lower_chain);
        let ankle = knee.xform(leg.end_local);
        let at = to_figure.xform(ankle);
        let lift = floor as f64 + raised - at.y as f64;
        if lift <= 0.0 {
            continue;
        }
        saved.push((upper, rig.rotation(upper)));
        saved.push((lower, rig.rotation(lower)));
        let mid = chain_pose(rig, leg.upper_chain).origin + ankle;
        let mut pole = knee.origin - Vec3::new(mid.x * 0.5, mid.y * 0.5, mid.z * 0.5);
        if (pole.length_squared() as f64) < 1e-8 {
            pole = to_skel.basis.xform(Vec3::new(0.0, 0.0, -1.0));
        }
        let target = to_skel.xform(at + UP.scale(lift));
        two_bone_to(
            rig,
            leg.upper_chain,
            leg.lower_chain,
            leg.end_local,
            target,
            pole,
            1.0,
        );
    }
}

/// One figure's carry style and stance blends, as `DotHumanoidCarryPose.apply` reads them.
#[derive(Clone, Copy, Debug, Default)]
pub struct Carry {
    /// Ready, sprint, aim, reload and swap grips.
    pub grips: [Vec3; 5],
    /// The same stances' turns, `Quaternion.from_euler` of the style's degrees.
    pub turns: [Quat; 5],
    pub walk_sway: Vec3,
    pub run_sway: Vec3,
    /// The style's foregrip in the gun's frame.
    pub support: Vec3,
    pub follow: f64,
    pub kick: f64,
    pub kick_climb: f64,
    /// The arm's weight (the carry's weight less the overlay's hold).
    pub weight: f64,
    pub sprint: f64,
    pub aim: f64,
    pub reload: f64,
    pub swap: f64,
    pub kick_now: f64,
    pub reload_time: f64,
    pub phase: f64,
    pub pitch: f64,
    pub size: f64,
    pub speed: f64,
    /// Whether the other hand takes the foregrip (`two_handed and explicit and swap < 0.5`).
    pub support_on: bool,
}

/// What `DotHumanoidCarryPose._measure` found for a body.
#[derive(Clone, Copy, Debug, Default)]
pub struct CarryRig {
    /// Upper arm, lower arm, hand.
    pub arm: [i32; 3],
    pub shoulder: i32,
    pub hand_frame: Xform,
    pub hand_frame_inv: Xform,
    pub chest_ref: Xform,
}

/// `DotHumanoidCarryPose.CHEST_PITCH`.
pub const CHEST_PITCH: f64 = 0.22;
/// `DotHumanoidCarryPose.POLE`.
pub const CARRY_POLE: Vec3 = Vec3::new(0.5, -1.0, 0.25);
/// `DotHumanoidCarryPose.BELT`.
const BELT: Vec3 = Vec3::new(-0.12, -0.26, -0.06);
/// `DotHumanoidCarryPose.WELL`.
const WELL: Vec3 = Vec3::new(0.0, -0.10, -0.08);
/// `DotHumanoidCarryPose.RELOAD_PATH`.
const RELOAD_PATH: f64 = 1.1;

/// `DotHumanoidCarryPose.apply` from the stance blend on (the caller has checked the weight, the
/// style and the bones): writes the gun arm and returns the other hand's point, or `None`.
pub fn carry<R: Rig>(
    rig: &mut R,
    c: &Carry,
    m: &CarryRig,
    chest_chain: &[i32],
    to_figure: &Xform,
) -> Option<Vec3> {
    let mut grip = lerp3(c.grips[0], c.grips[1], c.sprint);
    let mut turn = c.turns[0].slerp(c.turns[1], c.sprint as f32);
    grip = lerp3(grip, c.grips[2], c.aim);
    turn = turn.slerp(c.turns[2], c.aim as f32);
    grip = lerp3(grip, c.grips[3], c.reload);
    turn = turn.slerp(c.turns[3], c.reload as f32);
    grip = lerp3(grip, c.grips[4], c.swap);
    turn = turn.slerp(c.turns[4], c.swap as f32);
    let chest_global = chain_pose(rig, chest_chain);
    let shoulder_global = chest_global * rig.pose(m.shoulder);
    let chest = *to_figure * chest_global;
    let chest_ref = *to_figure * m.chest_ref;
    let lean = chest.basis.rotation_quaternion() * chest_ref.basis.rotation_quaternion().inverse();
    let follow_share = lerpf(c.follow, 0.25, c.aim);
    let follow = Quat::IDENTITY.slerp(lean, follow_share as f32);
    grip = chest.origin + follow.xform(grip.scale(c.size) - chest_ref.origin);
    turn = follow * turn;
    if c.phase >= 0.0 {
        let amount =
            clampf(c.speed / 1.2, 0.0, 1.0) * (1.0 - 0.75 * c.aim) * (1.0 - 0.5 * c.reload);
        let amp = lerp3(
            c.walk_sway,
            c.run_sway,
            clampf((c.speed - 2.0) / 3.0, 0.0, 1.0),
        )
        .scale(amount);
        let a = std::f64::consts::TAU * c.phase;
        let bob = Vec3::new(
            (amp.y as f64 * a.sin()) as f32,
            (amp.x as f64 * (2.0 * a).sin()) as f32,
            0.0,
        );
        grip = grip + bob.scale(c.size);
        let deg = amp.z as f64 * (std::f64::consts::PI / 180.0);
        let e = Vec3::new(
            (0.4 * deg * (2.0 * a).sin()) as f32,
            (deg * a.sin()) as f32,
            (-0.6 * deg * a.sin()) as f32,
        );
        turn = Quat::from_euler(e) * turn;
    }
    let shoulder = to_figure.xform((shoulder_global * rig.pose(m.arm[0])).origin);
    let share = lerpf(0.5, 1.0, c.aim) * (1.0 - 0.7 * c.sprint * (1.0 - c.aim))
        - CHEST_PITCH * follow_share;
    let tilt = Quat::from_axis_angle(RIGHT, (c.pitch * share) as f32);
    grip = shoulder + tilt.xform(grip - shoulder);
    turn = tilt * turn;
    if c.kick_now > 0.0 {
        grip = grip + turn.xform(Vec3::new(0.0, 0.0, (c.kick * c.kick_now * c.size) as f32));
        turn = turn * Quat::from_axis_angle(RIGHT, (c.kick_climb.to_radians() * c.kick_now) as f32);
    }
    let hand =
        to_figure.affine_inverse() * Xform::new(Basis::from_quat(turn), grip) * m.hand_frame_inv;
    let pole = to_figure.basis.inverse().xform(CARRY_POLE);
    two_bone_under(rig, m.arm, &shoulder_global, hand.origin, pole, c.weight);
    let upper_global = shoulder_global * rig.pose(m.arm[0]);
    let lower_global = upper_global * rig.pose(m.arm[1]);
    let lower = lower_global.basis.rotation_quaternion();
    let wanted = (lower.inverse() * hand.basis.rotation_quaternion()).normalized();
    let r = rig.rotation(m.arm[2]).slerp(wanted, c.weight as f32);
    rig.set_rotation(m.arm[2], r);
    if !c.support_on {
        return None;
    }
    Some(support(
        c,
        m,
        &(lower_global * rig.pose(m.arm[2])),
        to_figure,
        chest.origin,
    ))
}

/// `DotHumanoidCarryPose._support`.
fn support(c: &Carry, m: &CarryRig, hand_global: &Xform, to_figure: &Xform, chest: Vec3) -> Vec3 {
    let gun = *hand_global * m.hand_frame;
    let fore = gun.xform(c.support.scale(c.size));
    if c.reload <= 0.0 {
        return fore;
    }
    let well = gun.xform(WELL.scale(c.size));
    let belt = to_figure.affine_inverse().xform(chest + BELT.scale(c.size));
    let t = c.reload_time / RELOAD_PATH;
    let path = if t < 0.15 {
        lerp3(fore, well, smoothstep(0.0, 0.15, t))
    } else if t < 0.4 {
        lerp3(well, belt, smoothstep(0.15, 0.4, t))
    } else if t < 0.7 {
        lerp3(belt, well, smoothstep(0.4, 0.7, t))
    } else if t < 0.85 {
        well
    } else if t < 1.0 {
        lerp3(well, fore, smoothstep(0.85, 1.0, t))
    } else {
        fore
    };
    lerp3(fore, path, c.reload)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A chain of bones each a unit up its parent, every pose turned by its rotation.
    struct Line {
        rot: Vec<Quat>,
        parent: Vec<i32>,
    }

    impl Rig for Line {
        fn pose(&self, bone: i32) -> Xform {
            let o = if self.parent[bone as usize] < 0 {
                Vec3::default()
            } else {
                Vec3::new(0.0, 1.0, 0.0)
            };
            Xform::new(Basis::from_quat(self.rot[bone as usize]), o)
        }
        fn rotation(&self, bone: i32) -> Quat {
            self.rot[bone as usize]
        }
        fn parent(&self, bone: i32) -> i32 {
            self.parent[bone as usize]
        }
        fn set_rotation(&mut self, bone: i32, q: Quat) {
            self.rot[bone as usize] = q;
        }
    }

    fn arm() -> Line {
        Line {
            rot: vec![Quat::IDENTITY; 4],
            parent: vec![-1, 0, 1, 2],
        }
    }

    #[test]
    fn two_bone_reaches_a_target_in_range() {
        let mut rig = arm();
        let root = global_pose(&rig, 0);
        let target = Vec3::new(1.2, 1.5, 0.3);
        two_bone_under(
            &mut rig,
            [1, 2, 3],
            &root,
            target,
            Vec3::new(0.0, 0.0, -1.0),
            1.0,
        );
        let end = global_pose(&rig, 3).origin;
        assert!((end - target).length() < 1e-4, "end {end:?}");
    }

    #[test]
    fn two_bone_at_zero_weight_keeps_the_pose() {
        let mut rig = arm();
        let root = global_pose(&rig, 0);
        two_bone_under(
            &mut rig,
            [1, 2, 3],
            &root,
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, -1.0),
            0.0,
        );
        assert!(
            rig.rot
                .iter()
                .all(|q| (q.w - 1.0).abs() < 1e-6 && q.x.abs() + q.y.abs() + q.z.abs() < 1e-6)
        );
    }

    #[test]
    fn on_floor_lifts_only_an_ankle_under_the_floor() {
        // The figure's frame upside down: the line's top end (the ankle) is its lowest point.
        let flip = Xform::new(
            Basis {
                rows: [
                    Vec3::new(1.0, 0.0, 0.0),
                    Vec3::new(0.0, -1.0, 0.0),
                    Vec3::new(0.0, 0.0, -1.0),
                ],
            },
            Vec3::default(),
        );
        let leg = FloorLeg {
            end_local: Vec3::new(0.0, 1.0, 0.0),
            upper_chain: &[0, 1],
            lower_chain: &[0, 1, 2],
        };
        let mut rig = arm();
        let mut saved = Vec::new();
        on_floor(&mut rig, &[leg], &flip, &[-3.5], 0.0, &mut saved);
        assert!(saved.is_empty() && rig.rot.iter().all(|&q| q == Quat::IDENTITY));
        on_floor(&mut rig, &[leg], &flip, &[-3.0], 0.5, &mut saved);
        let ankle = global_pose(&rig, 2).xform(leg.end_local);
        assert!((ankle.y - 2.5).abs() < 1e-4, "ankle {ankle:?}");
        assert_eq!(saved, vec![(1, Quat::IDENTITY), (2, Quat::IDENTITY)]);
    }

    #[test]
    fn aim_bends_each_bone_by_its_share() {
        let mut rig = arm();
        aim(
            &mut rig,
            &[1.0, 0.0, 0.5, 0.0],
            &Xform::IDENTITY,
            0.4,
            0.0,
            0.6,
        );
        let want = Quat::from_axis_angle(RIGHT, 0.2);
        let got = rig.rot[1];
        assert!((got.x - want.x).abs() < 1e-6 && (got.w - want.w).abs() < 1e-6);
    }

    #[test]
    fn aim_on_a_collapsed_frame_does_nothing() {
        let mut rig = arm();
        let flat = Xform::new(
            Basis {
                rows: [Vec3::default(); 3],
            },
            Vec3::default(),
        );
        aim(&mut rig, &[1.0, 0.0, 1.0, 0.0], &flat, 0.4, 0.0, 0.6);
        assert_eq!(rig.rot[1], Quat::IDENTITY);
    }
}
