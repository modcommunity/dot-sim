//! `DotSimFigurePose`: a humanoid figure's pose layers after the clip (dot-player-char's aim bend,
//! gun arm and support arm) over a `Skeleton3D`, one call per layer instead of hundreds of script
//! operations. The maths is `dot_sim_figure`'s; this file only moves values across.

use dot_sim_figure::{self as fig, Carry, CarryRig, Quat, Rig, Vec3 as V3, Xform};
use godot::classes::Skeleton3D;
use godot::prelude::*;

/// Scalars in [`DotSimFigurePose::carry`]'s `values`, in order.
const CARRY_VALUES: usize = 15;
/// Vectors in its `stance`: five grips, walk sway, run sway, the foregrip.
const CARRY_VECTORS: usize = 8;

fn v3(v: Vector3) -> V3 {
    V3::new(v.x, v.y, v.z)
}

fn quat(q: Quaternion) -> Quat {
    Quat::new(q.x, q.y, q.z, q.w)
}

fn xform(t: Transform3D) -> Xform {
    let r = t.basis.rows;
    fig::Xform::new(
        fig::Basis {
            rows: [v3(r[0]), v3(r[1]), v3(r[2])],
        },
        v3(t.origin),
    )
}

/// A length-prefixed chain at the start of `c`, and what follows it.
fn split_chain(c: &[i32]) -> Option<(&[i32], &[i32])> {
    let (&len, rest) = c.split_first()?;
    let len = usize::try_from(len)
        .ok()
        .filter(|&l| l > 0 && l <= rest.len())?;
    Some(rest.split_at(len))
}

/// The engine's skeleton behind [`Rig`].
struct Skel<'a>(&'a mut Gd<Skeleton3D>);

impl Rig for Skel<'_> {
    fn pose(&self, bone: i32) -> Xform {
        xform(self.0.get_bone_pose(bone))
    }

    fn rotation(&self, bone: i32) -> Quat {
        quat(self.0.get_bone_pose_rotation(bone))
    }

    fn parent(&self, bone: i32) -> i32 {
        self.0.get_bone_parent(bone)
    }

    fn set_rotation(&mut self, bone: i32, q: Quat) {
        self.0
            .set_bone_pose_rotation(bone, Quaternion::new(q.x, q.y, q.z, q.w));
    }
}

#[derive(GodotClass)]
#[class(base = RefCounted, init)]
pub struct DotSimFigurePose {}

#[godot_api]
impl DotSimFigurePose {
    /// `DotHumanoidAim.apply_plan`: `plan` four numbers a bone (index, parent, share, arm 0 or 1).
    #[func]
    fn aim(
        &self,
        mut skeleton: Gd<Skeleton3D>,
        plan: PackedFloat64Array,
        to_figure: Transform3D,
        pitch: f64,
        sprint_blend: f64,
        sprint_drop: f64,
    ) {
        fig::aim(
            &mut Skel(&mut skeleton),
            plan.as_slice(),
            &xform(to_figure),
            pitch,
            sprint_blend,
            sprint_drop,
        );
    }

    /// `DotHumanoidIk.two_bone_under` under `chain_pose(chain)`: `bones` upper, lower, end.
    #[func]
    fn two_bone_chain(
        &self,
        mut skeleton: Gd<Skeleton3D>,
        bones: PackedInt32Array,
        chain: PackedInt32Array,
        target: Vector3,
        pole: Vector3,
        weight: f64,
    ) {
        let b = bones.as_slice();
        if b.len() < 3 {
            return;
        }
        let mut rig = Skel(&mut skeleton);
        let parent = fig::chain_pose(&rig, chain.as_slice());
        fig::two_bone_under(
            &mut rig,
            [b[0], b[1], b[2]],
            &parent,
            v3(target),
            v3(pole),
            weight,
        );
    }

    /// `DotHumanoidLegs.on_floor` past its checks. `chains`: per leg the upper leg's `chain_to`
    /// length then that chain, the lower leg's length then its chain; `ends`: per leg the ankle in
    /// the lower leg's frame. Answers each bent bone with its rotation from before (bone, x, y, z,
    /// w), for the script's restore.
    #[func]
    fn floor(
        &self,
        mut skeleton: Gd<Skeleton3D>,
        chains: PackedInt32Array,
        ends: PackedVector3Array,
        to_figure: Transform3D,
        floor_y: PackedFloat32Array,
        raised: f64,
    ) -> PackedFloat32Array {
        let (c, e) = (chains.as_slice(), ends.as_slice());
        let mut legs = Vec::with_capacity(e.len());
        let mut rest = c;
        for &end in e {
            let Some((upper, after)) = split_chain(rest) else {
                break;
            };
            let Some((lower, after)) = split_chain(after) else {
                break;
            };
            rest = after;
            legs.push(fig::FloorLeg {
                end_local: v3(end),
                upper_chain: upper,
                lower_chain: lower,
            });
        }
        let mut saved = Vec::new();
        fig::on_floor(
            &mut Skel(&mut skeleton),
            &legs,
            &xform(to_figure),
            floor_y.as_slice(),
            raised,
            &mut saved,
        );
        let out: Vec<f32> = saved
            .iter()
            .flat_map(|&(bone, q)| [bone as f32, q.x, q.y, q.z, q.w])
            .collect();
        PackedFloat32Array::from(out.as_slice())
    }

    /// `DotHumanoidCarryPose.apply` past its checks. `bones`: upper arm, lower arm, hand, shoulder,
    /// then the chain from the root to the chest; `frames`: hand frame, its inverse, chest at the
    /// held pose; `stance`: five grips, walk and run sway, the foregrip; `turns`: five quaternions
    /// as x, y, z, w; `values`: follow, kick, kick climb, weight, sprint, aim, reload, swap, kick
    /// now, reload time, phase, pitch, size, speed, support on. Answers the other hand's point in
    /// skeleton space, or `Vector3.INF`.
    #[func]
    #[allow(clippy::too_many_arguments)]
    fn carry(
        &self,
        mut skeleton: Gd<Skeleton3D>,
        bones: PackedInt32Array,
        to_figure: Transform3D,
        hand_frame: Transform3D,
        hand_frame_inv: Transform3D,
        chest_ref: Transform3D,
        stance: PackedVector3Array,
        turns: PackedFloat32Array,
        values: PackedFloat64Array,
    ) -> Vector3 {
        let inf = Vector3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY);
        let (b, s, t, v) = (
            bones.as_slice(),
            stance.as_slice(),
            turns.as_slice(),
            values.as_slice(),
        );
        if b.len() < 5 || s.len() < CARRY_VECTORS || t.len() < 20 || v.len() < CARRY_VALUES {
            return inf;
        }
        let m = CarryRig {
            arm: [b[0], b[1], b[2]],
            shoulder: b[3],
            hand_frame: xform(hand_frame),
            hand_frame_inv: xform(hand_frame_inv),
            chest_ref: xform(chest_ref),
        };
        let q = |i: usize| Quat::new(t[i * 4], t[i * 4 + 1], t[i * 4 + 2], t[i * 4 + 3]);
        let c = Carry {
            grips: [v3(s[0]), v3(s[1]), v3(s[2]), v3(s[3]), v3(s[4])],
            turns: [q(0), q(1), q(2), q(3), q(4)],
            walk_sway: v3(s[5]),
            run_sway: v3(s[6]),
            support: v3(s[7]),
            follow: v[0],
            kick: v[1],
            kick_climb: v[2],
            weight: v[3],
            sprint: v[4],
            aim: v[5],
            reload: v[6],
            swap: v[7],
            kick_now: v[8],
            reload_time: v[9],
            phase: v[10],
            pitch: v[11],
            size: v[12],
            speed: v[13],
            support_on: v[14] != 0.0,
        };
        match fig::carry(&mut Skel(&mut skeleton), &c, &m, &b[4..], &xform(to_figure)) {
            Some(p) => Vector3::new(p.x, p.y, p.z),
            None => inf,
        }
    }
}
