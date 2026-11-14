//! `DotSimRagdollPose`: a skinned ragdoll's bones put where its parts are, each physics step.
//!
//! The twin of `DotPlayerRagdoll._pose_bones` (dot-player-char): every carried bone's global pose
//! is `(base_inverse * part) * offset`, the part's global transform read from the engine, the
//! product in the engine's f32 order (`dot_sim_math::Xform`), and the bones written through
//! `Skeleton3D.set_bone_global_pose` in the same order as the script writes them. When two parts
//! carry one bone the later part's wins, as the script's dictionary keeps the last write. So every
//! bone pose is the script's to the bit; only the script's dictionary and variant math go.

use dot_sim_math::{Vec3, Xform};
use godot::classes::{Node3D, Skeleton3D};
use godot::prelude::*;

/// One bone write: the bone, the part (an index into the bodies) and the bone's offset in it.
struct Write {
    bone: i32,
    part: usize,
    offset: Xform,
}

/// Poses one ragdoll's skeleton; made per body, planned once its parts carry their bones.
#[derive(GodotClass)]
#[class(base = RefCounted, init)]
pub struct DotSimRagdollPose {
    bodies: Vec<Gd<Node3D>>,
    writes: Vec<Write>,
    /// Scratch: each part's `base_inverse * global_transform` this step.
    at: Vec<Option<Xform>>,
}

#[godot_api]
impl DotSimRagdollPose {
    /// Plans the writes. Entry `i` of [param bones], [param offsets] and [param part_of] is one
    /// part's carried bone (parts in order, each part's bones in order), [param part_of] its index
    /// into [param bodies]; [param order] is the write order (parents first). Returns how many
    /// bones each [method pose] writes.
    #[func]
    fn plan(
        &mut self,
        bodies: Array<Gd<Node3D>>,
        part_of: PackedInt32Array,
        bones: PackedInt32Array,
        offsets: Array<Transform3D>,
        order: PackedInt32Array,
    ) -> i32 {
        self.bodies = bodies.iter_shared().collect();
        self.at = vec![None; self.bodies.len()];
        let parts = part_of.as_slice();
        let picked = write_plan(bones.as_slice(), order.as_slice());
        self.writes = picked
            .into_iter()
            .filter_map(|e| {
                let part = usize::try_from(*parts.get(e)?).ok()?;
                if part >= self.bodies.len() {
                    return None;
                }
                Some(Write {
                    bone: bones.as_slice()[e],
                    part,
                    offset: from_godot(&offsets.get(e)?),
                })
            })
            .collect();
        self.writes.len() as i32
    }

    /// Writes every planned bone's global pose into [param skeleton], parts read where they are
    /// now; [param base_inverse] is the inverse of the skeleton's transform when built.
    #[func]
    fn pose(&mut self, skeleton: Option<Gd<Skeleton3D>>, base_inverse: Transform3D) {
        let Some(mut skeleton) = skeleton else {
            return;
        };
        let base_inverse = from_godot(&base_inverse);
        for (at, body) in self.at.iter_mut().zip(&self.bodies) {
            // A freed part writes nothing: its bones keep their last pose.
            *at = body
                .is_instance_valid()
                .then(|| base_inverse.mul(&from_godot(&body.get_global_transform())));
        }
        for w in &self.writes {
            if let Some(at) = &self.at[w.part] {
                skeleton.set_bone_global_pose(w.bone, to_godot(&at.mul(&w.offset)));
            }
        }
    }
}

/// For each bone of [param order] that an entry carries, the last such entry, in write order.
fn write_plan(bones: &[i32], order: &[i32]) -> Vec<usize> {
    let mut last = std::collections::HashMap::with_capacity(bones.len());
    for (e, &b) in bones.iter().enumerate() {
        last.insert(b, e);
    }
    order.iter().filter_map(|b| last.get(b).copied()).collect()
}

fn from_godot(t: &Transform3D) -> Xform {
    let v = |v: Vector3| Vec3::new(v.x, v.y, v.z);
    Xform {
        rows: [v(t.basis.rows[0]), v(t.basis.rows[1]), v(t.basis.rows[2])],
        origin: v(t.origin),
    }
}

fn to_godot(t: &Xform) -> Transform3D {
    let v = |v: Vec3| Vector3::new(v.x, v.y, v.z);
    Transform3D {
        basis: Basis {
            rows: [v(t.rows[0]), v(t.rows[1]), v(t.rows[2])],
        },
        origin: v(t.origin),
    }
}

#[cfg(test)]
mod tests {
    use super::write_plan;

    #[test]
    fn writes_follow_the_order_and_the_last_carrier_wins() {
        // Entries: bone 3 (part 0), bone 1 (part 0), bone 3 again (part 1), bone 5 (part 1).
        // Order 0..6 writes bone 1 from entry 1, bone 3 from entry 2 (the later part), bone 5 from 3.
        assert_eq!(
            write_plan(&[3, 1, 3, 5], &[0, 1, 2, 3, 4, 5]),
            vec![1, 2, 3]
        );
        // A bone outside the order is never written.
        assert_eq!(write_plan(&[3, 1], &[1]), vec![1]);
        assert!(write_plan(&[], &[0, 1]).is_empty());
    }
}
