//! `DotSimViewArms`: the view arms' elbow clearance scan (dot-player-char `DotViewArmsClear.scan`),
//! bit for bit the script's. Presentation only: it never feeds the simulation.

use dot_sim_arms::{Aabb as Box3, Vec3 as V3, Xform, scan};
use godot::prelude::*;

#[inline]
fn v3(v: Vector3) -> V3 {
    V3::new(v.x, v.y, v.z)
}

/// Stateless; one per view arms node.
#[derive(GodotClass)]
#[class(base = RefCounted, init)]
pub struct DotSimViewArms {}

#[godot_api]
impl DotSimViewArms {
    /// `DotViewArmsClear.scan`'s `[turned.x, turned.y, turned.z, turn, depth]`.
    #[allow(clippy::too_many_arguments)]
    #[func]
    fn clear_pole(
        &self,
        shoulder: Vector3,
        target: Vector3,
        lengths: Vector2,
        pole: Vector3,
        dir: Vector3,
        to_anchor: Transform3D,
        bounds: Aabb,
        up: Vector3,
        prev_turn: f64,
    ) -> PackedFloat64Array {
        let rows = to_anchor.basis.rows;
        let xf = Xform {
            rows: [v3(rows[0]), v3(rows[1]), v3(rows[2])],
            origin: v3(to_anchor.origin),
        };
        let b = Box3 {
            position: v3(bounds.position),
            size: v3(bounds.size),
        };
        let c = scan(
            v3(shoulder),
            v3(target),
            lengths.x,
            lengths.y,
            v3(pole),
            v3(dir),
            &xf,
            &b,
            v3(up),
            prev_turn,
        );
        PackedFloat64Array::from(
            &[
                c.turned.x as f64,
                c.turned.y as f64,
                c.turned.z as f64,
                c.turn,
                c.depth,
            ][..],
        )
    }
}
