//! `DotSimFpsBody`: dot-player-controller's `DotFpsPhysicsBody` queries (sweep, rest contact,
//! overlap) run natively over the same physics space, to the bit (see `dot-sim-motor`).
//!
//! A hit comes back as a `PackedFloat64Array`, empty for a miss, else
//! `[fraction, nx, ny, nz, px, py, pz, depth, id_lo, id_hi]`: the collider id in two 32-bit
//! halves, since an object id need not fit a double.

use dot_sim_motor::slide::{Aabb as SlideBox, Mover, Tun};
use dot_sim_motor::{Body, Contact, Hit, Masks, Space, Vec3 as V3};
use godot::classes::{
    CapsuleShape3D, Os, PhysicsDirectSpaceState3D, PhysicsRayQueryParameters3D,
    PhysicsShapeQueryParameters3D,
};
use godot::prelude::*;

/// The engine side of [`Space`]: one capsule and one set of query parameters, written only when a
/// value changes (a property write on an engine object is a call).
struct GdSpace {
    space: Option<Gd<PhysicsDirectSpaceState3D>>,
    shape: Gd<CapsuleShape3D>,
    params: Gd<PhysicsShapeQueryParameters3D>,
    ray: Gd<PhysicsRayQueryParameters3D>,
    ray_mask: i64,
    at: Option<V3>,
    moving: bool,
}

#[inline]
fn gv(v: V3) -> Vector3 {
    Vector3::new(v.x, v.y, v.z)
}

#[inline]
fn mv(v: Vector3) -> V3 {
    V3::new(v.x, v.y, v.z)
}

fn contact(d: &VarDictionary, normal_key: &str, point_key: &str) -> Option<Contact> {
    if d.is_empty() {
        return None;
    }
    let normal = d
        .get(normal_key)
        .and_then(|v| v.try_to::<Vector3>().ok())
        .unwrap_or(Vector3::ZERO);
    let point = d
        .get(point_key)
        .and_then(|v| v.try_to::<Vector3>().ok())
        .unwrap_or(Vector3::ZERO);
    let collider_id = d
        .get("collider_id")
        .and_then(|v| v.try_to::<i64>().ok())
        .unwrap_or(0);
    Some(Contact {
        normal: mv(normal),
        point: mv(point),
        collider_id,
    })
}

impl GdSpace {
    fn new() -> Self {
        let shape = CapsuleShape3D::new_gd();
        let mut params = PhysicsShapeQueryParameters3D::new_gd();
        params.set_shape(&shape);
        params.set_collide_with_bodies(true);
        params.set_collide_with_areas(false);
        let mut ray = PhysicsRayQueryParameters3D::new_gd();
        ray.set_collide_with_areas(false);
        Self {
            space: None,
            shape,
            params,
            ray,
            ray_mask: -1,
            at: None,
            moving: false,
        }
    }

    fn place(&mut self, at: V3, motion: Option<V3>) {
        if self.at != Some(at) {
            self.params
                .set_transform(Transform3D::new(Basis::IDENTITY, gv(at)));
            self.at = Some(at);
        }
        match motion {
            Some(m) => {
                self.params.set_motion(gv(m));
                self.moving = true;
            }
            None if self.moving => {
                self.params.set_motion(Vector3::ZERO);
                self.moving = false;
            }
            None => {}
        }
    }
}

impl Space for GdSpace {
    fn set_radius(&mut self, radius: f32) {
        self.shape.set_radius(radius);
    }

    fn set_height(&mut self, height: f32) {
        self.shape.set_height(height);
    }

    fn set_mask(&mut self, mask: i64) {
        self.params.set_collision_mask(mask as u32);
    }

    fn cast(&mut self, from: V3, motion: V3) -> Option<(f32, f32)> {
        self.place(from, Some(motion));
        let space = self.space.as_mut()?;
        let r = space.cast_motion(&self.params);
        let s = r.as_slice();
        if s.len() < 2 {
            return None;
        }
        Some((s[0], s[1]))
    }

    fn rest(&mut self, at: V3) -> Option<Contact> {
        self.place(at, None);
        let space = self.space.as_ref()?;
        contact(&space.get_rest_info(&self.params), "normal", "point")
    }

    fn overlap(&mut self, at: V3) -> bool {
        self.place(at, None);
        let Some(space) = self.space.as_mut() else {
            return false;
        };
        !space
            .intersect_shape_ex(&self.params)
            .max_results(1)
            .done()
            .is_empty()
    }

    fn ray(&mut self, from: V3, to: V3, mask: i64) -> Option<Contact> {
        self.ray.set_from(gv(from));
        self.ray.set_to(gv(to));
        if mask != self.ray_mask {
            self.ray.set_collision_mask(mask as u32);
            self.ray_mask = mask;
        }
        let space = self.space.as_mut()?;
        contact(&space.intersect_ray(&self.ray), "normal", "position")
    }
}

/// A collision body for one player's motor, over a physics space the script hands it.
#[derive(GodotClass)]
#[class(base = RefCounted)]
pub struct DotSimFpsBody {
    body: Body,
    space: GdSpace,
    boxes: Vec<SlideBox>,
    /// The engine checks a slide normal only in debug builds; the script's answer follows that.
    math_checks: bool,
}

#[godot_api]
impl IRefCounted for DotSimFpsBody {
    fn init(_base: Base<RefCounted>) -> Self {
        Self {
            body: Body::default(),
            space: GdSpace::new(),
            boxes: Vec::new(),
            math_checks: Os::singleton().is_debug_build(),
        }
    }
}

fn packed(h: &Hit) -> PackedFloat64Array {
    if !h.hit {
        return PackedFloat64Array::new();
    }
    let id = h.collider_id as u64;
    PackedFloat64Array::from(
        &[
            h.fraction,
            h.normal.x as f64,
            h.normal.y as f64,
            h.normal.z as f64,
            h.point.x as f64,
            h.point.y as f64,
            h.point.z as f64,
            h.depth,
            (id & 0xffff_ffff) as f64,
            (id >> 32) as f64,
        ][..],
    )
}

#[godot_api]
impl DotSimFpsBody {
    /// The physics space queried; a new one forgets the rest memo.
    #[func]
    fn set_space(&mut self, space: Option<Gd<PhysicsDirectSpaceState3D>>) {
        self.space.space = space;
        self.body.invalidate();
    }

    /// Bodies left out of every query (the player's own collider); forgets the rest memo.
    #[func]
    fn set_exclude(&mut self, exclude: Array<Rid>) {
        self.space.params.set_exclude(&exclude);
        self.space.ray.set_exclude(&exclude);
        self.body.invalidate();
    }

    /// Whether a motor tick may answer a repeated rest query from memory (`DotFpsBody.perf_caches`).
    #[func]
    fn set_perf_caches(&mut self, on: bool) {
        self.body.perf_caches = on;
    }

    #[func]
    fn begin_tick(&mut self) {
        self.body.begin_tick();
    }

    #[func]
    fn end_tick(&mut self) {
        self.body.end_tick();
    }

    /// `DotFpsPhysicsBody.sweep`.
    #[func]
    fn sweep(
        &mut self,
        from: Vector3,
        motion: Vector3,
        height: f64,
        radius: f64,
        collision_mask: i64,
        pass_out_mask: i64,
    ) -> PackedFloat64Array {
        let m = Masks {
            collision: collision_mask,
            pass_out: pass_out_mask,
        };
        let h = self
            .body
            .sweep(&mut self.space, mv(from), mv(motion), height, radius, m);
        packed(&h)
    }

    /// `DotFpsPhysicsBody.rest_contact`.
    #[func]
    fn rest_contact(
        &mut self,
        at: Vector3,
        height: f64,
        radius: f64,
        collision_mask: i64,
        pass_out_mask: i64,
    ) -> PackedFloat64Array {
        let m = Masks {
            collision: collision_mask,
            pass_out: pass_out_mask,
        };
        let h = self
            .body
            .rest_contact(&mut self.space, mv(at), height, radius, m);
        packed(&h)
    }

    /// `DotFpsPhysicsBody.overlaps`.
    #[func]
    fn overlaps(
        &mut self,
        at: Vector3,
        height: f64,
        radius: f64,
        collision_mask: i64,
        pass_out_mask: i64,
    ) -> bool {
        let m = Masks {
            collision: collision_mask,
            pass_out: pass_out_mask,
        };
        self.body
            .overlaps(&mut self.space, mv(at), height, radius, m)
    }

    /// `DotFpsMotor._move` (mode 0, `flag` = walking) or `move_and_slide` (mode 1, `flag` = step),
    /// against the map and, when `has_hulls`, the boxes in `hulls` (`DotFpsHullBody`).
    /// `tun` is `[max_slide_iterations, crease_slide, skin_width, max_slide_planes, step_height,
    /// radius, max_slope_radians]`. Returns `[px, py, pz, vx, vy, vz, travelled, flags,
    /// crease_legs, split_legs, world_touches, hull_blocks, hull_stops]`, flags being
    /// blocked | ran_out << 1 | stopped_on_duplicate << 2 | met_steep_face << 3.
    #[func]
    #[allow(clippy::too_many_arguments)]
    fn move_slide(
        &mut self,
        from: Vector3,
        velocity: Vector3,
        motion: Vector3,
        height: f64,
        mode: i64,
        flag: bool,
        tun: PackedFloat64Array,
        has_hulls: bool,
        hulls: VarArray,
        collision_mask: i64,
        pass_out_mask: i64,
    ) -> PackedFloat64Array {
        let p = tun.as_slice();
        if p.len() < 7 {
            return PackedFloat64Array::new();
        }
        let t = Tun {
            max_slide_iterations: p[0] as i64,
            crease_slide: p[1] != 0.0,
            skin_width: p[2],
            max_slide_planes: p[3] as i64,
            step_height: p[4],
            radius: p[5],
            max_slope: p[6],
            math_checks: self.math_checks,
        };
        self.boxes.clear();
        if has_hulls {
            for v in hulls.iter_shared() {
                let b = v.try_to::<Aabb>().unwrap_or_default();
                self.boxes.push(SlideBox {
                    pos: mv(b.position),
                    size: mv(b.size),
                });
            }
        }
        let m = Masks {
            collision: collision_mask,
            pass_out: pass_out_mask,
        };
        let boxes = if has_hulls {
            Some(&self.boxes[..])
        } else {
            None
        };
        let mut mover = Mover::new(&mut self.body, &mut self.space, m, boxes, t);
        let (from, velocity, motion) = (mv(from), mv(velocity), mv(motion));
        let r = if mode == 0 {
            mover.move_walking(from, velocity, motion, height, flag)
        } else {
            mover.move_and_slide(from, velocity, motion, height, flag)
        };
        let c = mover.counters;
        let flags = r.blocked as i64
            | (r.ran_out as i64) << 1
            | (r.stopped_on_duplicate as i64) << 2
            | (r.met_steep_face as i64) << 3;
        PackedFloat64Array::from(
            &[
                r.position.x as f64,
                r.position.y as f64,
                r.position.z as f64,
                r.velocity.x as f64,
                r.velocity.y as f64,
                r.velocity.z as f64,
                r.travelled,
                flags as f64,
                c.crease_leg_ticks as f64,
                c.split_leg_ticks as f64,
                c.world_touches as f64,
                c.hull_blocks as f64,
                c.hull_stops as f64,
            ][..],
        )
    }

    /// Queries counted as the script counts them; the script resets it each tick.
    #[func]
    fn query_count(&self) -> i64 {
        self.body.query_count
    }

    #[func]
    fn set_query_count(&mut self, n: i64) {
        self.body.query_count = n;
    }

    #[func]
    fn flipped_normals(&self) -> i64 {
        self.body.flipped_normals
    }

    #[func]
    fn rest_memo_hits(&self) -> i64 {
        self.body.rest_memo_hits
    }
}
