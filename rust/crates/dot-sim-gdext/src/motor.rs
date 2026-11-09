//! `DotSimFpsBody`: dot-player-controller's `DotFpsPhysicsBody` queries (sweep, rest contact,
//! overlap) run natively over the same physics space, to the bit (see `dot-sim-motor`).
//!
//! A hit comes back as a `PackedFloat64Array`, empty for a miss, else
//! `[fraction, nx, ny, nz, px, py, pz, depth, id_lo, id_hi]`: the collider id in two 32-bit
//! halves, since an object id need not fit a double.

use dot_sim_motor::full::{self, FullIn, FullTun, MoveCounters, Moves};
use dot_sim_motor::slide::{Aabb as SlideBox, Mover, Tun};
use dot_sim_motor::tick::{self, Post, State, TickCounters, TickTun};
use dot_sim_motor::{Body, Contact, Hit, Masks, Space, Vec3 as V3};
use godot::classes::{
    CapsuleShape3D, Os, PhysicsDirectSpaceState3D, PhysicsRayQueryParameters3D,
    PhysicsShapeQueryParameters3D,
};
use godot::prelude::*;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::Instant;

/// Whether the engine queries and the whole tick are timed (`DotSimFpsBody.set_profile`): off,
/// nothing but one relaxed load per query.
static PROFILE: AtomicBool = AtomicBool::new(false);
/// Per slot a count then nanoseconds: cast, rest, overlap, ray, then the `motor_tick` calls.
static PROF: [AtomicU64; 2 * PROF_SLOTS] = [const { AtomicU64::new(0) }; 2 * PROF_SLOTS];
const PROF_SLOTS: usize = 5;
const PROF_TICK: usize = 4;

#[inline]
fn prof_start() -> Option<Instant> {
    PROFILE.load(Ordering::Relaxed).then(Instant::now)
}

#[inline]
fn prof_end(slot: usize, at: Option<Instant>) {
    if let Some(t) = at {
        PROF[2 * slot].fetch_add(1, Ordering::Relaxed);
        PROF[2 * slot + 1].fetch_add(t.elapsed().as_nanos() as u64, Ordering::Relaxed);
    }
}

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
        let at = prof_start();
        let r = space.cast_motion(&self.params);
        prof_end(0, at);
        let s = r.as_slice();
        if s.len() < 2 {
            return None;
        }
        Some((s[0], s[1]))
    }

    fn rest(&mut self, at: V3) -> Option<Contact> {
        self.place(at, None);
        let space = self.space.as_ref()?;
        let at = prof_start();
        let r = contact(&space.get_rest_info(&self.params), "normal", "point");
        prof_end(1, at);
        r
    }

    fn overlap(&mut self, at: V3) -> bool {
        self.place(at, None);
        let Some(space) = self.space.as_mut() else {
            return false;
        };
        let at = prof_start();
        let r = !space
            .intersect_shape_ex(&self.params)
            .max_results(1)
            .done()
            .is_empty();
        prof_end(2, at);
        r
    }

    fn ray(&mut self, from: V3, to: V3, mask: i64) -> Option<Contact> {
        self.ray.set_from(gv(from));
        self.ray.set_to(gv(to));
        if mask != self.ray_mask {
            self.ray.set_collision_mask(mask as u32);
            self.ray_mask = mask;
        }
        let space = self.space.as_mut()?;
        let at = prof_start();
        let r = contact(&space.intersect_ray(&self.ray), "normal", "position");
        prof_end(3, at);
        r
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

/// Tick tunables: the slide's seven (see `move_slide`), then `jump_velocity, ground_snap,
/// ground_snap_distance, crest_launch_speed_scale, max_speed, stop_speed, air_friction,
/// max_velocity, backward_speed_scale, max_air_wish_speed, wall_slide_speed,
/// wall_run_vertical_damping`.
const TICK_TUN_LEN: usize = 19;

fn slide_tun(p: &[f64], math_checks: bool) -> Tun {
    Tun {
        max_slide_iterations: p[0] as i64,
        crease_slide: p[1] != 0.0,
        skin_width: p[2],
        max_slide_planes: p[3] as i64,
        step_height: p[4],
        radius: p[5],
        max_slope: p[6],
        math_checks,
    }
}

fn tick_tun(p: &[f64]) -> TickTun {
    TickTun {
        jump_velocity: p[7],
        ground_snap: p[8] != 0.0,
        ground_snap_distance: p[9],
        crest_launch_speed_scale: p[10],
        max_speed: p[11],
        stop_speed: p[12],
        air_friction: p[13],
        max_velocity: p[14],
        backward_speed_scale: p[15],
        max_air_wish_speed: p[16],
        wall_slide_speed: p[17],
        wall_run_vertical_damping: p[18],
    }
}

#[inline]
fn v3(p: &[f64], i: usize) -> V3 {
    V3::new(p[i] as f32, p[i + 1] as f32, p[i + 2] as f32)
}

/// The state block both tick halves take: `[px, py, pz, vx, vy, vz, nx, ny, nz, mode, id_lo,
/// id_hi, height]`.
const STATE_LEN: usize = 13;

fn read_state(p: &[f64]) -> State {
    State {
        position: v3(p, 0),
        velocity: v3(p, 3),
        ground_normal: v3(p, 6),
        mode: p[9] as i64,
        ground_id: ((p[10] as u64) | ((p[11] as u64) << 32)) as i64,
        height: p[12],
        surface_cleared: false,
    }
}

/// `[px, py, pz, vx, vy, vz, nx, ny, nz, mode, id_lo, id_hi, flags, depenetrated, embedded,
/// lifted, stuck, duplicate_plane, crease_legs, split_legs, world_touches, hull_blocks,
/// hull_stops]`.
fn tick_out(
    s: &State,
    flags: i64,
    c: &TickCounters,
    m: &dot_sim_motor::slide::Counters,
) -> PackedFloat64Array {
    let id = s.ground_id as u64;
    PackedFloat64Array::from(
        &[
            s.position.x as f64,
            s.position.y as f64,
            s.position.z as f64,
            s.velocity.x as f64,
            s.velocity.y as f64,
            s.velocity.z as f64,
            s.ground_normal.x as f64,
            s.ground_normal.y as f64,
            s.ground_normal.z as f64,
            s.mode as f64,
            (id & 0xffff_ffff) as f64,
            (id >> 32) as f64,
            flags as f64,
            c.depenetrated as f64,
            c.embedded as f64,
            c.lifted as f64,
            c.stuck as f64,
            c.duplicate_plane as f64,
            m.crease_leg_ticks as f64,
            m.split_leg_ticks as f64,
            m.world_touches as f64,
            m.hull_blocks as f64,
            m.hull_stops as f64,
        ][..],
    )
}

/// Reads a packed array front to back.
struct Rd<'a> {
    p: &'a [f64],
    i: usize,
}

impl Rd<'_> {
    fn f(&mut self) -> f64 {
        self.i += 1;
        self.p[self.i - 1]
    }

    fn b(&mut self) -> bool {
        self.f() != 0.0
    }

    fn n(&mut self) -> i64 {
        self.f() as i64
    }

    fn v(&mut self) -> V3 {
        V3::new(self.f() as f32, self.f() as f32, self.f() as f32)
    }
}

/// The move tunables after the tick ones, in [`FullTun`]'s field order.
const FULL_TUN_LEN: usize = 56;

fn full_tun(p: &[f64]) -> FullTun {
    let mut r = Rd { p, i: TICK_TUN_LEN };
    FullTun {
        can_jump: r.b(),
        auto_hop: r.b(),
        jump_buffer_time: r.f(),
        jump_cooldown: r.f(),
        coyote_time: r.f(),
        bhop_speed_cap_scale: r.f(),
        jump_adds_to_velocity: r.b(),
        launch_enabled: r.b(),
        launch_button: r.n(),
        launch_from_air: r.b(),
        launch_velocity: r.f(),
        launch_forward: r.f(),
        launch_cooldown: r.f(),
        dash_enabled: r.b(),
        dash_button: r.n(),
        dash_from_air: r.b(),
        dash_speed: r.f(),
        dash_cooldown: r.f(),
        dash_lift: r.f(),
        slide_enabled: r.b(),
        slide_duration: r.f(),
        slide_end_speed: r.f(),
        can_crouch: r.b(),
        slide_requires_sprint: r.b(),
        slide_min_speed: r.f(),
        slide_boost: r.f(),
        slide_max_speed: r.f(),
        slide_cooldown: r.f(),
        friction: r.f(),
        slide_friction: r.f(),
        accelerate: r.f(),
        slide_accelerate: r.f(),
        wall_run_enabled: r.b(),
        wall_jump_enabled: r.b(),
        wall_run_auto: r.b(),
        wall_run_on_jump: r.b(),
        wall_run_jump_min_speed: r.f(),
        wall_run_min_speed: r.f(),
        wall_max_angle_from_vertical: r.f(),
        wall_reach: r.f(),
        wall_run_max_time: r.f(),
        wall_run_end_speed: r.f(),
        wall_run_requires_forward: r.b(),
        wall_run_max_fall_speed: r.f(),
        wall_run_max_approach_angle: r.f(),
        wall_run_min_height: r.f(),
        wall_run_min_length: r.f(),
        wall_run_cooldown: r.f(),
        wall_run_jump_max_angle: r.f(),
        wall_run_turn_view: r.b(),
        wall_run_turn_view_max_angle: r.f(),
        wall_jump_max_per_wall: r.n(),
        wall_jump_max_per_air: r.n(),
        wall_jump_look_blend: r.f(),
        wall_jump_push_speed: r.f(),
        wall_jump_up_speed: r.f(),
    }
}

/// The inputs after the state block in `motor_tick`.
const FULL_IN_LEN: usize = 42;

fn full_in(p: &[f64]) -> (FullIn, Moves) {
    let mut r = Rd { p, i: STATE_LEN };
    let mut inp = FullIn {
        rest_ok: r.b(),
        rest_position: r.v(),
        delta: r.f(),
        buttons: r.n(),
        previous_buttons: r.n(),
        move_x: r.f(),
        move_y: r.f(),
        yaw: r.f(),
        ..FullIn::default()
    };
    let effects = r.n();
    inp.deny_move = effects & 1 != 0;
    inp.deny_jump = effects & 2 != 0;
    inp.deny_crouch = effects & 4 != 0;
    inp.effect_jump = r.f();
    inp.launch = r.f();
    inp.speed_base = r.f();
    inp.effect_max_speed = r.f();
    inp.surface_max_speed_scale = r.f();
    inp.surface_friction = r.f();
    inp.effect_friction = r.f();
    inp.surface_accelerate = r.f();
    inp.effect_accelerate = r.f();
    inp.air_accel = r.f();
    inp.push = r.v();
    inp.gravity = r.f();
    inp.wall_run_gravity = r.f();
    let mv = Moves {
        time_since_jump: r.f(),
        time_since_grounded: r.f(),
        time_since_jump_pressed: r.f(),
        slide_time: r.f(),
        slide_cooldown_left: r.f(),
        launch_cooldown_left: r.f(),
        dash_cooldown_left: r.f(),
        wall_run_cooldown_left: r.f(),
        wall_run_time: r.f(),
        wall_run_spent: r.b(),
        wall_jump_count: r.n(),
        wall_jumps_in_air: r.n(),
        wall_jump_heading: r.n(),
        wall_normal: r.v(),
    };
    (inp, mv)
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

    /// Fills the hull boxes: `hulls_mode` 0 none, 1 the boxes in `hulls`, 2 the ones the last call
    /// took (the hull body gathers once a tick, so the second half of a tick need not send them).
    fn take_hulls(&mut self, hulls_mode: i64, hulls: &VarArray) -> bool {
        match hulls_mode {
            1 => {
                self.boxes.clear();
                for v in hulls.iter_shared() {
                    let b = v.try_to::<Aabb>().unwrap_or_default();
                    self.boxes.push(SlideBox {
                        pos: mv(b.position),
                        size: mv(b.size),
                    });
                }
                true
            }
            2 => true,
            _ => false,
        }
    }

    /// The start of a motor tick (`DotFpsMotor._simulate` from `_depenetrate` to the start
    /// ground check), for a plain GROUND or AIR walker without surfaces. `state` is the state
    /// block (see `tick_out`) then `[rest_ok, rest_px, rest_py, rest_pz]`, `tun` the tick
    /// tunables. Flags: start_settled | reused << 1 | surface_cleared << 2.
    #[func]
    #[allow(clippy::too_many_arguments)]
    fn motor_pre(
        &mut self,
        state: PackedFloat64Array,
        tun: PackedFloat64Array,
        hulls_mode: i64,
        hulls: VarArray,
        collision_mask: i64,
        pass_out_mask: i64,
    ) -> PackedFloat64Array {
        let (sp, tp) = (state.as_slice(), tun.as_slice());
        if sp.len() < STATE_LEN + 4 || tp.len() < TICK_TUN_LEN {
            return PackedFloat64Array::new();
        }
        let t = slide_tun(tp, self.math_checks);
        let tt = tick_tun(tp);
        let has_hulls = self.take_hulls(hulls_mode, &hulls);
        let m = Masks {
            collision: collision_mask,
            pass_out: pass_out_mask,
        };
        let boxes = if has_hulls {
            Some(&self.boxes[..])
        } else {
            None
        };
        let mut s = read_state(sp);
        let rest_ok = sp[STATE_LEN] != 0.0;
        let rest_position = v3(sp, STATE_LEN + 1);
        let mut c = TickCounters::default();
        let mut mover = Mover::new(&mut self.body, &mut self.space, m, boxes, t);
        let out = tick::pre(&mut mover, &mut s, &tt, rest_ok, rest_position, &mut c);
        let flags =
            out.start_settled as i64 | (out.reused as i64) << 1 | (s.surface_cleared as i64) << 2;
        tick_out(&s, flags, &c, &mover.counters)
    }

    /// The rest of a motor tick after the script's jump, wall and slide moves: friction, the wish
    /// move, the push or gravity and wall slide, the clamp, the slide and the end ground check.
    /// `state` is the state block then `[delta, flags, move_x, move_y, yaw, speed_base,
    /// effect_max_speed, surface_max_speed_scale, friction_base, surface_friction,
    /// effect_friction, ground_accel, air_accel, push x y z, gravity, start px py pz, start vx vy
    /// vz]`, flags being jumped | sliding << 1 | deny_move << 2 | wall_slide_ok << 3 |
    /// start_settled << 4. Out flags: surface_cleared << 2 | stamp << 3 | nonfinite << 4.
    #[func]
    #[allow(clippy::too_many_arguments)]
    fn motor_post(
        &mut self,
        state: PackedFloat64Array,
        tun: PackedFloat64Array,
        hulls_mode: i64,
        hulls: VarArray,
        collision_mask: i64,
        pass_out_mask: i64,
    ) -> PackedFloat64Array {
        let (sp, tp) = (state.as_slice(), tun.as_slice());
        if sp.len() < STATE_LEN + 23 || tp.len() < TICK_TUN_LEN {
            return PackedFloat64Array::new();
        }
        let t = slide_tun(tp, self.math_checks);
        let tt = tick_tun(tp);
        let has_hulls = self.take_hulls(hulls_mode, &hulls);
        let m = Masks {
            collision: collision_mask,
            pass_out: pass_out_mask,
        };
        let boxes = if has_hulls {
            Some(&self.boxes[..])
        } else {
            None
        };
        let mut s = read_state(sp);
        let q = &sp[STATE_LEN..];
        let f = q[1] as i64;
        let p = Post {
            delta: q[0],
            jumped: f & 1 != 0,
            sliding: f & 2 != 0,
            deny_move: f & 4 != 0,
            wall_slide_ok: f & 8 != 0,
            start_settled: f & 16 != 0,
            move_x: q[2],
            move_y: q[3],
            yaw: q[4],
            speed_base: q[5],
            effect_max_speed: q[6],
            surface_max_speed_scale: q[7],
            friction_base: q[8],
            surface_friction: q[9],
            effect_friction: q[10],
            ground_accel: q[11],
            air_accel: q[12],
            push: v3(q, 13),
            gravity: q[16],
            start_position: v3(q, 17),
            start_velocity: v3(q, 20),
            wall_running: false,
            wall_run_gravity: 0.0,
        };
        let mut c = TickCounters::default();
        let mut mover = Mover::new(&mut self.body, &mut self.space, m, boxes, t);
        let out = tick::post(&mut mover, &mut s, &tt, &p, &mut c);
        let flags =
            (s.surface_cleared as i64) << 2 | (out.stamp as i64) << 3 | (out.nonfinite as i64) << 4;
        tick_out(&s, flags, &c, &mover.counters)
    }

    /// A whole motor tick from `_depenetrate` to the end ground check (see `dot_sim_motor::full`).
    /// `state` is the state block then the inputs and timers `full_in` reads, `tun` the tick
    /// tunables then `full_tun`'s. Answers `tick_out`, then the timers in `full_in`'s order,
    /// `view_turn`, and the slide, launch, dash, wall run and wall jump counts.
    #[func]
    #[allow(clippy::too_many_arguments)]
    fn motor_tick(
        &mut self,
        state: PackedFloat64Array,
        tun: PackedFloat64Array,
        hulls_mode: i64,
        hulls: VarArray,
        collision_mask: i64,
        pass_out_mask: i64,
    ) -> PackedFloat64Array {
        let at = prof_start();
        let (sp, tp) = (state.as_slice(), tun.as_slice());
        if sp.len() < STATE_LEN + FULL_IN_LEN || tp.len() < TICK_TUN_LEN + FULL_TUN_LEN {
            return PackedFloat64Array::new();
        }
        let t = slide_tun(tp, self.math_checks);
        let tt = tick_tun(tp);
        let ft = full_tun(tp);
        let has_hulls = self.take_hulls(hulls_mode, &hulls);
        let m = Masks {
            collision: collision_mask,
            pass_out: pass_out_mask,
        };
        let boxes = if has_hulls {
            Some(&self.boxes[..])
        } else {
            None
        };
        let mut s = read_state(sp);
        let (inp, mut mv) = full_in(sp);
        let mut c = TickCounters::default();
        let mut mc = MoveCounters::default();
        let mut mover = Mover::new(&mut self.body, &mut self.space, m, boxes, t);
        let out = full::tick(&mut mover, &mut s, &tt, &ft, &inp, &mut mv, &mut c, &mut mc);
        let flags = out.pre.start_settled as i64
            | (out.pre.reused as i64) << 1
            | (s.surface_cleared as i64) << 2
            | (out.post.stamp as i64) << 3
            | (out.post.nonfinite as i64) << 4;
        let mut o = tick_out(&s, flags, &c, &mover.counters);
        o.extend_array(&PackedFloat64Array::from(
            &[
                mv.time_since_jump,
                mv.time_since_grounded,
                mv.time_since_jump_pressed,
                mv.slide_time,
                mv.slide_cooldown_left,
                mv.launch_cooldown_left,
                mv.dash_cooldown_left,
                mv.wall_run_cooldown_left,
                mv.wall_run_time,
                mv.wall_run_spent as i64 as f64,
                mv.wall_jump_count as f64,
                mv.wall_jumps_in_air as f64,
                mv.wall_jump_heading as f64,
                mv.wall_normal.x as f64,
                mv.wall_normal.y as f64,
                mv.wall_normal.z as f64,
                out.view_turn,
                mc.slides as f64,
                mc.launches as f64,
                mc.dashes as f64,
                mc.wall_runs as f64,
                mc.wall_jumps as f64,
            ][..],
        ));
        prof_end(PROF_TICK, at);
        o
    }

    /// Times every engine query and every `motor_tick` from now on (all bodies), for a profile run.
    #[func]
    fn set_profile(on: bool) {
        PROFILE.store(on, Ordering::Relaxed);
    }

    /// Per slot the count then the microseconds since the last reset: cast, rest, overlap, ray,
    /// then whole `motor_tick` calls. The tick less the four queries is the motor's own work and
    /// the engine call boundary.
    #[func]
    fn profile_counters() -> PackedInt64Array {
        (0..PROF.len())
            .map(|i| {
                let v = PROF[i].load(Ordering::Relaxed) as i64;
                if i % 2 == 1 { v / 1000 } else { v }
            })
            .collect()
    }

    #[func]
    fn reset_profile() {
        for p in &PROF {
            p.store(0, Ordering::Relaxed);
        }
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
