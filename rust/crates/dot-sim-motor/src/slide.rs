//! The motor's collide-and-slide: dot-player-controller's `DotFpsMotor._slide`, its duplicate-plane
//! legs, `_slide_with_step` and `_prefer_step`, over a [`Body`] and optionally other players' hulls
//! (`DotFpsHullBody`), spelled operation for operation so a native client and a web client agree to the bit.

use crate::{Body, DOWN, Hit, Masks, Space, UP, Vec3};
use dot_sim_math::{clampf, maxf, minf};

/// `DotFpsMotorBase.SWEEP_TOLERANCE`.
const SWEEP_TOLERANCE: f64 = 0.02;
/// `DotFpsHullBody.HULL_ID_BASE`.
pub const HULL_ID_BASE: i64 = 1 << 24;
/// `DotFpsFlatBody.SWEEP_REJECT_MARGIN`.
const SWEEP_REJECT_MARGIN: f64 = 1e-3;

/// The tunables the slide reads.
#[derive(Clone, Copy, Debug)]
pub struct Tun {
    pub max_slide_iterations: i64,
    pub crease_slide: bool,
    pub skin_width: f64,
    pub max_slide_planes: i64,
    pub step_height: f64,
    pub radius: f64,
    /// `max_slope_radians()`, already in radians.
    pub max_slope: f64,
    /// The engine checks `Vector3.slide`'s normal (debug builds) and answers zero for one not normalized.
    pub math_checks: bool,
}

/// An engine `AABB`: position and size, `f32`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Aabb {
    pub pos: Vec3,
    pub size: Vec3,
}

impl Aabb {
    /// `AABB.intersects`, strict on every face.
    #[inline]
    fn intersects(&self, o: &Aabb) -> bool {
        !(self.pos.x >= o.pos.x + o.size.x
            || self.pos.x + self.size.x <= o.pos.x
            || self.pos.y >= o.pos.y + o.size.y
            || self.pos.y + self.size.y <= o.pos.y
            || self.pos.z >= o.pos.z + o.size.z
            || self.pos.z + self.size.z <= o.pos.z)
    }
}

/// `DotFpsMotor.MoveResult`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MoveResult {
    pub position: Vec3,
    pub velocity: Vec3,
    pub travelled: f64,
    pub blocked: bool,
    pub ran_out: bool,
    pub stopped_on_duplicate: bool,
    pub met_steep_face: bool,
}

/// What a move added to the script's counters.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Counters {
    pub crease_leg_ticks: i64,
    pub split_leg_ticks: i64,
    pub world_touches: i64,
    pub hull_blocks: i64,
    pub hull_stops: i64,
}

const PLANE_ADDED: i32 = 0;
const PLANE_DUPLICATE: i32 = 1;
const PLANE_FULL: i32 = 2;

#[inline]
fn signf(v: f64) -> f64 {
    if v > 0.0 {
        1.0
    } else if v < 0.0 {
        -1.0
    } else {
        0.0
    }
}

#[inline]
fn vmin(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(
        if a.x < b.x { a.x } else { b.x },
        if a.y < b.y { a.y } else { b.y },
        if a.z < b.z { a.z } else { b.z },
    )
}

#[inline]
fn vmax(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(
        if a.x > b.x { a.x } else { b.x },
        if a.y > b.y { a.y } else { b.y },
        if a.z > b.z { a.z } else { b.z },
    )
}

#[inline]
fn comp(v: Vec3, i: usize) -> f32 {
    match i {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}

/// `Vector3(a.x - b.x, 0, a.z - b.z).length()`: the differences are script floats, rounded on construction.
#[inline]
fn flat_distance(a: Vec3, b: Vec3) -> f64 {
    Vec3::new(
        ((a.x as f64) - (b.x as f64)) as f32,
        0.0,
        ((a.z as f64) - (b.z as f64)) as f32,
    )
    .length() as f64
}

#[inline]
fn capsule_centre(feet: Vec3, height: f64) -> Vec3 {
    feet + UP.scale(height * 0.5)
}

/// `DotFpsFlatBody._slab_sweep`: the entry fraction (rounded through `Vector2`) and axis, or none.
fn slab_sweep(start: &Aabb, motion: Vec3, b: &Aabb) -> Option<(f64, usize)> {
    let mut entry = f64::NEG_INFINITY;
    let mut exit_t = f64::INFINITY;
    let mut axis: i32 = -1;
    for i in 0..3 {
        let a_min = comp(start.pos, i) as f64;
        let a_max = comp(start.pos, i) as f64 + comp(start.size, i) as f64;
        let b_min = comp(b.pos, i) as f64;
        let b_max = comp(b.pos, i) as f64 + comp(b.size, i) as f64;
        let d = comp(motion, i) as f64;
        if d.abs() < 1e-9 {
            if a_max <= b_min || a_min >= b_max {
                return None;
            }
            continue;
        }
        let mut t1 = (b_min - a_max) / d;
        let mut t2 = (b_max - a_min) / d;
        if t1 > t2 {
            std::mem::swap(&mut t1, &mut t2);
        }
        if t1 > entry {
            entry = t1;
            axis = i as i32;
        }
        exit_t = minf(exit_t, t2);
        if entry > exit_t {
            return None;
        }
    }
    if axis < 0 || entry < 0.0 || entry > 1.0 {
        return None;
    }
    Some((entry as f32 as f64, axis as usize))
}

/// `DotFpsFlatBody.sweep` over boxes only (no floor, no planes, broad phase on): the hull body's `_near`.
fn boxes_sweep(
    from: Vec3,
    motion: Vec3,
    height: f64,
    radius: f64,
    boxes: &[Aabb],
) -> Option<(Hit, usize)> {
    if (motion.length_squared() as f64) <= 0.0 {
        return None;
    }
    let mut best = 1.0f64;
    let mut best_normal = UP;
    let mut best_index = 0usize;
    let mut found = false;

    let half = Vec3::new(radius as f32, (height * 0.5) as f32, radius as f32);
    let start = Aabb {
        pos: from - half,
        size: half.scale(2.0),
    };
    let s_min = start.pos;
    let s_max = start.pos + start.size;
    let e_min = s_min + motion;
    let e_max = s_max + motion;
    let lo_x = minf(s_min.x as f64, e_min.x as f64) - SWEEP_REJECT_MARGIN;
    let lo_y = minf(s_min.y as f64, e_min.y as f64) - SWEEP_REJECT_MARGIN;
    let lo_z = minf(s_min.z as f64, e_min.z as f64) - SWEEP_REJECT_MARGIN;
    let hi_x = maxf(s_max.x as f64, e_max.x as f64) + SWEEP_REJECT_MARGIN;
    let hi_y = maxf(s_max.y as f64, e_max.y as f64) + SWEEP_REJECT_MARGIN;
    let hi_z = maxf(s_max.z as f64, e_max.z as f64) + SWEEP_REJECT_MARGIN;

    for (index, b) in boxes.iter().enumerate() {
        let c_min = b.pos;
        let c_max = c_min + b.size;
        if (c_max.x as f64) < lo_x
            || (c_min.x as f64) > hi_x
            || (c_max.y as f64) < lo_y
            || (c_min.y as f64) > hi_y
            || (c_max.z as f64) < lo_z
            || (c_min.z as f64) > hi_z
        {
            continue;
        }
        let Some((t, axis)) = slab_sweep(&start, motion, b) else {
            continue;
        };
        if t < 0.0 || t >= best {
            continue;
        }
        best = t;
        let mut n = [0.0f32; 3];
        n[axis] = -signf(comp(motion, axis) as f64) as f32;
        best_normal = Vec3::new(n[0], n[1], n[2]);
        best_index = index;
        found = true;
    }
    if !found {
        return None;
    }
    let fraction = clampf(best, 0.0, 1.0);
    let centre = from + motion.scale(fraction);
    let point = centre
        - Vec3::new(
            (signf(best_normal.x as f64) * radius) as f32,
            (signf(best_normal.y as f64) * height * 0.5) as f32,
            (signf(best_normal.z as f64) * radius) as f32,
        );
    Some((
        Hit {
            hit: true,
            fraction,
            normal: best_normal,
            point,
            collider_id: 0,
            depth: 0.0,
        },
        best_index,
    ))
}

/// One move's queries: the body, the space it asks, the hulls beside it, and what the script would have counted.
pub struct Mover<'a, S: Space> {
    pub body: &'a mut Body,
    pub space: &'a mut S,
    pub masks: Masks,
    /// Other players' hulls; `None` for a body without them (`DotFpsPhysicsBody`), which counts no touches.
    pub hulls: Option<&'a [Aabb]>,
    pub t: Tun,
    pub counters: Counters,
    near: Vec<Aabb>,
    near_index: Vec<usize>,
}

impl<'a, S: Space> Mover<'a, S> {
    pub fn new(
        body: &'a mut Body,
        space: &'a mut S,
        masks: Masks,
        hulls: Option<&'a [Aabb]>,
        t: Tun,
    ) -> Self {
        Self {
            body,
            space,
            masks,
            hulls,
            t,
            counters: Counters::default(),
            near: Vec::new(),
            near_index: Vec::new(),
        }
    }

    /// The motor body's `sweep`: the world, then (`DotFpsHullBody.sweep`) the hulls the swept bounds touch.
    pub fn body_sweep(&mut self, from: Vec3, motion: Vec3, height: f64, radius: f64) -> Hit {
        let world = self
            .body
            .sweep(self.space, from, motion, height, radius, self.masks);
        let Some(boxes) = self.hulls else {
            return world;
        };
        if world.hit {
            self.counters.world_touches += 1;
        }
        if boxes.is_empty() {
            return world;
        }
        let half = Vec3::new(radius as f32, (height * 0.5) as f32, radius as f32);
        let lo = from - half;
        let hi = from + half;
        let end = from + motion;
        let skin = Vec3::new(1.0, 1.0, 1.0).scale(1e-3);
        let pos = vmin(lo, end - half) - skin;
        let swept = Aabb {
            pos,
            size: (vmax(hi, end + half) + skin) - pos,
        };
        self.near.clear();
        self.near_index.clear();
        for (i, b) in boxes.iter().enumerate() {
            if b.intersects(&swept) {
                self.near.push(*b);
                self.near_index.push(i);
            }
        }
        if self.near.is_empty() {
            return world;
        }
        let Some((mut other, k)) = boxes_sweep(from, motion, height, radius, &self.near) else {
            return world;
        };
        if world.hit && world.fraction <= other.fraction {
            return world;
        }
        self.counters.hull_stops += 1;
        self.counters.hull_blocks += 1;
        other.collider_id = HULL_ID_BASE + self.near_index[k] as i64;
        other
    }

    fn rest_contact(&mut self, at: Vec3, height: f64, radius: f64) -> Hit {
        self.body
            .rest_contact(self.space, at, height, radius, self.masks)
    }

    /// `DotFpsMotorWalls._sweep`: sweeps a margin further and gives it back.
    fn sweep(&mut self, centre: Vec3, motion: Vec3, height: f64) -> Hit {
        let length = motion.length() as f64;
        if length <= 0.0 {
            return Hit::MISS;
        }
        let direction = motion.div_scalar(length);
        let margin = maxf(self.t.skin_width, length * SWEEP_TOLERANCE);
        let radius = self.t.radius;
        let mut hit = self.body_sweep(centre, motion + direction.scale(margin), height, radius);
        if !hit.hit {
            return hit;
        }
        let travelled = clampf(hit.fraction * (length + margin) - margin, 0.0, length);
        hit.fraction = travelled / length;
        hit
    }

    #[inline]
    fn slide_along(&self, v: Vec3, n: Vec3) -> Vec3 {
        if self.t.math_checks && !n.is_normalized() {
            return Vec3::default();
        }
        v - n.scale(v.dot(n) as f64)
    }

    #[inline]
    fn is_floor(&self, n: Vec3) -> bool {
        (n.angle_to(UP) as f64) <= self.t.max_slope
    }

    fn remember_plane(&self, planes: &mut Vec<Vec3>, normal: Vec3) -> i32 {
        for existing in planes.iter() {
            if (existing.dot(normal) as f64) > 0.999 {
                return PLANE_DUPLICATE;
            }
        }
        if planes.len() as i64 >= self.t.max_slide_planes {
            return PLANE_FULL;
        }
        planes.push(normal);
        PLANE_ADDED
    }

    fn resolve_planes(&self, planes: &[Vec3], original: Vec3) -> Vec3 {
        let count = planes.len();
        for i in 0..count {
            let candidate = self.slide_along(original, planes[i]);
            let clear = (0..count).all(|j| j == i || (candidate.dot(planes[j]) as f64) >= -1e-5);
            if clear {
                return candidate;
            }
        }
        for i in 0..count {
            for j in (i + 1)..count {
                let crease = planes[i].cross(planes[j]);
                if (crease.length_squared() as f64) < 1e-12 {
                    continue;
                }
                let crease = crease.normalized();
                let along = crease.scale(crease.dot(original) as f64);
                let leaves =
                    (0..count).all(|k| k == i || k == j || (along.dot(planes[k]) as f64) >= -1e-5);
                if leaves {
                    return along;
                }
            }
        }
        Vec3::default()
    }

    /// `DotFpsMotor._slide`.
    pub fn slide(&mut self, from: Vec3, velocity: Vec3, motion: Vec3, height: f64) -> MoveResult {
        let mut result = MoveResult {
            position: from,
            velocity,
            ..Default::default()
        };
        let start = from;
        let mut current = motion;
        let mut time_left = 1.0f64;
        let mut planes: Vec<Vec3> = Vec::with_capacity(8);

        for _ in 0..self.t.max_slide_iterations.max(0) {
            let step = current.scale(time_left);
            if (step.length_squared() as f64) <= 1e-12 {
                break;
            }
            let centre = capsule_centre(result.position, height);
            let hit = self.sweep(centre, step, height);
            if !hit.hit {
                result.position = result.position + step;
                time_left = 0.0;
                break;
            }
            result.blocked = true;
            if (hit.normal.y as f64) > 0.0 && !self.is_floor(hit.normal) {
                result.met_steep_face = true;
            }
            result.position = result.position + step.scale(hit.fraction);
            time_left -= time_left * hit.fraction;

            if !self.t.crease_slide {
                current = self.slide_along(current, hit.normal);
                result.velocity = self.slide_along(result.velocity, hit.normal);
                continue;
            }

            let outcome = self.remember_plane(&mut planes, hit.normal);
            if outcome == PLANE_FULL {
                current = Vec3::default();
                result.velocity = Vec3::default();
                break;
            }
            if outcome == PLANE_DUPLICATE {
                let rest = current.scale(time_left) + hit.normal.scale(self.t.skin_width);
                let clear = self.sweep(capsule_centre(result.position, height), rest, height);
                let spent = if !clear.hit { 1.0 } else { clear.fraction };
                result.position = result.position + rest.scale(spent);
                if spent > 1e-6 {
                    if spent < 1.0 {
                        let left = time_left * (1.0 - spent);
                        self.leave_crease(
                            &mut result,
                            &planes,
                            motion,
                            velocity,
                            left,
                            hit.normal,
                            height,
                        );
                    }
                    time_left = 0.0;
                    break;
                }
                if self.leave_crease(
                    &mut result,
                    &planes,
                    motion,
                    velocity,
                    time_left,
                    hit.normal,
                    height,
                ) {
                    time_left = 0.0;
                    break;
                }
                if self.leave_face(&mut result, current.scale(time_left), hit.normal, height) {
                    time_left = 0.0;
                    break;
                }
                if self.split_leg(&mut result, current.scale(time_left), height) {
                    time_left = 0.0;
                    break;
                }
                result.stopped_on_duplicate = true;
                break;
            }
            current = self.resolve_planes(&planes, motion);
            result.velocity = self.resolve_planes(&planes, velocity);
        }

        if time_left > 1e-6
            && (current.length_squared() as f64) > 1e-12
            && !result.stopped_on_duplicate
        {
            result.ran_out = true;
        }
        result.travelled = flat_distance(result.position, start);
        result
    }

    /// `DotFpsMotor._leave_face`.
    fn leave_face(
        &mut self,
        result: &mut MoveResult,
        leg: Vec3,
        normal: Vec3,
        height: f64,
    ) -> bool {
        if self.t.skin_width <= 0.0 || (leg.length_squared() as f64) <= 1e-12 {
            return false;
        }
        for scale in [1.0, 2.0, 4.0] {
            let lift = normal.scale(self.t.skin_width).scale(scale);
            if self
                .sweep(capsule_centre(result.position, height), lift, height)
                .hit
            {
                return false;
            }
            let from = result.position + lift;
            let along = self.sweep(capsule_centre(from, height), leg, height);
            let spent = if !along.hit { 1.0 } else { along.fraction };
            if spent > 1e-6 {
                result.position = from + leg.scale(spent);
                return true;
            }
        }
        false
    }

    /// `DotFpsMotor._leave_crease`.
    #[allow(clippy::too_many_arguments)]
    fn leave_crease(
        &mut self,
        result: &mut MoveResult,
        planes: &[Vec3],
        motion: Vec3,
        velocity: Vec3,
        time_left: f64,
        normal: Vec3,
        height: f64,
    ) -> bool {
        if self.t.skin_width <= 0.0 {
            return false;
        }
        let mut other = Vec3::default();
        for scale in [1.0, 2.0, 4.0] {
            let at = capsule_centre(
                result.position + normal.scale(self.t.skin_width).scale(scale),
                height,
            );
            let probe = self.rest_contact(at, height, self.t.radius);
            if probe.hit && (probe.normal.dot(normal) as f64) < 0.999 {
                other = probe.normal;
                break;
            }
        }
        if other == Vec3::default() {
            return false;
        }
        let c = normal.dot(other) as f64;
        if c < -0.9 {
            return false;
        }
        let mut wedge: Vec<Vec3> = planes.to_vec();
        if self.remember_plane(&mut wedge, other) == PLANE_FULL {
            return false;
        }
        let leg = self.resolve_planes(&wedge, motion).scale(time_left);
        if (leg.length_squared() as f64) <= 1e-12 {
            return false;
        }
        let off_both = (normal + other).scale(self.t.skin_width / (1.0 + c));
        for scale in [0.0, 1.0, 2.0, 4.0] {
            let lift = off_both.scale(scale);
            if scale > 0.0
                && self
                    .sweep(capsule_centre(result.position, height), lift, height)
                    .hit
            {
                continue;
            }
            let from = result.position + lift;
            let along = self.sweep(capsule_centre(from, height), leg, height);
            let spent = if !along.hit { 1.0 } else { along.fraction };
            if spent > 1e-6 && (scale > 0.0 || !along.hit) {
                result.position = from + leg.scale(spent);
                result.velocity = self.resolve_planes(&wedge, velocity);
                self.counters.crease_leg_ticks += 1;
                return true;
            }
        }
        false
    }

    /// `DotFpsMotor._split_leg`.
    fn split_leg(&mut self, result: &mut MoveResult, leg: Vec3, height: f64) -> bool {
        for part in [Vec3::new(0.0, leg.y, 0.0), Vec3::new(leg.x, 0.0, leg.z)] {
            if (part.length_squared() as f64) <= 1e-12 {
                continue;
            }
            let along = self.sweep(capsule_centre(result.position, height), part, height);
            let spent = if !along.hit { 1.0 } else { along.fraction };
            if spent > 1e-6 {
                result.position = result.position + part.scale(spent);
                self.counters.split_leg_ticks += 1;
                return true;
            }
        }
        false
    }

    /// `DotFpsMotor._slide_with_step`: up, across, down.
    pub fn slide_with_step(
        &mut self,
        from: Vec3,
        velocity: Vec3,
        motion: Vec3,
        height: f64,
    ) -> MoveResult {
        let skin = self.t.skin_width;
        let centre = capsule_centre(from, height);
        let seat = self.sweep(centre, DOWN.scale(skin * 2.0 + 0.02), height);
        let mut hover = 0.0;
        if seat.hit {
            hover = maxf((skin * 2.0 + 0.02) * seat.fraction - skin, 0.0);
        }
        let floor_y = from.y as f64 - hover;
        let lift = UP.scale(self.t.step_height - hover);

        let up = self.sweep(centre, lift, height);
        if up.hit {
            return MoveResult {
                position: from,
                velocity,
                blocked: true,
                ..Default::default()
            };
        }
        let raised = from + lift;
        let horizontal = Vec3::new(motion.x, 0.0, motion.z);
        let across = self.slide(raised, velocity, horizontal, height);

        let drop = DOWN.scale(lift.y as f64 + skin);
        let landing = self.sweep(capsule_centre(across.position, height), drop, height);

        let mut result = MoveResult {
            velocity: across.velocity,
            blocked: across.blocked,
            ..Default::default()
        };
        let too_high =
            landing.hit && (landing.point.y as f64) - floor_y > self.t.step_height + skin;
        if !landing.hit
            || too_high
            || !(self.is_floor(landing.normal)
                || self.edge_floor_normal(&landing, across.position, floor_y) != Vec3::default())
        {
            result.position = from;
            result.travelled = 0.0;
            result.blocked = true;
            return result;
        }
        result.position = across.position + drop.scale(landing.fraction);
        result.travelled = flat_distance(result.position, from);
        result
    }

    /// `DotFpsMotor._edge_floor_normal`.
    fn edge_floor_normal(&mut self, landing: &Hit, feet: Vec3, start_y: f64) -> Vec3 {
        if (landing.point.y as f64) - start_y > self.t.step_height + self.t.skin_width {
            return Vec3::default();
        }
        let outward = Vec3::new(
            ((landing.point.x as f64) - (feet.x as f64)) as f32,
            0.0,
            ((landing.point.z as f64) - (feet.z as f64)) as f32,
        );
        if (outward.length_squared() as f64) < 1e-8 {
            return Vec3::default();
        }
        let probe_radius = minf(0.05, self.t.radius * 0.25);
        let probe_height = probe_radius * 2.0;
        let at = landing.point + outward.normalized().scale(probe_radius + 0.01);
        let centre = at + UP.scale(probe_radius + 0.05);
        let hit = self.body_sweep(centre, DOWN.scale(0.1), probe_height, probe_radius);
        if !hit.hit || !self.is_floor(hit.normal) {
            return Vec3::default();
        }
        let bottom = centre.y as f64 - probe_radius + (-0.1 * hit.fraction);
        if (bottom - landing.point.y as f64).abs() < 0.02 {
            hit.normal
        } else {
            Vec3::default()
        }
    }

    /// `DotFpsMotor._move`'s slide and `_prefer_step`: the plain slide, or the stepped one when it got further.
    /// Stepping is skipped when only another player's hull stopped the slide.
    pub fn move_walking(
        &mut self,
        from: Vec3,
        velocity: Vec3,
        motion: Vec3,
        height: f64,
        walking: bool,
    ) -> MoveResult {
        let hulls_before = self.counters.hull_blocks;
        let world_before = self.counters.world_touches;
        let plain = self.slide(from, velocity, motion, height);
        let hull_only =
            self.counters.hull_blocks > hulls_before && self.counters.world_touches == world_before;
        let skin = self.t.skin_width;
        let flat = (motion.x * motion.x + motion.z * motion.z).sqrt() as f64;
        if walking
            && plain.blocked
            && self.t.step_height > 0.0
            && !hull_only
            && plain.travelled + skin * 2.0 + 1e-4 < flat
        {
            let stepped = self.slide_with_step(from, velocity, motion, height);
            if stepped.travelled > plain.travelled + skin * 2.0 {
                return stepped;
            }
        }
        plain
    }

    /// `DotFpsMotor.move_and_slide`'s collision: the slide, and with `step` the stepped one when it got further.
    pub fn move_and_slide(
        &mut self,
        from: Vec3,
        velocity: Vec3,
        motion: Vec3,
        height: f64,
        step: bool,
    ) -> MoveResult {
        let result = self.slide(from, velocity, motion, height);
        if step && result.blocked && self.t.step_height > 0.0 {
            let stepped = self.slide_with_step(from, velocity, motion, height);
            if stepped.travelled > result.travelled + self.t.skin_width * 2.0 {
                return stepped;
            }
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Contact;

    /// Open space: nothing is ever hit, so only the hulls stop a sweep.
    #[derive(Default)]
    struct Empty;

    impl Space for Empty {
        fn set_radius(&mut self, _: f32) {}
        fn set_height(&mut self, _: f32) {}
        fn set_mask(&mut self, _: i64) {}
        fn cast(&mut self, _: Vec3, _: Vec3) -> Option<(f32, f32)> {
            Some((1.0, 1.0))
        }
        fn rest(&mut self, _: Vec3) -> Option<Contact> {
            None
        }
        fn overlap(&mut self, _: Vec3) -> bool {
            false
        }
        fn ray(&mut self, _: Vec3, _: Vec3, _: i64) -> Option<Contact> {
            None
        }
    }

    const TUN: Tun = Tun {
        max_slide_iterations: 5,
        crease_slide: true,
        skin_width: 0.001,
        max_slide_planes: 5,
        step_height: 0.3429,
        radius: 0.4,
        max_slope: 46.0 * std::f64::consts::PI / 180.0,
        math_checks: true,
    };

    fn hull(feet: Vec3) -> Aabb {
        Aabb {
            pos: feet - Vec3::new(0.4, 0.0, 0.4),
            size: Vec3::new(0.8, 1.8, 0.8),
        }
    }

    #[test]
    fn a_free_move_goes_the_whole_way() {
        let (mut b, mut s) = (Body::default(), Empty);
        let mut m = Mover::new(&mut b, &mut s, Masks::default(), Some(&[]), TUN);
        let r = m.slide(
            Vec3::default(),
            Vec3::new(5.0, 0.0, 0.0),
            Vec3::new(0.1, 0.0, 0.0),
            1.8,
        );
        assert_eq!(r.position, Vec3::new(0.1, 0.0, 0.0));
        assert!(!r.blocked && !r.ran_out);
        assert_eq!(m.counters, Counters::default());
    }

    #[test]
    fn a_hull_ahead_stops_the_move_and_counts_as_a_hull_block() {
        let (mut b, mut s) = (Body::default(), Empty);
        let boxes = [
            hull(Vec3::new(5.0, 0.0, 0.0)),
            hull(Vec3::new(1.0, 0.0, 0.0)),
        ];
        let mut m = Mover::new(&mut b, &mut s, Masks::default(), Some(&boxes), TUN);
        let from = Vec3::default();
        let r = m.move_walking(
            from,
            Vec3::new(10.0, 0.0, 0.0),
            Vec3::new(0.5, 0.0, 0.0),
            1.8,
            true,
        );
        assert!(r.blocked);
        // The hull's face is at x = 0.6, so the feet stop a skin short of 0.2.
        assert!(
            r.position.x < 0.2 && r.position.x > 0.19,
            "{:?}",
            r.position
        );
        assert_eq!(r.velocity.x, 0.0);
        assert!(m.counters.hull_blocks >= 1);
        assert_eq!(m.counters.world_touches, 0);
        assert_eq!(m.counters.hull_blocks, m.counters.hull_stops);
    }

    #[test]
    fn a_hull_hit_reports_its_index_in_the_full_list() {
        let (mut b, mut s) = (Body::default(), Empty);
        let boxes = [
            hull(Vec3::new(-5.0, 0.0, 0.0)),
            hull(Vec3::new(1.0, 0.0, 0.0)),
        ];
        let mut m = Mover::new(&mut b, &mut s, Masks::default(), Some(&boxes), TUN);
        let h = m.body_sweep(Vec3::new(0.0, 0.9, 0.0), Vec3::new(1.0, 0.0, 0.0), 1.8, 0.4);
        assert!(h.hit);
        assert_eq!(h.collider_id, HULL_ID_BASE + 1);
        assert_eq!(h.normal, Vec3::new(-1.0, 0.0, 0.0));
        // Bit-exactness against the script is the parity suite's job; here only the geometry.
        assert!((h.fraction - 0.2).abs() < 1e-6, "{}", h.fraction);
    }

    #[test]
    fn a_diagonal_into_a_hull_slides_along_it() {
        let (mut b, mut s) = (Body::default(), Empty);
        let boxes = [hull(Vec3::new(0.85, 0.0, 0.0))];
        let mut m = Mover::new(&mut b, &mut s, Masks::default(), Some(&boxes), TUN);
        let r = m.slide(
            Vec3::default(),
            Vec3::new(3.0, 0.0, 3.0),
            Vec3::new(0.3, 0.0, 0.3),
            1.8,
        );
        assert!(r.blocked);
        assert_eq!(r.velocity.x, 0.0);
        assert_eq!(r.velocity.z, 3.0);
        assert!(r.position.z > 0.29);
    }

    #[test]
    fn planes_resolve_to_the_crease_or_nothing() {
        let (mut b, mut s) = (Body::default(), Empty);
        let m = Mover::new(&mut b, &mut s, Masks::default(), None, TUN);
        let a = Vec3::new(-1.0, 0.0, 0.0);
        let c = Vec3::new(0.0, 0.0, -1.0);
        let v = Vec3::new(1.0, 0.5, 1.0);
        assert_eq!(m.resolve_planes(&[a, c], v), Vec3::new(0.0, 0.5, 0.0));
        assert_eq!(
            m.resolve_planes(&[a, -a], Vec3::new(1.0, 0.0, 0.0)),
            Vec3::default()
        );
        assert_eq!(m.slide_along(v, Vec3::new(2.0, 0.0, 0.0)), Vec3::default());
    }
}
