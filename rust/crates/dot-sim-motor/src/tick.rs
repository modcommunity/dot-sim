//! The rest of `DotFpsMotor._simulate` around the slide, for a plain walker (GROUND or AIR, no
//! surfaces, no edge friction, not wall running): the ground check at the start of the tick
//! ([`pre`]), then friction, the wish move, gravity, the clamp, the slide and the ground check at
//! the end ([`post`]). The script keeps the jump, the wall moves, launches, slides and crouch, which
//! run between the two and only write state.
//!
//! Every line follows the script's arithmetic to the bit: script floats are `f64`, vectors `f32`,
//! and a component written from a script float is rounded on the store.

use crate::slide::{Mover, capsule_centre};
use crate::{DOWN, Hit, Space, UP, Vec3};
use dot_sim_math::{maxf, minf};

/// `DotFpsState.Mode.GROUND`.
pub const GROUND: i64 = 0;
/// `DotFpsState.Mode.AIR`.
pub const AIR: i64 = 1;
/// `DotFpsState.Mode.NOCLIP`.
pub const NOCLIP: i64 = 2;

/// `DotFpsMotor.LEAVING_SPEED`.
const LEAVING_SPEED: f64 = 0.1;

/// `DotFpsMotor.QUADRANTS`.
const QUADRANTS: [(f32, f32); 4] = [(-1.0, -1.0), (1.0, 1.0), (-1.0, 1.0), (1.0, -1.0)];

/// The player state the tick reads and writes.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct State {
    pub position: Vec3,
    pub velocity: Vec3,
    pub ground_normal: Vec3,
    pub mode: i64,
    pub ground_id: i64,
    /// `_height(state)`, fixed for the part of the tick run here.
    pub height: f64,
    /// Set when the script's code would have written `state.surface = &""`.
    pub surface_cleared: bool,
}

/// The tunables the tick reads beyond the slide's [`crate::slide::Tun`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TickTun {
    /// `tunables.jump_velocity()`.
    pub jump_velocity: f64,
    pub ground_snap: bool,
    pub ground_snap_distance: f64,
    pub crest_launch_speed_scale: f64,
    pub max_speed: f64,
    pub stop_speed: f64,
    pub air_friction: f64,
    pub max_velocity: f64,
    pub backward_speed_scale: f64,
    pub max_air_wish_speed: f64,
    pub wall_slide_speed: f64,
    pub wall_run_vertical_damping: f64,
}

/// What the script decided for this tick before [`post`] runs, already multiplied out where the
/// script multiplies tunables, surface and effects in one expression.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Post {
    pub delta: f64,
    /// `jumped or launched`.
    pub jumped: bool,
    /// `state.is_sliding()`.
    pub sliding: bool,
    /// `_effects.deny_move`.
    pub deny_move: bool,
    /// `_wall_slides` without its falling check, which needs the velocity after gravity.
    pub wall_slide_ok: bool,
    /// What [`pre`] answered.
    pub start_settled: bool,
    pub move_x: f64,
    pub move_y: f64,
    /// `state.yaw`, degrees.
    pub yaw: f64,
    /// `tunables.speed_for(buttons, load_fraction)`.
    pub speed_base: f64,
    /// `_effects.max_speed`.
    pub effect_max_speed: f64,
    /// `_surface.max_speed_scale`.
    pub surface_max_speed_scale: f64,
    /// `slide_friction` while sliding, else `friction`.
    pub friction_base: f64,
    /// `_surface.friction_scale`.
    pub surface_friction: f64,
    /// `_effects.friction`.
    pub effect_friction: f64,
    /// `ground_accel * _surface.accelerate_scale * _effects.accelerate`.
    pub ground_accel: f64,
    /// `tunables.air_accelerate * _effects.air_accelerate`.
    pub air_accel: f64,
    /// `_surface.push`.
    pub push: Vec3,
    /// `tunables.gravity * _effects.gravity`.
    pub gravity: f64,
    pub start_position: Vec3,
    pub start_velocity: Vec3,
}

/// The script counters a tick adds to; the slide's own are in [`crate::slide::Counters`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TickCounters {
    pub depenetrated: i64,
    pub embedded: i64,
    pub lifted: i64,
    pub stuck: i64,
    pub duplicate_plane: i64,
}

/// What [`pre`] answered.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PreOut {
    pub start_settled: bool,
    /// `_reuse_rest` said yes (`reused_ground_ticks`).
    pub reused: bool,
}

/// What [`post`] answered.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PostOut {
    /// The tick ended settled: the script counts `settled_ground_ticks` and stamps the rest.
    pub stamp: bool,
    /// The velocity went non-finite and was zeroed (the script warns).
    pub nonfinite: bool,
}

/// `Vector2(x, y).length()`.
#[inline]
fn len2(x: f32, y: f32) -> f32 {
    (x * x + y * y).sqrt()
}

/// `Vector3.is_equal_approx`.
#[inline]
fn approx(a: f32, b: f32) -> bool {
    if a == b {
        return true;
    }
    let mut tolerance = 0.00001f32 * a.abs();
    if tolerance < 0.00001 {
        tolerance = 0.00001;
    }
    (a - b).abs() < tolerance
}

#[inline]
fn add_y(v: Vec3, by: f64) -> Vec3 {
    Vec3::new(v.x, ((v.y as f64) + by) as f32, v.z)
}

/// `DotFpsMotor._depenetrate`.
fn depenetrate<S: Space>(m: &mut Mover<'_, S>, s: &mut State, c: &mut TickCounters) -> bool {
    let centre = capsule_centre(s.position, s.height);
    let radius = m.t.radius;
    let resting = m.rest_contact(centre, s.height, radius);
    if !resting.hit || resting.depth <= 0.0 {
        return false;
    }
    let skin = m.t.skin_width;
    let push = minf(resting.depth + skin, maxf(m.t.step_height, skin * 2.0));
    s.position = s.position + resting.normal.scale(push);
    c.depenetrated += 1;
    true
}

/// `DotFpsMotor._quadrant_floor_probe`.
fn quadrant_floor<S: Space>(m: &mut Mover<'_, S>, centre: Vec3, probe: Vec3, height: f64) -> Hit {
    let half = m.t.radius * 0.5;
    for (qx, qz) in QUADRANTS {
        let offset = Vec3::new(qx, 0.0, qz).normalized().scale(half);
        let hit = m.sweep_r(centre + offset, probe, height, half);
        if hit.hit && m.is_floor(hit.normal) {
            return hit;
        }
    }
    Hit::MISS
}

/// `DotFpsMotor._standable_normal`.
fn standable_normal<S: Space>(m: &mut Mover<'_, S>, s: &State, hit: &Hit) -> Vec3 {
    if !hit.hit {
        return Vec3::default();
    }
    if m.is_floor(hit.normal) {
        return hit.normal;
    }
    let toward = Vec3::new(
        ((hit.point.x as f64) - (s.position.x as f64)) as f32,
        0.0,
        ((hit.point.z as f64) - (s.position.z as f64)) as f32,
    );
    let horizontal = Vec3::new(s.velocity.x, 0.0, s.velocity.z);
    if (horizontal.dot(toward) as f64) <= 0.0 {
        return Vec3::default();
    }
    m.edge_floor_normal(hit, s.position, s.position.y as f64)
}

/// `DotFpsMotor._keep_skin`.
fn keep_skin<S: Space>(m: &Mover<'_, S>, s: &mut State, hit: &Hit, probe: Vec3) {
    let gap = (probe.length() as f64) * hit.fraction;
    if gap < m.t.skin_width * 0.5 && (hit.normal.y as f64) > 0.0 {
        s.position = add_y(s.position, m.t.skin_width - gap);
    }
}

/// `DotFpsMotor._lift_clear`.
fn lift_clear<S: Space>(m: &mut Mover<'_, S>, s: &mut State) -> bool {
    let mut lift = maxf(m.t.skin_width * 2.0, 0.0005);
    let limit = maxf(m.t.step_height, lift);
    let mut moved = 0.0f64;
    let radius = m.t.radius;
    while moved < limit {
        moved = minf(moved + lift, limit);
        let centre = capsule_centre(s.position + UP.scale(moved), s.height);
        let inside = m.body.overlaps(m.space, centre, s.height, radius, m.masks);
        if !inside {
            s.position = add_y(s.position, moved);
            return true;
        }
        if moved >= limit {
            break;
        }
        lift *= 2.0;
    }
    false
}

#[inline]
fn set_air(s: &mut State) {
    s.mode = AIR;
    s.ground_normal = UP;
    s.ground_id = 0;
}

/// `DotFpsMotor._categorise_ground_settled` with no surfaces.
pub fn categorise<S: Space>(
    m: &mut Mover<'_, S>,
    s: &mut State,
    tt: &TickTun,
    c: &mut TickCounters,
) -> bool {
    if s.mode == NOCLIP {
        return false;
    }
    let was_grounded = s.mode == GROUND;
    let position_in = s.position;
    let velocity_in = s.velocity;
    let vy = s.velocity.y as f64;
    let rising = vy > LEAVING_SPEED;

    if rising && !was_grounded && vy > maxf(tt.jump_velocity * 0.5, LEAVING_SPEED) {
        set_air(s);
        return false;
    }

    let height = s.height;
    let centre = capsule_centre(s.position, height);
    let probe = DOWN.scale(m.t.skin_width * 2.0 + 0.02);

    let mut hit = m.sweep(centre, probe, height);
    let mut floor = standable_normal(m, s, &hit);

    if floor == Vec3::default() && hit.hit && !m.is_floor(hit.normal) {
        let quarter = quadrant_floor(m, centre, probe, height);
        if quarter.hit {
            hit = quarter;
            floor = quarter.normal;
        }
    }

    if rising {
        let walking = floor != Vec3::default()
            && ((s.velocity.dot(floor) as f64) <= LEAVING_SPEED
                || (s.velocity.dot(s.ground_normal) as f64) <= LEAVING_SPEED);
        if !walking {
            set_air(s);
            return false;
        }
        s.velocity = m.slide_along(s.velocity, floor);
    }

    if floor != Vec3::default() {
        s.mode = GROUND;
        s.ground_normal = floor;
        s.ground_id = hit.collider_id;
        s.surface_cleared = true;
        keep_skin(m, s, &hit, probe);
        return was_grounded && s.position == position_in && s.velocity == velocity_in;
    }

    let radius = m.t.radius;
    if was_grounded && m.body.overlaps(m.space, centre, height, radius, m.masks) {
        c.embedded += 1;
        if !hit.hit && lift_clear(m, s) {
            c.lifted += 1;
            let recovered = m.sweep(capsule_centre(s.position, height), probe, height);
            if recovered.hit && m.is_floor(recovered.normal) {
                s.mode = GROUND;
                s.ground_normal = recovered.normal;
                s.ground_id = recovered.collider_id;
                return false;
            }
        }
        s.mode = GROUND;
        return false;
    }

    set_air(s);
    s.surface_cleared = true;
    false
}

/// The start of the tick: `_depenetrate`, then `_reuse_rest` (the script decided everything but
/// the position, in `rest_ok`) or `_categorise_ground_settled`.
pub fn pre<S: Space>(
    m: &mut Mover<'_, S>,
    s: &mut State,
    tt: &TickTun,
    rest_ok: bool,
    rest_position: Vec3,
    c: &mut TickCounters,
) -> PreOut {
    depenetrate(m, s, c);
    if rest_ok && s.mode == GROUND && s.position == rest_position {
        return PreOut {
            start_settled: true,
            reused: true,
        };
    }
    PreOut {
        start_settled: categorise(m, s, tt, c),
        reused: false,
    }
}

/// `DotFpsMotor._apply_friction` with no edge friction.
fn apply_friction(s: &mut State, tt: &TickTun, p: &Post) {
    let speed = s.velocity.length() as f64;
    if speed <= 0.0 {
        return;
    }
    let control = maxf(speed, tt.stop_speed);
    let edge = 1.0f64;
    let drop = control * p.friction_base * p.surface_friction * p.effect_friction * edge * p.delta;
    let scale = maxf(speed - drop, 0.0) / speed;
    s.velocity = s.velocity.scale(scale);
}

/// `DotFpsMotor._apply_air_friction`.
fn apply_air_friction(s: &mut State, tt: &TickTun, p: &Post) {
    let horizontal = Vec3::new(s.velocity.x, 0.0, s.velocity.z);
    let speed = horizontal.length() as f64;
    if speed <= 0.0 {
        return;
    }
    let scale = maxf(speed - speed * tt.air_friction * p.delta, 0.0) / speed;
    s.velocity.x = ((s.velocity.x as f64) * scale) as f32;
    s.velocity.z = ((s.velocity.z as f64) * scale) as f32;
}

/// `DotFpsMotor._project_on_slope`.
fn project_on_slope<S: Space>(m: &Mover<'_, S>, direction: Vec3, normal: Vec3) -> Vec3 {
    if approx(normal.x, UP.x) && approx(normal.y, UP.y) && approx(normal.z, UP.z) {
        return direction;
    }
    let projected = m.slide_along(direction, normal);
    if (projected.length_squared() as f64) < 1e-8 {
        return direction;
    }
    projected.normalized()
}

/// `DotFpsMotor.accelerate`.
fn accelerate(s: &mut State, direction: Vec3, wish_speed: f64, accel: f64, delta: f64, cap: f64) {
    let target = if cap < 0.0 {
        wish_speed
    } else {
        minf(wish_speed, cap)
    };
    let current = s.velocity.dot(direction) as f64;
    let add = target - current;
    if add <= 0.0 {
        return;
    }
    let step = accel * wish_speed * delta;
    s.velocity = s.velocity + direction.scale(minf(step, add));
}

/// `DotFpsMotor._apply_wish_move`; returns the ground cap it set (0 when none).
fn apply_wish_move<S: Space>(m: &Mover<'_, S>, s: &mut State, tt: &TickTun, p: &Post) -> f64 {
    if p.deny_move {
        return 0.0;
    }
    let yaw = p.yaw * (std::f64::consts::PI / 180.0);
    let right = Vec3::new(yaw.cos() as f32, 0.0, (-yaw.sin()) as f32);
    let forward = Vec3::new((-yaw.sin() * 1.0) as f32, 0.0, (-yaw.cos() * 1.0) as f32);
    let mut wish = right.scale(p.move_x) + forward.scale(p.move_y);
    wish.y = 0.0;
    let magnitude = minf(wish.length() as f64, 1.0);
    if magnitude <= 0.0 {
        return 0.0;
    }
    let mut target = p.speed_base * magnitude * p.effect_max_speed;
    if p.move_y < 0.0 {
        target *= tt.backward_speed_scale;
    }
    let mut direction = wish.normalized();
    if s.mode == GROUND {
        target *= p.surface_max_speed_scale;
        direction = project_on_slope(m, direction, s.ground_normal);
        let cap = maxf(len2(s.velocity.x, s.velocity.z) as f64, target);
        accelerate(s, direction, target, p.ground_accel, p.delta, -1.0);
        cap
    } else {
        accelerate(
            s,
            direction,
            target,
            p.air_accel,
            p.delta,
            tt.max_air_wish_speed,
        );
        0.0
    }
}

/// `DotFpsMotor._clamp_velocity`; true when it zeroed a non-finite velocity.
fn clamp_velocity(s: &mut State, tt: &TickTun) -> bool {
    let v = s.velocity;
    if !(v.x.is_finite() && v.y.is_finite() && v.z.is_finite()) {
        s.velocity = Vec3::default();
        return true;
    }
    let p_len = tt.max_velocity as f32;
    let l = v.length();
    if l > 0.0 && p_len < l {
        s.velocity = Vec3::new(v.x / l * p_len, v.y / l * p_len, v.z / l * p_len);
    }
    false
}

/// `DotFpsMotor._snap_to_ground`.
fn snap_to_ground<S: Space>(m: &mut Mover<'_, S>, s: &mut State, tt: &TickTun, stood_on: Vec3) {
    if (s.velocity.dot(stood_on) as f64) > LEAVING_SPEED {
        return;
    }
    let height = s.height;
    let centre = capsule_centre(s.position, height);
    let probe = DOWN.scale(tt.ground_snap_distance);
    let mut hit = m.sweep(centre, probe, height);
    if hit.hit && !m.is_floor(hit.normal) {
        hit = quadrant_floor(m, centre, probe, height);
    }
    if !hit.hit || !m.is_floor(hit.normal) {
        return;
    }
    if tt.crest_launch_speed_scale > 0.0
        && (s.velocity.dot(hit.normal) as f64) > LEAVING_SPEED
        && (s.velocity.length() as f64) >= tt.max_speed * tt.crest_launch_speed_scale
    {
        return;
    }
    s.position = s.position + probe.scale(hit.fraction);
    s.mode = GROUND;
    s.ground_normal = hit.normal;
    s.ground_id = hit.collider_id;
    if (s.velocity.y as f64) < 0.0 {
        s.velocity.y = 0.0;
    } else if (s.velocity.dot(hit.normal) as f64) > LEAVING_SPEED {
        s.velocity = m.slide_along(s.velocity, hit.normal);
    }
}

/// `DotFpsMotor._move`, with the native slide.
fn move_tick<S: Space>(
    m: &mut Mover<'_, S>,
    s: &mut State,
    tt: &TickTun,
    p: &Post,
    ground_cap: f64,
    c: &mut TickCounters,
) -> bool {
    let height = s.height;
    let motion = s.velocity.scale(p.delta);
    if (motion.length_squared() as f64) <= 0.0 {
        return false;
    }
    let was_grounded = s.mode == GROUND;
    let velocity_in = s.velocity;
    let stood_on = s.ground_normal;
    let walking = was_grounded && !p.jumped;

    let plain = m.move_walking(s.position, s.velocity, motion, height, walking);
    s.position = plain.position;
    s.velocity = plain.velocity;

    if walking && plain.blocked && ground_cap > 0.0 {
        let (mut fx, mut fz) = (s.velocity.x, s.velocity.z);
        if (len2(fx, fz) as f64) > ground_cap {
            let l = fx * fx + fz * fz;
            if l != 0.0 {
                let l = l.sqrt();
                fx /= l;
                fz /= l;
            }
            let cap = ground_cap as f32;
            s.velocity.x = fx * cap;
            s.velocity.z = fz * cap;
        }
    }

    if walking && plain.met_steep_face {
        let lift = (s.velocity.dot(stood_on) as f64) - maxf(velocity_in.dot(stood_on) as f64, 0.0);
        if lift > 0.0 {
            s.velocity = s.velocity - stood_on.scale(lift);
        }
    }

    if plain.ran_out {
        c.stuck += 1;
    }
    if plain.stopped_on_duplicate {
        c.duplicate_plane += 1;
    }

    if walking && tt.ground_snap {
        if categorise(m, s, tt, c) {
            return true;
        }
        if s.mode != GROUND {
            snap_to_ground(m, s, tt, stood_on);
        }
    }
    false
}

/// From friction to the end of the tick: friction, the wish move, the push or gravity and the
/// wall slide, the clamp, the move, and the ground check the next tick starts from.
pub fn post<S: Space>(
    m: &mut Mover<'_, S>,
    s: &mut State,
    tt: &TickTun,
    p: &Post,
    c: &mut TickCounters,
) -> PostOut {
    if s.mode == GROUND && !p.jumped {
        apply_friction(s, tt, p);
    } else if tt.air_friction > 0.0 {
        apply_air_friction(s, tt, p);
    }

    let mut ground_cap = apply_wish_move(m, s, tt, p);

    if s.mode == GROUND {
        if p.push != Vec3::default() {
            s.velocity = s.velocity + p.push.scale(p.delta);
            if ground_cap > 0.0 {
                ground_cap += (len2(p.push.x, p.push.z) as f64) * p.delta;
            }
        }
    } else {
        let falling_at = s.velocity.y as f64;
        s.velocity.y = (falling_at - p.gravity * p.delta) as f32;
        if p.wall_slide_ok && tt.wall_slide_speed > 0.0 && (s.velocity.y as f64) < 0.0 {
            let cap = -tt.wall_slide_speed;
            if (s.velocity.y as f64) < cap {
                if falling_at >= cap {
                    s.velocity.y = cap as f32;
                } else {
                    s.velocity.y = (cap
                        + (falling_at - cap) * (-tt.wall_run_vertical_damping * p.delta).exp())
                        as f32;
                }
            }
        }
    }

    let nonfinite = clamp_velocity(s, tt);
    let settled = move_tick(m, s, tt, p, ground_cap, c);

    if settled
        || (p.start_settled
            && s.mode == GROUND
            && s.position == p.start_position
            && s.velocity == p.start_velocity)
    {
        return PostOut {
            stamp: true,
            nonfinite,
        };
    }
    categorise(m, s, tt, c);
    PostOut {
        stamp: false,
        nonfinite,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::slide::Tun;
    use crate::{Body, Contact, Masks};

    /// Open space: nothing is ever hit.
    struct Floor;

    impl Space for Floor {
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
        step_height: 0.3,
        radius: 0.3,
        max_slope: 0.785,
        math_checks: true,
    };

    const TT: TickTun = TickTun {
        jump_velocity: 5.0,
        ground_snap: true,
        ground_snap_distance: 0.3,
        crest_launch_speed_scale: 0.0,
        max_speed: 6.0,
        stop_speed: 1.5,
        air_friction: 0.0,
        max_velocity: 50.0,
        backward_speed_scale: 0.9,
        max_air_wish_speed: 0.8,
        wall_slide_speed: 0.0,
        wall_run_vertical_damping: 0.0,
    };

    fn post_in(delta: f64) -> Post {
        Post {
            delta,
            speed_base: 6.0,
            effect_max_speed: 1.0,
            surface_max_speed_scale: 1.0,
            friction_base: 6.0,
            surface_friction: 1.0,
            effect_friction: 1.0,
            ground_accel: 10.0,
            air_accel: 12.0,
            gravity: 20.0,
            ..Post::default()
        }
    }

    #[test]
    fn falling_in_open_space_stays_in_air_and_gains_speed() {
        let mut body = Body::default();
        let mut space = Floor;
        let masks = Masks {
            collision: 1,
            pass_out: 0,
        };
        let mut m = Mover::new(&mut body, &mut space, masks, None, TUN);
        let mut s = State {
            mode: AIR,
            ground_normal: UP,
            height: 1.8,
            ..State::default()
        };
        let mut c = TickCounters::default();
        let pre_out = pre(&mut m, &mut s, &TT, false, Vec3::default(), &mut c);
        assert!(!pre_out.start_settled);
        assert_eq!(s.mode, AIR);
        let p = post_in(1.0 / 64.0);
        let out = post(&mut m, &mut s, &TT, &p, &mut c);
        assert!(!out.stamp);
        assert_eq!(s.mode, AIR);
        assert_eq!(s.velocity.y, (0.0f64 - 20.0 * (1.0 / 64.0)) as f32);
        assert!(s.position.y < 0.0);
    }

    #[test]
    fn wish_accelerates_forward_on_the_ground_basis() {
        let mut body = Body::default();
        let mut space = Floor;
        let masks = Masks {
            collision: 1,
            pass_out: 0,
        };
        let m = Mover::new(&mut body, &mut space, masks, None, TUN);
        let mut s = State {
            mode: GROUND,
            ground_normal: UP,
            height: 1.8,
            ..State::default()
        };
        let mut p = post_in(1.0 / 64.0);
        p.move_y = 1.0;
        let cap = apply_wish_move(&m, &mut s, &TT, &p);
        assert_eq!(cap, 6.0);
        assert!(s.velocity.z < 0.0);
        assert_eq!(s.velocity.x, 0.0);
    }

    #[test]
    fn clamp_limits_and_zeroes_non_finite() {
        let mut s = State {
            velocity: Vec3::new(100.0, 0.0, 0.0),
            ..State::default()
        };
        assert!(!clamp_velocity(&mut s, &TT));
        assert_eq!(s.velocity.x, 50.0);
        s.velocity.y = f32::NAN;
        assert!(clamp_velocity(&mut s, &TT));
        assert_eq!(s.velocity, Vec3::default());
    }
}
