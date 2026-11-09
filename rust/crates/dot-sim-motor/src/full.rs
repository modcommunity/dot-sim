//! The whole of `DotFpsMotor._simulate` from the start ground check to the end one, for a plain
//! walker (GROUND or AIR, no surfaces, no edge friction): [`tick::pre`], the jump, the wall moves,
//! the launch, the dash, the slide and the wall run, then [`tick::post`]. The script keeps the
//! timers, effects, noclip, crouch and the idle tick, which run before it.
//!
//! The same arithmetic rules as [`crate::tick`]: script floats are `f64`, vectors `f32`.

use crate::slide::{Mover, capsule_centre};
use crate::tick::{self, AIR, GROUND, LEAVING_SPEED, Post, PostOut, PreOut, State, TickCounters, TickTun, len2};
use crate::{Hit, Space, UP, Vec3};
use dot_sim_math::{maxf, minf, rad_to_deg};
use std::f64::consts::{PI, TAU};

/// `DotFpsCommand.BUTTON_JUMP`.
pub const BUTTON_JUMP: i64 = 1;
/// `DotFpsCommand.BUTTON_CROUCH`.
pub const BUTTON_CROUCH: i64 = 2;
/// `DotFpsCommand.BUTTON_SPRINT`.
pub const BUTTON_SPRINT: i64 = 4;

const WALL_HEADINGS: i64 = 8;
const WALL_JUMP_COUNT_MAX: i64 = 3;
const WALL_PROBE_MIN_SPEED: f64 = 0.5;
const WALL_RUN_TIME_SLACK: f64 = 1e-4;
/// `DotFpsTunables.WALL_RUN_TIME_LIMIT`.
const WALL_RUN_TIME_LIMIT: f64 = 1.9;
const WALL_ENTRY_MIN_AIR: f64 = 0.08;
const WALL_PROBE_STEP: f64 = 0.25;
const WALL_PROBE_STANDOFF: f64 = 0.12;
const WALL_PROBE_RADIUS: f64 = 0.05;
const WALL_PROBE_HEIGHT: f64 = 0.12;

/// The movement tunables the moves read beyond [`TickTun`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FullTun {
    pub can_jump: bool,
    pub auto_hop: bool,
    pub jump_buffer_time: f64,
    pub jump_cooldown: f64,
    pub coyote_time: f64,
    pub bhop_speed_cap_scale: f64,
    pub jump_adds_to_velocity: bool,
    pub launch_enabled: bool,
    pub launch_button: i64,
    pub launch_from_air: bool,
    pub launch_velocity: f64,
    pub launch_forward: f64,
    pub launch_cooldown: f64,
    pub dash_enabled: bool,
    pub dash_button: i64,
    pub dash_from_air: bool,
    pub dash_speed: f64,
    pub dash_cooldown: f64,
    pub dash_lift: f64,
    pub slide_enabled: bool,
    pub slide_duration: f64,
    pub slide_end_speed: f64,
    pub can_crouch: bool,
    pub slide_requires_sprint: bool,
    pub slide_min_speed: f64,
    pub slide_boost: f64,
    pub slide_max_speed: f64,
    pub slide_cooldown: f64,
    pub friction: f64,
    pub slide_friction: f64,
    pub accelerate: f64,
    pub slide_accelerate: f64,
    pub wall_run_enabled: bool,
    pub wall_jump_enabled: bool,
    pub wall_run_auto: bool,
    pub wall_run_on_jump: bool,
    pub wall_run_jump_min_speed: f64,
    pub wall_run_min_speed: f64,
    pub wall_max_angle_from_vertical: f64,
    pub wall_reach: f64,
    pub wall_run_max_time: f64,
    pub wall_run_end_speed: f64,
    pub wall_run_requires_forward: bool,
    pub wall_run_max_fall_speed: f64,
    pub wall_run_max_approach_angle: f64,
    pub wall_run_min_height: f64,
    pub wall_run_min_length: f64,
    pub wall_run_cooldown: f64,
    pub wall_run_jump_max_angle: f64,
    pub wall_run_turn_view: bool,
    pub wall_run_turn_view_max_angle: f64,
    pub wall_jump_max_per_wall: i64,
    pub wall_jump_max_per_air: i64,
    pub wall_jump_look_blend: f64,
    pub wall_jump_push_speed: f64,
    pub wall_jump_up_speed: f64,
}

/// The command, effects and surface for this tick, multiplied out where the script multiplies.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FullIn {
    pub rest_ok: bool,
    pub rest_position: Vec3,
    pub delta: f64,
    pub buttons: i64,
    pub previous_buttons: i64,
    pub move_x: f64,
    pub move_y: f64,
    /// `state.yaw`, degrees.
    pub yaw: f64,
    pub deny_move: bool,
    pub deny_jump: bool,
    pub deny_crouch: bool,
    /// `_effects.jump`.
    pub effect_jump: f64,
    /// `tunables.jump_velocity() * _surface.jump_scale * _effects.jump`.
    pub launch: f64,
    pub speed_base: f64,
    pub effect_max_speed: f64,
    pub surface_max_speed_scale: f64,
    pub surface_friction: f64,
    pub effect_friction: f64,
    pub surface_accelerate: f64,
    pub effect_accelerate: f64,
    pub air_accel: f64,
    pub push: Vec3,
    pub gravity: f64,
    pub wall_run_gravity: f64,
}

/// The state's timers and wall fields the moves read and write.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Moves {
    pub time_since_jump: f64,
    pub time_since_grounded: f64,
    pub time_since_jump_pressed: f64,
    pub slide_time: f64,
    pub slide_cooldown_left: f64,
    pub launch_cooldown_left: f64,
    pub dash_cooldown_left: f64,
    pub wall_run_cooldown_left: f64,
    pub wall_run_time: f64,
    pub wall_run_spent: bool,
    pub wall_jump_count: i64,
    pub wall_jumps_in_air: i64,
    pub wall_jump_heading: i64,
    pub wall_normal: Vec3,
}

impl Moves {
    fn sliding(&self) -> bool {
        self.slide_time >= 0.0
    }

    fn wall_running(&self) -> bool {
        self.wall_run_time >= 0.0
    }
}

/// The motor's move counters this tick added to.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MoveCounters {
    pub slides: i64,
    pub launches: i64,
    pub dashes: i64,
    pub wall_runs: i64,
    pub wall_jumps: i64,
}

/// What [`tick`] answered.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FullOut {
    pub pre: PreOut,
    pub post: PostOut,
    /// `DotFpsMotor.view_turn`.
    pub view_turn: f64,
}

/// Everything one tick's moves read.
struct Ctx<'a> {
    tt: &'a TickTun,
    ft: &'a FullTun,
    inp: &'a FullIn,
}

fn deg_to_rad(d: f64) -> f64 {
    d * (PI / 180.0)
}

/// `_view_basis(yaw, 0.0)`: forward and right.
fn view_basis(yaw_degrees: f64) -> (Vec3, Vec3) {
    let yaw = deg_to_rad(yaw_degrees);
    let forward = Vec3::new((-yaw.sin()) as f32, 0.0, (-yaw.cos()) as f32);
    let right = Vec3::new(yaw.cos() as f32, 0.0, (-yaw.sin()) as f32);
    (forward, right)
}

/// `DotFpsMotorWalls.forward_for`.
fn forward_for(yaw_degrees: f64) -> Vec3 {
    let yaw = deg_to_rad(yaw_degrees);
    Vec3::new((-yaw.sin()) as f32, 0.0, (-yaw.cos()) as f32)
}

/// The script's `wrapf` on floats.
fn wrapf(value: f64, min: f64, max: f64) -> f64 {
    let range = max - min;
    if range.abs() < 0.00001 {
        return min;
    }
    let result = value - range * ((value - min) / range).floor();
    if approx64(result, max) {
        return min;
    }
    result
}

/// `Math::is_equal_approx` on doubles.
fn approx64(a: f64, b: f64) -> bool {
    if a == b {
        return true;
    }
    let mut tolerance = 0.00001 * a.abs();
    if tolerance < 0.00001 {
        tolerance = 0.00001;
    }
    (a - b).abs() < tolerance
}

fn horizontal_speed(s: &State) -> f64 {
    len2(s.velocity.x, s.velocity.z) as f64
}

fn flat(v: Vec3) -> Vec3 {
    Vec3::new(v.x, 0.0, v.z)
}

/// `DotFpsMotorWalls._flat_normal`.
fn flat_normal(normal: Vec3) -> Vec3 {
    let f = flat(normal);
    if (f.length_squared() as f64) > 1e-8 {
        f.normalized()
    } else {
        Vec3::default()
    }
}

/// `DotFpsMotorWalls.wall_heading_of`.
pub fn wall_heading_of(normal: Vec3) -> i64 {
    let turns = (normal.x as f64).atan2(normal.z as f64) / TAU;
    ((turns * WALL_HEADINGS as f64).round() as i64).rem_euclid(WALL_HEADINGS)
}

fn same_wall(a: i64, b: i64) -> bool {
    let apart = (a - b).rem_euclid(WALL_HEADINGS);
    apart <= 1 || apart >= WALL_HEADINGS - 1
}

/// `DotFpsMotor._try_jump`.
fn try_jump(s: &mut State, x: &Ctx, mv: &mut Moves) -> bool {
    let (ft, inp) = (x.ft, x.inp);
    if !ft.can_jump || inp.deny_jump {
        return false;
    }
    let wants = if ft.auto_hop {
        inp.buttons & BUTTON_JUMP != 0
    } else {
        mv.time_since_jump_pressed <= ft.jump_buffer_time
    };
    if !wants || mv.time_since_jump < ft.jump_cooldown {
        return false;
    }
    if !(s.mode == GROUND || mv.time_since_grounded <= ft.coyote_time) {
        return false;
    }
    let normal = if s.mode == GROUND { s.ground_normal } else { UP };
    if (s.velocity.dot(normal) as f64) > LEAVING_SPEED {
        return false;
    }
    let launch = inp.launch;
    if launch <= 0.0 {
        return false;
    }
    if ft.bhop_speed_cap_scale > 0.0 {
        let cap = x.tt.max_speed * ft.bhop_speed_cap_scale;
        let speed = horizontal_speed(s);
        if speed > cap && speed > 0.0 {
            let scale = cap / speed;
            s.velocity.x = ((s.velocity.x as f64) * scale) as f32;
            s.velocity.z = ((s.velocity.z as f64) * scale) as f32;
        }
    }
    s.velocity.y = if ft.jump_adds_to_velocity {
        (maxf(s.velocity.y as f64, 0.0) + launch) as f32
    } else {
        launch as f32
    };
    s.mode = AIR;
    mv.time_since_jump = 0.0;
    mv.time_since_grounded = 1000.0;
    mv.time_since_jump_pressed = 1000.0;
    true
}

/// `DotFpsMotor._try_launch`.
fn try_launch(s: &mut State, x: &Ctx, mv: &mut Moves, mc: &mut MoveCounters) -> bool {
    let (ft, inp) = (x.ft, x.inp);
    if !ft.launch_enabled || ft.launch_button <= 0 || inp.deny_jump || inp.deny_move {
        return false;
    }
    if mv.launch_cooldown_left > 0.0 {
        return false;
    }
    if (inp.buttons & !inp.previous_buttons) & ft.launch_button == 0 {
        return false;
    }
    if !ft.launch_from_air && !(s.mode == GROUND || mv.time_since_grounded <= ft.coyote_time) {
        return false;
    }
    s.velocity.y = maxf(s.velocity.y as f64, ft.launch_velocity) as f32;
    if ft.launch_forward > 0.0 {
        let mut forward = view_basis(inp.yaw).0;
        forward.y = 0.0;
        if (forward.length_squared() as f64) > 0.0 {
            s.velocity = s.velocity + forward.normalized().scale(ft.launch_forward);
        }
    }
    s.mode = AIR;
    mv.time_since_grounded = 1000.0;
    mv.launch_cooldown_left = ft.launch_cooldown;
    mc.launches += 1;
    true
}

/// `DotFpsMotor._try_dash`.
fn try_dash(s: &mut State, x: &Ctx, mv: &mut Moves, mc: &mut MoveCounters) -> bool {
    let (ft, inp) = (x.ft, x.inp);
    if !ft.dash_enabled || ft.dash_button <= 0 || inp.deny_move || mv.dash_cooldown_left > 0.0 {
        return false;
    }
    if (inp.buttons & !inp.previous_buttons) & ft.dash_button == 0 {
        return false;
    }
    if !ft.dash_from_air && s.mode != GROUND {
        return false;
    }
    let (forward, right) = view_basis(inp.yaw);
    let mut direction = right.scale(inp.move_x) + forward.scale(inp.move_y);
    direction.y = 0.0;
    if (direction.length_squared() as f64) < 0.0001 {
        direction = forward;
        direction.y = 0.0;
    }
    direction = direction.normalized();
    let f = flat(s.velocity);
    let along = f.dot(direction) as f64;
    let across = f - direction.scale(along);
    let speed = maxf(along, ft.dash_speed);
    let dashed = direction.scale(speed) + across;
    s.velocity.x = dashed.x;
    s.velocity.z = dashed.z;
    mv.dash_cooldown_left = ft.dash_cooldown;
    mc.dashes += 1;
    if ft.dash_lift > 0.0 {
        s.velocity.y = maxf(s.velocity.y as f64, ft.dash_lift) as f32;
        s.mode = AIR;
        mv.time_since_grounded = 1000.0;
        return true;
    }
    false
}

fn end_slide(x: &Ctx, mv: &mut Moves) {
    mv.slide_time = -1.0;
    mv.slide_cooldown_left = x.ft.slide_cooldown;
}

/// `DotFpsMotor._update_slide`.
fn update_slide(s: &mut State, x: &Ctx, mv: &mut Moves, left_ground: bool, mc: &mut MoveCounters) {
    let (ft, inp) = (x.ft, x.inp);
    if !ft.slide_enabled {
        if mv.sliding() {
            end_slide(x, mv);
        }
        return;
    }
    if mv.sliding() {
        mv.slide_time += inp.delta;
        if left_ground
            || s.mode != GROUND
            || inp.buttons & BUTTON_CROUCH == 0
            || mv.slide_time >= ft.slide_duration
            || horizontal_speed(s) < ft.slide_end_speed
            || inp.deny_move
        {
            end_slide(x, mv);
        }
        return;
    }
    if left_ground || s.mode != GROUND || mv.slide_cooldown_left > 0.0 {
        return;
    }
    if !ft.can_crouch || inp.deny_crouch || inp.deny_move {
        return;
    }
    if (inp.buttons & !inp.previous_buttons) & BUTTON_CROUCH == 0 {
        return;
    }
    if ft.slide_requires_sprint && inp.buttons & BUTTON_SPRINT == 0 {
        return;
    }
    let speed = horizontal_speed(s);
    if speed < ft.slide_min_speed || speed <= 0.0 {
        return;
    }
    mv.slide_time = 0.0;
    mc.slides += 1;
    if ft.slide_boost > 0.0 && speed < ft.slide_max_speed {
        let boosted = minf(speed + ft.slide_boost, ft.slide_max_speed);
        let scale = boosted / speed;
        s.velocity.x = ((s.velocity.x as f64) * scale) as f32;
        s.velocity.z = ((s.velocity.z as f64) * scale) as f32;
    }
}

/// `DotFpsMotorWalls._wall_contact`; also writes `wall_normal`.
fn wall_contact<S: Space>(m: &mut Mover<'_, S>, s: &State, x: &Ctx, mv: &mut Moves) -> Option<Hit> {
    let (ft, inp) = (x.ft, x.inp);
    if !ft.wall_run_enabled && !ft.wall_jump_enabled && x.tt.wall_slide_speed <= 0.0 {
        return None;
    }
    mv.wall_normal = Vec3::default();
    if s.mode != AIR {
        return None;
    }
    let wants_run = ft.wall_run_enabled
        && (mv.wall_running()
            || (!mv.wall_run_spent
                && horizontal_speed(s)
                    >= if ft.wall_run_auto {
                        ft.wall_run_jump_min_speed
                    } else {
                        ft.wall_run_min_speed
                    }));
    let wants_jump = (ft.wall_jump_enabled || (ft.wall_run_enabled && ft.wall_run_on_jump))
        && mv.time_since_jump_pressed <= ft.jump_buffer_time;
    let wants_slide = x.tt.wall_slide_speed > 0.0
        && (s.velocity.y as f64) < 0.0
        && (inp.move_x != 0.0 || inp.move_y != 0.0 || mv.wall_run_spent);
    if !wants_run && !wants_jump && !wants_slide {
        return None;
    }

    let mut directions = [Vec3::default(); 6];
    let mut count = 0;
    let f = flat(s.velocity);
    if (f.length() as f64) >= WALL_PROBE_MIN_SPEED {
        let side = f.normalized().cross(UP);
        directions[0] = side;
        directions[1] = -side;
        count = 2;
    }
    let (forward, right) = view_basis(inp.yaw);
    for d in [forward, -forward, right, -right] {
        directions[count] = d;
        count += 1;
    }

    let height = s.height;
    let centre = capsule_centre(s.position, height);
    let steepest = deg_to_rad(90.0 - ft.wall_max_angle_from_vertical).cos();
    let sideways = if mv.wall_running() && count == 6 { 2 } else { 0 };
    let mut best: Option<Hit> = None;
    let mut beside: Option<Hit> = None;
    for (index, &direction) in directions[..count].iter().enumerate() {
        let hit = m.sweep(centre, direction.scale(ft.wall_reach), height);
        if !hit.hit || (hit.normal.y.abs() as f64) > steepest {
            continue;
        }
        if (hit.normal.dot(direction) as f64) >= 0.0 {
            continue;
        }
        if index < sideways && beside.is_none_or(|b| hit.fraction < b.fraction) {
            beside = Some(hit);
        }
        if best.is_none_or(|b| hit.fraction < b.fraction) {
            best = Some(hit);
        }
    }
    if beside.is_some() {
        best = beside;
    }
    if let Some(b) = best {
        mv.wall_normal = b.normal;
    }
    best
}

/// `DotFpsMotorWalls._wall_probe`: 2 inside solid, 1 the face found, 0 not.
fn wall_probe<S: Space>(m: &mut Mover<'_, S>, at: Vec3, normal: Vec3, y: f64) -> i64 {
    let origin = Vec3::new(at.x, y as f32, at.z) + normal.scale(WALL_PROBE_STANDOFF);
    if m.body.overlaps(m.space, origin, WALL_PROBE_HEIGHT, WALL_PROBE_RADIUS, m.masks) {
        return 2;
    }
    let motion = (-normal).scale(WALL_PROBE_STANDOFF + 0.2);
    let hit = m.sweep_r(origin, motion, WALL_PROBE_HEIGHT, WALL_PROBE_RADIUS);
    if hit.hit && (flat_normal(hit.normal).dot(normal) as f64) > 0.9 {
        return 1;
    }
    0
}

fn wall_extent<S: Space>(m: &mut Mover<'_, S>, base: Vec3, normal: Vec3, y: f64, sign_y: f64, limit: f64) -> f64 {
    let mut reached = 0.0;
    while reached < limit {
        let found = wall_probe(m, base, normal, y + sign_y * (reached + WALL_PROBE_STEP));
        if found == 2 {
            return reached;
        }
        if found == 0 {
            break;
        }
        reached += WALL_PROBE_STEP;
    }
    reached
}

fn wall_extent_along<S: Space>(m: &mut Mover<'_, S>, base: Vec3, normal: Vec3, tangent: Vec3, y: f64, limit: f64) -> f64 {
    let mut reached = 0.0;
    while reached < limit {
        let at = base + tangent.scale(reached + WALL_PROBE_STEP);
        if wall_probe(m, at, normal, y) != 1 {
            break;
        }
        reached += WALL_PROBE_STEP;
    }
    reached
}

/// `DotFpsMotorWalls._wall_runnable`.
fn wall_runnable<S: Space>(m: &mut Mover<'_, S>, s: &State, x: &Ctx, wall: &Hit) -> bool {
    let (min_height, min_length) = (x.ft.wall_run_min_height, x.ft.wall_run_min_length);
    if min_height <= 0.0 && min_length <= 0.0 {
        return true;
    }
    let normal = flat_normal(wall.normal);
    if normal == Vec3::default() {
        return false;
    }
    let height = s.height;
    let feet_y = s.position.y as f64;
    let base = Vec3::new(wall.point.x, 0.0, wall.point.z);
    let mut contact_y = f64::NEG_INFINITY;
    let mut level = feet_y + height;
    while level >= feet_y + 0.05 && contact_y == f64::NEG_INFINITY {
        if wall_probe(m, base, normal, level) == 1 {
            contact_y = level;
        }
        level -= WALL_PROBE_STEP * 0.5;
    }
    if contact_y == f64::NEG_INFINITY {
        return false;
    }
    if min_height > 0.0 {
        let top = wall_extent(m, base, normal, contact_y, 1.0, min_height);
        let bottom = wall_extent(m, base, normal, contact_y, -1.0, min_height);
        if top + bottom < min_height - 0.001 || contact_y + top < feet_y + height * 0.5 {
            return false;
        }
    }
    if min_length > 0.0 {
        let mut tangent = normal.cross(UP);
        if (tangent.dot(flat(s.velocity)) as f64) < 0.0 {
            tangent = -tangent;
        }
        let ahead = wall_extent_along(m, base, normal, tangent, contact_y, min_length);
        if ahead < min_length - 0.001 {
            return false;
        }
    }
    true
}

fn reset_walls(mv: &mut Moves) {
    mv.wall_run_time = -1.0;
    mv.wall_run_spent = false;
    mv.wall_jump_count = 0;
    mv.wall_jumps_in_air = 0;
}

fn end_wall_run(x: &Ctx, mv: &mut Moves) {
    mv.wall_run_time = -1.0;
    mv.wall_run_spent = true;
    mv.wall_run_cooldown_left = x.ft.wall_run_cooldown;
}

fn speed_along_wall(s: &State, wall: &Hit) -> f64 {
    let normal = flat_normal(wall.normal);
    let f = flat(s.velocity);
    (f - normal.scale(f.dot(normal) as f64)).length() as f64
}

/// `DotFpsMotorWalls._turn_along`.
fn turn_along(s: &mut State, x: &Ctx, mv: &mut Moves, turned: Vec3, view_turn: &mut f64, mc: &mut MoveCounters) {
    s.velocity.x = turned.x;
    s.velocity.z = turned.z;
    mv.wall_run_time = 0.0;
    mc.wall_runs += 1;
    if x.ft.wall_run_turn_view {
        let view = forward_for(x.inp.yaw);
        let limit = x.ft.wall_run_turn_view_max_angle;
        if limit >= 180.0 || (view.dot(turned.normalized()) as f64) > deg_to_rad(limit).cos() + 1e-6 {
            let heading = rad_to_deg((-(turned.x as f64)).atan2(-(turned.z as f64)));
            *view_turn = wrapf(heading - x.inp.yaw, -180.0, 180.0);
        }
    }
}

/// The checks `_try_jump_run` and `_try_auto_run` share after their own: the run's heading, or none.
fn run_heading<S: Space>(m: &mut Mover<'_, S>, s: &State, x: &Ctx, wall: &Hit) -> Option<Vec3> {
    let normal = flat_normal(wall.normal);
    let f = flat(s.velocity);
    let speed = f.length() as f64;
    if normal == Vec3::default() || speed < x.ft.wall_run_jump_min_speed {
        return None;
    }
    let into = -(f.dot(normal)) as f64;
    if into < -LEAVING_SPEED {
        return None;
    }
    let mut tangent = normal.cross(UP);
    if (tangent.dot(f) as f64) < 0.0 {
        tangent = -tangent;
    }
    let approach = rad_to_deg(maxf(into, 0.0).atan2(tangent.dot(f).abs() as f64));
    if approach > x.ft.wall_run_jump_max_angle {
        return None;
    }
    if !wall_runnable(m, s, x, wall) {
        return None;
    }
    Some(tangent.scale(speed))
}

/// `DotFpsMotorWalls._try_jump_run`.
fn try_jump_run<S: Space>(
    m: &mut Mover<'_, S>,
    s: &mut State,
    x: &Ctx,
    mv: &mut Moves,
    wall: Option<&Hit>,
    view_turn: &mut f64,
    mc: &mut MoveCounters,
) -> bool {
    let (ft, inp) = (x.ft, x.inp);
    let Some(wall) = wall else { return false };
    if !(ft.wall_run_enabled && ft.wall_run_on_jump) || ft.wall_run_auto {
        return false;
    }
    if s.mode != AIR || mv.wall_running() || mv.wall_run_spent {
        return false;
    }
    if mv.wall_run_cooldown_left > 0.0 || inp.deny_move || inp.deny_jump {
        return false;
    }
    if mv.time_since_grounded < WALL_ENTRY_MIN_AIR || mv.time_since_jump_pressed > ft.jump_buffer_time {
        return false;
    }
    let Some(turned) = run_heading(m, s, x, wall) else { return false };
    turn_along(s, x, mv, turned, view_turn, mc);
    mv.time_since_jump_pressed = 1000.0;
    true
}

/// `DotFpsMotorWalls._try_auto_run`.
fn try_auto_run<S: Space>(
    m: &mut Mover<'_, S>,
    s: &mut State,
    x: &Ctx,
    mv: &mut Moves,
    wall: Option<&Hit>,
    view_turn: &mut f64,
    mc: &mut MoveCounters,
) {
    let (ft, inp) = (x.ft, x.inp);
    let Some(wall) = wall else { return };
    if !(ft.wall_run_enabled && ft.wall_run_auto) {
        return;
    }
    if s.mode != AIR || mv.wall_running() || mv.wall_run_spent {
        return;
    }
    if mv.wall_run_cooldown_left > 0.0 || inp.deny_move || mv.time_since_grounded < WALL_ENTRY_MIN_AIR {
        return;
    }
    if ft.wall_run_requires_forward && inp.move_y <= 0.0 {
        return;
    }
    if (s.velocity.y as f64) < -ft.wall_run_max_fall_speed {
        return;
    }
    if let Some(turned) = run_heading(m, s, x, wall) {
        turn_along(s, x, mv, turned, view_turn, mc);
    }
}

/// `DotFpsMotorWalls._try_wall_jump`.
fn try_wall_jump(s: &mut State, x: &Ctx, mv: &mut Moves, wall: Option<&Hit>, mc: &mut MoveCounters) -> bool {
    let (ft, inp) = (x.ft, x.inp);
    let Some(wall) = wall else { return false };
    if !ft.wall_jump_enabled {
        return false;
    }
    if !ft.can_jump || inp.deny_jump || inp.deny_move || inp.effect_jump <= 0.0 || s.mode != AIR {
        return false;
    }
    if mv.time_since_jump_pressed > ft.jump_buffer_time || mv.time_since_jump < ft.jump_cooldown {
        return false;
    }
    let normal = flat_normal(wall.normal);
    if normal == Vec3::default() {
        return false;
    }
    let heading = wall_heading_of(normal);
    let same = mv.wall_jump_count > 0 && same_wall(heading, mv.wall_jump_heading);
    if same && ft.wall_jump_max_per_wall > 0 && mv.wall_jump_count >= ft.wall_jump_max_per_wall {
        return false;
    }
    if ft.wall_jump_max_per_air > 0 && mv.wall_jumps_in_air >= ft.wall_jump_max_per_air {
        return false;
    }
    let mut look = view_basis(inp.yaw).0;
    look.y = 0.0;
    look = look.normalized();
    look = look - normal.scale(minf(look.dot(normal) as f64, 0.0));
    let blend = ft.wall_jump_look_blend;
    let mut push = normal.scale(1.0 - blend) + look.scale(blend);
    push = if (push.length_squared() as f64) > 1e-8 { push.normalized() } else { normal };
    let f = flat(s.velocity);
    let kept = f - normal.scale(minf(f.dot(normal) as f64, 0.0));
    let jumped = kept + push.scale(ft.wall_jump_push_speed).scale(inp.effect_jump);
    s.velocity.x = jumped.x;
    s.velocity.z = jumped.z;
    s.velocity.y = maxf(s.velocity.y as f64, ft.wall_jump_up_speed * inp.effect_jump) as f32;
    mv.time_since_jump = 0.0;
    mv.time_since_jump_pressed = 1000.0;
    mv.time_since_grounded = 1000.0;
    mv.wall_run_time = -1.0;
    mv.wall_run_spent = false;
    mv.wall_jump_count = if same { (mv.wall_jump_count + 1).min(WALL_JUMP_COUNT_MAX) } else { 1 };
    mv.wall_jumps_in_air = (mv.wall_jumps_in_air + 1).min(WALL_JUMP_COUNT_MAX);
    mv.wall_jump_heading = heading;
    mc.wall_jumps += 1;
    true
}

/// `DotFpsMotorWalls._update_wall_run`.
fn update_wall_run<S: Space>(
    m: &mut Mover<'_, S>,
    s: &State,
    x: &Ctx,
    mv: &mut Moves,
    wall: Option<&Hit>,
    left_this_tick: bool,
    mc: &mut MoveCounters,
) {
    let (ft, inp) = (x.ft, x.inp);
    if !ft.wall_run_enabled && !ft.wall_jump_enabled {
        return;
    }
    if s.mode == GROUND {
        reset_walls(mv);
        return;
    }
    if !ft.wall_run_enabled {
        if mv.wall_running() {
            end_wall_run(x, mv);
        }
        return;
    }
    if mv.wall_running() {
        mv.wall_run_time += inp.delta;
        let ends = left_this_tick
            || match wall {
                None => true,
                Some(w) => {
                    inp.deny_move
                        || mv.wall_run_time
                            >= minf(ft.wall_run_max_time, WALL_RUN_TIME_LIMIT) - WALL_RUN_TIME_SLACK
                        || speed_along_wall(s, w) < ft.wall_run_end_speed
                        || (ft.wall_run_requires_forward && inp.move_y <= 0.0)
                        || (!ft.wall_jump_enabled
                            && ft.wall_run_on_jump
                            && mv.time_since_jump_pressed <= ft.jump_buffer_time)
                }
            };
        if ends && mv.wall_running() {
            end_wall_run(x, mv);
        }
        return;
    }
    let Some(wall) = wall else { return };
    if left_this_tick || mv.wall_run_spent || inp.deny_move {
        return;
    }
    if s.mode != AIR || mv.wall_run_cooldown_left > 0.0 || mv.time_since_grounded < WALL_ENTRY_MIN_AIR {
        return;
    }
    if ft.wall_run_requires_forward && inp.move_y <= 0.0 {
        return;
    }
    if (s.velocity.y as f64) < -ft.wall_run_max_fall_speed {
        return;
    }
    let normal = flat_normal(wall.normal);
    let into = -(flat(s.velocity).dot(normal)) as f64;
    if into < -LEAVING_SPEED {
        return;
    }
    let along = speed_along_wall(s, wall);
    if along < ft.wall_run_min_speed {
        return;
    }
    let approach = rad_to_deg(maxf(into, 0.0).atan2(along));
    if approach > ft.wall_run_max_approach_angle || !wall_runnable(m, s, x, wall) {
        return;
    }
    mv.wall_run_time = 0.0;
    mc.wall_runs += 1;
}

/// `DotFpsMotorWalls._wall_slide_ok`.
fn wall_slide_ok(s: &State, x: &Ctx, mv: &Moves, wall: Option<&Hit>) -> bool {
    let Some(wall) = wall else { return false };
    if x.tt.wall_slide_speed <= 0.0 || s.mode != AIR || mv.wall_running() {
        return false;
    }
    if mv.wall_run_spent {
        return true;
    }
    let (forward, right) = view_basis(x.inp.yaw);
    let mut wish = right.scale(x.inp.move_x) + forward.scale(x.inp.move_y);
    wish.y = 0.0;
    if (wish.length_squared() as f64) < 1e-6 {
        return false;
    }
    (wish.normalized().dot(-flat_normal(wall.normal)) as f64) > 0.5
}

/// One tick from the start ground check to the end one, in the script's order.
#[allow(clippy::too_many_arguments)]
pub fn tick<S: Space>(
    m: &mut Mover<'_, S>,
    s: &mut State,
    tt: &TickTun,
    ft: &FullTun,
    inp: &FullIn,
    mv: &mut Moves,
    c: &mut TickCounters,
    mc: &mut MoveCounters,
) -> FullOut {
    let x = Ctx { tt, ft, inp };
    let pre = tick::pre(m, s, tt, inp.rest_ok, inp.rest_position, c);
    let start_position = s.position;
    let start_velocity = s.velocity;
    let mut view_turn = 0.0;

    let jumped = try_jump(s, &x, mv);
    let wall = wall_contact(m, s, &x, mv);
    let wall = wall.as_ref();
    let mut wall_jumped = false;
    if !jumped && !try_jump_run(m, s, &x, mv, wall, &mut view_turn, mc) {
        wall_jumped = try_wall_jump(s, &x, mv, wall, mc);
        if !wall_jumped {
            try_auto_run(m, s, &x, mv, wall, &mut view_turn, mc);
        }
    }
    let mut launched = try_launch(s, &x, mv, mc);
    launched = try_dash(s, &x, mv, mc) || launched;

    update_slide(s, &x, mv, jumped || launched, mc);
    update_wall_run(m, s, &x, mv, wall, jumped || wall_jumped || launched, mc);

    let sliding = mv.sliding();
    let p = Post {
        delta: inp.delta,
        jumped: jumped || launched,
        sliding,
        deny_move: inp.deny_move,
        wall_slide_ok: wall_slide_ok(s, &x, mv, wall),
        start_settled: pre.start_settled,
        move_x: inp.move_x,
        move_y: inp.move_y,
        yaw: inp.yaw,
        speed_base: inp.speed_base,
        effect_max_speed: inp.effect_max_speed,
        surface_max_speed_scale: inp.surface_max_speed_scale,
        friction_base: if sliding { ft.slide_friction } else { ft.friction },
        surface_friction: inp.surface_friction,
        effect_friction: inp.effect_friction,
        ground_accel: (if sliding { ft.slide_accelerate } else { ft.accelerate })
            * inp.surface_accelerate
            * inp.effect_accelerate,
        air_accel: inp.air_accel,
        push: inp.push,
        gravity: inp.gravity,
        start_position,
        start_velocity,
        wall_running: mv.wall_running(),
        wall_run_gravity: inp.wall_run_gravity,
    };
    let post = tick::post(m, s, tt, &p, c);
    FullOut { pre, post, view_turn }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::slide::Tun;
    use crate::{Body, Contact, Masks};

    /// Open space: nothing is ever hit.
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
        max_slope: 46.0 * PI / 180.0,
        math_checks: true,
    };

    #[test]
    fn wrapf_is_the_engine_wrap() {
        assert_eq!(wrapf(0.5, -PI, PI), 0.5);
        assert_eq!(wrapf(PI + 0.25, -PI, PI), -PI + 0.25);
        assert_eq!(wrapf(PI, -PI, PI), -PI);
        assert_eq!(wrapf(3.0, 1.0, 1.0), 1.0);
    }

    #[test]
    fn a_dash_fires_once_on_the_press_edge() {
        let (mut b, mut sp) = (Body::default(), Empty);
        let mut m = Mover::new(&mut b, &mut sp, Masks::default(), Some(&[]), TUN);
        let tt = TickTun { max_velocity: 3500.0, ..TickTun::default() };
        let ft = FullTun {
            dash_enabled: true,
            dash_button: 32,
            dash_from_air: true,
            dash_speed: 10.0,
            dash_cooldown: 0.5,
            ..FullTun::default()
        };
        let mut inp = FullIn { delta: 1.0 / 66.0, buttons: 32, speed_base: 6.0, ..FullIn::default() };
        let mut s = State { height: 1.8, mode: AIR, ..State::default() };
        let (mut mv, mut c, mut mc) = (Moves::default(), TickCounters::default(), MoveCounters::default());
        tick(&mut m, &mut s, &tt, &ft, &inp, &mut mv, &mut c, &mut mc);
        assert_eq!(mc.dashes, 1);
        assert!(mv.dash_cooldown_left > 0.0);
        assert!((horizontal_speed(&s) - 10.0).abs() < 1e-4);
        inp.previous_buttons = 32;
        mv.dash_cooldown_left = 0.0;
        tick(&mut m, &mut s, &tt, &ft, &inp, &mut mv, &mut c, &mut mc);
        assert_eq!(mc.dashes, 1, "a held button does not dash again");
    }
}
