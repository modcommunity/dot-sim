//! DotBotAim's private half: the aim error, the flick overshoot, the tracked velocity, the turn,
//! the bursts. The view (yaw, pitch) and the engagement's public times stay with the caller and
//! are passed in, so the script keeps owning what games read.

use crate::num::{
    Stream, Vec2, Vec3, clampf, deg_to_rad, finite3, lerp3, maxf, minf, rad_to_deg, scale3, wrapf,
};

/// Seconds of a target's velocity error that bot_track_lag turns into aim error.
pub const LAG_REACH: f64 = 0.15;
/// Seconds a bot with no prediction trails a moving target by.
pub const TRACK_LAG: f64 = 0.08;
/// Seconds a flinch takes to halve.
pub const FLINCH_HALF_LIFE: f64 = 0.3;

/// The profile fields the aim reads (DotBotProfile's names).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AimProfile {
    pub prediction: f64,
    pub track_lag: f64,
    pub overshoot: f64,
    pub aim_floor: f64,
    pub aim_settle: f64,
    pub fire_cone: f64,
    pub turn_smooth: f64,
    pub turn_rate: f64,
    pub max_range: f64,
    pub burst_range: f64,
    pub burst: f64,
    pub burst_pause: f64,
}

/// The view, command degrees: yaw 0 looks down -Z, positive turns left.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct View {
    pub yaw: f64,
    pub pitch: f64,
}

/// The aim's private state, as DotBotAim's underscored fields, and its random stream.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aim {
    pub err_mag0: f64,
    pub err_angle: f64,
    pub err_spin: f64,
    pub over: Vec2,
    pub vel_est: Vec3,
    pub prev_gap: f64,
    pub flinch: f64,
    pub want: Vec2,
    pub want_prev: Vec2,
    pub in_burst: bool,
    pub burst_until: f64,
    pub pause_until: f64,
    pub rng: Stream,
}

impl Default for Aim {
    fn default() -> Self {
        Self::new(0)
    }
}

/// (yaw, pitch) in command degrees from `from` toward `to`.
#[inline]
pub fn angles_to(from: Vec3, to: Vec3) -> Vec2 {
    let d = to - from;
    let flat = Vec2::new(d.x, d.z).length() as f64;
    Vec2::of(
        rad_to_deg((-(d.x as f64)).atan2(-(d.z as f64))),
        rad_to_deg((d.y as f64).atan2(flat)),
    )
}

/// The direction a (yaw, pitch) looks along, -Z forward at yaw 0.
#[inline]
pub fn direction(yaw: f64, pitch: f64) -> Vec3 {
    let y = deg_to_rad(yaw);
    let p = deg_to_rad(pitch);
    Vec3::new(
        (-y.sin() * p.cos()) as f32,
        p.sin() as f32,
        (-y.cos() * p.cos()) as f32,
    )
}

impl Aim {
    /// A fresh aim whose stream has `key` (the script's `stream(&"dot_bot_aim")`).
    pub fn new(key: i64) -> Self {
        Self {
            err_mag0: 0.0,
            err_angle: 0.0,
            err_spin: 1.0,
            over: Vec2::ZERO,
            vel_est: Vec3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY),
            prev_gap: 0.0,
            flinch: 0.0,
            want: Vec2::NAN,
            want_prev: Vec2::NAN,
            in_burst: false,
            burst_until: f64::NEG_INFINITY,
            pause_until: f64::NEG_INFINITY,
            rng: Stream::new(key, 0),
        }
    }

    /// `engage` past its continue check: draws the reaction and, for a new engagement, the error.
    /// Returns the new `react_at`.
    #[allow(clippy::too_many_arguments)]
    pub fn engage(
        &mut self,
        resumed: bool,
        now: f64,
        off: f64,
        reaction: f64,
        react_jitter: f64,
        notice: f64,
        aim_error: f64,
    ) -> f64 {
        let mut jitter = 0.85 + 0.3 * self.rng.next_unit();
        self.in_burst = false;
        if resumed {
            return now + reaction * jitter * 0.5;
        }
        if react_jitter > 0.0 {
            let u = self.rng.next_unit();
            jitter *= 1.0 + react_jitter * 1.2 * u * u;
        }
        let react_at = now + reaction * jitter + notice * clampf(off / 90.0, 0.0, 2.0);
        self.err_mag0 = aim_error * (0.75 + 0.5 * self.rng.next_unit());
        self.err_angle = self.rng.next_unit() * std::f64::consts::TAU;
        let spin = 0.6 + self.rng.next_unit();
        self.err_spin = spin
            * if self.rng.next_unit() < 0.5 {
                1.0
            } else {
                -1.0
            };
        self.want_prev = Vec2::NAN;
        self.over = Vec2::ZERO;
        self.prev_gap = 0.0;
        self.vel_est = Vec3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY);
        react_at
    }

    pub fn release(&mut self) {
        self.in_burst = false;
        self.over = Vec2::ZERO;
        self.want = Vec2::NAN;
    }

    pub fn forget(&mut self) {
        self.release();
        self.want_prev = Vec2::NAN;
    }

    pub fn hurt(&mut self, flinch: f64) {
        self.flinch += flinch;
    }

    #[inline]
    pub fn decay(&mut self, dt: f64) {
        if dt > 0.0 {
            self.flinch *= 0.5f64.powf(dt / FLINCH_HALF_LIFE);
        }
    }

    /// Degrees of aim error at `now`.
    #[inline]
    pub fn error_deg(&self, now: f64, engaged_at: f64, aim_floor: f64, aim_settle: f64) -> f64 {
        let age = if engaged_at > f64::NEG_INFINITY {
            maxf(now - engaged_at, 0.0)
        } else {
            0.0
        };
        let settled = aim_floor
            + maxf(self.err_mag0 - aim_floor, 0.0) * 0.5f64.powf(age / maxf(aim_settle, 0.01));
        settled + self.flinch
    }

    /// `track` without an aim solver; returns nothing, the view is turned in place.
    #[allow(clippy::too_many_arguments)]
    pub fn track(
        &mut self,
        v: &mut View,
        eye: Vec3,
        point: Vec3,
        velocity: Vec3,
        now: f64,
        dt: f64,
        projectile_speed: f64,
        engaged_at: f64,
        p: &AimProfile,
    ) {
        self.decay(dt);
        let mut aim = point - scale3(scale3(velocity, 1.0 - p.prediction), TRACK_LAG);
        if p.track_lag > 0.0 {
            if finite3(self.vel_est) && dt > 0.0 {
                self.vel_est = lerp3(self.vel_est, velocity, 1.0 - (-dt / p.track_lag).exp());
            } else {
                self.vel_est = velocity;
            }
            aim = aim + scale3(self.vel_est - velocity, LAG_REACH);
        }
        if projectile_speed > 0.0 {
            let lead = eye.distance_to(aim) as f64 / projectile_speed;
            aim = aim + scale3(scale3(velocity, lead), p.prediction);
        }
        let mut want = angles_to(eye, aim);
        self.err_angle += self.err_spin * dt;
        let e = self.error_deg(now, engaged_at, p.aim_floor, p.aim_settle)
            * (0.55 + 0.45 * (self.err_angle * 1.7 + 0.3).sin());
        want = want + Vec2::of(e * self.err_angle.cos(), e * self.err_angle.sin() * 0.6);
        if p.overshoot > 0.0 {
            want = want + self.overshoot(v, want, dt, p.overshoot);
        }
        self.want = want;
        self.turn_to(v, want, dt, p.prediction, p.turn_smooth, p.turn_rate);
    }

    fn overshoot(&mut self, v: &View, want: Vec2, dt: f64, overshoot: f64) -> Vec2 {
        let gap = Vec2::of(
            wrapf(want.x as f64 - v.yaw, -180.0, 180.0),
            want.y as f64 - v.pitch,
        );
        let size = gap.length() as f64;
        if size > 20.0 && self.prev_gap < 10.0 && self.over.is_zero() {
            self.over = gap
                .normalized()
                .scale(minf(size, 90.0))
                .scale(overshoot)
                .scale(0.1 + 0.2 * self.rng.next_unit());
        } else if (size < 6.0 || (gap.dot(self.over) as f64) < 0.0) && dt > 0.0 {
            self.over = self.over.scale(0.5f64.powf(dt / 0.12));
            if (self.over.length() as f64) < 0.05 {
                self.over = Vec2::ZERO;
            }
        }
        self.prev_gap = size;
        self.over
    }

    /// `look_at_point`: the view toward `point` with no aim error.
    #[allow(clippy::too_many_arguments)]
    pub fn look(
        &mut self,
        v: &mut View,
        eye: Vec3,
        point: Vec3,
        dt: f64,
        prediction: f64,
        turn_smooth: f64,
        turn_rate: f64,
    ) {
        self.decay(dt);
        self.want = Vec2::NAN;
        self.turn_to(
            v,
            angles_to(eye, point),
            dt,
            prediction,
            turn_smooth,
            turn_rate,
        );
    }

    /// `turn_to`: carry the target's motion, close the rest smoothly, cap at turn_rate.
    pub fn turn_to(
        &mut self,
        v: &mut View,
        want: Vec2,
        dt: f64,
        prediction: f64,
        turn_smooth: f64,
        turn_rate: f64,
    ) {
        if dt <= 0.0 {
            self.want_prev = want;
            return;
        }
        let mut carry = Vec2::ZERO;
        if !self.want_prev.x.is_nan() {
            carry = Vec2::of(
                wrapf(want.x as f64 - self.want_prev.x as f64, -180.0, 180.0),
                want.y as f64 - self.want_prev.y as f64,
            )
            .scale(0.5 + 0.5 * prediction);
        }
        self.want_prev = want;

        let diff = Vec2::of(
            wrapf(want.x as f64 - v.yaw, -180.0, 180.0),
            want.y as f64 - v.pitch,
        );
        let k = clampf(turn_smooth * dt * 12.0, 0.0, 1.0);
        let mut step = carry + (diff - carry).scale(k);
        if step.length() > diff.length() {
            step = diff;
        }
        let limit = turn_rate * dt;
        if (step.length() as f64) > limit {
            step = step.normalized().scale(limit);
        }
        v.yaw = wrapf(v.yaw + step.x as f64, -180.0, 180.0);
        v.pitch = clampf(v.pitch + step.y as f64, -85.0, 85.0);
    }

    /// Whether the view is within `fire_cone` of where the bot means to aim.
    #[inline]
    pub fn on_aim(&self, v: &View, fire_cone: f64) -> bool {
        if self.want.x.is_nan() {
            return false;
        }
        let to = direction(self.want.x as f64, self.want.y as f64);
        rad_to_deg(direction(v.yaw, v.pitch).angle_to(to) as f64) <= fire_cone
    }

    /// Whether the trigger is down this tick.
    pub fn trigger(
        &mut self,
        v: &View,
        now: f64,
        distance: f64,
        react_at: f64,
        p: &AimProfile,
    ) -> bool {
        let can = now >= react_at && self.on_aim(v, p.fire_cone) && distance <= p.max_range;
        let mut down = false;
        if can && distance > p.burst_range {
            if !self.in_burst && now >= self.pause_until {
                self.in_burst = true;
                self.burst_until = now + p.burst * (0.8 + 0.4 * self.rng.next_unit());
            }
            if self.in_burst && now >= self.burst_until {
                self.in_burst = false;
                self.pause_until = now + p.burst_pause * (0.8 + 0.4 * self.rng.next_unit());
            }
            down = self.in_burst;
        } else if can {
            down = true;
            self.in_burst = false;
        } else {
            self.in_burst = false;
        }
        down
    }

    /// The state as a flat list, DotBotAim's order (see the binding's `state`).
    pub fn to_array(&self) -> [f64; 18] {
        [
            self.err_mag0,
            self.err_angle,
            self.err_spin,
            self.over.x as f64,
            self.over.y as f64,
            self.vel_est.x as f64,
            self.vel_est.y as f64,
            self.vel_est.z as f64,
            self.prev_gap,
            self.flinch,
            self.want.x as f64,
            self.want.y as f64,
            self.want_prev.x as f64,
            self.want_prev.y as f64,
            if self.in_burst { 1.0 } else { 0.0 },
            self.burst_until,
            self.pause_until,
            self.rng.counter as f64,
        ]
    }

    /// The inverse of [`Aim::to_array`]; a short slice leaves the rest alone.
    pub fn load(&mut self, a: &[f64]) {
        if a.len() < 18 {
            return;
        }
        self.err_mag0 = a[0];
        self.err_angle = a[1];
        self.err_spin = a[2];
        self.over = Vec2::of(a[3], a[4]);
        self.vel_est = Vec3::new(a[5] as f32, a[6] as f32, a[7] as f32);
        self.prev_gap = a[8];
        self.flinch = a[9];
        self.want = Vec2::of(a[10], a[11]);
        self.want_prev = Vec2::of(a[12], a[13]);
        self.in_burst = a[14] != 0.0;
        self.burst_until = a[15];
        self.pause_until = a[16];
        self.rng.counter = a[17] as i64;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile() -> AimProfile {
        AimProfile {
            prediction: 0.6,
            track_lag: 0.05,
            overshoot: 0.4,
            aim_floor: 1.0,
            aim_settle: 0.4,
            fire_cone: 4.0,
            turn_smooth: 0.8,
            turn_rate: 540.0,
            max_range: 60.0,
            burst_range: 12.0,
            burst: 0.4,
            burst_pause: 0.3,
        }
    }

    #[test]
    fn angles_and_direction_agree() {
        let a = angles_to(Vec3::new(0.0, 0.0, 0.0), Vec3::new(-1.0, 0.0, -1.0));
        assert!((a.x - 45.0).abs() < 1e-4);
        let d = direction(a.x as f64, a.y as f64);
        assert!((d.x + std::f32::consts::FRAC_1_SQRT_2).abs() < 1e-5);
    }

    #[test]
    fn tracking_a_still_target_settles_on_it() {
        let p = profile();
        let mut aim = Aim::new(crate::num::mix(42));
        let mut v = View {
            yaw: 90.0,
            pitch: 0.0,
        };
        let eye = Vec3::new(0.0, 1.6, 0.0);
        let point = Vec3::new(0.0, 1.6, -10.0);
        let _ = aim.engage(false, 0.0, 90.0, 0.2, 0.3, 0.1, 0.0);
        let mut fired = false;
        for i in 0..200 {
            let now = i as f64 / 60.0;
            aim.track(
                &mut v,
                eye,
                point,
                Vec3::new(0.0, 0.0, 0.0),
                now,
                1.0 / 60.0,
                0.0,
                0.0,
                &p,
            );
            fired |= aim.trigger(&v, now, 10.0, 0.3, &p);
        }
        assert!(v.yaw.abs() < 2.0, "yaw {}", v.yaw);
        assert!(fired);
    }

    #[test]
    fn state_round_trips() {
        let mut a = Aim::new(9);
        a.flinch = 2.5;
        a.over = Vec2::new(1.5, -0.25);
        a.rng.counter = 17;
        let mut b = Aim::new(9);
        b.load(&a.to_array());
        assert_eq!(
            a.to_array().map(f64::to_bits),
            b.to_array().map(f64::to_bits)
        );
    }
}
