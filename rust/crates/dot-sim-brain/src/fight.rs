//! DotBotBrain's `_fight` (and `_set_move`): track and trigger, then range, strafing, stopping
//! to shoot, crouching and jumping, turned into the legs' move in the view's frame.

use crate::aim::{Aim, AimProfile, View};
use crate::num::{Stream, Vec2, Vec3, deg_to_rad, lerpf, move_toward};

/// Health fraction under which a bot that rolled for cover backs off (the brain's constant).
pub const RETREAT_HEALTH: f64 = 0.35;

/// The profile fields the fight step reads beyond the aim's.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FightProfile {
    pub aim: AimProfile,
    pub range: f64,
    pub strafe: f64,
    pub strafe_jitter: f64,
    pub strafe_time: f64,
    pub jump: f64,
}

/// What the brain knows going into the step: the bodies, the engagement, the rolls, the strafe.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FightIn {
    pub eye: Vec3,
    pub point: Vec3,
    pub velocity: Vec3,
    pub me_pos: Vec3,
    pub c_pos: Vec3,
    pub now: f64,
    pub dt: f64,
    pub projectile_speed: f64,
    pub health: f64,
    pub react_at: f64,
    pub engaged_at: f64,
    pub roll_stop: bool,
    pub roll_crouch: bool,
    pub roll_cover: bool,
    pub hesitate_until: f64,
}

/// The brain's strafe state, carried between steps.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Strafe {
    pub side: f64,
    pub until: f64,
    pub eased: f64,
}

impl Default for Strafe {
    fn default() -> Self {
        Self {
            side: 0.0,
            until: f64::NEG_INFINITY,
            eased: 0.0,
        }
    }
}

/// The step's intent.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FightOut {
    pub fire: bool,
    pub crouch: bool,
    pub jump: bool,
    pub move_: Vec2,
    pub retreat: bool,
    /// 0 fight, 1 close (out of range, going in), 2 flee.
    pub scene: u8,
}

/// The engagement's first shot, as `note_shot` records it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FirstShot {
    pub at: f64,
    pub error: f64,
}

/// One `_fight` step. `aim` and `view` are the brain's aim; `rng` is the brain's own stream.
#[allow(clippy::too_many_arguments)]
pub fn fight(
    aim: &mut Aim,
    view: &mut View,
    first: &mut FirstShot,
    strafe: &mut Strafe,
    rng: &mut Stream,
    i: &FightIn,
    p: &FightProfile,
) -> FightOut {
    let mut out = FightOut::default();
    aim.track(
        view,
        i.eye,
        i.point,
        i.velocity,
        i.now,
        i.dt,
        i.projectile_speed,
        i.engaged_at,
        &p.aim,
    );

    let flat = Vec2::of(
        i.c_pos.x as f64 - i.me_pos.x as f64,
        i.c_pos.z as f64 - i.me_pos.z as f64,
    );
    let dist = flat.length() as f64;
    let trigger = aim.trigger(view, i.now, dist, i.react_at, &p.aim);
    out.fire = trigger;
    if trigger && first.at.is_nan() {
        first.at = i.now;
        let d = crate::aim::direction(view.yaw, view.pitch);
        first.error = crate::num::rad_to_deg(d.angle_to(i.point - i.eye) as f64);
    }

    let dir = if dist > 0.01 {
        flat.div_scalar(dist)
    } else {
        Vec2::new(0.0, -1.0)
    };
    let mut forward = 0.0;
    if dist > p.range * 1.3 {
        forward = 1.0;
    } else if dist < p.range * 0.7 {
        forward = -0.7;
    }

    let retreat = i.roll_cover && i.health < RETREAT_HEALTH;
    if retreat {
        forward = -1.0;
    }
    out.retreat = retreat;
    out.scene = if retreat {
        2
    } else if dist > p.range * 1.3 {
        1
    } else {
        0
    };

    if i.now >= strafe.until {
        strafe.side = if rng.next_unit() < p.strafe {
            if rng.next_unit() < 0.5 { 1.0 } else { -1.0 }
        } else {
            0.0
        };
        if p.strafe_jitter > 0.0 {
            let u = rng.next_unit();
            let a = 0.5 + rng.next_unit();
            strafe.until = i.now + p.strafe_time * lerpf(a, 0.2 + 2.0 * u * u, p.strafe_jitter);
            if strafe.side != 0.0 && rng.next_unit() < 0.25 * p.strafe_jitter {
                strafe.side = 0.0;
            }
        } else {
            strafe.until = i.now + p.strafe_time * (0.5 + rng.next_unit());
        }
    }
    let mut side = strafe.side;
    if p.strafe_jitter > 0.0 {
        strafe.eased = move_toward(strafe.eased, strafe.side, i.dt * 5.0);
        side = strafe.eased;
    }
    if i.now < i.hesitate_until {
        forward = 0.0;
        side = 0.0;
    }

    let long_burst = trigger && dist > p.aim.burst_range * 0.6;
    if long_burst && i.roll_stop && !retreat {
        forward = 0.0;
        side = 0.0;
    }
    out.crouch = trigger && i.roll_crouch && dist > p.aim.burst_range && !retreat;
    if out.crouch {
        side = 0.0;
    }
    if !out.crouch && dist < p.aim.burst_range && i.dt > 0.0 && rng.next_unit() < p.jump * i.dt {
        out.jump = true;
    }

    let world_dir = dir.scale(forward) + Vec2::new(-dir.y, dir.x).scale(side);
    out.move_ = set_move(view.yaw, world_dir);
    out
}

/// `_set_move`: a flat world direction into the legs' (right, forward) at this yaw, at most 1 long.
#[inline]
pub fn set_move(yaw: f64, world_dir: Vec2) -> Vec2 {
    let y = deg_to_rad(yaw);
    let forward = Vec2::of(-y.sin(), -y.cos());
    let right = Vec2::of(y.cos(), -y.sin());
    Vec2::new(world_dir.dot(right), world_dir.dot(forward)).limit_length(1.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::num::mix;

    fn profile() -> FightProfile {
        FightProfile {
            aim: AimProfile {
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
            },
            range: 10.0,
            strafe: 0.8,
            strafe_jitter: 0.6,
            strafe_time: 0.8,
            jump: 0.05,
        }
    }

    #[test]
    fn a_far_target_is_closed_on_and_shot_at() {
        let p = profile();
        let mut aim = Aim::new(mix(3));
        let mut view = View::default();
        let mut first = FirstShot {
            at: f64::NAN,
            error: f64::NAN,
        };
        let mut strafe = Strafe::default();
        let mut rng = Stream::new(mix(4), 0);
        let mut moved_forward = false;
        for k in 0..120 {
            let now = k as f64 / 30.0;
            let i = FightIn {
                eye: Vec3::new(0.0, 1.6, 0.0),
                point: Vec3::new(0.0, 1.4, -30.0),
                velocity: Vec3::new(0.0, 0.0, 0.0),
                me_pos: Vec3::new(0.0, 0.0, 0.0),
                c_pos: Vec3::new(0.0, 0.0, -30.0),
                now,
                dt: 1.0 / 30.0,
                projectile_speed: 0.0,
                health: 1.0,
                react_at: 0.25,
                engaged_at: 0.0,
                roll_stop: false,
                roll_crouch: false,
                roll_cover: false,
                hesitate_until: f64::NEG_INFINITY,
            };
            let o = fight(
                &mut aim,
                &mut view,
                &mut first,
                &mut strafe,
                &mut rng,
                &i,
                &p,
            );
            assert_eq!(o.scene, 1);
            assert!(o.move_.length() <= 1.0 + 1e-6);
            moved_forward |= o.move_.y > 0.5;
        }
        assert!(moved_forward);
        assert!(!first.at.is_nan(), "never fired");
    }

    #[test]
    fn set_move_turns_world_into_view_frame() {
        // Facing -Z (yaw 0), a world step toward -Z is straight ahead.
        let m = set_move(0.0, Vec2::new(0.0, -1.0));
        assert!((m.y - 1.0).abs() < 1e-6 && m.x.abs() < 1e-6);
    }
}
