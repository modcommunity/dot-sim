//! `DotSimBotMind`: one bot brain's aim state and its fight step, native. DotBotAim keeps its
//! public fields (the view, the engagement's times) and hands each call the ones it needs; the
//! private ones (aim error, overshoot, tracked velocity, bursts, the aim's random stream) live here.
//! The view comes back as `f64`s, never a `Vector2`, so the script's yaw keeps its precision.

use dot_sim_brain::aim::direction;
use dot_sim_brain::fight::FirstShot;
use dot_sim_brain::num::rad_to_deg;
use dot_sim_brain::{
    Aim, AimProfile, FightIn, FightProfile, Strafe, Stream, Vec2 as V2, Vec3 as V3, View, fight,
};
use godot::prelude::*;

/// The `io` layout of [`DotSimBotMind::fight`] (and DotBotBrain's packing of it).
mod io {
    pub const YAW: usize = 0;
    pub const PITCH: usize = 1;
    pub const REACT_AT: usize = 2;
    pub const ENGAGED_AT: usize = 3;
    pub const FIRST_SHOT_AT: usize = 4;
    pub const FIRST_SHOT_ERROR: usize = 5;
    pub const COUNTER: usize = 6;
    pub const NOW: usize = 7;
    pub const DT: usize = 8;
    pub const SPEED: usize = 9;
    pub const HEALTH: usize = 10;
    pub const ROLL_STOP: usize = 11;
    pub const ROLL_CROUCH: usize = 12;
    pub const ROLL_COVER: usize = 13;
    pub const HESITATE_UNTIL: usize = 14;
    pub const STRAFE: usize = 15;
    pub const STRAFE_UNTIL: usize = 16;
    pub const STRAFE_EASED: usize = 17;
    /// The profile from here: prediction, track_lag, overshoot, aim_floor, aim_settle, fire_cone,
    /// turn_smooth, turn_rate, max_range, burst_range, burst, burst_pause, range, strafe,
    /// strafe_jitter, strafe_time, jump.
    pub const PROFILE: usize = 18;
    pub const LEN: usize = PROFILE + 17;
}

fn v3(v: Vector3) -> V3 {
    V3::new(v.x, v.y, v.z)
}

fn aim_profile(a: &[f64]) -> AimProfile {
    AimProfile {
        prediction: a[0],
        track_lag: a[1],
        overshoot: a[2],
        aim_floor: a[3],
        aim_settle: a[4],
        fire_cone: a[5],
        turn_smooth: a[6],
        turn_rate: a[7],
        max_range: a[8],
        burst_range: a[9],
        burst: a[10],
        burst_pause: a[11],
    }
}

fn view_out(v: View) -> PackedFloat64Array {
    PackedFloat64Array::from(&[v.yaw, v.pitch][..])
}

/// One brain's native aim and fight step.
#[derive(GodotClass)]
#[class(base = RefCounted)]
pub struct DotSimBotMind {
    aim: Aim,
    brain_key: i64,
}

#[godot_api]
impl IRefCounted for DotSimBotMind {
    fn init(_base: Base<RefCounted>) -> Self {
        Self {
            aim: Aim::new(0),
            brain_key: 0,
        }
    }
}

#[godot_api]
impl DotSimBotMind {
    /// The aim's stream key (`stream(&"dot_bot_aim")`) and the brain's (`stream(&"dot_bot")`).
    #[func]
    fn set_keys(&mut self, aim_key: i64, brain_key: i64) {
        self.aim.rng.key = aim_key;
        self.brain_key = brain_key;
    }

    /// The private state: err_mag0, err_angle, err_spin, over (2), vel_est (3), prev_gap, flinch,
    /// want (2), want_prev (2), in_burst, burst_until, pause_until, the aim stream's counter.
    #[func]
    fn state(&self) -> PackedFloat64Array {
        PackedFloat64Array::from(&self.aim.to_array()[..])
    }

    /// The inverse of [`Self::state`].
    #[func]
    fn load(&mut self, state: PackedFloat64Array) {
        self.aim.load(state.as_slice());
    }

    /// `engage` past its continue check; returns `react_at`.
    #[func]
    #[allow(clippy::too_many_arguments)]
    fn engage(
        &mut self,
        resumed: bool,
        now: f64,
        off: f64,
        reaction: f64,
        react_jitter: f64,
        notice: f64,
        aim_error: f64,
    ) -> f64 {
        self.aim
            .engage(resumed, now, off, reaction, react_jitter, notice, aim_error)
    }

    #[func]
    fn release(&mut self) {
        self.aim.release();
    }

    #[func]
    fn forget(&mut self) {
        self.aim.forget();
    }

    #[func]
    fn hurt(&mut self, flinch: f64) {
        self.aim.hurt(flinch);
    }

    #[func]
    fn error_deg(&self, now: f64, engaged_at: f64, aim_floor: f64, aim_settle: f64) -> f64 {
        self.aim.error_deg(now, engaged_at, aim_floor, aim_settle)
    }

    #[func]
    fn on_aim(&self, yaw: f64, pitch: f64, fire_cone: f64) -> bool {
        self.aim.on_aim(&View { yaw, pitch }, fire_cone)
    }

    /// `turn_to`; returns the new [yaw, pitch].
    #[func]
    #[allow(clippy::too_many_arguments)]
    fn turn(
        &mut self,
        yaw: f64,
        pitch: f64,
        want: Vector2,
        dt: f64,
        prediction: f64,
        turn_smooth: f64,
        turn_rate: f64,
    ) -> PackedFloat64Array {
        let mut v = View { yaw, pitch };
        self.aim.turn_to(
            &mut v,
            V2::new(want.x, want.y),
            dt,
            prediction,
            turn_smooth,
            turn_rate,
        );
        view_out(v)
    }

    /// `look_at_point`; returns the new [yaw, pitch].
    #[func]
    #[allow(clippy::too_many_arguments)]
    fn look(
        &mut self,
        yaw: f64,
        pitch: f64,
        eye: Vector3,
        point: Vector3,
        dt: f64,
        prediction: f64,
        turn_smooth: f64,
        turn_rate: f64,
    ) -> PackedFloat64Array {
        let mut v = View { yaw, pitch };
        self.aim.look(
            &mut v,
            v3(eye),
            v3(point),
            dt,
            prediction,
            turn_smooth,
            turn_rate,
        );
        view_out(v)
    }

    /// `track` with no aim solver. `io`: yaw, pitch, engaged_at, now, dt, projectile speed, then
    /// the aim profile (prediction .. burst_pause, as [`Self::fight`]). Returns [yaw, pitch].
    #[func]
    fn track(
        &mut self,
        eye: Vector3,
        point: Vector3,
        velocity: Vector3,
        io: PackedFloat64Array,
    ) -> PackedFloat64Array {
        let a = io.as_slice();
        if a.len() < 18 {
            godot_error!("DotSimBotMind.track: io holds {} of 18 values", a.len());
            return PackedFloat64Array::new();
        }
        let p = aim_profile(&a[6..18]);
        let mut v = View {
            yaw: a[0],
            pitch: a[1],
        };
        self.aim.track(
            &mut v,
            v3(eye),
            v3(point),
            v3(velocity),
            a[3],
            a[4],
            a[5],
            a[2],
            &p,
        );
        view_out(v)
    }

    /// `trigger`. `io`: yaw, pitch, react_at, then the aim profile.
    #[func]
    fn trigger(&mut self, now: f64, distance: f64, io: PackedFloat64Array) -> bool {
        let a = io.as_slice();
        if a.len() < 15 {
            godot_error!("DotSimBotMind.trigger: io holds {} of 15 values", a.len());
            return false;
        }
        let p = aim_profile(&a[3..15]);
        self.aim.trigger(
            &View {
                yaw: a[0],
                pitch: a[1],
            },
            now,
            distance,
            a[2],
            &p,
        )
    }

    /// `note_shot`'s error: degrees between the view and `point` from `eye`.
    #[func]
    fn shot_error(&self, yaw: f64, pitch: f64, eye: Vector3, point: Vector3) -> f64 {
        rad_to_deg(direction(yaw, pitch).angle_to(v3(point) - v3(eye)) as f64)
    }

    /// DotBotBrain's whole fight step (see the `io` layout in the source). Returns yaw, pitch,
    /// first_shot_at, first_shot_error, the brain stream's counter, fire, crouch, jump, move (2),
    /// scene (0 fight, 1 close, 2 flee), retreat, strafe, strafe_until, strafe_eased.
    #[func]
    fn fight(
        &mut self,
        eye: Vector3,
        point: Vector3,
        velocity: Vector3,
        me_pos: Vector3,
        c_pos: Vector3,
        io: PackedFloat64Array,
    ) -> PackedFloat64Array {
        let a = io.as_slice();
        if a.len() < io::LEN {
            godot_error!(
                "DotSimBotMind.fight: io holds {} of {} values",
                a.len(),
                io::LEN
            );
            return PackedFloat64Array::new();
        }
        let pr = &a[io::PROFILE..io::LEN];
        let p = FightProfile {
            aim: aim_profile(&pr[..12]),
            range: pr[12],
            strafe: pr[13],
            strafe_jitter: pr[14],
            strafe_time: pr[15],
            jump: pr[16],
        };
        let i = FightIn {
            eye: v3(eye),
            point: v3(point),
            velocity: v3(velocity),
            me_pos: v3(me_pos),
            c_pos: v3(c_pos),
            now: a[io::NOW],
            dt: a[io::DT],
            projectile_speed: a[io::SPEED],
            health: a[io::HEALTH],
            react_at: a[io::REACT_AT],
            engaged_at: a[io::ENGAGED_AT],
            roll_stop: a[io::ROLL_STOP] != 0.0,
            roll_crouch: a[io::ROLL_CROUCH] != 0.0,
            roll_cover: a[io::ROLL_COVER] != 0.0,
            hesitate_until: a[io::HESITATE_UNTIL],
        };
        let mut view = View {
            yaw: a[io::YAW],
            pitch: a[io::PITCH],
        };
        let mut first = FirstShot {
            at: a[io::FIRST_SHOT_AT],
            error: a[io::FIRST_SHOT_ERROR],
        };
        let mut strafe = Strafe {
            side: a[io::STRAFE],
            until: a[io::STRAFE_UNTIL],
            eased: a[io::STRAFE_EASED],
        };
        let mut rng = Stream::new(self.brain_key, a[io::COUNTER] as i64);
        let o = fight(
            &mut self.aim,
            &mut view,
            &mut first,
            &mut strafe,
            &mut rng,
            &i,
            &p,
        );
        let b = |x: bool| if x { 1.0 } else { 0.0 };
        PackedFloat64Array::from(
            &[
                view.yaw,
                view.pitch,
                first.at,
                first.error,
                rng.counter as f64,
                b(o.fire),
                b(o.crouch),
                b(o.jump),
                o.move_.x as f64,
                o.move_.y as f64,
                o.scene as f64,
                b(o.retreat),
                strafe.side,
                strafe.until,
                strafe.eased,
            ][..],
        )
    }
}
