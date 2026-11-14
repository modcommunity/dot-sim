//! `DotSimThinkTier`: dot-bot's think-rate tiers (`DotBotThinkTier`) for one tick, natively.
//! The script keeps the settings and hands them over with [`DotSimThinkTier::configure`].

use dot_sim_bots::Vec3 as V3;
use dot_sim_bots::tier::{Tier, TierConfig};
use godot::prelude::*;

fn v3(p: &Vector3) -> V3 {
    V3::new(p.x, p.y, p.z)
}

/// The tiers of one tick: people's eyes and looks, the pinned bots, and the counts.
#[derive(GodotClass)]
#[class(base = RefCounted, init)]
pub struct DotSimThinkTier {
    tier: Tier,
    eyes: Vec<V3>,
    looks: Vec<V3>,
    ids: Vec<i64>,
    at: Vec<V3>,
}

#[godot_api]
impl DotSimThinkTier {
    /// The settings, in this order: above, near_every, far_every, far_range, close_range, nearest,
    /// engaged_far_every, engaged_range, idle_view_every, load_scale, view_range, view_cos,
    /// body_near_every, body_far_every, body_above. Missing values keep their defaults.
    #[func]
    fn configure(&mut self, values: PackedFloat64Array) {
        let v = values.as_slice();
        let d = TierConfig::default();
        let f = |i: usize, def: f64| v.get(i).copied().unwrap_or(def);
        let n = |i: usize, def: i64| v.get(i).map_or(def, |x| *x as i64);
        self.tier.config = TierConfig {
            above: n(0, d.above),
            near_every: n(1, d.near_every),
            far_every: n(2, d.far_every),
            far_range: f(3, d.far_range),
            close_range: f(4, d.close_range),
            nearest: n(5, d.nearest),
            engaged_far_every: n(6, d.engaged_far_every),
            engaged_range: f(7, d.engaged_range),
            idle_view_every: n(8, d.idle_view_every),
            load_scale: f(9, d.load_scale),
            view_range: f(10, d.view_range),
            view_cos: f(11, d.view_cos),
            body_near_every: n(12, d.body_near_every),
            body_far_every: n(13, d.body_far_every),
            body_above: n(14, d.body_above),
        };
    }

    /// Starts a tick (the script's `begin`); returns the bots pinned.
    #[func]
    fn begin(
        &mut self,
        players: i64,
        eyes: PackedVector3Array,
        looks: PackedVector3Array,
        bot_ids: PackedInt32Array,
        bot_positions: PackedVector3Array,
    ) -> i64 {
        self.eyes.clear();
        self.eyes.extend(eyes.as_slice().iter().map(v3));
        self.looks.clear();
        self.looks.extend(looks.as_slice().iter().map(v3));
        self.ids.clear();
        self.ids
            .extend(bot_ids.as_slice().iter().map(|i| i64::from(*i)));
        self.at.clear();
        self.at.extend(bot_positions.as_slice().iter().map(v3));
        self.tier
            .begin(players, &self.eyes, &self.looks, &self.ids, &self.at);
        self.tier.pinned_len() as i64
    }

    /// Whether the last begin turned tiers on.
    #[func]
    fn is_on(&self) -> bool {
        self.tier.is_on()
    }

    /// The script's `period`, counted.
    #[func]
    fn period(&mut self, id: i64, position: Vector3, engaged: bool, idle: bool) -> i64 {
        self.tier.period(id, v3(&position), engaged, idle)
    }

    /// The script's `body_period`, counted above 1.
    #[func]
    fn body_period(&mut self, id: i64, position: Vector3, engaged: bool, idle: bool) -> i64 {
        self.tier.body_period(id, v3(&position), engaged, idle)
    }

    /// One think's schedule: (ticks to the next beat, body period), from one classification.
    #[func]
    #[allow(clippy::too_many_arguments)]
    fn schedule(
        &mut self,
        id: i64,
        position: Vector3,
        engaged: bool,
        idle: bool,
        tick: i64,
        seed: i64,
        floor: i64,
        coastable: bool,
    ) -> Vector2i {
        let s = self.tier.schedule(
            id,
            v3(&position),
            engaged,
            idle,
            tick,
            seed,
            floor,
            coastable,
        );
        Vector2i::new(s.every as i32, s.body as i32)
    }

    /// The counts since the last call, then zeroed: off, engaged, pinned, close, view, idle_view,
    /// engaged_far, near, far, then body near and far.
    #[func]
    fn take_counts(&mut self) -> PackedInt64Array {
        PackedInt64Array::from(&self.tier.take_counts()[..])
    }
}
