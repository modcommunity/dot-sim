//! Think-rate tiers for a crowd of bots: dot-bot's `DotBotThinkTier`, the same arithmetic in the
//! same precision (engine vectors in `f32`, script floats in `f64`), so a game that schedules its
//! bots here gets the periods the script gives.
//!
//! [`Tier::schedule`] is one think's whole answer (the script's `schedule`): the period, the ticks
//! to the bot's next beat, and the body period of a bot that coasts, from one classification.

use crate::Vec3;
use dot_sim_math::{maxf, minf};

/// The longest period `load_scale` may stretch a tier to (DotBotThinkTier.MAX_PERIOD).
pub const MAX_PERIOD: i64 = 8;

/// Why a bot got its period; the order of [`Tier::counts`] and of the script's reasons.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(usize)]
pub enum Reason {
    Off = 0,
    Engaged,
    Pinned,
    Close,
    View,
    IdleView,
    EngagedFar,
    Near,
    Far,
}

/// Number of [`Reason`]s.
pub const REASONS: usize = 9;

/// The tier's settings: the script's public fields (`view_cos` is its `_view_cos`, worked out there).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TierConfig {
    pub above: i64,
    pub near_every: i64,
    pub far_every: i64,
    pub far_range: f64,
    pub close_range: f64,
    pub nearest: i64,
    pub engaged_far_every: i64,
    pub engaged_range: f64,
    pub idle_view_every: i64,
    pub load_scale: f64,
    pub view_range: f64,
    pub view_cos: f64,
    pub body_near_every: i64,
    pub body_far_every: i64,
    pub body_above: i64,
}

impl Default for TierConfig {
    fn default() -> Self {
        Self {
            above: 16,
            near_every: 2,
            far_every: 4,
            far_range: 40.0,
            close_range: 12.0,
            nearest: 3,
            engaged_far_every: 1,
            engaged_range: 25.0,
            idle_view_every: 2,
            load_scale: 1.0,
            view_range: 60.0,
            view_cos: 60.0f64.to_radians().cos(),
            body_near_every: 2,
            body_far_every: 4,
            body_above: 24,
        }
    }
}

/// One think's schedule: see [`Tier::schedule`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Schedule {
    /// The tier's period (the script's `period`), before the caller's floor.
    pub period: i64,
    /// Ticks to the bot's next beat at `max(period, floor)` (1: thinks next tick).
    pub every: i64,
    /// The motor's period while the bot coasts (1 when it does not).
    pub body: i64,
}

/// The tier of one tick: the people's eyes and looks and the pinned (nearest) bots.
#[derive(Clone, Debug, Default)]
pub struct Tier {
    pub config: TierConfig,
    on: bool,
    body_on: bool,
    eyes: Vec<Vec3>,
    looks: Vec<Vec3>,
    pinned: Vec<i64>,
    /// Answers of the period by [`Reason`] since the last [`Tier::take_counts`].
    pub counts: [i64; REASONS],
    /// Body periods above 1 by reason: near, far.
    pub body_counts: [i64; 2],
    best_d: Vec<f32>,
    best_i: Vec<usize>,
}

/// Ticks from a think at `tick` to the next beat of a bot with `seed` at period `every` (1..every).
#[inline]
pub fn until_beat(tick: i64, seed: i64, every: i64) -> i64 {
    if every <= 1 {
        1
    } else {
        every - tick.wrapping_add(seed).rem_euclid(every)
    }
}

impl Tier {
    /// Whether tiers apply to a game of `players` players.
    pub fn applies(&self, players: i64) -> bool {
        let c = &self.config;
        c.above >= 0 && players > c.above && c.near_every.max(c.far_every) > 1
    }

    /// Whether the last [`Tier::begin`] turned tiers on.
    pub fn is_on(&self) -> bool {
        self.on
    }

    /// Bots pinned by the last [`Tier::begin`].
    pub fn pinned_len(&self) -> usize {
        self.pinned.len()
    }

    /// People's eyes given to the last [`Tier::begin`].
    pub fn people(&self) -> usize {
        self.eyes.len()
    }

    /// Starts a tick, as the script's `begin`: eyes and looks parallel, bot ids and positions
    /// parallel. Pins each person's `nearest` bots.
    pub fn begin(
        &mut self,
        players: i64,
        eyes: &[Vec3],
        looks: &[Vec3],
        bot_ids: &[i64],
        bot_positions: &[Vec3],
    ) {
        self.on = self.applies(players);
        self.body_on = self.on && self.config.body_above >= 0 && players > self.config.body_above;
        self.eyes.clear();
        self.eyes.extend_from_slice(eyes);
        self.looks.clear();
        self.looks.extend_from_slice(looks);
        self.pinned.clear();
        if !self.on || self.config.nearest <= 0 {
            return;
        }
        let n = bot_ids.len().min(bot_positions.len());
        let k = (self.config.nearest as usize).min(n);
        if k == 0 {
            return;
        }
        for eye in eyes {
            self.best_d.clear();
            self.best_i.clear();
            for (i, p) in bot_positions[..n].iter().enumerate() {
                // The engine's distance_squared_to: (to - from).length_squared().
                let d = (*eye - *p).length_squared();
                if self.best_d.len() == k && d >= self.best_d[k - 1] {
                    continue;
                }
                let mut at = self.best_d.len();
                while at > 0 && self.best_d[at - 1] > d {
                    at -= 1;
                }
                self.best_d.insert(at, d);
                self.best_i.insert(at, i);
                if self.best_d.len() > k {
                    self.best_d.truncate(k);
                    self.best_i.truncate(k);
                }
            }
            for &i in &self.best_i {
                let id = bot_ids[i];
                if !self.pinned.contains(&id) {
                    self.pinned.push(id);
                }
            }
        }
    }

    /// The reason for a bot and the nearest person's squared distance (the script's `_classify`).
    pub fn classify(&self, id: i64, position: Vec3, engaged: bool, idle: bool) -> (Reason, f64) {
        let c = &self.config;
        let mut best_sq = f64::INFINITY;
        if !self.on {
            return (Reason::Off, best_sq);
        }
        if engaged && c.engaged_far_every <= 1 {
            return (Reason::Engaged, best_sq);
        }
        if self.pinned.contains(&id) {
            return (Reason::Pinned, best_sq);
        }
        let close_sq = c.close_range * c.close_range;
        let view_sq = c.view_range * c.view_range;
        let reach = maxf(c.engaged_range, c.close_range);
        let engaged_sq = reach * reach;
        let mut seen = false;
        for (eye, look) in self.eyes.iter().zip(self.looks.iter()) {
            let to = position - *eye;
            let d_sq = f64::from(to.length_squared());
            if d_sq < close_sq {
                return (Reason::Close, best_sq);
            }
            if engaged && d_sq < engaged_sq {
                return (Reason::Engaged, best_sq);
            }
            if d_sq < view_sq && f64::from(to.dot(*look)) >= c.view_cos * d_sq.sqrt() {
                if engaged || !idle || c.idle_view_every <= 1 {
                    return (Reason::View, best_sq);
                }
                seen = true;
            }
            best_sq = minf(best_sq, d_sq);
        }
        let reason = if seen {
            Reason::IdleView
        } else if engaged {
            Reason::EngagedFar
        } else if best_sq < c.far_range * c.far_range {
            Reason::Near
        } else {
            Reason::Far
        };
        (reason, best_sq)
    }

    /// `every` stretched by `load_scale`; 1 stays 1.
    fn scaled(&self, every: i64) -> i64 {
        if every <= 1 {
            return 1;
        }
        let s = (every as f64 * maxf(self.config.load_scale, 1.0) - 0.001).ceil();
        (s as i64).clamp(1, MAX_PERIOD.max(every))
    }

    fn unseen_every(&self, best_sq: f64) -> i64 {
        let c = &self.config;
        if best_sq < c.far_range * c.far_range {
            self.scaled(c.near_every)
        } else {
            self.scaled(c.far_every)
        }
    }

    /// The period of a classified bot, counted, as the script's `period`.
    fn period_of(&mut self, reason: Reason, best_sq: f64) -> i64 {
        let every = match reason {
            Reason::IdleView => self.config.idle_view_every.min(self.unseen_every(best_sq)),
            Reason::EngagedFar => self.scaled(self.config.engaged_far_every),
            Reason::Near | Reason::Far => self.unseen_every(best_sq),
            _ => 1,
        };
        self.counts[reason as usize] += 1;
        every
    }

    /// Whether body periods can be above 1 this tick.
    fn body_live(&self) -> bool {
        let c = &self.config;
        self.body_on && c.body_near_every.max(c.body_far_every) > 1
    }

    /// The body period of a classified bot, counted above 1, as the script's `body_period`.
    fn body_of(&mut self, reason: Reason) -> i64 {
        let c = &self.config;
        let (every, slot) = match reason {
            Reason::Near => (c.body_near_every.clamp(1, MAX_PERIOD), 0),
            Reason::Far => (c.body_far_every.clamp(1, MAX_PERIOD), 1),
            _ => (1, 0),
        };
        if every > 1 {
            self.body_counts[slot] += 1;
        }
        every
    }

    /// Ticks between thinks for a bot (the script's `period`), counted.
    pub fn period(&mut self, id: i64, position: Vec3, engaged: bool, idle: bool) -> i64 {
        let (reason, best_sq) = self.classify(id, position, engaged, idle);
        self.period_of(reason, best_sq)
    }

    /// Ticks between motor ticks for a bot (the script's `body_period`), counted above 1.
    pub fn body_period(&mut self, id: i64, position: Vec3, engaged: bool, idle: bool) -> i64 {
        if !self.body_live() {
            return 1;
        }
        let (reason, _) = self.classify(id, position, engaged, idle);
        self.body_of(reason)
    }

    /// One think's schedule from one classification: the period, the ticks to the bot's next
    /// beat at `max(period, floor)` from `tick` with `seed`, and the body period when the bot
    /// coasts (that count above 1 and `coastable`). The same answers and counts as the script's
    /// `period`, `until_beat` and (for a coasting bot) `body_period`.
    #[allow(clippy::too_many_arguments)]
    pub fn schedule(
        &mut self,
        id: i64,
        position: Vec3,
        engaged: bool,
        idle: bool,
        tick: i64,
        seed: i64,
        floor: i64,
        coastable: bool,
    ) -> Schedule {
        let (reason, best_sq) = self.classify(id, position, engaged, idle);
        let period = self.period_of(reason, best_sq);
        let every = until_beat(tick, seed, period.max(floor));
        let body = if every > 1 && coastable && self.body_live() {
            self.body_of(reason)
        } else {
            1
        };
        Schedule {
            period,
            every,
            body,
        }
    }

    /// The counts since the last call, then zeroed: the period reasons, then body near and far.
    pub fn take_counts(&mut self) -> [i64; REASONS + 2] {
        let mut out = [0; REASONS + 2];
        out[..REASONS].copy_from_slice(&self.counts);
        out[REASONS..].copy_from_slice(&self.body_counts);
        self.counts = [0; REASONS];
        self.body_counts = [0; 2];
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(x: f32, y: f32, z: f32) -> Vec3 {
        Vec3::new(x, y, z)
    }

    fn tier(players: i64) -> Tier {
        let mut t = Tier::default();
        t.begin(
            players,
            &[v(0.0, 1.6, 0.0)],
            &[v(0.0, 0.0, -1.0)],
            &[10, 11, 12, 13, 14],
            &[
                v(0.0, 0.0, -5.0),
                v(0.0, 0.0, 30.0),
                v(25.0, 0.0, 0.0),
                v(0.0, 0.0, 100.0),
                v(0.0, 0.0, -20.0),
            ],
        );
        t
    }

    #[test]
    fn off_at_or_below_above() {
        let mut t = tier(16);
        assert!(!t.is_on());
        assert_eq!(t.period(13, v(0.0, 0.0, 100.0), false, false), 1);
        assert_eq!(t.counts[Reason::Off as usize], 1);
    }

    #[test]
    fn nearest_three_are_pinned() {
        let t = tier(32);
        assert_eq!(t.pinned_len(), 3);
        assert_eq!(
            t.classify(10, v(0.0, 0.0, -5.0), false, false).0,
            Reason::Pinned
        );
        assert_eq!(
            t.classify(11, v(0.0, 0.0, 30.0), false, false).0,
            Reason::Near
        );
        assert_eq!(
            t.classify(13, v(0.0, 0.0, 100.0), false, false).0,
            Reason::Far
        );
        assert_eq!(
            t.classify(99, v(0.0, 0.0, -50.0), false, false).0,
            Reason::View
        );
        assert_eq!(
            t.classify(99, v(0.0, 0.0, -50.0), false, true).0,
            Reason::IdleView
        );
        assert_eq!(
            t.classify(99, v(0.0, 0.0, 5.0), false, true).0,
            Reason::Close
        );
    }

    #[test]
    fn schedule_matches_period_and_beat() {
        let mut t = tier(32);
        let s = t.schedule(13, v(0.0, 0.0, 100.0), false, false, 7, 3, 1, true);
        assert_eq!(s.period, 4);
        assert_eq!(s.every, until_beat(7, 3, 4));
        assert_eq!(s.body, 4);
        assert_eq!(t.body_counts, [0, 1]);
        let s = t.schedule(13, v(0.0, 0.0, 100.0), false, false, 7, 3, 1, false);
        assert_eq!(s.body, 1);
        assert_eq!(t.counts[Reason::Far as usize], 2);
        let c = t.take_counts();
        assert_eq!(c[Reason::Far as usize], 2);
        assert_eq!(c[REASONS + 1], 1);
        assert_eq!(t.counts, [0; REASONS]);
    }

    #[test]
    fn body_off_below_body_above() {
        let mut t = tier(20);
        assert_eq!(t.body_period(13, v(0.0, 0.0, 100.0), false, false), 1);
        let s = t.schedule(13, v(0.0, 0.0, 100.0), false, false, 0, 0, 1, true);
        assert_eq!(s.body, 1);
        assert_eq!(t.body_counts, [0, 0]);
    }

    #[test]
    fn load_scale_stretches_and_caps() {
        let mut t = tier(32);
        t.config.load_scale = 3.0;
        assert_eq!(t.period(13, v(0.0, 0.0, 100.0), false, false), 8);
        t.config.load_scale = 1.5;
        assert_eq!(t.period(11, v(0.0, 0.0, 30.0), false, false), 3);
    }

    #[test]
    fn beat_spreads() {
        assert_eq!(until_beat(0, 0, 1), 1);
        assert_eq!(until_beat(0, 0, 4), 4);
        assert_eq!(until_beat(5, 0, 4), 3);
        assert_eq!(until_beat(-3, 1, 4), 2);
    }
}
