//! A [`Space`] that answers a repeated shape query from memory while the world cannot change.
//!
//! One motor tick asks the engine the same rest or cast more than once: the seat sweep repeats
//! categorise's, a step repeats the blocked slide's probe, a fallback re-asks the spot a sweep
//! started from. Between [`MemoSpace::open`] and [`MemoSpace::close`] the caller promises that
//! nothing the space sees moves, so the second answer is the first. A key is the query's bits
//! (position, motion, mask, radius, height), never a tolerance, so every answer is the one the
//! engine would give. Rays and overlaps are passed through.

use crate::{Contact, Space, Vec3};

/// How many distinct rests and casts one tick remembers; the oldest is dropped first.
const SLOTS: usize = 8;

#[inline]
fn bits(v: Vec3) -> [u32; 3] {
    [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]
}

/// The capsule and mask the next shape query runs with, as bits.
#[derive(Clone, Copy, Default, PartialEq, Eq)]
struct Shape {
    mask: u32,
    radius: u32,
    height: u32,
}

#[derive(Clone, Copy, Default, PartialEq, Eq)]
struct RestKey {
    at: [u32; 3],
    shape: Shape,
}

#[derive(Clone, Copy, Default, PartialEq, Eq)]
struct CastKey {
    from: [u32; 3],
    motion: [u32; 3],
    shape: Shape,
}

/// A small ring of keyed answers.
struct Ring<K, V> {
    slots: [(K, V); SLOTS],
    len: usize,
    next: usize,
}

impl<K: Copy + Default + Eq, V: Copy + Default> Ring<K, V> {
    fn new() -> Self {
        Self {
            slots: [(K::default(), V::default()); SLOTS],
            len: 0,
            next: 0,
        }
    }

    fn get(&self, k: &K) -> Option<V> {
        self.slots[..self.len]
            .iter()
            .find(|(sk, _)| sk == k)
            .map(|(_, v)| *v)
    }

    fn put(&mut self, k: K, v: V) {
        self.slots[self.next] = (k, v);
        self.next = (self.next + 1) % SLOTS;
        self.len = (self.len + 1).min(SLOTS);
    }

    fn clear(&mut self) {
        self.len = 0;
        self.next = 0;
    }
}

/// `inner` with a rest and cast memo that lives only while open.
pub struct MemoSpace<S: Space> {
    pub inner: S,
    on: bool,
    shape: Shape,
    rests: Ring<RestKey, Option<Contact>>,
    casts: Ring<CastKey, Option<(f32, f32)>>,
    /// Engine queries answered from memory, rests then casts, since the space was made.
    pub hits: [u64; 2],
}

impl<S: Space> MemoSpace<S> {
    pub fn new(inner: S) -> Self {
        Self {
            inner,
            on: false,
            shape: Shape::default(),
            rests: Ring::new(),
            casts: Ring::new(),
            hits: [0; 2],
        }
    }

    /// Starts remembering: from here until [`MemoSpace::close`] nothing the space sees may move,
    /// and the exclusion list and the space itself stay as they are.
    pub fn open(&mut self) {
        self.forget();
        self.on = true;
    }

    /// Stops remembering and forgets.
    pub fn close(&mut self) {
        self.on = false;
        self.forget();
    }

    /// Forgets every answer (the space or its exclusions changed); stays open or closed.
    pub fn forget(&mut self) {
        self.rests.clear();
        self.casts.clear();
    }
}

impl<S: Space> Space for MemoSpace<S> {
    fn set_radius(&mut self, radius: f32) {
        self.shape.radius = radius.to_bits();
        self.inner.set_radius(radius);
    }

    fn set_height(&mut self, height: f32) {
        self.shape.height = height.to_bits();
        self.inner.set_height(height);
    }

    fn set_mask(&mut self, mask: i64) {
        self.shape.mask = mask as u32;
        self.inner.set_mask(mask);
    }

    fn cast(&mut self, from: Vec3, motion: Vec3) -> Option<(f32, f32)> {
        if !self.on {
            return self.inner.cast(from, motion);
        }
        let k = CastKey {
            from: bits(from),
            motion: bits(motion),
            shape: self.shape,
        };
        if let Some(v) = self.casts.get(&k) {
            self.hits[1] += 1;
            return v;
        }
        let v = self.inner.cast(from, motion);
        self.casts.put(k, v);
        v
    }

    fn rest(&mut self, at: Vec3) -> Option<Contact> {
        if !self.on {
            return self.inner.rest(at);
        }
        let k = RestKey {
            at: bits(at),
            shape: self.shape,
        };
        if let Some(v) = self.rests.get(&k) {
            self.hits[0] += 1;
            return v;
        }
        let v = self.inner.rest(at);
        self.rests.put(k, v);
        v
    }

    fn overlap(&mut self, at: Vec3) -> bool {
        self.inner.overlap(at)
    }

    fn ray(&mut self, from: Vec3, to: Vec3, mask: i64) -> Option<Contact> {
        self.inner.ray(from, to, mask)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::full::{self, FullIn, FullTun, MoveCounters, Moves};
    use crate::slide::{Mover, Tun};
    use crate::tick::{AIR, GROUND, Post, State, TickCounters, TickTun, post, pre};
    use crate::{Body, Masks, UP};

    /// A floor at y = 0 inside walls at x and z = +-`half` (1 when unset), answered
    /// analytically; counts what reached it.
    #[derive(Default)]
    struct Room {
        radius: f32,
        height: f32,
        mask: i64,
        asked: [u32; 4],
        half: f32,
    }

    impl Room {
        fn sized(half: f32) -> Self {
            Self {
                half,
                ..Self::default()
            }
        }

        fn half(&self) -> f32 {
            if self.half > 0.0 { self.half } else { 1.0 }
        }

        /// The wall `at` touches: its inward normal.
        fn wall(&self, at: Vec3) -> Option<Vec3> {
            let (r, h) = (self.radius, self.half());
            [
                (at.x + r >= h, Vec3::new(-1.0, 0.0, 0.0)),
                (at.x - r <= -h, Vec3::new(1.0, 0.0, 0.0)),
                (at.z + r >= h, Vec3::new(0.0, 0.0, -1.0)),
                (at.z - r <= -h, Vec3::new(0.0, 0.0, 1.0)),
            ]
            .into_iter()
            .find(|w| w.0)
            .map(|w| w.1)
        }
    }

    impl Space for Room {
        fn set_radius(&mut self, radius: f32) {
            self.radius = radius;
        }
        fn set_height(&mut self, height: f32) {
            self.height = height;
        }
        fn set_mask(&mut self, mask: i64) {
            self.mask = mask;
        }
        fn cast(&mut self, from: Vec3, motion: Vec3) -> Option<(f32, f32)> {
            self.asked[0] += 1;
            let mut t = 1.0f32;
            let b = from.y - self.height * 0.5;
            if motion.y < 0.0 && b + motion.y < 0.0 {
                t = t.min((b / -motion.y).max(0.0));
            }
            let room = self.half() - self.radius;
            for (p, d) in [(from.x, motion.x), (from.z, motion.z)] {
                if d > 0.0 && p + d > room {
                    t = t.min(((room - p) / d).max(0.0));
                }
                if d < 0.0 && p + d < -room {
                    t = t.min(((-room - p) / d).max(0.0));
                }
            }
            if t >= 1.0 {
                return Some((1.0, 1.0));
            }
            Some((t, (t + 0.001).min(1.0)))
        }
        fn rest(&mut self, at: Vec3) -> Option<Contact> {
            self.asked[1] += 1;
            if let Some(n) = self.wall(at) {
                return Some(Contact {
                    normal: n,
                    point: at - n.scale(self.radius as f64),
                    collider_id: 9,
                });
            }
            if at.y - self.height * 0.5 <= 0.0 {
                return Some(Contact {
                    normal: UP,
                    point: Vec3::new(at.x, 0.0, at.z),
                    collider_id: 7 + self.mask,
                });
            }
            None
        }
        fn overlap(&mut self, at: Vec3) -> bool {
            self.asked[2] += 1;
            at.y - self.height * 0.5 < 0.0 || self.wall(at).is_some()
        }
        fn ray(&mut self, from: Vec3, to: Vec3, _mask: i64) -> Option<Contact> {
            self.asked[3] += 1;
            (from.y >= 0.0 && to.y <= 0.0).then_some(Contact {
                normal: UP,
                point: Vec3::new(from.x, 0.0, from.z),
                collider_id: 7,
            })
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

    const M: Masks = Masks {
        collision: 1,
        pass_out: 0,
    };

    /// Walks into a corner for `n` ticks over `space`, each tick opened then closed through
    /// `bracket`; answers each tick's state and counters.
    fn walk<S: Space>(
        space: &mut S,
        n: usize,
        mut bracket: impl FnMut(&mut S, bool),
    ) -> Vec<(State, TickCounters)> {
        let mut body = Body::default();
        let mut s = State {
            mode: GROUND,
            ground_normal: UP,
            height: 1.8,
            position: Vec3::new(0.0, 0.9, 0.0),
            ..State::default()
        };
        let mut p = Post {
            delta: 1.0 / 64.0,
            speed_base: 6.0,
            effect_max_speed: 1.0,
            surface_max_speed_scale: 1.0,
            friction_base: 6.0,
            surface_friction: 1.0,
            effect_friction: 1.0,
            ground_accel: 10.0,
            air_accel: 12.0,
            gravity: 20.0,
            move_x: 0.6,
            move_y: 0.8,
            yaw: 30.0,
            ..Post::default()
        };
        let mut out = Vec::new();
        for _ in 0..n {
            p.start_position = s.position;
            p.start_velocity = s.velocity;
            body.begin_tick();
            bracket(space, true);
            let mut c = TickCounters::default();
            let mut m = Mover::new(&mut body, space, M, None, TUN);
            pre(&mut m, &mut s, &TT, false, Vec3::default(), &mut c);
            post(&mut m, &mut s, &TT, &p, &mut c);
            bracket(space, false);
            body.end_tick();
            out.push((s, c));
        }
        out
    }

    #[test]
    fn a_remembered_tick_matches_the_plain_one_with_fewer_queries() {
        let mut plain = Room::default();
        let want = walk(&mut plain, 120, |_, _| {});
        let mut memo = MemoSpace::new(Room::default());
        let got = walk(
            &mut memo,
            120,
            |m, open| {
                if open { m.open() } else { m.close() }
            },
        );
        assert_eq!(want.len(), got.len());
        for (i, (w, g)) in want.iter().zip(&got).enumerate() {
            assert_eq!(w.0, g.0, "state at tick {i}");
            assert_eq!(w.1, g.1, "counters at tick {i}");
        }
        let end = want.last().unwrap().0.position;
        assert!(
            end.x.abs().max(end.z.abs()) > 0.6,
            "reached a wall: {end:?}"
        );
        let (a, b) = (plain.asked, memo.inner.asked);
        assert_eq!(a[2..], b[2..], "overlaps and rays pass through");
        let hits = memo.hits.iter().sum::<u64>() as u32;
        assert_eq!(b[0] + b[1] + hits, a[0] + a[1]);
        assert!(hits > 0, "some query repeated");
    }

    /// Bounds off the wall at x = `half` in the air along +z, `runs` times from spots along it,
    /// each tick through the whole native tick ([`crate::full::tick`]) with auto wall runs that
    /// probe the wall with their small capsule; answers each tick's state, moves and counters.
    fn wall_runs<S: Space>(
        space: &mut S,
        half: f32,
        runs: usize,
        mut bracket: impl FnMut(&mut S, bool),
    ) -> Vec<(State, Moves, TickCounters, MoveCounters)> {
        let tt = TickTun {
            max_velocity: 50.0,
            max_speed: 8.0,
            max_air_wish_speed: 0.8,
            ground_snap: true,
            ground_snap_distance: 0.3,
            jump_velocity: 5.0,
            ..TickTun::default()
        };
        let ft = FullTun {
            friction: 6.0,
            accelerate: 10.0,
            wall_run_enabled: true,
            wall_run_auto: true,
            wall_run_jump_min_speed: 4.0,
            wall_run_min_speed: 3.0,
            wall_max_angle_from_vertical: 30.0,
            wall_reach: 0.6,
            wall_run_max_time: 0.6,
            wall_run_end_speed: 2.0,
            wall_run_requires_forward: true,
            wall_run_max_fall_speed: 8.0,
            wall_run_max_approach_angle: 60.0,
            wall_run_min_height: 1.0,
            wall_run_min_length: 1.0,
            wall_run_cooldown: 0.2,
            wall_run_jump_max_angle: 60.0,
            ..FullTun::default()
        };
        let inp = FullIn {
            delta: 1.0 / 64.0,
            move_y: 1.0,
            yaw: 180.0,
            speed_base: 8.0,
            effect_max_speed: 1.0,
            surface_max_speed_scale: 1.0,
            surface_friction: 1.0,
            effect_friction: 1.0,
            surface_accelerate: 1.0,
            effect_accelerate: 1.0,
            air_accel: 12.0,
            gravity: 20.0,
            wall_run_gravity: 4.0,
            ..FullIn::default()
        };
        let masks = Masks {
            collision: 1 | 8,
            pass_out: 8,
        };
        let tun = Tun { radius: 0.4, ..TUN };
        let mut body = Body::default();
        let mut out = Vec::new();
        for run in 0..runs {
            let mut s = State {
                mode: AIR,
                ground_normal: UP,
                height: 1.8,
                position: Vec3::new(half - 0.55, 1.2, -half + 2.0 + run as f32),
                velocity: Vec3::new(1.5, 2.0, 7.0),
                ..State::default()
            };
            let mut mv = Moves {
                time_since_grounded: 1000.0,
                time_since_jump_pressed: 1000.0,
                time_since_jump: 1000.0,
                wall_run_time: -1.0,
                slide_time: -1.0,
                ..Moves::default()
            };
            for _ in 0..90 {
                body.begin_tick();
                bracket(space, true);
                let mut c = TickCounters::default();
                let mut mc = MoveCounters::default();
                let mut m = Mover::new(&mut body, space, masks, None, tun);
                full::tick(&mut m, &mut s, &tt, &ft, &inp, &mut mv, &mut c, &mut mc);
                bracket(space, false);
                body.end_tick();
                mv.wall_run_cooldown_left = (mv.wall_run_cooldown_left - inp.delta).max(0.0);
                out.push((s, mv, c, mc));
            }
        }
        out
    }

    #[test]
    fn a_remembered_wall_run_matches_the_plain_one() {
        let half = 12.0;
        let mut plain = Room::sized(half);
        let want = wall_runs(&mut plain, half, 6, |_, _| {});
        let mut memo = MemoSpace::new(Room::sized(half));
        let got = wall_runs(
            &mut memo,
            half,
            6,
            |m, open| {
                if open { m.open() } else { m.close() }
            },
        );
        assert_eq!(want.len(), got.len());
        for (i, (w, g)) in want.iter().zip(&got).enumerate() {
            assert_eq!(w.0, g.0, "state at tick {i}");
            assert_eq!(w.1, g.1, "moves at tick {i}");
            assert_eq!(w.2, g.2, "counters at tick {i}");
            assert_eq!(w.3, g.3, "move counters at tick {i}");
        }
        let runs: i64 = want.iter().map(|t| t.3.wall_runs).sum();
        assert!(runs >= 6, "every bounce wall runs: {runs}");
        let (a, b) = (plain.asked, memo.inner.asked);
        assert_eq!(a[2..], b[2..], "overlaps and rays pass through");
        let hits = memo.hits.iter().sum::<u64>() as u32;
        assert_eq!(b[0] + b[1] + hits, a[0] + a[1]);
    }

    #[test]
    fn closed_passes_every_query_through() {
        let mut m = MemoSpace::new(Room::default());
        m.set_height(1.8);
        for _ in 0..3 {
            m.rest(Vec3::new(0.0, 0.5, 0.0));
            m.cast(Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, -1.0, 0.0));
        }
        assert_eq!(m.inner.asked[..2], [3, 3]);
        assert_eq!(m.hits, [0, 0]);
    }

    #[test]
    fn the_key_is_the_shape_and_the_bits() {
        let mut m = MemoSpace::new(Room::default());
        m.set_height(1.8);
        m.set_radius(0.3);
        m.set_mask(1);
        m.open();
        let at = Vec3::new(0.0, 0.5, 0.0);
        let one = m.rest(at);
        assert_eq!(m.rest(at), one);
        assert_eq!(m.inner.asked[1], 1);
        m.set_mask(2);
        let two = m.rest(at);
        assert_ne!(two, one, "a new mask is a new query");
        m.set_mask(1);
        m.rest(Vec3::new(-0.0, 0.5, 0.0));
        assert_eq!(m.inner.asked[1], 3, "-0.0 is not 0.0");
        m.set_radius(0.31);
        m.rest(at);
        m.set_height(1.0);
        m.rest(at);
        assert_eq!(m.inner.asked[1], 5);
        m.close();
        m.open();
        m.rest(at);
        assert_eq!(m.inner.asked[1], 6, "close forgets");
    }

    #[test]
    fn the_oldest_answer_is_dropped_first() {
        let mut m = MemoSpace::new(Room::default());
        m.set_height(1.8);
        m.open();
        for i in 0..=SLOTS {
            m.rest(Vec3::new(i as f32 * 0.01, 0.5, 0.0));
        }
        m.rest(Vec3::new(SLOTS as f32 * 0.01, 0.5, 0.0));
        assert_eq!(m.inner.asked[1] as usize, SLOTS + 1);
        m.rest(Vec3::new(0.0, 0.5, 0.0));
        assert_eq!(m.inner.asked[1] as usize, SLOTS + 2);
    }
}
