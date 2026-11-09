//! The player motor's collision body: dot-player-controller's `DotFpsPhysicsBody` (sweep, rest
//! contact, overlap and their fallbacks), spelled operation for operation so that a native client
//! and a web client running the script agree to the bit.
//!
//! The engine's physics queries stay the engine's: this crate decides which ones to ask and what to
//! make of the answers, and reaches the physics server through [`Space`]. Every vector operation is
//! [`Vec3`]'s (`f32`, engine order); every scalar the script keeps in a `float` is an `f64` here.

pub use dot_sim_math::Vec3;
use dot_sim_math::{clampf, maxf, minf};

pub mod slide;
pub mod tick;

/// `Vector3.UP`.
pub const UP: Vec3 = Vec3::new(0.0, 1.0, 0.0);
/// `Vector3.DOWN`.
pub const DOWN: Vec3 = Vec3::new(0.0, -1.0, 0.0);

/// How much deeper than `unsafe` a sweep looks for a contact the first rest query missed (metres).
pub const CONTACT_RETRY_DEPTHS: [f64; 3] = [0.001, 0.002, 0.004];
/// How deep `unsafe` may be before a sweep's contact counts as late (metres).
pub const LATE_CONTACT_DEPTH: f64 = 0.0005;
/// How clear a capsule moved out along a reversed normal must be for the reversal to be believed.
pub const FLIP_CHECK_CLEARANCE: f64 = 0.0005;

/// One contact the engine reported: a rest query's or a ray's.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Contact {
    pub normal: Vec3,
    pub point: Vec3,
    pub collider_id: i64,
}

/// The physics queries a body makes, over a capsule shape the body sizes.
///
/// Shape queries use the mask last given to [`Space::set_mask`] and the shape last given to
/// [`Space::set_radius`] and [`Space::set_height`]; every query but [`Space::cast`] runs with zero motion.
pub trait Space {
    /// The capsule's radius.
    fn set_radius(&mut self, radius: f32);
    /// The capsule's total height, caps included.
    fn set_height(&mut self, height: f32);
    /// The shape queries' collision mask (the engine keeps the low 32 bits).
    fn set_mask(&mut self, mask: i64);
    /// `cast_motion` from `from` along `motion`: `(safe, unsafe)`, or `None` for an empty answer.
    fn cast(&mut self, from: Vec3, motion: Vec3) -> Option<(f32, f32)>;
    /// `get_rest_info` at `at`.
    fn rest(&mut self, at: Vec3) -> Option<Contact>;
    /// `intersect_shape` at `at`, one result: whether anything is there.
    fn overlap(&mut self, at: Vec3) -> bool;
    /// `intersect_ray` from `from` to `to` under `mask`, areas excluded.
    fn ray(&mut self, from: Vec3, to: Vec3, mask: i64) -> Option<Contact>;
}

/// `DotFpsBody.Hit`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hit {
    pub hit: bool,
    pub fraction: f64,
    pub normal: Vec3,
    pub point: Vec3,
    pub collider_id: i64,
    pub depth: f64,
}

impl Hit {
    /// `Hit.miss()`.
    pub const MISS: Hit = Hit {
        hit: false,
        fraction: 1.0,
        normal: UP,
        point: Vec3::new(0.0, 0.0, 0.0),
        collider_id: 0,
        depth: 0.0,
    };
}

/// `DotFpsBody.capsule_support`: how far the capsule reaches from its centre along `normal`.
#[inline]
pub fn capsule_support(normal: Vec3, height: f64, radius: f64) -> f64 {
    maxf(height * 0.5 - radius, 0.0) * (normal.y as f64).abs() + radius
}

/// The masks a query is filtered by: the body's `collision_mask` and `pass_out_mask`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Masks {
    pub collision: i64,
    pub pass_out: i64,
}

/// A body's state between queries: what the shape was last set to, the in-tick rest memo, counters.
#[derive(Debug)]
pub struct Body {
    cfg_mask: i64,
    cfg_radius: f64,
    cfg_height: f64,
    pub perf_caches: bool,
    in_tick: bool,
    rm_valid: bool,
    rm_at: Vec3,
    rm_height: f64,
    rm_radius: f64,
    rm_mask: i64,
    rm_found: Option<(Vec3, Vec3, i64, f64)>,
    /// Queries counted the way the script counts them.
    pub query_count: i64,
    /// Rest contacts turned round (see [`Body::rest_contact`]).
    pub flipped_normals: i64,
    /// Rest queries answered from the in-tick memo.
    pub rest_memo_hits: i64,
}

impl Default for Body {
    fn default() -> Self {
        Self {
            cfg_mask: -1,
            cfg_radius: f64::NAN,
            cfg_height: f64::NAN,
            perf_caches: true,
            in_tick: false,
            rm_valid: false,
            rm_at: Vec3::default(),
            rm_height: 0.0,
            rm_radius: 0.0,
            rm_mask: 0,
            rm_found: None,
            query_count: 0,
            flipped_normals: 0,
            rest_memo_hits: 0,
        }
    }
}

impl Body {
    /// Opens a motor tick: the rest memo is live until [`Body::end_tick`].
    pub fn begin_tick(&mut self) {
        self.in_tick = true;
        self.rm_valid = false;
    }

    /// Closes a motor tick.
    pub fn end_tick(&mut self) {
        self.in_tick = false;
        self.rm_valid = false;
    }

    /// Forgets the memo: the exclusion list or the space changed.
    pub fn invalidate(&mut self) {
        self.rm_valid = false;
    }

    fn set_mask<S: Space>(&mut self, s: &mut S, mask: i64) {
        if mask != self.cfg_mask {
            s.set_mask(mask);
            self.cfg_mask = mask;
        }
    }

    fn configure<S: Space>(&mut self, s: &mut S, height: f64, radius: f64, m: Masks) {
        let total = maxf(height, radius * 2.0);
        // Each written only when it differs from what this body last wrote, as the script does.
        if radius != self.cfg_radius {
            s.set_radius(radius as f32);
            self.cfg_radius = radius;
        }
        if total != self.cfg_height {
            s.set_height(total as f32);
            self.cfg_height = total;
        }
        self.set_mask(s, m.collision & !m.pass_out);
    }

    /// `DotFpsPhysicsBody.sweep`.
    pub fn sweep<S: Space>(
        &mut self,
        s: &mut S,
        from: Vec3,
        motion: Vec3,
        height: f64,
        radius: f64,
        m: Masks,
    ) -> Hit {
        let mut result = Hit::MISS;
        if (motion.length_squared() as f64) <= 0.0 {
            return result;
        }
        self.configure(s, height, radius, m);
        self.query_count += 1;

        self.set_mask(s, m.collision);
        if m.pass_out != 0 {
            self.set_mask(s, m.collision & m.pass_out);
            if s.overlap(from) {
                self.set_mask(s, m.collision & !m.pass_out);
            } else {
                self.set_mask(s, m.collision);
            }
        }

        let Some((safe, unsafe_)) = s.cast(from, motion) else {
            return result;
        };
        let safe = safe as f64;
        let unsafe_ = unsafe_ as f64;

        if safe >= 1.0 {
            self.set_mask(s, m.collision & !m.pass_out);
            if let Some(resting) = self.blocking_rest_contact(s, from, motion, height, radius, m) {
                return resting;
            }
            if (motion.y as f64) < 0.0 {
                let below = self.downward_ray_fallback(s, from, motion, height, m);
                if below.hit {
                    return below;
                }
            }
            return self
                .end_overlap_fallback(s, from, motion, height, radius, m)
                .unwrap_or(result);
        }

        result.hit = true;
        result.fraction = clampf(safe, 0.0, 1.0);

        let mut contact = s.rest(from + motion.scale(unsafe_));
        if contact.is_none() {
            let along = motion.normalized();
            for depth in CONTACT_RETRY_DEPTHS {
                contact = s.rest(from + motion.scale(unsafe_) + along.scale(depth));
                if contact.is_some() {
                    break;
                }
            }
        }

        if contact.is_none() && (motion.normalized().dot(DOWN) as f64) >= 0.7 {
            return self.downward_ray_fallback(s, from, motion, height, m);
        }

        let Some(c) = contact else {
            return self
                .end_overlap_fallback(s, from, motion, height, radius, m)
                .unwrap_or(Hit::MISS);
        };

        result.normal = c.normal;
        result.point = c.point;
        result.collider_id = c.collider_id;

        if !result.normal.is_normalized() {
            result.normal = -motion.normalized();
            return result;
        }

        let at_contact = from + motion.scale(unsafe_);
        result.depth = maxf(
            result.normal.dot(
                result.point
                    - (at_contact
                        - result
                            .normal
                            .scale(capsule_support(result.normal, height, radius))),
            ) as f64,
            0.0,
        );
        self.turn_round_flipped(s, &mut result, at_contact, height, radius, m);
        result.depth = 0.0;

        let approach = -(motion.dot(result.normal) as f64);
        if approach > 1e-9 {
            let at_unsafe = from + motion.scale(unsafe_);
            let deepest = at_unsafe
                - result
                    .normal
                    .scale(capsule_support(result.normal, height, radius));
            let depth = result.normal.dot(result.point - deepest) as f64;
            if depth > LATE_CONTACT_DEPTH {
                result.fraction =
                    clampf(minf(result.fraction, unsafe_ - depth / approach), 0.0, 1.0);
            }
        }
        result
    }

    fn downward_ray_fallback<S: Space>(
        &mut self,
        s: &mut S,
        from: Vec3,
        motion: Vec3,
        height: f64,
        m: Masks,
    ) -> Hit {
        let mut result = Hit::MISS;
        let length = motion.length() as f64;
        if length <= 0.0 {
            return result;
        }
        let direction = motion.div_scalar(length);
        if (direction.dot(DOWN) as f64) < 0.7 {
            return result;
        }
        let foot = from - UP.scale(height * 0.5);
        self.query_count += 1;
        let Some(c) = s.ray(foot, foot + motion, m.collision & !m.pass_out) else {
            return result;
        };
        result.hit = true;
        result.fraction = clampf(foot.distance_to(c.point) as f64 / length, 0.0, 1.0);
        result.normal = c.normal;
        result.point = c.point;
        result.collider_id = c.collider_id;
        if !result.normal.is_normalized() {
            result.normal = UP;
        }
        result
    }

    fn end_overlap_fallback<S: Space>(
        &mut self,
        s: &mut S,
        from: Vec3,
        motion: Vec3,
        height: f64,
        radius: f64,
        m: Masks,
    ) -> Option<Hit> {
        let at = from + motion;
        let mut end = self.raw_rest_or_none(s, at, height, radius, m)?;
        self.turn_round_flipped(s, &mut end, at, height, radius, m);
        let approach = -(motion.dot(end.normal) as f64);
        if approach <= 1e-9 || end.depth <= 0.0 {
            return None;
        }
        Some(Hit {
            hit: true,
            fraction: clampf(1.0 - end.depth / approach, 0.0, 1.0),
            normal: end.normal,
            point: end.point,
            collider_id: end.collider_id,
            depth: 0.0,
        })
    }

    fn blocking_rest_contact<S: Space>(
        &mut self,
        s: &mut S,
        from: Vec3,
        motion: Vec3,
        height: f64,
        radius: f64,
        m: Masks,
    ) -> Option<Hit> {
        let mut resting = self.raw_rest_or_none(s, from, height, radius, m)?;
        self.turn_round_flipped(s, &mut resting, from, height, radius, m);
        if (motion.dot(resting.normal) as f64) >= 0.0 {
            return None;
        }
        Some(resting)
    }

    /// `DotFpsPhysicsBody.rest_contact`: the engine's rest answer at `at`, its normal turned round
    /// when it points into the surface it came from.
    pub fn rest_contact<S: Space>(
        &mut self,
        s: &mut S,
        at: Vec3,
        height: f64,
        radius: f64,
        m: Masks,
    ) -> Hit {
        let Some(mut result) = self.raw_rest_or_none(s, at, height, radius, m) else {
            return Hit::MISS;
        };
        self.turn_round_flipped(s, &mut result, at, height, radius, m);
        result
    }

    fn turn_round_flipped<S: Space>(
        &mut self,
        s: &mut S,
        result: &mut Hit,
        at: Vec3,
        height: f64,
        radius: f64,
        m: Masks,
    ) {
        let support = capsule_support(result.normal, height, radius);
        if result.depth <= support {
            return;
        }
        let reversed_depth = maxf(support * 2.0 - result.depth, 0.0);
        let probe = at - result.normal.scale(reversed_depth + FLIP_CHECK_CLEARANCE);
        if let Some(after) = self.raw_rest_or_none(s, probe, height, radius, m)
            && after.depth > FLIP_CHECK_CLEARANCE
        {
            return;
        }
        result.normal = -result.normal;
        result.depth = reversed_depth;
        self.flipped_normals += 1;
    }

    fn raw_rest_or_none<S: Space>(
        &mut self,
        s: &mut S,
        at: Vec3,
        height: f64,
        radius: f64,
        m: Masks,
    ) -> Option<Hit> {
        self.configure(s, height, radius, m);
        let memo = self.perf_caches && self.in_tick;
        if memo
            && self.rm_valid
            && at == self.rm_at
            && height == self.rm_height
            && radius == self.rm_radius
            && self.cfg_mask == self.rm_mask
        {
            self.rest_memo_hits += 1;
            let (normal, point, collider_id, depth) = self.rm_found?;
            return Some(Hit {
                hit: true,
                fraction: 0.0,
                normal,
                point,
                collider_id,
                depth,
            });
        }
        let found = self.raw_rest_query(s, at, height, radius);
        if memo {
            self.rm_valid = true;
            self.rm_at = at;
            self.rm_height = height;
            self.rm_radius = radius;
            self.rm_mask = self.cfg_mask;
            self.rm_found = found.map(|h| (h.normal, h.point, h.collider_id, h.depth));
        }
        found
    }

    fn raw_rest_query<S: Space>(
        &mut self,
        s: &mut S,
        at: Vec3,
        height: f64,
        radius: f64,
    ) -> Option<Hit> {
        self.query_count += 1;
        let c = s.rest(at)?;
        let normal = c.normal;
        if !normal.is_normalized() {
            return None;
        }
        let deepest = at - normal.scale(capsule_support(normal, height, radius));
        Some(Hit {
            hit: true,
            fraction: 0.0,
            normal,
            point: c.point,
            collider_id: c.collider_id,
            depth: maxf(normal.dot(c.point - deepest) as f64, 0.0),
        })
    }

    /// `DotFpsPhysicsBody.overlaps`.
    pub fn overlaps<S: Space>(
        &mut self,
        s: &mut S,
        at: Vec3,
        height: f64,
        radius: f64,
        m: Masks,
    ) -> bool {
        self.configure(s, height, radius, m);
        self.query_count += 1;
        s.overlap(at)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A floor plane at y = 0 under a capsule, answered analytically; records what was asked.
    #[derive(Default)]
    struct Floor {
        radius: f32,
        height: f32,
        mask: i64,
        asked: Vec<&'static str>,
        flip: bool,
    }

    impl Floor {
        fn bottom(&self, at: Vec3) -> f32 {
            at.y - self.height * 0.5
        }
    }

    impl Space for Floor {
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
            self.asked.push("cast");
            let b = self.bottom(from);
            if motion.y >= 0.0 || b + motion.y > 0.0 {
                return Some((1.0, 1.0));
            }
            let t = b / -motion.y;
            Some((t, (t + 0.001).min(1.0)))
        }
        fn rest(&mut self, at: Vec3) -> Option<Contact> {
            self.asked.push("rest");
            let b = self.bottom(at);
            if b > 0.0 {
                return None;
            }
            let n = if self.flip { DOWN } else { UP };
            Some(Contact {
                normal: n,
                point: Vec3::new(at.x, 0.0, at.z),
                collider_id: 7,
            })
        }
        fn overlap(&mut self, at: Vec3) -> bool {
            self.asked.push("overlap");
            self.bottom(at) < 0.0
        }
        fn ray(&mut self, from: Vec3, to: Vec3, _mask: i64) -> Option<Contact> {
            self.asked.push("ray");
            if from.y >= 0.0 && to.y <= 0.0 {
                return Some(Contact {
                    normal: UP,
                    point: Vec3::new(from.x, 0.0, from.z),
                    collider_id: 7,
                });
            }
            None
        }
    }

    const M: Masks = Masks {
        collision: 1,
        pass_out: 0,
    };

    #[test]
    fn zero_motion_is_a_miss_without_a_query() {
        let mut s = Floor::default();
        let mut b = Body::default();
        let h = b.sweep(
            &mut s,
            Vec3::new(0.0, 5.0, 0.0),
            Vec3::default(),
            1.8,
            0.4,
            M,
        );
        assert_eq!(h, Hit::MISS);
        assert!(s.asked.is_empty());
        assert_eq!(b.query_count, 0);
    }

    #[test]
    fn falling_onto_the_floor_hits_it_with_its_normal() {
        let mut s = Floor::default();
        let mut b = Body::default();
        let h = b.sweep(
            &mut s,
            Vec3::new(0.0, 1.4, 0.0),
            Vec3::new(0.0, -1.0, 0.0),
            1.8,
            0.4,
            M,
        );
        assert!(h.hit);
        assert_eq!(h.normal, UP);
        assert_eq!(h.collider_id, 7);
        assert!((h.fraction - 0.5).abs() < 1e-6, "{}", h.fraction);
        assert_eq!(h.depth, 0.0);
        assert_eq!(s.mask, 1);
    }

    #[test]
    fn rest_memo_answers_the_same_question_once_per_tick() {
        let mut s = Floor::default();
        let mut b = Body::default();
        b.begin_tick();
        let at = Vec3::new(0.0, 0.85, 0.0);
        let a = b.rest_contact(&mut s, at, 1.8, 0.4, M);
        let c = b.rest_contact(&mut s, at, 1.8, 0.4, M);
        assert_eq!(a, c);
        assert_eq!(b.rest_memo_hits, 1);
        assert_eq!(b.query_count, 1);
        b.end_tick();
        b.rest_contact(&mut s, at, 1.8, 0.4, M);
        assert_eq!(b.query_count, 2);
    }

    #[test]
    fn a_reversed_normal_is_turned_round_when_the_probe_is_clear() {
        let mut s = Floor {
            flip: true,
            ..Default::default()
        };
        let mut b = Body::default();
        // 0.01 into the floor; the engine says DOWN, so the depth reads as the far side.
        let at = Vec3::new(0.0, 0.89, 0.0);
        let h = b.rest_contact(&mut s, at, 1.8, 0.4, M);
        assert!(h.hit);
        // DOWN: support is 0.9, depth = DOWN.(point - (at + 0.9 UP)) = 1.79 > 0.9, the probe
        // moves the capsule up 0.0105 out of the floor and finds nothing, so it is turned round.
        assert_eq!(h.normal, UP);
        assert_eq!(b.flipped_normals, 1);
    }

    #[test]
    fn pass_out_layers_are_left_out_while_inside_them() {
        let mut s = Floor::default();
        let mut b = Body::default();
        let m = Masks {
            collision: 0b1001,
            pass_out: 0b1000,
        };
        b.sweep(
            &mut s,
            Vec3::new(0.0, 0.5, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            1.8,
            0.4,
            m,
        );
        assert_eq!(s.asked[0], "overlap");
        assert_eq!(s.asked[1], "cast");
    }
}
