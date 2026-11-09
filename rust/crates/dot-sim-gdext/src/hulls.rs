//! `DotSimHullWorld`: where every player stands during a server's players' loop, so a motor tick
//! gathers the other players' standing hulls natively (`DotSimFpsBody` hulls mode 3) instead of
//! asking the game script once per player and tick.
//!
//! The answer is the script gather's to the bit (game-arena `_add_hulls_local`): the roster in
//! order, skipping the mover, a slot without a controller, one farther than the reach (the
//! squared distance in `f32`, as `Vector3.distance_squared_to`), a dead one and a pair the
//! pair callable lets pass; each box `AABB(feet - Vector3(r, 0, r), Vector3(r * 2, h, r * 2))`.

use dot_sim_motor::Vec3 as V3;
use dot_sim_motor::slide::Aabb as SlideBox;
use godot::prelude::*;

/// Per slot in [`DotSimHullWorld::begin`]'s data: x, y, z, radius, stand height, flags.
const SLOT_LEN: usize = 6;
/// The slot has a controller and a state.
const PRESENT: i64 = 1;
/// The player is alive.
const ALIVE: i64 = 2;
/// The slot may gather natively (no lag compensation for this mover).
const NATIVE: i64 = 4;

/// Where the players stand during one players' loop, and this loop's pair answers.
#[derive(GodotClass)]
#[class(base = RefCounted, init)]
pub struct DotSimHullWorld {
    live: bool,
    ids: Vec<i64>,
    feet: Vec<V3>,
    radius: Vec<f64>,
    height: Vec<f64>,
    flags: Vec<i64>,
    reach_sq: f64,
    pair: Option<Callable>,
    /// Pair answers this loop, row-major by slot: 0 unasked, 1 blocks, 2 passes.
    memo: Vec<u8>,
    gathers: i64,
    pair_calls: i64,
}

#[godot_api]
impl DotSimHullWorld {
    /// Starts a players' loop: `ids` the roster in order, `data` per slot x, y, z, radius, stand
    /// height and flags (1 present, 2 alive, 4 may gather natively), `pair` the symmetric
    /// `(id, other_id) -> bool` blocks test (invalid: every pair blocks), `reach_sq` the squared
    /// reach. The pair answers of an earlier loop are dropped.
    #[func]
    fn begin(
        &mut self,
        ids: PackedInt64Array,
        data: PackedFloat64Array,
        pair: Callable,
        reach_sq: f64,
    ) {
        let (ids, d) = (ids.as_slice(), data.as_slice());
        let n = ids.len().min(d.len() / SLOT_LEN);
        self.ids.clear();
        self.feet.clear();
        self.radius.clear();
        self.height.clear();
        self.flags.clear();
        for (i, &id) in ids.iter().enumerate().take(n) {
            let s = &d[i * SLOT_LEN..(i + 1) * SLOT_LEN];
            self.ids.push(id);
            self.feet
                .push(V3::new(s[0] as f32, s[1] as f32, s[2] as f32));
            self.radius.push(s[3]);
            self.height.push(s[4]);
            self.flags.push(s[5] as i64);
        }
        self.memo.clear();
        self.memo.resize(n * n, 0);
        self.pair = pair.is_valid().then_some(pair);
        self.reach_sq = reach_sq;
        self.live = true;
    }

    /// Slot `slot` has ticked: it now stands at `x, y, z`.
    #[func]
    fn moved(&mut self, slot: i64, x: f64, y: f64, z: f64) {
        if let Some(f) = usize::try_from(slot)
            .ok()
            .and_then(|s| self.feet.get_mut(s))
        {
            *f = V3::new(x as f32, y as f32, z as f32);
        }
    }

    /// Slot `slot` died (or came back) during the loop.
    #[func]
    fn set_alive(&mut self, slot: i64, alive: bool) {
        if let Some(f) = usize::try_from(slot)
            .ok()
            .and_then(|s| self.flags.get_mut(s))
        {
            *f = if alive { *f | ALIVE } else { *f & !ALIVE };
        }
    }

    /// The loop is over: a motor tick asking from now on gets no answer and gathers by script.
    #[func]
    fn end(&mut self) {
        self.live = false;
    }

    #[func]
    fn is_live(&self) -> bool {
        self.live
    }

    /// The boxes slot `slot` standing at `x, y, z` stops at, as an Array of AABB, whether or not
    /// the slot may gather natively (a test's check against the script gather); empty when the
    /// world is not live.
    #[func]
    fn gather(&mut self, slot: i64, x: f64, y: f64, z: f64) -> VarArray {
        let mut boxes = Vec::new();
        let me = V3::new(x as f32, y as f32, z as f32);
        let mut out = VarArray::new();
        if self.fill(slot, me, true, &mut boxes) {
            for b in boxes {
                let aabb = Aabb::new(
                    Vector3::new(b.pos.x, b.pos.y, b.pos.z),
                    Vector3::new(b.size.x, b.size.y, b.size.z),
                );
                out.push(&aabb.to_variant());
            }
        }
        out
    }

    /// Native gathers and pair-callable calls since the last reset.
    #[func]
    fn counters(&self) -> PackedInt64Array {
        PackedInt64Array::from(&[self.gathers, self.pair_calls][..])
    }

    #[func]
    fn reset_counters(&mut self) {
        self.gathers = 0;
        self.pair_calls = 0;
    }
}

impl DotSimHullWorld {
    /// Fills `out` with the boxes slot `slot`, standing at `me`, stops at. False (nothing
    /// written) when the world is not live or, unless `any`, the slot may not gather natively.
    pub fn fill(&mut self, slot: i64, me: V3, any: bool, out: &mut Vec<SlideBox>) -> bool {
        let n = self.ids.len();
        let Ok(me_i) = usize::try_from(slot) else {
            return false;
        };
        if !self.live || me_i >= n || (!any && self.flags[me_i] & NATIVE == 0) {
            return false;
        }
        out.clear();
        self.gathers += 1;
        let id = self.ids[me_i];
        for i in 0..n {
            if i == me_i || self.flags[i] & PRESENT == 0 {
                continue;
            }
            let f = self.feet[i];
            let (dx, dy, dz) = (me.x - f.x, me.y - f.y, me.z - f.z);
            let d2 = dx * dx + dy * dy + dz * dz;
            if d2 as f64 > self.reach_sq || self.flags[i] & ALIVE == 0 {
                continue;
            }
            if let Some(pair) = &self.pair {
                let mut known = self.memo[me_i * n + i];
                if known == 0 {
                    self.pair_calls += 1;
                    let v = pair.call(&[id.to_variant(), self.ids[i].to_variant()]);
                    known = if v.booleanize() { 1 } else { 2 };
                    self.memo[me_i * n + i] = known;
                    self.memo[i * n + me_i] = known;
                }
                if known == 2 {
                    continue;
                }
            }
            let r = self.radius[i];
            let rf = r as f32;
            let w = (r * 2.0) as f32;
            out.push(SlideBox {
                pos: V3::new(f.x - rf, f.y - 0.0, f.z - rf),
                size: V3::new(w, self.height[i] as f32, w),
            });
        }
        true
    }
}
