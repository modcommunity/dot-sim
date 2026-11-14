//! Snapshot interpolation for remote entities, bit-identical to dot-net's `DotNetInterpolator`.
//!
//! The script stays the specification: `Track.push`, `sample`, `_extrapolate`, `_continue`,
//! `_blend`, `_blend_value` and `_ease_resume` are transcribed here operation for operation.
//! Script floats are f64 (`lerpf`, the render tick, the blend weight); `Vector3` and `Vector2`
//! are f32, and a script float meeting one (`Vector3.lerp`'s weight, `/ span`, `* ahead`) is
//! rounded to f32 first, as the engine's `real_t` parameters do. No fused multiply-add, and `exp`
//! is the platform's, as the engine's is.
//!
//! A sample's values keep the script dictionary's order (pushed order, merges in place, the
//! blend's newer keys then the older-only ones). Samples are shared the way the script shares
//! its dictionaries: a duplicate tick merges into the stored sample, and the two samples
//! `_ease_resume` remembers see that merge.
//!
//! Values other than floats and 2D/3D vectors are `Other` and stay opaque: the binding decides
//! which of those blend continuously (a quaternion's slerp, a colour's lerp, in the engine) and
//! the rest switch at the midpoint.

use std::cell::RefCell;
use std::collections::{HashMap, HashSet, VecDeque};
use std::rc::Rc;

#[cfg(test)]
mod tests;

/// Samples kept per entity: `DotNetInterpolator.TRACK_CAPACITY`.
pub const TRACK_CAPACITY: usize = 32;

/// A value the crate does not do arithmetic on: the binding's variant.
pub trait Other: Clone {
    /// Whether the two hold the same type (`typeof` in the script).
    fn same_kind(&self, other: &Self) -> bool;
    /// The continuous blend toward `to` at `alpha`, or `None` for a discrete value (which then
    /// switches at the midpoint).
    fn blend(&self, to: &Self, alpha: f64) -> Option<Self>;
}

/// One property's value as the script holds it.
#[derive(Clone, Debug, PartialEq)]
pub enum Value<O> {
    Float(f64),
    Vec3([f32; 3]),
    Vec2([f32; 2]),
    Other(O),
}

/// A dictionary's entries in its order: interned property, value.
pub type Values<O> = Vec<(u32, Value<O>)>;

fn find<O>(values: &Values<O>, key: u32) -> Option<&Value<O>> {
    values.iter().find(|(k, _)| *k == key).map(|(_, v)| v)
}

/// The config numbers the interpolator reads, as the script computes them.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// `config.max_extrapolation_sec * float(config.tick_rate)`.
    pub max_ticks: f64,
    /// `config.resume_ease_max`.
    pub ease_max: f64,
    /// `config.resume_ease_sec`.
    pub ease_sec: f64,
}

/// The script's counters: `_samples`, `_stalls`, `_extrapolations`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Counts {
    pub samples: i64,
    pub stalls: i64,
    pub extrapolations: i64,
}

/// One snapshot of one entity: `{tick, values}`.
#[derive(Debug)]
pub struct Sample<O> {
    pub tick: i64,
    pub values: Values<O>,
}

type Shared<O> = Rc<RefCell<Sample<O>>>;

/// A position offset left over from a resumed extrapolation.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Offset {
    Vec3([f32; 3]),
    Vec2([f32; 2]),
}

/// One entity's recent samples, ordered by tick, and what `_ease_resume` carries between frames.
/// `P` is the binding's write plan, kept with the track as the script keeps it.
pub struct Track<O, P> {
    samples: VecDeque<Shared<O>>,
    offsets: Vec<(u32, Offset)>,
    ahead_from: Option<(Shared<O>, Shared<O>)>,
    pub plan: P,
}

impl<O, P: Default> Track<O, P> {
    fn new() -> Self {
        Track {
            samples: VecDeque::new(),
            offsets: Vec::new(),
            ahead_from: None,
            plan: P::default(),
        }
    }
}

impl<O, P> Track<O, P> {
    /// `Track.push`: in tick order, a duplicate tick merged into the stored sample.
    fn push(&mut self, tick: i64, values: Values<O>) {
        let s = &mut self.samples;
        let mut index = s.len();
        while index > 0 && s[index - 1].borrow().tick > tick {
            index -= 1;
        }
        if index > 0 && s[index - 1].borrow().tick == tick {
            let mut existing = s[index - 1].borrow_mut();
            for (k, v) in values {
                match existing.values.iter_mut().find(|(e, _)| *e == k) {
                    Some(slot) => slot.1 = v,
                    None => existing.values.push((k, v)),
                }
            }
            return;
        }
        s.insert(index, Rc::new(RefCell::new(Sample { tick, values })));
        while s.len() > TRACK_CAPACITY {
            s.pop_front();
        }
    }

    pub fn len(&self) -> usize {
        self.samples.len()
    }

    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }
}

fn lerp32(a: f32, b: f32, w: f32) -> f32 {
    a + (b - a) * w
}

fn len3(v: [f32; 3]) -> f32 {
    let x2 = v[0] * v[0];
    let y2 = v[1] * v[1];
    let z2 = v[2] * v[2];
    (x2 + y2 + z2).sqrt()
}

fn len2(v: [f32; 2]) -> f32 {
    (v[0] * v[0] + v[1] * v[1]).sqrt()
}

/// `_blend_value`, with `_blend`'s inlined float and `Vector3` cases (the same arithmetic).
fn blend_value<O: Other>(from: &Value<O>, to: &Value<O>, alpha: f64) -> Value<O> {
    match (from, to) {
        (Value::Float(a), Value::Float(b)) => Value::Float(a + (b - a) * alpha),
        (Value::Vec3(a), Value::Vec3(b)) => {
            let w = alpha as f32;
            Value::Vec3([
                lerp32(a[0], b[0], w),
                lerp32(a[1], b[1], w),
                lerp32(a[2], b[2], w),
            ])
        }
        (Value::Vec2(a), Value::Vec2(b)) => {
            let w = alpha as f32;
            Value::Vec2([lerp32(a[0], b[0], w), lerp32(a[1], b[1], w)])
        }
        (Value::Other(a), Value::Other(b)) if a.same_kind(b) => Value::Other(
            a.blend(b, alpha)
                .unwrap_or_else(|| if alpha >= 0.5 { b.clone() } else { a.clone() }),
        ),
        _ => to.clone(),
    }
}

/// `_blend`: newer's keys in order, then those only the older sample has.
fn blend<O: Other>(older: &Values<O>, newer: &Values<O>, alpha: f64) -> Values<O> {
    let mut out: Values<O> = Vec::with_capacity(newer.len().max(older.len()));
    let mut shared = 0;
    for (k, to) in newer {
        match find(older, *k) {
            None => out.push((*k, to.clone())),
            Some(from) => {
                shared += 1;
                out.push((*k, blend_value(from, to, alpha)));
            }
        }
    }
    if shared < older.len() {
        for (k, v) in older {
            if find(&out, *k).is_none() {
                out.push((*k, v.clone()));
            }
        }
    }
    out
}

/// `_continue`: the straight line through `previous` and `newest` at `render`, positions only.
fn continue_line<O: Clone>(
    newest: &Sample<O>,
    previous: &Sample<O>,
    render: f64,
    max_ticks: f64,
) -> Values<O> {
    let ahead = render - newest.tick as f64;
    if ahead <= 0.0 || ahead > max_ticks {
        return newest.values.clone();
    }
    let span = newest.tick as f64 - previous.tick as f64;
    if span <= 0.0 {
        return newest.values.clone();
    }
    let (s, a) = (span as f32, ahead as f32);
    let mut out = newest.values.clone();
    for (i, (k, current)) in newest.values.iter().enumerate() {
        let Some(before) = find(&previous.values, *k) else {
            continue;
        };
        match (current, before) {
            (Value::Vec3(c), Value::Vec3(b)) => {
                let v = [(c[0] - b[0]) / s, (c[1] - b[1]) / s, (c[2] - b[2]) / s];
                out[i].1 = Value::Vec3([c[0] + v[0] * a, c[1] + v[1] * a, c[2] + v[2] * a]);
            }
            (Value::Vec2(c), Value::Vec2(b)) => {
                let v = [(c[0] - b[0]) / s, (c[1] - b[1]) / s];
                out[i].1 = Value::Vec2([c[0] + v[0] * a, c[1] + v[1] * a]);
            }
            _ => {}
        }
    }
    out
}

/// `sample` for one track at `render` (`server_tick - delay_ticks()`), counting as it does.
fn sample_track<O: Other, P>(
    track: &Track<O, P>,
    render: f64,
    cfg: &Config,
    counts: &mut Counts,
) -> Values<O> {
    let s = &track.samples;
    let n = s.len();
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        return s[0].borrow().values.clone();
    }
    counts.samples += 1;

    let newest = s[n - 1].borrow();
    if render >= newest.tick as f64 {
        counts.stalls += 1;
        // `_extrapolate`.
        let ahead = render - newest.tick as f64;
        if ahead <= 0.0 || cfg.max_ticks <= 0.0 || ahead > cfg.max_ticks {
            return newest.values.clone();
        }
        counts.extrapolations += 1;
        return continue_line(&newest, &s[n - 2].borrow(), render, cfg.max_ticks);
    }
    if render <= s[0].borrow().tick as f64 {
        return s[0].borrow().values.clone();
    }
    for i in (0..n - 1).rev() {
        let older = s[i].borrow();
        let older_tick = older.tick as f64;
        if render <= older_tick {
            continue;
        }
        let newer = s[i + 1].borrow();
        let newer_tick = newer.tick as f64;
        if render <= newer_tick {
            let span = newer_tick - older_tick;
            let alpha = if span <= 0.0 {
                0.0
            } else {
                (render - older_tick) / span
            }
            .clamp(0.0, 1.0);
            return blend(&older.values, &newer.values, alpha);
        }
    }
    newest.values.clone()
}

/// `_ease_resume`: adds what is left of a jump out of an extrapolation, and notes what this
/// frame extrapolated from.
fn ease_resume<O: Other, P>(
    track: &mut Track<O, P>,
    values: Values<O>,
    render: f64,
    frame_elapsed: f64,
    cfg: &Config,
) -> Values<O> {
    if frame_elapsed <= 0.0 {
        return values;
    }
    let n = track.samples.len();
    let ahead = n >= 2 && render >= track.samples[n - 1].borrow().tick as f64;

    if let Some((from0, from1)) = &track.ahead_from {
        let resumed = !ahead || track.samples[n - 1].borrow().tick != from0.borrow().tick;
        if resumed {
            let was = continue_line(&from0.borrow(), &from1.borrow(), render, cfg.max_ticks);
            for (k, now) in &values {
                let total = match (now, find(&was, *k)) {
                    (Value::Vec3(v), Some(Value::Vec3(t))) => {
                        let base = match find_offset(&track.offsets, *k) {
                            Some(Offset::Vec3(o)) => o,
                            _ => [v[0] * 0.0, v[1] * 0.0, v[2] * 0.0],
                        };
                        let jump = [t[0] - v[0], t[1] - v[1], t[2] - v[2]];
                        let total = [base[0] + jump[0], base[1] + jump[1], base[2] + jump[2]];
                        (Offset::Vec3(total), len3(total))
                    }
                    (Value::Vec2(v), Some(Value::Vec2(t))) => {
                        let base = match find_offset(&track.offsets, *k) {
                            Some(Offset::Vec2(o)) => o,
                            _ => [v[0] * 0.0, v[1] * 0.0],
                        };
                        let jump = [t[0] - v[0], t[1] - v[1]];
                        let total = [base[0] + jump[0], base[1] + jump[1]];
                        (Offset::Vec2(total), len2(total))
                    }
                    _ => continue,
                };
                let at = track.offsets.iter().position(|(o, _)| o == k);
                if total.1 as f64 <= cfg.ease_max {
                    match at {
                        Some(i) => track.offsets[i].1 = total.0,
                        None => track.offsets.push((*k, total.0)),
                    }
                } else if let Some(i) = at {
                    track.offsets.remove(i);
                }
            }
        }
    }

    track.ahead_from = if ahead {
        Some((track.samples[n - 1].clone(), track.samples[n - 2].clone()))
    } else {
        None
    };

    if track.offsets.is_empty() {
        return values;
    }

    let keep = (-frame_elapsed / cfg.ease_sec).exp() as f32;
    let mut out = values;
    let mut i = 0;
    while i < track.offsets.len() {
        let (k, offset) = track.offsets[i];
        let (scaled, length) = match offset {
            Offset::Vec3(o) => {
                let v = [o[0] * keep, o[1] * keep, o[2] * keep];
                (Offset::Vec3(v), len3(v))
            }
            Offset::Vec2(o) => {
                let v = [o[0] * keep, o[1] * keep];
                (Offset::Vec2(v), len2(v))
            }
        };
        let small = (length as f64) < 0.0005;
        let slot = match out.iter().position(|(o, _)| *o == k) {
            Some(slot) if !small => slot,
            _ => {
                track.offsets.remove(i);
                continue;
            }
        };
        track.offsets[i].1 = scaled;
        match (&mut out[slot].1, scaled) {
            (Value::Vec3(v), Offset::Vec3(o)) => *v = [v[0] + o[0], v[1] + o[1], v[2] + o[2]],
            (Value::Vec2(v), Offset::Vec2(o)) => *v = [v[0] + o[0], v[1] + o[1]],
            _ => {}
        }
        i += 1;
    }
    out
}

fn find_offset(offsets: &[(u32, Offset)], key: u32) -> Option<Offset> {
    offsets.iter().find(|(k, _)| *k == key).map(|(_, o)| *o)
}

/// Every entity's track, and the counters: the native half of a `DotNetInterpolator`.
pub struct Interp<O, P> {
    tracks: HashMap<i64, Track<O, P>>,
    pub counts: Counts,
}

impl<O, P> Default for Interp<O, P> {
    fn default() -> Self {
        Interp {
            tracks: HashMap::new(),
            counts: Counts::default(),
        }
    }
}

impl<O: Other, P: Default> Interp<O, P> {
    /// `push`'s track half: the binding mirrors every push here.
    pub fn push(&mut self, net_id: i64, tick: i64, values: Values<O>) {
        self.tracks
            .entry(net_id)
            .or_insert_with(Track::new)
            .push(tick, values);
    }

    /// `sample(net_id, server_tick)` with `render = server_tick - delay_ticks()`: empty when
    /// there is nothing to show.
    pub fn sample(&mut self, net_id: i64, render: f64, cfg: &Config) -> Values<O> {
        match self.tracks.get(&net_id) {
            Some(track) => sample_track(track, render, cfg, &mut self.counts),
            None => Vec::new(),
        }
    }

    /// What `apply` writes: the sample with any resume easing added, or `None` where `apply`
    /// returns early (no track, or nothing sampled).
    pub fn draw(
        &mut self,
        net_id: i64,
        render: f64,
        frame_elapsed: f64,
        cfg: &Config,
    ) -> Option<Values<O>> {
        let track = self.tracks.get_mut(&net_id)?;
        let values = sample_track(track, render, cfg, &mut self.counts);
        if values.is_empty() {
            return None;
        }
        Some(ease_resume(track, values, render, frame_elapsed, cfg))
    }

    /// The binding's write plan for `net_id`'s track.
    pub fn plan_mut(&mut self, net_id: i64) -> Option<&mut P> {
        self.tracks.get_mut(&net_id).map(|t| &mut t.plan)
    }

    pub fn forget(&mut self, net_id: i64) {
        self.tracks.remove(&net_id);
    }

    pub fn clear(&mut self) {
        self.tracks.clear();
    }

    /// Drops every track not in `live`.
    pub fn prune(&mut self, live: &[i64]) {
        let keep: HashSet<i64> = live.iter().copied().collect();
        self.tracks.retain(|id, _| keep.contains(id));
    }

    pub fn track_count(&self) -> usize {
        self.tracks.len()
    }

    pub fn track(&self, net_id: i64) -> Option<&Track<O, P>> {
        self.tracks.get(&net_id)
    }
}
