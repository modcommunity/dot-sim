//! dot-audio's per-body probes, pure: the footstep stride and landing of `DotAudioFootsteps.advance`,
//! the gate of `DotAudioSlide`/`DotAudioSurf`'s tick (reach, speed, whether the body is asked at
//! all) and `DotAudioSurfRules`' ramp test with its per-physics-frame cache.
//!
//! Each is the script's arithmetic in the script's precision: positions are `Vector3` (`f32`), the
//! script's floats `f64`, and a `Vector2(...).length()` is an `f32` square root widened after.
//! The engine questions (is the body grounded, is it surfing, is it out of earshot) are closures,
//! asked lazily in the script's order so a question the script skips is never asked.

use std::collections::HashMap;

pub use dot_sim_math::{Vec3, clampf, maxf};

/// The script's `lerpf`: `from + (to - from) * weight`.
#[inline]
pub fn lerpf(from: f64, to: f64, weight: f64) -> f64 {
    from + (to - from) * weight
}

/// `Vector2(pos.x - last.x, pos.z - last.z).length()`: the ground distance between two frames, in
/// `f32`, widened to the script's float.
#[inline]
pub fn moved(last: Vec3, pos: Vec3) -> f64 {
    let dx = pos.x - last.x;
    let dz = pos.z - last.z;
    (dx * dx + dz * dz).sqrt() as f64
}

/// `from.distance_squared_to(to)`: `(to - from).length_squared()` in `f32`.
#[inline]
pub fn distance_squared(from: Vec3, to: Vec3) -> f32 {
    (to - from).length_squared()
}

/// The probe ray of footsteps, slide and surf: from `pos + Vector3.UP * 0.3` to
/// `pos + Vector3.DOWN * depth`.
#[inline]
pub fn ray_down(pos: Vec3, depth: f64) -> (Vec3, Vec3) {
    (
        pos + Vec3::new(0.0, 1.0, 0.0).scale(0.3),
        pos + Vec3::new(0.0, -1.0, 0.0).scale(depth),
    )
}

/// `DotAudioSurfRules.is_ramp_normal`: steeper than walkable and not a wall.
#[inline]
pub fn is_ramp(normal_y: f32, min_normal_y: f64, max_normal_y: f64) -> bool {
    let y = normal_y as f64;
    y > min_normal_y && y < max_normal_y
}

/// `DotAudioSurfRules`' `_seen`: one verdict per body per physics frame, emptied past 256 bodies.
#[derive(Default, Debug, Clone)]
pub struct SeenCache {
    seen: HashMap<i64, (u64, bool)>,
}

impl SeenCache {
    /// The verdict already given to body `id` in physics frame `frame`.
    #[inline]
    pub fn get(&self, id: i64, frame: u64) -> Option<bool> {
        match self.seen.get(&id) {
            Some(&(f, yes)) if f == frame => Some(yes),
            _ => None,
        }
    }

    /// Records a verdict; the whole cache is dropped first when it holds more than 256 bodies.
    #[inline]
    pub fn put(&mut self, id: i64, frame: u64, yes: bool) {
        if self.seen.len() > 256 {
            self.seen.clear();
        }
        self.seen.insert(id, (frame, yes));
    }

    pub fn len(&self) -> usize {
        self.seen.len()
    }

    pub fn is_empty(&self) -> bool {
        self.seen.is_empty()
    }
}

/// `DotAudioFootsteps`' tuning, as the script holds it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StepTun {
    pub quiet_speed: f64,
    pub run_speed: f64,
    pub stride_walk: f64,
    pub stride_run: f64,
    pub walk_loudness: f64,
    pub landing_air_time: f64,
}

impl Default for StepTun {
    fn default() -> Self {
        Self {
            quiet_speed: 1.4,
            run_speed: 6.0,
            stride_walk: 1.5,
            stride_run: 2.3,
            walk_loudness: 0.45,
            landing_air_time: 0.3,
        }
    }
}

impl StepTun {
    /// From `[quiet_speed, run_speed, stride_walk, stride_run, walk_loudness, landing_air_time]`;
    /// missing values keep the script's defaults.
    pub fn from_slice(v: &[f64]) -> Self {
        let d = Self::default();
        let at = |i: usize, or: f64| v.get(i).copied().unwrap_or(or);
        Self {
            quiet_speed: at(0, d.quiet_speed),
            run_speed: at(1, d.run_speed),
            stride_walk: at(2, d.stride_walk),
            stride_run: at(3, d.stride_run),
            walk_loudness: at(4, d.walk_loudness),
            landing_air_time: at(5, d.landing_air_time),
        }
    }
}

/// One body's footstep state: the script's `{last, travelled, air, grounded}`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StepState {
    pub last: Vec3,
    pub travelled: f64,
    pub air: f64,
    pub grounded: bool,
}

impl Default for StepState {
    fn default() -> Self {
        Self {
            last: Vec3::new(0.0, 0.0, 0.0),
            travelled: 0.0,
            air: 0.0,
            grounded: true,
        }
    }
}

/// The steps of one advance, in the order the script plays them: a landing (loudness 1.0) and
/// then a stride step with its loudness. Both can come in one frame.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Steps {
    pub landing: bool,
    pub stride: Option<f64>,
}

/// `DotAudioFootsteps.advance` for one body and one physics frame. `out_of_earshot`, `surfing` and
/// `grounded` are asked in that order, each only when the script would ask it.
pub fn step_advance(
    st: &mut StepState,
    tun: &StepTun,
    pos: Vec3,
    delta: f64,
    out_of_earshot: impl FnOnce() -> bool,
    surfing: impl FnOnce() -> bool,
    grounded: impl FnOnce() -> bool,
) -> Steps {
    let mut out = Steps::default();
    let last = st.last;
    st.last = pos;
    let moved = moved(last, pos);
    // A respawn or a correction is not a walk.
    if moved > 4.0 || delta <= 0.0 {
        st.travelled = 0.0;
        return out;
    }
    if out_of_earshot() || surfing() {
        st.travelled = 0.0;
        st.air = 0.0;
        st.grounded = true;
        return out;
    }
    if !grounded() {
        st.air += delta;
        st.grounded = false;
        return out;
    }
    if !st.grounded {
        st.grounded = true;
        if st.air >= tun.landing_air_time {
            out.landing = true;
            st.travelled = 0.0;
        }
        st.air = 0.0;
    }
    let speed = moved / delta;
    if speed < tun.quiet_speed {
        st.travelled = 0.0;
        return out;
    }
    let t = clampf(
        (speed - tun.quiet_speed) / maxf(tun.run_speed - tun.quiet_speed, 0.01),
        0.0,
        1.0,
    );
    st.travelled += moved;
    if st.travelled >= lerpf(tun.stride_walk, tun.stride_run, t) {
        st.travelled = 0.0;
        out.stride = Some(lerpf(tun.walk_loudness, 1.0, t));
    }
    out
}

/// `DotAudioSlide.advance`'s speed: ground distance over `maxf(delta, 0.0001)`, a jump past 60 m/s
/// (a respawn or a correction) read as standing still.
#[inline]
pub fn slide_speed(last: Vec3, pos: Vec3, delta: f64) -> f64 {
    let speed = moved(last, pos) / maxf(delta, 0.0001);
    if speed > 60.0 { 0.0 } else { speed }
}

/// What a slide tick does with one body, as far as it can be decided without the script.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlideGate {
    /// Out of reach and not playing: only its position is kept.
    KeepFar,
    /// Out of reach and playing: the script's `out_of_reach` stops its loop.
    StopFar,
    /// Not playing and not worth asking (switched off, or slower than the quiet speed): only its
    /// position is kept, the `sliding` question is not asked.
    KeepQuiet,
    /// Everything else: the script's `advance`.
    Advance,
}

/// The gate of `DotAudioSlide._physics_process` and the head of `advance` for one body.
/// `ear_sq` is `pos.distance_squared_to(ear)`, `reach_sq` the tick's `reach_squared()` (0: every
/// body is in reach). `quiet` is the speed under which a body not yet playing is not asked
/// (`DotAudioSurf`'s `rules.quiet_speed`), or `None` when every body is asked (`DotAudioSlide`).
#[allow(clippy::neg_cmp_op_on_partial_ord)] // the script's `speed >= quiet`, NaN included
pub fn slide_gate(
    ear_sq: f32,
    reach_sq: f64,
    on: bool,
    enabled: bool,
    speed: impl FnOnce() -> f64,
    quiet: Option<f64>,
) -> SlideGate {
    if reach_sq > 0.0 && ear_sq as f64 > reach_sq {
        return if on {
            SlideGate::StopFar
        } else {
            SlideGate::KeepFar
        };
    }
    if on {
        return SlideGate::Advance;
    }
    if !enabled {
        return SlideGate::KeepQuiet;
    }
    match quiet {
        Some(q) if !(speed() >= q) => SlideGate::KeepQuiet,
        _ => SlideGate::Advance,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(x: f32, y: f32, z: f32) -> Vec3 {
        Vec3::new(x, y, z)
    }

    /// Walks a body `n` frames at `speed` m/s on the ground and counts the steps.
    fn walk(st: &mut StepState, tun: &StepTun, speed: f32, n: usize) -> Vec<f64> {
        let mut out = Vec::new();
        let mut pos = st.last;
        for _ in 0..n {
            pos = pos + v(speed / 60.0, 0.0, 0.0);
            let s = step_advance(st, tun, pos, 1.0 / 60.0, || false, || false, || true);
            if s.landing {
                out.push(1.0);
            }
            if let Some(l) = s.stride {
                out.push(l);
            }
        }
        out
    }

    #[test]
    fn walking_steps_by_distance() {
        let tun = StepTun::default();
        let mut st = StepState::default();
        let steps = walk(&mut st, &tun, 4.0, 180);
        assert!((5..=9).contains(&steps.len()), "{}", steps.len());
        let mut st = StepState::default();
        assert!(walk(&mut st, &tun, 1.0, 180).is_empty(), "slow is silent");
        let mut st = StepState::default();
        let run = walk(&mut st, &tun, 8.0, 60);
        assert!(!run.is_empty() && run.iter().all(|&l| l == 1.0));
    }

    #[test]
    fn landing_after_air_time() {
        let tun = StepTun::default();
        let mut st = StepState::default();
        let mut pos = v(0.0, 0.0, 0.0);
        for _ in 0..30 {
            pos = pos + v(4.0 / 60.0, 0.0, 0.0);
            let s = step_advance(&mut st, &tun, pos, 1.0 / 60.0, || false, || false, || false);
            assert_eq!(s, Steps::default());
        }
        assert!(!st.grounded && st.air >= 0.49);
        pos = pos + v(4.0 / 60.0, 0.0, 0.0);
        let s = step_advance(&mut st, &tun, pos, 1.0 / 60.0, || false, || false, || true);
        assert!(s.landing && st.grounded && st.air == 0.0);
    }

    #[test]
    fn questions_are_asked_lazily() {
        let tun = StepTun::default();
        let mut st = StepState::default();
        // A teleport asks nothing.
        let s = step_advance(
            &mut st,
            &tun,
            v(50.0, 0.0, 0.0),
            1.0 / 60.0,
            || panic!("earshot"),
            || panic!("surf"),
            || panic!("ground"),
        );
        assert_eq!(s, Steps::default());
        // Out of earshot asks neither surf nor ground, and resets the air time.
        st.air = 1.0;
        st.grounded = false;
        let _ = step_advance(
            &mut st,
            &tun,
            v(50.1, 0.0, 0.0),
            1.0 / 60.0,
            || true,
            || panic!("surf"),
            || panic!("ground"),
        );
        assert!(st.grounded && st.air == 0.0 && st.travelled == 0.0);
    }

    #[test]
    fn moved_is_f32() {
        // The script's Vector2 length rounds in f32 before widening.
        let a = v(0.1, 0.0, 0.2);
        let b = v(0.4, 7.0, 0.9);
        let dx = 0.4f32 - 0.1f32;
        let dz = 0.9f32 - 0.2f32;
        assert_eq!(moved(a, b), (dx * dx + dz * dz).sqrt() as f64);
    }

    #[test]
    fn ray_is_the_scripts() {
        let (from, to) = ray_down(v(1.0, 2.0, 3.0), 1.2);
        assert_eq!(from.y, 2.0f32 + 0.3f32);
        assert_eq!(to.y, 2.0f32 - 1.2f64 as f32);
        assert_eq!((from.x, to.z), (1.0, 3.0));
    }

    #[test]
    fn ramps_and_cache() {
        assert!(is_ramp(0.5, 0.02, 0.707));
        assert!(!is_ramp(1.0, 0.02, 0.707) && !is_ramp(0.0, 0.02, 0.707));
        assert!(
            !is_ramp(0.707, 0.02, 0.707),
            "0.707f32 widened is above 0.707"
        );
        let mut c = SeenCache::default();
        c.put(7, 3, true);
        assert_eq!(c.get(7, 3), Some(true));
        assert_eq!(c.get(7, 4), None);
        for i in 0..300 {
            c.put(i + 100, 3, false);
        }
        assert!(c.len() <= 257);
    }

    #[test]
    fn slide_gates() {
        let fast = || 10.0;
        assert_eq!(
            slide_gate(500.0, 400.0, false, true, fast, None),
            SlideGate::KeepFar
        );
        assert_eq!(
            slide_gate(500.0, 400.0, true, true, fast, None),
            SlideGate::StopFar
        );
        assert_eq!(
            slide_gate(500.0, 0.0, false, true, fast, None),
            SlideGate::Advance
        );
        assert_eq!(
            slide_gate(1.0, 400.0, false, false, fast, None),
            SlideGate::KeepQuiet
        );
        assert_eq!(
            slide_gate(1.0, 400.0, true, false, fast, None),
            SlideGate::Advance
        );
        assert_eq!(
            slide_gate(1.0, 400.0, false, true, || 3.0, Some(4.0)),
            SlideGate::KeepQuiet
        );
        assert_eq!(
            slide_gate(1.0, 400.0, false, true, || 4.0, Some(4.0)),
            SlideGate::Advance
        );
        assert_eq!(slide_speed(v(0.0, 0.0, 0.0), v(2.0, 0.0, 0.0), 0.01), 0.0);
        assert_eq!(slide_speed(v(0.0, 0.0, 0.0), v(0.5, 0.0, 0.0), 0.1), 5.0);
    }
}
