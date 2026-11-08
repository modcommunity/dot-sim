//! The engine's scalar helpers, its `Vector2`, and dot-core's `DotRandomStream`, rounded and
//! ordered as the engine and the script do them.

pub use dot_sim_math::{Vec3, clampf, maxf, minf, rad_to_deg};

/// `CMP_EPSILON`, the engine's tolerance for approximate comparisons.
const CMP_EPSILON: f64 = 0.00001;

/// Two `f32` components, as the engine's `Vector2`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    pub const ZERO: Self = Self::new(0.0, 0.0);
    pub const NAN: Self = Self::new(f32::NAN, f32::NAN);

    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    /// `Vector2(float, float)` from script floats: each rounded to `f32`.
    #[inline]
    pub fn of(x: f64, y: f64) -> Self {
        Self::new(x as f32, y as f32)
    }

    #[inline]
    pub fn dot(self, o: Self) -> f32 {
        self.x * o.x + self.y * o.y
    }

    #[inline]
    pub fn length(self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    /// `Vector2.normalized()`: zero stays zero.
    #[inline]
    pub fn normalized(self) -> Self {
        let l = self.x * self.x + self.y * self.y;
        if l == 0.0 {
            return self;
        }
        let l = l.sqrt();
        Self::new(self.x / l, self.y / l)
    }

    /// `Vector2 * float`: the scalar rounded to `f32` first.
    #[inline]
    pub fn scale(self, s: f64) -> Self {
        let s = s as f32;
        Self::new(self.x * s, self.y * s)
    }

    /// `Vector2 / float`, a division per component.
    #[inline]
    pub fn div_scalar(self, s: f64) -> Self {
        let s = s as f32;
        Self::new(self.x / s, self.y / s)
    }

    /// `Vector2.limit_length(len)`.
    #[inline]
    pub fn limit_length(self, len: f64) -> Self {
        let len = len as f32;
        let l = self.length();
        if l > 0.0 && len < l {
            let v = Self::new(self.x / l, self.y / l);
            return Self::new(v.x * len, v.y * len);
        }
        self
    }

    /// Exactly zero in both components (`== Vector2.ZERO`).
    #[inline]
    pub fn is_zero(self) -> bool {
        self.x == 0.0 && self.y == 0.0
    }
}

impl std::ops::Add for Vec2 {
    type Output = Self;

    #[inline]
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y)
    }
}

impl std::ops::Sub for Vec2 {
    type Output = Self;

    #[inline]
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y)
    }
}

/// `Vector3 * float`: the scalar rounded to `f32` first.
#[inline]
pub fn scale3(v: Vec3, s: f64) -> Vec3 {
    let s = s as f32;
    Vec3::new(v.x * s, v.y * s, v.z * s)
}

/// `Vector3.lerp(to, weight)`: per component in `f32`, the weight rounded first.
#[inline]
pub fn lerp3(a: Vec3, b: Vec3, w: f64) -> Vec3 {
    let w = w as f32;
    Vec3::new(
        a.x + (b.x - a.x) * w,
        a.y + (b.y - a.y) * w,
        a.z + (b.z - a.z) * w,
    )
}

/// `Vector3.is_finite()`.
#[inline]
pub fn finite3(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

/// `is_equal_approx(a, b)`.
#[inline]
pub fn is_equal_approx(a: f64, b: f64) -> bool {
    if a == b {
        return true;
    }
    let mut tolerance = CMP_EPSILON * a.abs();
    if tolerance < CMP_EPSILON {
        tolerance = CMP_EPSILON;
    }
    (a - b).abs() < tolerance
}

/// `wrapf(value, min, max)`.
#[inline]
pub fn wrapf(value: f64, min: f64, max: f64) -> f64 {
    let range = max - min;
    if range.abs() < CMP_EPSILON {
        return min;
    }
    let result = value - range * ((value - min) / range).floor();
    if is_equal_approx(result, max) {
        return min;
    }
    result
}

/// `lerpf(from, to, weight)`.
#[inline]
pub fn lerpf(from: f64, to: f64, w: f64) -> f64 {
    from + (to - from) * w
}

/// `move_toward(from, to, delta)`.
#[inline]
pub fn move_toward(from: f64, to: f64, delta: f64) -> f64 {
    let d = to - from;
    if d.abs() <= delta {
        return to;
    }
    let s = if d > 0.0 {
        1.0
    } else if d < 0.0 {
        -1.0
    } else {
        0.0
    };
    from + s * delta
}

/// `deg_to_rad`.
#[inline]
pub fn deg_to_rad(d: f64) -> f64 {
    d * (std::f64::consts::PI / 180.0)
}

const GAMMA: i64 = -7046029254386353131;
const MIX_A: i64 = -4658895280553007687;
const MIX_B: i64 = -7723592293110705685;

#[inline]
fn ushr(z: i64, n: u32) -> i64 {
    ((z as u64) >> n) as i64
}

/// dot-core's `DotRandomStream._mix` (the splitmix64 finaliser over wrapping `i64`).
#[inline]
pub fn mix(x: i64) -> i64 {
    let mut z = x.wrapping_add(GAMMA);
    z = (z ^ ushr(z, 30)).wrapping_mul(MIX_A);
    z = (z ^ ushr(z, 27)).wrapping_mul(MIX_B);
    z ^ ushr(z, 31)
}

/// One `DotRandomStream`: its key (made by the script) and its counter.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stream {
    pub key: i64,
    pub counter: i64,
}

impl Stream {
    pub const fn new(key: i64, counter: i64) -> Self {
        Self { key, counter }
    }

    /// `unit_at(index)`: in [0, 1), 53 bits.
    #[inline]
    pub fn unit_at(&self, index: i64) -> f64 {
        let v = mix(self.key ^ mix(index.wrapping_add(GAMMA)));
        ((v as u64) >> 11) as f64 * (1.0 / 9007199254740992.0)
    }

    /// `next_unit()`.
    #[inline]
    pub fn next_unit(&mut self) -> f64 {
        let v = self.unit_at(self.counter);
        self.counter = self.counter.wrapping_add(1);
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrapf_matches_engine_edges() {
        assert_eq!(wrapf(190.0, -180.0, 180.0), -170.0);
        assert_eq!(wrapf(-190.0, -180.0, 180.0), 170.0);
        assert_eq!(wrapf(180.0, -180.0, 180.0), -180.0);
        assert_eq!(wrapf(179.999999, -180.0, 180.0), -180.0);
        assert_eq!(wrapf(12.5, -180.0, 180.0), 12.5);
    }

    #[test]
    fn move_toward_steps_and_lands() {
        assert_eq!(move_toward(0.0, 1.0, 0.25), 0.25);
        assert_eq!(move_toward(0.9, 1.0, 0.25), 1.0);
        assert_eq!(move_toward(0.0, -1.0, 0.25), -0.25);
    }

    #[test]
    fn limit_length_keeps_short_vectors() {
        let v = Vec2::new(0.3, 0.4);
        assert_eq!(v.limit_length(1.0), v);
        let l = Vec2::new(3.0, 4.0).limit_length(1.0);
        assert!((l.length() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn stream_is_deterministic_and_in_unit_range() {
        let mut a = Stream::new(mix(7), 0);
        let mut b = a;
        for _ in 0..100 {
            let u = a.next_unit();
            assert!((0.0..1.0).contains(&u));
            assert_eq!(u, b.next_unit());
        }
        assert_eq!(a.counter, 100);
    }

    #[test]
    fn mix_is_splitmix64() {
        // splitmix64's first output for seed 0 is 0xE220A8397B1DCDAF.
        assert_eq!(mix(0) as u64, 0xE220_A839_7B1D_CDAF);
    }
}
