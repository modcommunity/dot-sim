//! Vector arithmetic rounded the way the engine rounds it.
//!
//! Engine vectors are `f32` while script floats are `f64`; every operation here is spelled in
//! the engine's order and precision so a native kernel and the script it replaces agree to the bit.

/// Three `f32` components, as the engine's `Vector3`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    /// A vector from its components.
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Dot product, summed x, y, z as the engine does.
    #[inline]
    pub fn dot(self, o: Self) -> f32 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    /// Cross product.
    #[inline]
    pub fn cross(self, o: Self) -> Self {
        Self::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }

    /// Length in `f32`, as `Vector3.length()`.
    #[inline]
    pub fn length(self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Squared length in `f32`, as `Vector3.length_squared()`.
    #[inline]
    pub fn length_squared(self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Unit vector, as `Vector3.normalized()`: zero for a zero vector, else each component divided by the length.
    #[inline]
    pub fn normalized(self) -> Self {
        let lsq = self.length_squared();
        if lsq == 0.0 {
            return Self::default();
        }
        let l = lsq.sqrt();
        Self::new(self.x / l, self.y / l, self.z / l)
    }

    /// `Vector3.is_normalized()`: the squared length equal to one within `UNIT_EPSILON` (0.001), in `f32`.
    #[inline]
    pub fn is_normalized(self) -> bool {
        let lsq = self.length_squared();
        lsq == 1.0 || (lsq - 1.0).abs() < 0.001
    }

    /// `Vector3 * float`: the script's `f64` scalar rounded to `f32` first, then each component multiplied.
    #[inline]
    pub fn scale(self, s: f64) -> Self {
        let s = s as f32;
        Self::new(self.x * s, self.y * s, self.z * s)
    }

    /// `Vector3 / float`, rounded the same way as [`Vec3::scale`]; a division per component, not a reciprocal.
    #[inline]
    pub fn div_scalar(self, s: f64) -> Self {
        let s = s as f32;
        Self::new(self.x / s, self.y / s, self.z / s)
    }

    /// `Vector3.distance_to()`: the length of `to - self`.
    #[inline]
    pub fn distance_to(self, to: Self) -> f32 {
        (to - self).length()
    }

    /// Unsigned angle in radians, as `Vector3.angle_to()`: atan2 of the cross length and the dot.
    #[inline]
    pub fn angle_to(self, o: Self) -> f32 {
        self.cross(o).length().atan2(self.dot(o))
    }
}

impl std::ops::Sub for Vec3 {
    type Output = Self;

    #[inline]
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl std::ops::Add for Vec3 {
    type Output = Self;

    #[inline]
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl std::ops::Neg for Vec3 {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

/// The script's `maxf`: `a < b ? b : a`, so a NaN in `a` survives as it does there.
#[inline]
pub fn maxf(a: f64, b: f64) -> f64 {
    if a < b { b } else { a }
}

/// The script's `minf`: `a < b ? a : b`.
#[inline]
pub fn minf(a: f64, b: f64) -> f64 {
    if a < b { a } else { b }
}

/// The script's `clampf`: `v < lo ? lo : (v > hi ? hi : v)`.
#[inline]
pub fn clampf(v: f64, lo: f64, hi: f64) -> f64 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

/// Radians to degrees in `f64`, as the script's `rad_to_deg`.
#[inline]
pub fn rad_to_deg(r: f64) -> f64 {
    r * (180.0 / std::f64::consts::PI)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn angle_of_perpendicular_axes_is_ninety_degrees() {
        let a = Vec3::new(1.0, 0.0, 0.0).angle_to(Vec3::new(0.0, 0.0, -1.0));
        assert!((rad_to_deg(a as f64) - 90.0).abs() < 1e-5);
    }

    #[test]
    fn angle_to_self_is_zero_and_opposite_is_pi() {
        let v = Vec3::new(0.3, -2.0, 5.0);
        assert_eq!(v.angle_to(v), 0.0);
        let back = Vec3::new(-0.3, 2.0, -5.0);
        assert!((v.angle_to(back) - std::f32::consts::PI).abs() < 1e-6);
    }

    #[test]
    fn normalized_matches_engine_rounding() {
        let v = Vec3::new(0.3, -2.0, 5.0);
        let l = (0.3f32 * 0.3 + 2.0 * 2.0 + 5.0 * 5.0).sqrt();
        assert_eq!(v.normalized(), Vec3::new(0.3 / l, -2.0 / l, 5.0 / l));
        assert_eq!(Vec3::default().normalized(), Vec3::default());
        assert!(v.normalized().is_normalized());
        assert!(!Vec3::new(1.0, 0.1, 0.0).is_normalized());
    }

    #[test]
    fn scale_rounds_the_scalar_to_f32_first() {
        let v = Vec3::new(1.0, 3.0, -7.0);
        let s = 0.1f64;
        assert_eq!(v.scale(s), Vec3::new(0.1f32, 3.0 * 0.1f32, -7.0 * 0.1f32));
    }

    #[test]
    fn script_min_max_clamp_keep_their_nan_order() {
        assert!(maxf(f64::NAN, 0.0).is_nan());
        assert_eq!(maxf(0.0, f64::NAN), 0.0);
        assert_eq!(clampf(2.0, 0.0, 1.0), 1.0);
        assert_eq!(clampf(-2.0, 0.0, 1.0), 0.0);
        assert_eq!(minf(0.25, 0.5), 0.25);
    }

    #[test]
    fn length_is_computed_in_f32() {
        let v = Vec3::new(3.0, 4.0, 12.0);
        assert_eq!(v.length(), 13.0);
        assert_eq!(
            Vec3::new(0.1, 0.2, 0.3).length(),
            (0.1f32 * 0.1 + 0.2 * 0.2 + 0.3 * 0.3).sqrt()
        );
    }
}
