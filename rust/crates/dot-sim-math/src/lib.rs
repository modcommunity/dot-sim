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
    fn length_is_computed_in_f32() {
        let v = Vec3::new(3.0, 4.0, 12.0);
        assert_eq!(v.length(), 13.0);
        assert_eq!(Vec3::new(0.1, 0.2, 0.3).length(), (0.1f32 * 0.1 + 0.2 * 0.2 + 0.3 * 0.3).sqrt());
    }
}
