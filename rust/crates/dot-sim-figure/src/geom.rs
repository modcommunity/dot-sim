//! `Quaternion`, `Basis` and `Transform3D` as the engine spells them (`real_t` = `f32`), operation for
//! operation, so a pose computed here matches the script's to the bit or within a rounding.

use dot_sim_math::Vec3;

/// The engine's `CMP_EPSILON`.
const CMP_EPSILON: f32 = 0.00001;

/// `Vector3.lerp`: per component `from + (to - from) * weight`, the weight rounded to `f32`.
#[inline]
pub fn lerp3(a: Vec3, b: Vec3, w: f64) -> Vec3 {
    let w = w as f32;
    Vec3::new(
        a.x + (b.x - a.x) * w,
        a.y + (b.y - a.y) * w,
        a.z + (b.z - a.z) * w,
    )
}

/// The script's `lerpf`, in `f64`.
#[inline]
pub fn lerpf(a: f64, b: f64, w: f64) -> f64 {
    a + (b - a) * w
}

/// The script's `smoothstep`.
#[inline]
pub fn smoothstep(from: f64, to: f64, s: f64) -> f64 {
    if (from - to).abs() < 0.00001 {
        return if s <= from { 0.0 } else { 1.0 };
    }
    let s = dot_sim_math::clampf((s - from) / (to - from), 0.0, 1.0);
    s * s * (3.0 - 2.0 * s)
}

/// The engine's `Quaternion`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl Default for Quat {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Quat {
    pub const IDENTITY: Quat = Quat::new(0.0, 0.0, 0.0, 1.0);

    pub const fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self { x, y, z, w }
    }

    /// `Quaternion(axis, angle)`: a zero axis gives the zero quaternion.
    pub fn from_axis_angle(axis: Vec3, angle: f32) -> Self {
        let d = axis.length();
        if d == 0.0 {
            return Self::new(0.0, 0.0, 0.0, 0.0);
        }
        let sin_angle = (angle * 0.5).sin();
        let cos_angle = (angle * 0.5).cos();
        let s = sin_angle / d;
        Self::new(axis.x * s, axis.y * s, axis.z * s, cos_angle)
    }

    /// `Quaternion(from, to)`: the shortest arc.
    pub fn from_arc(v0: Vec3, v1: Vec3) -> Self {
        const ALMOST_ONE: f32 = 1.0 - CMP_EPSILON;
        let n0 = v0.normalized();
        let n1 = v1.normalized();
        let d = n0.dot(n1);
        if d.abs() > ALMOST_ONE {
            if d >= 0.0 {
                return Self::IDENTITY;
            }
            let a = any_perpendicular(n0);
            return Self::new(a.x, a.y, a.z, 0.0);
        }
        let c = n0.cross(n1);
        let s = ((1.0 + d) * 2.0).sqrt();
        let rs = 1.0 / s;
        Self::new(c.x * rs, c.y * rs, c.z * rs, s * 0.5)
    }

    /// `Quaternion.from_euler` (YXZ).
    pub fn from_euler(e: Vec3) -> Self {
        let half_a1 = e.y * 0.5;
        let half_a2 = e.x * 0.5;
        let half_a3 = e.z * 0.5;
        let (sin_a1, cos_a1) = (half_a1.sin(), half_a1.cos());
        let (sin_a2, cos_a2) = (half_a2.sin(), half_a2.cos());
        let (sin_a3, cos_a3) = (half_a3.sin(), half_a3.cos());
        Self::new(
            sin_a1 * cos_a2 * sin_a3 + cos_a1 * sin_a2 * cos_a3,
            sin_a1 * cos_a2 * cos_a3 - cos_a1 * sin_a2 * sin_a3,
            -sin_a1 * sin_a2 * cos_a3 + cos_a1 * cos_a2 * sin_a3,
            sin_a1 * sin_a2 * sin_a3 + cos_a1 * cos_a2 * cos_a3,
        )
    }

    #[inline]
    pub fn dot(self, q: Self) -> f32 {
        self.x * q.x + self.y * q.y + self.z * q.z + self.w * q.w
    }

    #[inline]
    pub fn length_squared(self) -> f32 {
        self.dot(self)
    }

    /// `normalized()`: times one over the length.
    #[inline]
    pub fn normalized(self) -> Self {
        let r = 1.0 / self.length_squared().sqrt();
        Self::new(self.x * r, self.y * r, self.z * r, self.w * r)
    }

    #[inline]
    pub fn inverse(self) -> Self {
        Self::new(-self.x, -self.y, -self.z, self.w)
    }

    /// `slerp`, with the engine's mixed `f64` scale terms.
    pub fn slerp(self, to: Self, weight: f32) -> Self {
        let mut cosom = self.dot(to);
        let to1 = if cosom < 0.0 {
            cosom = -cosom;
            Self::new(-to.x, -to.y, -to.z, -to.w)
        } else {
            to
        };
        let (scale0, scale1);
        if (1.0 - cosom) > CMP_EPSILON {
            let omega = cosom.acos();
            let sinom = omega.sin();
            scale0 = (((1.0 - weight as f64) * omega as f64).sin() / sinom as f64) as f32;
            scale1 = (weight * omega).sin() / sinom;
        } else {
            scale0 = 1.0 - weight;
            scale1 = weight;
        }
        Self::new(
            scale0 * self.x + scale1 * to1.x,
            scale0 * self.y + scale1 * to1.y,
            scale0 * self.z + scale1 * to1.z,
            scale0 * self.w + scale1 * to1.w,
        )
    }

    /// `Quaternion * Vector3`.
    #[inline]
    pub fn xform(self, v: Vec3) -> Vec3 {
        let u = Vec3::new(self.x, self.y, self.z);
        let uv = u.cross(v);
        let t = Vec3::new(uv.x * self.w, uv.y * self.w, uv.z * self.w) + u.cross(uv);
        v + Vec3::new(t.x * 2.0, t.y * 2.0, t.z * 2.0)
    }
}

impl std::ops::Mul for Quat {
    type Output = Self;

    #[inline]
    fn mul(self, q: Self) -> Self {
        let (x, y, z, w) = (self.x, self.y, self.z, self.w);
        Self::new(
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y + y * q.w + z * q.x - x * q.z,
            w * q.z + z * q.w + x * q.y - y * q.x,
            w * q.w - x * q.x - y * q.y - z * q.z,
        )
    }
}

/// `Vector3.get_any_perpendicular`.
fn any_perpendicular(v: Vec3) -> Vec3 {
    let other = if v.x.abs() <= v.y.abs() && v.x.abs() <= v.z.abs() {
        Vec3::new(1.0, 0.0, 0.0)
    } else if v.y.abs() <= v.z.abs() {
        Vec3::new(0.0, 1.0, 0.0)
    } else {
        Vec3::new(0.0, 0.0, 1.0)
    };
    v.cross(other).normalized()
}

/// The engine's `Basis`, by rows.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Basis {
    pub rows: [Vec3; 3],
}

impl Default for Basis {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Basis {
    pub const IDENTITY: Basis = Basis {
        rows: [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ],
    };

    #[allow(clippy::too_many_arguments)]
    #[inline]
    fn set(
        xx: f32,
        xy: f32,
        xz: f32,
        yx: f32,
        yy: f32,
        yz: f32,
        zx: f32,
        zy: f32,
        zz: f32,
    ) -> Self {
        Self {
            rows: [
                Vec3::new(xx, xy, xz),
                Vec3::new(yx, yy, yz),
                Vec3::new(zx, zy, zz),
            ],
        }
    }

    #[inline]
    fn at(&self, r: usize, c: usize) -> f32 {
        let v = self.rows[r];
        match c {
            0 => v.x,
            1 => v.y,
            _ => v.z,
        }
    }

    /// `Basis(Quaternion)`.
    pub fn from_quat(q: Quat) -> Self {
        let d = q.length_squared();
        let s = 2.0 / d;
        let (xs, ys, zs) = (q.x * s, q.y * s, q.z * s);
        let (wx, wy, wz) = (q.w * xs, q.w * ys, q.w * zs);
        let (xx, xy, xz) = (q.x * xs, q.x * ys, q.x * zs);
        let (yy, yz, zz) = (q.y * ys, q.y * zs, q.z * zs);
        Self::set(
            1.0 - (yy + zz),
            xy - wz,
            xz + wy,
            xy + wz,
            1.0 - (xx + zz),
            yz - wx,
            xz - wy,
            yz + wx,
            1.0 - (xx + yy),
        )
    }

    #[inline]
    pub fn xform(&self, v: Vec3) -> Vec3 {
        Vec3::new(
            self.rows[0].dot(v),
            self.rows[1].dot(v),
            self.rows[2].dot(v),
        )
    }

    pub fn determinant(&self) -> f32 {
        let m = |r, c| self.at(r, c);
        m(0, 0) * (m(1, 1) * m(2, 2) - m(2, 1) * m(1, 2))
            - m(1, 0) * (m(0, 1) * m(2, 2) - m(2, 1) * m(0, 2))
            + m(2, 0) * (m(0, 1) * m(1, 2) - m(1, 1) * m(0, 2))
    }

    /// `inverse()`, by cofactors.
    pub fn inverse(&self) -> Self {
        let m = |r, c| self.at(r, c);
        let cofac = |r1, c1, r2, c2| m(r1, c1) * m(r2, c2) - m(r1, c2) * m(r2, c1);
        let co = [cofac(1, 1, 2, 2), cofac(1, 2, 2, 0), cofac(1, 0, 2, 1)];
        let det = m(0, 0) * co[0] + m(0, 1) * co[1] + m(0, 2) * co[2];
        let s = 1.0 / det;
        Self::set(
            co[0] * s,
            cofac(0, 2, 2, 1) * s,
            cofac(0, 1, 1, 2) * s,
            co[1] * s,
            cofac(0, 0, 2, 2) * s,
            cofac(0, 2, 1, 0) * s,
            co[2] * s,
            cofac(0, 1, 2, 0) * s,
            cofac(0, 0, 1, 1) * s,
        )
    }

    fn column(&self, c: usize) -> Vec3 {
        Vec3::new(self.at(0, c), self.at(1, c), self.at(2, c))
    }

    /// `orthonormalized()`: Gram-Schmidt over the columns.
    fn orthonormalized(&self) -> Self {
        let x = self.column(0).normalized();
        let y = self.column(1);
        let y = (y - x.scale_f32(x.dot(y))).normalized();
        let z = self.column(2);
        let z = (z - x.scale_f32(x.dot(z)) - y.scale_f32(y.dot(z))).normalized();
        Self::set(x.x, y.x, z.x, x.y, y.y, z.y, x.z, y.z, z.z)
    }

    /// `get_quaternion()` of a rotation matrix.
    fn quaternion(&self) -> Quat {
        let m = |r, c| self.at(r, c);
        let trace = m(0, 0) + m(1, 1) + m(2, 2);
        let mut t = [0.0f32; 4];
        if trace > 0.0 {
            let mut s = (trace + 1.0).sqrt();
            t[3] = s * 0.5;
            s = 0.5 / s;
            t[0] = (m(2, 1) - m(1, 2)) * s;
            t[1] = (m(0, 2) - m(2, 0)) * s;
            t[2] = (m(1, 0) - m(0, 1)) * s;
        } else {
            let i = if m(0, 0) < m(1, 1) {
                if m(1, 1) < m(2, 2) { 2 } else { 1 }
            } else if m(0, 0) < m(2, 2) {
                2
            } else {
                0
            };
            let j = (i + 1) % 3;
            let k = (i + 2) % 3;
            let mut s = (m(i, i) - m(j, j) - m(k, k) + 1.0).sqrt();
            t[i] = s * 0.5;
            s = 0.5 / s;
            t[3] = (m(k, j) - m(j, k)) * s;
            t[j] = (m(j, i) + m(i, j)) * s;
            t[k] = (m(k, i) + m(i, k)) * s;
        }
        Quat::new(t[0], t[1], t[2], t[3])
    }

    /// `get_rotation_quaternion()`: orthonormalized, a reflection flipped out.
    pub fn rotation_quaternion(&self) -> Quat {
        let mut m = self.orthonormalized();
        if m.determinant() < 0.0 {
            for r in &mut m.rows {
                *r = -*r;
            }
        }
        m.quaternion()
    }
}

impl std::ops::Mul for Basis {
    type Output = Self;

    #[inline]
    fn mul(self, b: Self) -> Self {
        let tdot = |c: usize, v: Vec3| b.at(0, c) * v.x + b.at(1, c) * v.y + b.at(2, c) * v.z;
        let r = self.rows;
        Self::set(
            tdot(0, r[0]),
            tdot(1, r[0]),
            tdot(2, r[0]),
            tdot(0, r[1]),
            tdot(1, r[1]),
            tdot(2, r[1]),
            tdot(0, r[2]),
            tdot(1, r[2]),
            tdot(2, r[2]),
        )
    }
}

/// `Vec3 * real_t` with an `f32` factor (an engine-internal product, no `f64` rounding).
trait ScaleF32 {
    fn scale_f32(self, s: f32) -> Self;
}

impl ScaleF32 for Vec3 {
    #[inline]
    fn scale_f32(self, s: f32) -> Self {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
}

/// The engine's `Transform3D`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Xform {
    pub basis: Basis,
    pub origin: Vec3,
}

impl Xform {
    pub const IDENTITY: Xform = Xform {
        basis: Basis::IDENTITY,
        origin: Vec3::new(0.0, 0.0, 0.0),
    };

    pub fn new(basis: Basis, origin: Vec3) -> Self {
        Self { basis, origin }
    }

    /// `Transform3D * Vector3`.
    #[inline]
    pub fn xform(&self, v: Vec3) -> Vec3 {
        self.basis.xform(v) + self.origin
    }

    /// `affine_inverse()`.
    pub fn affine_inverse(&self) -> Self {
        let b = self.basis.inverse();
        Self {
            basis: b,
            origin: b.xform(-self.origin),
        }
    }
}

impl std::ops::Mul for Xform {
    type Output = Self;

    #[inline]
    fn mul(self, t: Self) -> Self {
        Self {
            origin: self.xform(t.origin),
            basis: self.basis * t.basis,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: Quat, b: Quat) -> bool {
        (a.x - b.x).abs() < 1e-6
            && (a.y - b.y).abs() < 1e-6
            && (a.z - b.z).abs() < 1e-6
            && (a.w - b.w).abs() < 1e-6
    }

    #[test]
    fn basis_round_trips_a_quaternion() {
        let q = Quat::from_euler(Vec3::new(0.3, -1.1, 0.7));
        assert!(close(Basis::from_quat(q).rotation_quaternion(), q));
    }

    #[test]
    fn arc_turns_one_vector_onto_another() {
        let a = Vec3::new(1.0, 0.2, -0.3).normalized();
        let b = Vec3::new(-0.2, 0.9, 0.4).normalized();
        let r = Quat::from_arc(a, b).xform(a);
        assert!((r - b).length() < 1e-6);
    }

    #[test]
    fn slerp_ends_are_its_ends() {
        let a = Quat::from_euler(Vec3::new(0.1, 0.2, 0.3));
        let b = Quat::from_euler(Vec3::new(-0.4, 0.9, 0.1));
        assert!(close(a.slerp(b, 0.0), a));
        assert!(close(a.slerp(b, 1.0), b));
    }

    #[test]
    fn affine_inverse_undoes_the_transform() {
        let t = Xform::new(
            Basis::from_quat(Quat::from_euler(Vec3::new(0.5, 1.0, -0.2))),
            Vec3::new(1.0, 2.0, 3.0),
        );
        let p = Vec3::new(0.3, -0.7, 2.0);
        assert!((t.affine_inverse().xform(t.xform(p)) - p).length() < 1e-5);
    }
}
