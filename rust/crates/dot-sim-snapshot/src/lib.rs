//! Snapshot encode and decode, bit-identical to dot-net's script codec.
//!
//! The wire format is frozen by `DotNetWriter`, `DotNetReader` and `DotNetVar.write` / `read` in
//! dot-net: this crate transcribes them, operation for operation, in the same float widths
//! (GDScript computes in f64, `Vector3` and `Quaternion` hold f32), with no fused multiply-add and
//! the platform's own `acos`, `atan2`, `sin`, `cos` and `sqrt` (the engine calls the same ones).
//! What it cannot reproduce exactly (a custom codec, a string's conversion, a float field wider
//! than 32 bits) it declines, and the script does that value.
//!
//! A behaviour's state is `count:6`, then per entry `index:6` and the value
//! (`DotNetBehaviour._encode` / `read_state`).

mod bits;
#[cfg(test)]
mod tests;

pub use bits::{Reader, Writer};

/// `DotNetVar.Type`, in its enum order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Bool,
    Uint,
    Int,
    Varint,
    Float,
    FloatRange,
    Vec3Position,
    Vec3Velocity,
    Vec3Range,
    Direction,
    Quaternion,
    Angle,
    String,
    Bytes,
    Custom,
}

impl Kind {
    pub fn from_index(i: i64) -> Option<Kind> {
        use Kind::*;
        Some(match i {
            0 => Bool,
            1 => Uint,
            2 => Int,
            3 => Varint,
            4 => Float,
            5 => FloatRange,
            6 => Vec3Position,
            7 => Vec3Velocity,
            8 => Vec3Range,
            9 => Direction,
            10 => Quaternion,
            11 => Angle,
            12 => String,
            13 => Bytes,
            14 => Custom,
            _ => return None,
        })
    }
}

/// One declaration, resolved against its config: the bits and range `DotNetVar.write` uses.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Decl {
    pub kind: Kind,
    pub bits: i64,
    pub min: f64,
    pub max: f64,
    /// False for what the script must do itself (see the crate docs).
    pub native: bool,
}

impl Decl {
    /// From a row of `DotNetBehaviour._native_spec()`: `[type, audience, max_rate, epsilon,
    /// on_spawn_only, bit_width, min, max, extent, position_bits, max_speed, velocity_bits,
    /// rotation_bits, slot]`.
    pub fn from_spec(row: &[f64]) -> Decl {
        let get = |i: usize| row.get(i).copied().unwrap_or(0.0);
        let kind = Kind::from_index(get(0) as i64).unwrap_or(Kind::Custom);
        let (bits, min, max) = match kind {
            Kind::Vec3Position => (get(9) as i64, -get(8), get(8)),
            Kind::Vec3Velocity => (get(11) as i64, -get(10), get(10)),
            Kind::Quaternion => (get(12) as i64, 0.0, 0.0),
            _ => (get(5) as i64, get(6), get(7)),
        };
        let native = match kind {
            Kind::String | Kind::Custom => false,
            Kind::Bool | Kind::Varint | Kind::Float | Kind::Bytes => true,
            // A width out of 1..=64 makes the script push an error; a float field past 32 bits
            // rounds through an int conversion C++ and Rust disagree on at the top.
            Kind::Uint | Kind::Int => (1..=64).contains(&bits),
            _ => (1..=32).contains(&bits),
        };
        Decl {
            kind,
            bits,
            min,
            max,
            native,
        }
    }
}

/// A value as `DotNetVar` reads it.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Bool(bool),
    Int(i64),
    Float(f64),
    Vec3([f32; 3]),
    Quat([f32; 4]),
    Bytes(Vec<u8>),
}

/// The script's 0.7071067811865476, the same double.
const LIMIT: f64 = std::f64::consts::FRAC_1_SQRT_2;
const PI: f64 = std::f64::consts::PI;
/// `DotNetReader.read_bytes`' default bound.
pub const MAX_READ_BYTES: i64 = 1 << 20;

/// GDScript's `int(float)` as the engine compiles it on x86-64 (`cvttsd2si`): a NaN or an
/// out-of-range value is `i64::MIN`. Callers mask the result to the field width.
#[cfg(not(target_family = "wasm"))]
#[inline]
pub fn to_int(v: f64) -> i64 {
    if (-9.223_372_036_854_776e18..9.223_372_036_854_776e18).contains(&v) {
        v as i64
    } else {
        i64::MIN
    }
}

/// GDScript's `int(float)` as Emscripten compiles it for the web (`i64.trunc_sat_f64_s`): a NaN
/// is 0 and an out-of-range value saturates, which is exactly Rust's `as`. Measured in Chromium
/// on the dlink export: `int(nan)` 0, `int(1e300)` `i64::MAX`, `int(-1e300)` `i64::MIN`.
#[cfg(target_family = "wasm")]
#[inline]
pub fn to_int(v: f64) -> i64 {
    v as i64
}

/// Godot's `CLAMP`: a NaN passes through.
#[inline]
fn clampd(v: f64, lo: f64, hi: f64) -> f64 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

#[inline]
fn mask(bits: i64) -> i64 {
    if bits >= 64 {
        -1
    } else {
        ((1u64 << bits) - 1) as i64
    }
}

/// Godot's `Math::fposmod`.
fn fposmod(x: f64, y: f64) -> f64 {
    let mut value = x % y;
    if (value < 0.0 && y > 0.0) || (value > 0.0 && y < 0.0) {
        value += y;
    }
    value + 0.0
}

/// `Vector3::normalized`, in f32 as the engine computes it.
pub fn v3_normalized(v: [f32; 3]) -> [f32; 3] {
    let lsq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    if lsq == 0.0 {
        [0.0; 3]
    } else {
        let l = lsq.sqrt();
        [v[0] / l, v[1] / l, v[2] / l]
    }
}

/// `Quaternion::normalized`: `*this * (1.0f / length())`.
pub fn q_normalized(q: [f32; 4]) -> [f32; 4] {
    let r = 1.0f32 / q_length_squared(q).sqrt();
    [q[0] * r, q[1] * r, q[2] * r, q[3] * r]
}

#[inline]
pub fn q_length_squared(q: [f32; 4]) -> f32 {
    q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]
}

impl Writer {
    pub fn write_float_range(&mut self, value: f64, min: f64, max: f64, bits: i64) {
        if max <= min {
            self.write_bits(0, bits);
            return;
        }
        let span = max - min;
        let normalised = clampd((value - min) / span, 0.0, 1.0);
        self.write_bits(to_int((normalised * mask(bits) as f64).round()), bits);
    }

    pub fn write_varint(&mut self, value: i64) {
        let mut v = value.max(0);
        loop {
            let group = v & 0x7F;
            v >>= 7;
            if v == 0 {
                self.write_bits(group, 8);
                return;
            }
            self.write_bits(group | 0x80, 8);
        }
    }

    pub fn write_svarint(&mut self, value: i64) {
        self.write_varint(value.wrapping_shl(1) ^ (value >> 63));
    }

    pub fn write_angle(&mut self, degrees: f64, bits: i64) {
        let wrapped = fposmod(degrees, 360.0);
        let steps = (1u64 << (bits & 63)).wrapping_sub(1) as i64;
        self.write_bits(
            to_int((wrapped / 360.0 * steps as f64).round()) & mask(bits),
            bits,
        );
    }

    pub fn write_float32(&mut self, value: f64) {
        for b in (value as f32).to_le_bytes() {
            self.write_bits(b as i64, 8);
        }
    }

    pub fn write_vec3_range(&mut self, v: [f32; 3], min: f64, max: f64, bits: i64) {
        for c in v {
            self.write_float_range(c as f64, min, max, bits);
        }
    }

    pub fn write_direction(&mut self, direction: [f32; 3], bits: i64) {
        let mut d = v3_normalized(direction);
        let finite = d.iter().all(|c| c.is_finite());
        if !finite || ((d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) as f64) < 0.5 {
            d = [0.0, 1.0, 0.0];
        }
        self.write_float_range(clampd(d[1] as f64, -1.0, 1.0).acos(), 0.0, PI, bits);
        self.write_float_range((d[2] as f64).atan2(d[0] as f64), -PI, PI, bits);
    }

    pub fn write_quaternion(&mut self, rotation: [f32; 4], bits: i64) {
        let q = q_normalized(rotation);
        let c = [q[0] as f64, q[1] as f64, q[2] as f64, q[3] as f64];
        let mut largest = 0usize;
        let mut largest_value = c[0].abs();
        for (i, v) in c.iter().enumerate().skip(1) {
            if v.abs() > largest_value {
                largest_value = v.abs();
                largest = i;
            }
        }
        let sign_flip = if c[largest] < 0.0 { -1.0 } else { 1.0 };
        self.write_bits(largest as i64, 2);
        for (i, v) in c.iter().enumerate() {
            if i != largest {
                self.write_float_range(clampd(v * sign_flip, -LIMIT, LIMIT), -LIMIT, LIMIT, bits);
            }
        }
    }

    pub fn write_bytes(&mut self, data: &[u8]) {
        self.write_varint(data.len() as i64);
        self.align();
        self.append_aligned(data);
    }

    /// One declaration's value, as `DotNetVar.write` writes it. False, part written, for a
    /// declaration or value the script must write.
    pub fn write_value(&mut self, d: &Decl, value: &Value) -> bool {
        if !d.native {
            return false;
        }
        match (d.kind, value) {
            (Kind::Bool, Value::Bool(b)) => self.write_bits(*b as i64, 1),
            (Kind::Uint, Value::Int(v)) => self.write_bits(*v, d.bits),
            (Kind::Int, Value::Int(v)) => self.write_bits(v & mask(d.bits), d.bits),
            (Kind::Varint, Value::Int(v)) => self.write_svarint(*v),
            (Kind::Float, Value::Float(v)) => self.write_float32(*v),
            (Kind::FloatRange, Value::Float(v)) => self.write_float_range(*v, d.min, d.max, d.bits),
            (Kind::Angle, Value::Float(v)) => self.write_angle(*v, d.bits),
            (Kind::Vec3Position | Kind::Vec3Velocity | Kind::Vec3Range, Value::Vec3(v)) => {
                self.write_vec3_range(*v, d.min, d.max, d.bits)
            }
            (Kind::Direction, Value::Vec3(v)) => self.write_direction(*v, d.bits),
            (Kind::Quaternion, Value::Quat(q)) => self.write_quaternion(*q, d.bits),
            (Kind::Bytes, Value::Bytes(b)) => self.write_bytes(b),
            _ => return false,
        }
        true
    }
}

impl Reader<'_> {
    pub fn read_int(&mut self, bits: i64) -> i64 {
        let raw = self.read_bits(bits);
        if bits >= 64 || bits <= 0 {
            return raw;
        }
        if raw & (1i64 << (bits - 1)) != 0 {
            raw.wrapping_sub(1i64 << bits)
        } else {
            raw
        }
    }

    pub fn read_varint(&mut self) -> i64 {
        let mut result = 0i64;
        let mut shift = 0u32;
        for _ in 0..10 {
            let group = self.read_bits(8);
            if self.exhausted {
                return 0;
            }
            result |= (group & 0x7F).wrapping_shl(shift);
            if group & 0x80 == 0 {
                return result;
            }
            shift += 7;
        }
        self.exhausted = true;
        0
    }

    pub fn read_svarint(&mut self) -> i64 {
        let raw = self.read_varint();
        (raw >> 1) ^ (raw & 1).wrapping_neg()
    }

    pub fn read_float_range(&mut self, min: f64, max: f64, bits: i64) -> f64 {
        if max <= min {
            return min;
        }
        let q = self.read_bits(bits);
        let normalised = q as f64 / mask(bits) as f64;
        min + normalised * (max - min)
    }

    pub fn read_angle(&mut self, bits: i64) -> f64 {
        let q = self.read_bits(bits);
        q as f64 / ((1u64 << (bits & 63)).wrapping_sub(1) as i64) as f64 * 360.0
    }

    pub fn read_float32(&mut self) -> f64 {
        let mut b = [0u8; 4];
        for x in b.iter_mut() {
            *x = self.read_bits(8) as u8;
        }
        if self.exhausted {
            return 0.0;
        }
        f32::from_le_bytes(b) as f64
    }

    pub fn read_vec3_range(&mut self, min: f64, max: f64, bits: i64) -> [f32; 3] {
        let x = self.read_float_range(min, max, bits);
        let y = self.read_float_range(min, max, bits);
        let z = self.read_float_range(min, max, bits);
        [x as f32, y as f32, z as f32]
    }

    pub fn read_direction(&mut self, bits: i64) -> [f32; 3] {
        let polar = self.read_float_range(0.0, PI, bits);
        let azimuth = self.read_float_range(-PI, PI, bits);
        let sin_polar = polar.sin();
        [
            (sin_polar * azimuth.cos()) as f32,
            polar.cos() as f32,
            (sin_polar * azimuth.sin()) as f32,
        ]
    }

    pub fn read_quaternion(&mut self, bits: i64) -> [f32; 4] {
        let largest = self.read_bits(2) as usize;
        let mut values = [0.0f64; 4];
        let mut sum_squares = 0.0f64;
        for (i, slot) in values.iter_mut().enumerate() {
            if i == largest {
                continue;
            }
            let v = self.read_float_range(-LIMIT, LIMIT, bits);
            *slot = v;
            sum_squares += v * v;
        }
        let rest = 1.0 - sum_squares;
        // maxf(0.0, rest): Godot's MAX, `0.0 > rest ? 0.0 : rest`.
        values[largest & 3] = (if 0.0 > rest { 0.0 } else { rest }).sqrt();
        let q = [
            values[0] as f32,
            values[1] as f32,
            values[2] as f32,
            values[3] as f32,
        ];
        if !q.iter().all(|c| c.is_finite()) || (q_length_squared(q) as f64) < 0.0001 {
            return [0.0, 0.0, 0.0, 1.0];
        }
        q_normalized(q)
    }

    pub fn read_bytes(&mut self, max_bytes: i64) -> Vec<u8> {
        let length = self.read_varint();
        if self.exhausted {
            return Vec::new();
        }
        if length < 0 || length > max_bytes {
            self.exhausted = true;
            return Vec::new();
        }
        self.align();
        let start = self.bit_pos >> 3;
        if (start + length) << 3 > self.end {
            self.exhausted = true;
            return Vec::new();
        }
        let out = self.bytes[start as usize..(start + length) as usize].to_vec();
        self.bit_pos = (start + length) << 3;
        out
    }

    /// One declaration's value, as `DotNetVar.read` reads it; `None` (nothing read) for a
    /// declaration the script must read.
    pub fn read_value(&mut self, d: &Decl) -> Option<Value> {
        if !d.native {
            return None;
        }
        Some(match d.kind {
            Kind::Bool => Value::Bool(self.read_bits(1) == 1),
            Kind::Uint => Value::Int(self.read_bits(d.bits)),
            Kind::Int => Value::Int(self.read_int(d.bits)),
            Kind::Varint => Value::Int(self.read_svarint()),
            Kind::Float => Value::Float(self.read_float32()),
            Kind::FloatRange => Value::Float(self.read_float_range(d.min, d.max, d.bits)),
            Kind::Angle => Value::Float(self.read_angle(d.bits)),
            Kind::Vec3Position | Kind::Vec3Velocity | Kind::Vec3Range => {
                Value::Vec3(self.read_vec3_range(d.min, d.max, d.bits))
            }
            Kind::Direction => Value::Vec3(self.read_direction(d.bits)),
            Kind::Quaternion => Value::Quat(self.read_quaternion(d.bits)),
            Kind::Bytes => Value::Bytes(self.read_bytes(MAX_READ_BYTES)),
            Kind::String | Kind::Custom => return None,
        })
    }
}

/// `DotNetBehaviour._encode`: `pad` zero bits, `count`, then each `(index, value)`. False when
/// any of them is one the script must write (`out` is then partly written).
pub fn encode_state(
    decls: &[Decl],
    pad: i64,
    count: i64,
    entries: &[(usize, Value)],
    out: &mut Writer,
) -> bool {
    out.reset();
    if pad > 0 {
        out.write_bits(0, pad);
    }
    out.write_bits(count, 6);
    for (index, value) in entries {
        let Some(d) = decls.get(*index) else {
            return false;
        };
        out.write_bits(*index as i64, 6);
        if !out.write_value(d, value) {
            return false;
        }
    }
    true
}

/// How [`decode_state`] stopped. The codes are the script's (`DotNetSimCodec`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stop {
    /// Every entry read.
    Done = 0,
    /// "Truncated state header."
    TruncatedHeader = 1,
    /// "Truncated state entry."
    TruncatedEntry = 2,
    /// "State references an unknown property index." (the index is `stopped_index`)
    UnknownIndex = 3,
    /// "Truncated property value." (declaration `stopped_index`)
    TruncatedValue = 4,
    /// Declaration `stopped_index` is the script's to read; its index bits are consumed.
    Script = 5,
}

/// What [`decode_state`] read.
#[derive(Default, Debug)]
pub struct Decoded {
    pub entries: Vec<(usize, Value)>,
    /// Entries left after the stop: the count header less those read, the stopped one included.
    pub remaining: i64,
    pub stopped_index: i64,
}

/// The reads of `DotNetBehaviour.read_state`, from the reader's position: the count header when
/// `remaining` is negative, else `remaining` more entries. Stops at the first error, or at a
/// declaration the script reads (it reads that one value and calls again with `remaining`).
pub fn decode_state(decls: &[Decl], r: &mut Reader<'_>, remaining: i64, out: &mut Decoded) -> Stop {
    out.entries.clear();
    out.stopped_index = -1;
    let mut left = remaining;
    if left < 0 {
        left = r.read_bits(6);
        if r.exhausted {
            out.remaining = 0;
            return Stop::TruncatedHeader;
        }
    }
    while left > 0 {
        out.remaining = left;
        let index = r.read_bits(6);
        if r.exhausted {
            return Stop::TruncatedEntry;
        }
        out.stopped_index = index;
        let Some(d) = decls.get(index as usize) else {
            return Stop::UnknownIndex;
        };
        let Some(value) = r.read_value(d) else {
            return Stop::Script;
        };
        if r.exhausted {
            return Stop::TruncatedValue;
        }
        out.entries.push((index as usize, value));
        left -= 1;
    }
    out.stopped_index = -1;
    out.remaining = 0;
    Stop::Done
}
