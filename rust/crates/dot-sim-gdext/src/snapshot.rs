//! `DotSimSnapshotCodec`: one behaviour's state encode and decode, byte for byte the script's.

use dot_sim_snapshot::{
    Decl, Decoded, Kind, Reader, Stop, Value, Writer, decode_state, encode_state, to_int,
};
use godot::prelude::*;

/// One behaviour class's declarations (`DotNetBehaviour._native_spec()` rows) and scratch.
#[derive(GodotClass)]
#[class(base = RefCounted, init)]
pub struct DotSimSnapshotCodec {
    decls: Vec<Decl>,
    writer: Writer,
    entries: Vec<(usize, Value)>,
    decoded: Decoded,
}

/// The value as `DotNetVar.write` converts it (what dot-net's C++ core accepts too), or `None`
/// for one only the script converts.
fn to_value(d: &Decl, v: &Variant) -> Option<Value> {
    let t = v.get_type();
    let int_of = || match t {
        VariantType::INT => Some(v.to::<i64>()),
        VariantType::FLOAT => Some(to_int(v.to::<f64>())),
        VariantType::BOOL => Some(v.to::<bool>() as i64),
        _ => None,
    };
    let float_of = || match t {
        VariantType::FLOAT => Some(v.to::<f64>()),
        VariantType::INT => Some(v.to::<i64>() as f64),
        VariantType::BOOL => Some(if v.to::<bool>() { 1.0 } else { 0.0 }),
        _ => None,
    };
    Some(match d.kind {
        Kind::Bool => Value::Bool(match t {
            VariantType::BOOL => v.to::<bool>(),
            VariantType::INT => v.to::<i64>() != 0,
            VariantType::FLOAT => v.to::<f64>() != 0.0,
            _ => return None,
        }),
        Kind::Uint | Kind::Int | Kind::Varint => Value::Int(int_of()?),
        Kind::Float | Kind::FloatRange | Kind::Angle => Value::Float(float_of()?),
        Kind::Vec3Position | Kind::Vec3Velocity | Kind::Vec3Range | Kind::Direction => {
            if t != VariantType::VECTOR3 {
                return None;
            }
            let p = v.to::<Vector3>();
            Value::Vec3([p.x, p.y, p.z])
        }
        Kind::Quaternion => {
            if t != VariantType::QUATERNION {
                return None;
            }
            let q = v.to::<Quaternion>();
            Value::Quat([q.x, q.y, q.z, q.w])
        }
        Kind::Bytes => {
            if t != VariantType::PACKED_BYTE_ARRAY {
                return None;
            }
            Value::Bytes(v.to::<PackedByteArray>().as_slice().to_vec())
        }
        Kind::String | Kind::Custom => return None,
    })
}

fn to_variant(v: &Value) -> Variant {
    match v {
        Value::Bool(b) => b.to_variant(),
        Value::Int(i) => i.to_variant(),
        Value::Float(f) => f.to_variant(),
        Value::Vec3(p) => Vector3::new(p[0], p[1], p[2]).to_variant(),
        Value::Quat(q) => Quaternion::new(q[0], q[1], q[2], q[3]).to_variant(),
        Value::Bytes(b) => PackedByteArray::from(b.as_slice()).to_variant(),
    }
}

#[godot_api]
impl DotSimSnapshotCodec {
    /// The declarations, one `PackedFloat64Array` per `DotNetVar` in registration order. Returns
    /// how many of them the codec does itself (the rest stop it, and the script does them).
    #[func]
    fn set_spec(&mut self, rows: VarArray) -> i64 {
        self.decls = rows
            .iter_shared()
            .map(|row| {
                let row = row.try_to::<PackedFloat64Array>().unwrap_or_default();
                Decl::from_spec(row.as_slice())
            })
            .collect();
        self.decls.iter().filter(|d| d.native).count() as i64
    }

    /// Whether declaration `index` is the codec's (false: the script's, or no such index).
    #[func]
    fn is_native(&self, index: i64) -> bool {
        usize::try_from(index)
            .ok()
            .and_then(|i| self.decls.get(i))
            .is_some_and(|d| d.native)
    }

    /// `DotNetBehaviour._encode`'s bytes: `pad` zero bits, `count` in 6 bits, then
    /// `indices[i]` and `values[i]` for each. `[bytes, bit_length]`, or empty when any value is
    /// one the script must write.
    #[func]
    fn encode(
        &mut self,
        pad: i64,
        count: i64,
        indices: PackedInt32Array,
        values: VarArray,
    ) -> VarArray {
        let n = indices.len();
        if values.len() != n {
            return VarArray::new();
        }
        self.entries.clear();
        for (i, index) in indices.as_slice().iter().enumerate() {
            let Some(d) = usize::try_from(*index).ok().and_then(|x| self.decls.get(x)) else {
                return VarArray::new();
            };
            if !d.native {
                return VarArray::new();
            }
            let Some(value) = to_value(d, &values.at(i)) else {
                return VarArray::new();
            };
            self.entries.push((*index as usize, value));
        }
        if !encode_state(&self.decls, pad, count, &self.entries, &mut self.writer) {
            return VarArray::new();
        }
        let mut out = VarArray::new();
        out.push(&PackedByteArray::from(self.writer.bytes.as_slice()).to_variant());
        out.push(&(self.writer.bit_length() as i64).to_variant());
        out
    }

    /// `DotNetBehaviour.read_state`'s reads from bit `bit_pos` of `bytes`, ending at `end_bits`
    /// (negative: the buffer's end): the count header when `remaining` is negative, else that
    /// many more entries. Fills `out` with `[bit position after, exhausted, entries left,
    /// stopped index, index, value, index, value...]` and returns the stop: 0 all read,
    /// 1 truncated header, 2 truncated entry, 3 unknown index, 4 truncated value, 5 the script
    /// reads declaration `stopped index` next (its index already read).
    #[func]
    fn decode(
        &mut self,
        bytes: PackedByteArray,
        bit_pos: i64,
        end_bits: i64,
        remaining: i64,
        mut out: VarArray,
    ) -> i64 {
        let mut r = Reader::at(bytes.as_slice(), bit_pos, end_bits);
        let stop = decode_state(&self.decls, &mut r, remaining, &mut self.decoded);
        out.clear();
        out.push(&r.bit_pos.to_variant());
        out.push(&r.exhausted.to_variant());
        out.push(&self.decoded.remaining.to_variant());
        out.push(&self.decoded.stopped_index.to_variant());
        for (index, value) in &self.decoded.entries {
            out.push(&(*index as i64).to_variant());
            out.push(&to_variant(value));
        }
        stop as i64
    }
}

// The stop codes are the script's contract.
const _: () = {
    assert!(Stop::Done as i64 == 0);
    assert!(Stop::Script as i64 == 5);
};
