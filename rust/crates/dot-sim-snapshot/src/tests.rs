//! Golden bytes worked by hand from the script codec, and round trips. The engine-side parity
//! suite (`examples/snapshot_selftest.tscn`) compares against `DotNetWriter` itself.

use super::*;

fn spec(kind: Kind, bits: f64, min: f64, max: f64) -> Decl {
    Decl::from_spec(&[
        kind as i64 as f64,
        0.0,
        0.0,
        0.0,
        0.0,
        bits,
        min,
        max,
        2048.0,
        19.0,
        100.0,
        12.0,
        9.0,
        0.0,
    ])
}

#[test]
fn bits_are_lsb_first() {
    let mut w = Writer::new();
    w.write_bits(5, 3);
    w.write_bits(1, 1);
    w.write_bits(0xAB, 8);
    assert_eq!(w.bytes, vec![0xBD, 0x0A]);
    assert_eq!(w.bit_length(), 12);
    let mut r = Reader::new(&w.bytes);
    assert_eq!(r.read_bits(3), 5);
    assert_eq!(r.read_bits(1), 1);
    assert_eq!(r.read_bits(8), 0xAB);
    assert!(!r.exhausted);
    assert_eq!(r.read_bits(5), 0);
    assert!(r.exhausted);
    // Sticky: even a read that would fit returns zero.
    assert_eq!(r.read_bits(1), 0);
}

#[test]
fn varints() {
    let mut w = Writer::new();
    w.write_varint(300);
    w.write_svarint(-1);
    w.write_svarint(1);
    w.write_varint(-5);
    assert_eq!(w.bytes, vec![0xAC, 0x02, 0x01, 0x02, 0x00]);
    let mut r = Reader::new(&w.bytes);
    assert_eq!(r.read_varint(), 300);
    assert_eq!(r.read_svarint(), -1);
    assert_eq!(r.read_svarint(), 1);
    assert_eq!(r.read_varint(), 0);
    // Ten continuation groups are malformed.
    let bad = [0xFFu8; 10];
    let mut r = Reader::new(&bad);
    assert_eq!(r.read_varint(), 0);
    assert!(r.exhausted);
}

#[test]
fn quantised_floats() {
    let mut w = Writer::new();
    w.write_float_range(0.5, 0.0, 1.0, 8);
    assert_eq!(w.bytes, vec![0x80]);
    let mut w = Writer::new();
    w.write_angle(-90.0, 9);
    let mut r = Reader::new(&w.bytes);
    assert_eq!(r.read_bits(9), 383);
    let mut w = Writer::new();
    w.write_float_range(f64::NAN, 0.0, 1.0, 8);
    w.write_float_range(1e300, 0.0, 1.0, 8);
    assert_eq!(w.bytes, vec![0x00, 0xFF]);
}

#[test]
fn identity_quaternion() {
    let mut w = Writer::new();
    w.write_quaternion([0.0, 0.0, 0.0, 1.0], 9);
    let mut r = Reader::new(&w.bytes);
    assert_eq!(r.read_bits(2), 3);
    for _ in 0..3 {
        assert_eq!(r.read_bits(9), 256);
    }
    let mut r = Reader::new(&w.bytes);
    let q = r.read_quaternion(9);
    assert!(q[3] > 0.9999 && q[0].abs() < 0.002);
}

#[test]
fn zero_quaternion_and_direction_are_deterministic() {
    let mut w = Writer::new();
    w.write_quaternion([0.0; 4], 9);
    w.write_direction([0.0; 3], 12);
    let mut again = Writer::new();
    again.write_quaternion([0.0; 4], 9);
    again.write_direction([0.0; 3], 12);
    assert_eq!(w.bytes, again.bytes);
}

#[test]
fn state_round_trip() {
    let decls = vec![
        spec(Kind::Bool, 1.0, 0.0, 0.0),
        spec(Kind::Uint, 10.0, 0.0, 0.0),
        spec(Kind::Int, 7.0, 0.0, 0.0),
        spec(Kind::Varint, 0.0, 0.0, 0.0),
        spec(Kind::Float, 0.0, 0.0, 0.0),
        spec(Kind::Vec3Position, 0.0, 0.0, 0.0),
        spec(Kind::Quaternion, 0.0, 0.0, 0.0),
        spec(Kind::Bytes, 0.0, 0.0, 0.0),
        spec(Kind::Custom, 0.0, 0.0, 0.0),
    ];
    let entries = vec![
        (0, Value::Bool(true)),
        (1, Value::Int(1000)),
        (2, Value::Int(-17)),
        (3, Value::Int(-123_456)),
        (4, Value::Float(1.5)),
        (5, Value::Vec3([0.0, 1.0, -2.0])),
        (6, Value::Quat([0.0, 0.0, 0.0, 1.0])),
        (7, Value::Bytes(vec![1, 2, 3])),
    ];
    for pad in 0..8 {
        let mut w = Writer::new();
        assert!(encode_state(
            &decls,
            pad,
            entries.len() as i64,
            &entries,
            &mut w
        ));
        let mut r = Reader::at(&w.bytes, pad, w.bit_length() as i64);
        let mut out = Decoded::default();
        assert_eq!(decode_state(&decls, &mut r, -1, &mut out), Stop::Done);
        assert_eq!(r.bit_pos, w.bit_length() as i64);
        assert_eq!(out.entries.len(), entries.len());
        for (a, b) in out.entries.iter().zip(&entries) {
            assert_eq!(a.0, b.0);
            match (&a.1, &b.1) {
                (Value::Vec3(x), Value::Vec3(y)) => {
                    for i in 0..3 {
                        assert!((x[i] - y[i]).abs() < 0.01);
                    }
                }
                (Value::Quat(x), _) => assert!(x[3] > 0.9999),
                _ => assert_eq!(a.1, b.1),
            }
        }
        // Truncated anywhere: an error, never a panic, and the entries before it.
        for cut in pad..w.bit_length() as i64 {
            let mut r = Reader::at(&w.bytes, pad, cut);
            let mut out = Decoded::default();
            let stop = decode_state(&decls, &mut r, -1, &mut out);
            assert!(matches!(
                stop,
                Stop::TruncatedHeader | Stop::TruncatedEntry | Stop::TruncatedValue
            ));
        }
    }
    // A custom declaration is the script's.
    let mut w = Writer::new();
    assert!(!encode_state(&decls, 0, 1, &[(8, Value::Int(1))], &mut w));
    let mut w = Writer::new();
    w.write_bits(2, 6);
    w.write_bits(0, 6);
    w.write_bits(1, 1);
    w.write_bits(8, 6);
    let mut r = Reader::new(&w.bytes);
    let mut out = Decoded::default();
    assert_eq!(decode_state(&decls, &mut r, -1, &mut out), Stop::Script);
    assert_eq!(
        (out.entries.len(), out.remaining, out.stopped_index),
        (1, 1, 8)
    );
    assert_eq!(r.bit_pos, 19);
    // An index past the declarations.
    let mut w = Writer::new();
    w.write_bits(1, 6);
    w.write_bits(40, 6);
    let mut r = Reader::new(&w.bytes);
    assert_eq!(
        decode_state(&decls, &mut r, -1, &mut out),
        Stop::UnknownIndex
    );
    assert_eq!(out.stopped_index, 40);
}

#[test]
fn wide_floats_are_the_scripts() {
    assert!(!spec(Kind::FloatRange, 40.0, 0.0, 1.0).native);
    assert!(spec(Kind::FloatRange, 32.0, 0.0, 1.0).native);
    assert!(spec(Kind::Uint, 64.0, 0.0, 0.0).native);
    assert!(!spec(Kind::String, 0.0, 0.0, 0.0).native);
}

/// Golden bytes from the script codec itself (`examples/snapshot_selftest.gd`, `_golden`): a
/// player state as game-arena sends it, default config, pad 3. A change here is a wire change.
#[test]
fn golden_player_state() {
    let decls = vec![
        spec(Kind::Vec3Position, 0.0, 0.0, 0.0),
        spec(Kind::Vec3Velocity, 0.0, 0.0, 0.0),
        spec(Kind::Angle, 12.0, 0.0, 0.0),
        spec(Kind::Angle, 10.0, 0.0, 0.0),
        spec(Kind::FloatRange, 14.0, 0.0, 2000.0),
        spec(Kind::Uint, 3.0, 0.0, 0.0),
        spec(Kind::Uint, 5.0, 0.0, 0.0),
        spec(Kind::Uint, 9.0, 0.0, 0.0),
        spec(Kind::Uint, 16.0, 0.0, 0.0),
        spec(Kind::Uint, 8.0, 0.0, 0.0),
        spec(Kind::Bool, 1.0, 0.0, 0.0),
        spec(Kind::String, 0.0, 0.0, 0.0),
        spec(Kind::Quaternion, 0.0, 0.0, 0.0),
    ];
    let half = 0.375f32;
    let entries = vec![
        (0, Value::Vec3([12.5, -3.25, 100.0])),
        (1, Value::Vec3([4.0, 0.0, -2.5])),
        (2, Value::Float(271.0)),
        (3, Value::Float(-12.0)),
        (4, Value::Float(87.5)),
        (5, Value::Int(5)),
        (7, Value::Int(300)),
        (10, Value::Bool(false)),
        (12, Value::Quat([0.0, half.sin(), 0.0, half.cos()])),
    ];
    let mut w = Writer::new();
    assert!(encode_state(
        &decls,
        3,
        entries.len() as i64,
        &entries,
        &mut w
    ));
    assert_eq!(w.bit_length(), 234);
    let hex: String = w.bytes.iter().map(|b| format!("{b:02x}")).collect();
    assert_eq!(
        hex,
        "48801f0382f9ef3f8641140220f3090b3c744f340b450f960a6680840102"
    );
}
