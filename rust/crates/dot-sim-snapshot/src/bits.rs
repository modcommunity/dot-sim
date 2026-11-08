//! `DotNetWriter` and `DotNetReader`'s bit packing: least significant bit first, bytes in order.

/// `DotNetWriter.MAX_BYTES`.
pub const MAX_BYTES: usize = 1 << 22;

/// A growing bit buffer, as `DotNetWriter`.
#[derive(Default, Debug, Clone)]
pub struct Writer {
    pub bytes: Vec<u8>,
    pub bit_pos: usize,
    pub overflowed: bool,
}

impl Writer {
    pub fn new() -> Writer {
        Writer::default()
    }

    pub fn reset(&mut self) {
        self.bytes.clear();
        self.bit_pos = 0;
        self.overflowed = false;
    }

    pub fn bit_length(&self) -> usize {
        self.bit_pos
    }

    /// The low `bits` of `value`; a width out of 1..=64 writes nothing, as the script's does.
    pub fn write_bits(&mut self, value: i64, bits: i64) {
        if bits <= 0 || bits > 64 {
            return;
        }
        let mut v = if bits >= 64 {
            value as u64
        } else {
            (value as u64) & ((1u64 << bits) - 1)
        };
        let mut remaining = bits as usize;
        while remaining > 0 {
            let byte_index = self.bit_pos >> 3;
            let bit_offset = self.bit_pos & 7;
            let take = (8 - bit_offset).min(remaining);
            if byte_index >= self.bytes.len() {
                if self.bytes.len() >= MAX_BYTES {
                    self.overflowed = true;
                    return;
                }
                self.bytes.push(0);
            }
            self.bytes[byte_index] |= ((v & ((1u64 << take) - 1)) << bit_offset) as u8;
            v >>= take;
            self.bit_pos += take;
            remaining -= take;
        }
    }

    pub fn align(&mut self) {
        let remainder = self.bit_pos & 7;
        if remainder != 0 {
            self.write_bits(0, 8 - remainder as i64);
        }
    }

    /// `write_bytes`' tail: the bytes at the (aligned) end.
    pub fn append_aligned(&mut self, data: &[u8]) {
        if self.bytes.len() + data.len() > MAX_BYTES {
            self.overflowed = true;
            return;
        }
        self.bytes.extend_from_slice(data);
        self.bit_pos = self.bytes.len() << 3;
    }
}

/// A bounded bit cursor over borrowed bytes, as `DotNetReader`: once a read runs past the end it
/// is exhausted, and every read after returns zero.
#[derive(Debug)]
pub struct Reader<'a> {
    pub bytes: &'a [u8],
    pub bit_pos: i64,
    /// The end, in bits (`limit_bits`, else the whole buffer).
    pub end: i64,
    pub exhausted: bool,
}

impl<'a> Reader<'a> {
    pub fn new(bytes: &'a [u8]) -> Reader<'a> {
        Reader {
            bytes,
            bit_pos: 0,
            end: (bytes.len() as i64) << 3,
            exhausted: false,
        }
    }

    /// At `bit_pos`, ending at `end_bits` (negative: the whole buffer; clamped to it).
    pub fn at(bytes: &'a [u8], bit_pos: i64, end_bits: i64) -> Reader<'a> {
        let full = (bytes.len() as i64) << 3;
        let end = if end_bits < 0 {
            full
        } else {
            end_bits.min(full)
        };
        Reader {
            bytes,
            bit_pos: bit_pos.clamp(0, end),
            end,
            exhausted: false,
        }
    }

    pub fn read_bits(&mut self, bits: i64) -> i64 {
        if bits <= 0 || bits > 64 || self.exhausted {
            return 0;
        }
        if self.bit_pos + bits > self.end {
            self.exhausted = true;
            return 0;
        }
        let mut result = 0u64;
        let mut written = 0u32;
        let mut remaining = bits;
        while remaining > 0 {
            let byte_index = (self.bit_pos >> 3) as usize;
            let bit_offset = (self.bit_pos & 7) as u32;
            let take = (8 - bit_offset as i64).min(remaining) as u32;
            let chunk = ((self.bytes[byte_index] >> bit_offset) as u64) & ((1u64 << take) - 1);
            result |= chunk << written;
            written += take;
            self.bit_pos += take as i64;
            remaining -= take as i64;
        }
        result as i64
    }

    pub fn align(&mut self) {
        let remainder = self.bit_pos & 7;
        if remainder != 0 {
            self.bit_pos += 8 - remainder;
        }
    }
}
