//! Base64url, encoding only.
//!
//! Both transports carry a Noise handshake in an HTTP header -- `Noise_X` on the POST and
//! `Noise_IK` on the WebSocket upgrade -- and a header value has to be printable, so the raw
//! bytes need an encoding. This is it, and it is here rather than in either transport because
//! having it twice is how the two would drift into disagreeing about padding.
//!
//! Unpadded, and url-safe (`-` and `_` rather than `+` and `/`). Both choices are what the
//! server's decoder expects; neither matters cryptographically, and both matter to whether a
//! header parses.
//!
//! Hand-rolled rather than pulled in: this is twenty lines that run twice per connection, and
//! a dependency for it is flash spent on nothing.

const ALPHABET: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";

/// Write `bytes` as base64url into any [`core::fmt::Write`].
///
/// Writing into the caller's sink rather than returning a buffer, because both callers are
/// building an HTTP head with `write!` and an intermediate `heapless::String` would be a
/// second copy of the handshake for no reason -- on a chip where `.bss` and `.stack` share
/// one pool.
pub fn write_base64_url<W: core::fmt::Write>(out: &mut W, bytes: &[u8]) -> core::fmt::Result {
    for chunk in bytes.chunks(3) {
        let b = [chunk[0], *chunk.get(1).unwrap_or(&0), *chunk.get(2).unwrap_or(&0)];
        let n = ((b[0] as u32) << 16) | ((b[1] as u32) << 8) | b[2] as u32;

        out.write_char(ALPHABET[(n >> 18) as usize & 63] as char)?;
        out.write_char(ALPHABET[(n >> 12) as usize & 63] as char)?;
        // Unpadded: a two-byte tail writes three characters and a one-byte tail writes two.
        if chunk.len() > 1 {
            out.write_char(ALPHABET[(n >> 6) as usize & 63] as char)?;
        }
        if chunk.len() > 2 {
            out.write_char(ALPHABET[n as usize & 63] as char)?;
        }
    }
    Ok(())
}

/// How many characters [`write_base64_url`] produces for `len` bytes.
///
/// Exact rather than an upper bound, so a buffer sized with it cannot be one character short
/// on a tail -- which is the only place unpadded base64 differs from padded.
pub const fn encoded_len(len: usize) -> usize {
    len / 3 * 4 + match len % 3 {
        0 => 0,
        1 => 2,
        _ => 3,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::String;

    fn encode(bytes: &[u8]) -> String {
        let mut out = String::new();
        write_base64_url(&mut out, bytes).unwrap();
        out
    }

    /// The RFC 4648 test vectors, in the url-safe unpadded form.
    ///
    /// Worth having verbatim: the three tail cases are the whole of what an encoder gets
    /// wrong, and they are exactly what the padded and unpadded forms disagree about.
    #[test]
    fn the_rfc_vectors_encode() {
        assert_eq!(encode(b""), "");
        assert_eq!(encode(b"f"), "Zg");
        assert_eq!(encode(b"fo"), "Zm8");
        assert_eq!(encode(b"foo"), "Zm9v");
        assert_eq!(encode(b"foob"), "Zm9vYg");
        assert_eq!(encode(b"fooba"), "Zm9vYmE");
        assert_eq!(encode(b"foobar"), "Zm9vYmFy");
    }

    /// The two bytes that separate url-safe base64 from the standard alphabet.
    ///
    /// A `+` or a `/` in a header value is not a parse error -- it decodes to something else,
    /// or to nothing -- so this failing quietly would look like a rejected handshake.
    #[test]
    fn the_url_safe_alphabet_avoids_plus_and_slash() {
        // 0xfb 0xff encodes to the last two alphabet positions, 62 and 63.
        let encoded = encode(&[0xfb, 0xff, 0xfe]);
        assert!(encoded.contains('-'), "{encoded} should use - for 62");
        assert!(encoded.contains('_'), "{encoded} should use _ for 63");
        assert!(!encoded.contains('+') && !encoded.contains('/'));
    }

    #[test]
    fn the_length_is_exact_for_every_tail() {
        for len in 0..64usize {
            let bytes = alloc::vec![0x5au8; len];
            assert_eq!(encoded_len(len), encode(&bytes).len(), "at {len} bytes");
        }
    }

    /// No padding, ever. The server's decoder tolerates `=` but the length above does not
    /// account for it, so emitting one would overrun a buffer sized by `encoded_len`.
    #[test]
    fn nothing_is_padded() {
        for len in 0..16usize {
            assert!(!encode(&alloc::vec![0u8; len]).contains('='));
        }
    }
}
