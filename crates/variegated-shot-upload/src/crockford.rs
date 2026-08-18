//! Crockford base32 for the two 32-byte Noise keys, with a check symbol.
//!
//! # Why this alphabet
//!
//! These keys are copied by a human out of a web page and into a form on a machine that has
//! no keyboard of its own. Crockford's alphabet exists for exactly that: it has no `i`, `l`,
//! `o` or `u`, so the four pairs a person actually confuses -- `1`/`l`/`i` and `0`/`o` --
//! cannot both be written, and the two that survive are accepted as aliases on the way back
//! in. Hyphens may be inserted anywhere for readability and are ignored on decode.
//!
//! It is also what [`variegated-plantlet-ts`] already uses for upload tokens and shot ids
//! (`apps/worker/src/machines.ts`), lowercased. Matching it means one alphabet across the
//! system rather than two that differ in a way nobody notices until a key round-trips wrong.
//!
//! # Why 53 characters and not 52
//!
//! 32 bytes is 256 bits, and 256 is not a multiple of five. The upload token dodges this --
//! it is 40 bytes, and 320 divides by five, so it is exactly 64 characters with nothing left
//! over. Here the last group is partial: 51 full groups cover 255 bits and the 52nd character
//! carries the final bit, left-aligned, so its low four bits are padding that must be zero.
//! [`decode_key`] rejects a non-zero remainder rather than ignoring it, because otherwise
//! sixteen distinct strings decode to the same key and a round-trip test passes while the
//! encoding is ambiguous.
//!
//! The 53rd character is Crockford's check symbol: the whole key read as a 256-bit big-endian
//! integer, modulo 37. It costs one character and it is the difference between a mistyped
//! server key reported as "that is not a valid key" and one reported as
//! `Handshake { code: -28160 }` three retries later. This crate has been on the wrong side of
//! that trade before -- see [`roots`], where the wrong trust anchor reached hardware twice
//! because a bad key and a broken network were indistinguishable from the log.
//!
//! [`roots`]: crate::roots
//! [`variegated-plantlet-ts`]: https://github.com/variegated-coffee/variegated-plantlet-ts

/// Raw key length. X25519 public keys and secrets are both 32 bytes.
pub const KEY_LEN: usize = 32;

/// 52 characters of payload plus one check symbol.
pub const ENCODED_LEN: usize = 53;

/// The 32 data symbols, lowercase. Byte-for-byte `machines.ts`'s `ALPHABET`.
const ALPHABET: &[u8; 32] = b"0123456789abcdefghjkmnpqrstvwxyz";

/// The check alphabet: the 32 data symbols plus five more, giving the 37 residues.
const CHECK_ALPHABET: &[u8; 37] = b"0123456789abcdefghjkmnpqrstvwxyz*~$=u";

/// Why a key would not decode.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrockfordError {
    /// Not [`ENCODED_LEN`] symbols once hyphens and surrounding whitespace are removed.
    WrongLength,
    /// A symbol outside the alphabet. `u` is not a data symbol in Crockford's alphabet and
    /// is refused here rather than aliased, because it is only legal as a check symbol.
    BadChar,
    /// The low four bits of the last data symbol were not zero. See the module docs: they
    /// are padding, and accepting a non-zero remainder would make the encoding ambiguous.
    PaddingNotZero,
    /// The check symbol did not match the payload. Almost always a typo, and saying so is
    /// the whole reason the symbol is there.
    CheckMismatch,
}

/// Decode a provisioned key.
///
/// Case-insensitive, hyphen-tolerant, and strict about everything else.
pub fn decode_key(text: &str) -> Result<[u8; KEY_LEN], CrockfordError> {
    let mut symbols = [0u8; ENCODED_LEN];
    let mut n = 0usize;

    for byte in text.bytes() {
        // Crockford permits hyphens as visual separators; whitespace survives a copy-paste
        // more often than anyone would like. Neither carries information.
        if byte == b'-' || byte.is_ascii_whitespace() {
            continue;
        }
        if n == ENCODED_LEN {
            return Err(CrockfordError::WrongLength);
        }
        symbols[n] = byte;
        n += 1;
    }

    if n != ENCODED_LEN {
        return Err(CrockfordError::WrongLength);
    }

    let mut key = [0u8; KEY_LEN];
    let mut acc = 0u16;
    let mut bits = 0u8;
    let mut out = 0usize;

    for &symbol in &symbols[..ENCODED_LEN - 1] {
        acc = (acc << 5) | u16::from(data_value(symbol)?);
        bits += 5;
        if bits >= 8 {
            bits -= 8;
            key[out] = (acc >> bits) as u8;
            out += 1;
        }
    }

    // 52 symbols is 260 bits against 256 of key, so four bits are always left here.
    if acc & 0x0f != 0 {
        return Err(CrockfordError::PaddingNotZero);
    }

    if data_or_check_value(symbols[ENCODED_LEN - 1])? != check_symbol_value(&key) {
        return Err(CrockfordError::CheckMismatch);
    }

    Ok(key)
}

/// Encode a key for display. Always [`ENCODED_LEN`] characters, never hyphenated.
pub fn encode_key(key: &[u8; KEY_LEN]) -> heapless::String<ENCODED_LEN> {
    let mut out = heapless::String::new();
    let mut acc = 0u16;
    let mut bits = 0u8;

    for &byte in key.iter() {
        acc = (acc << 8) | u16::from(byte);
        bits += 8;
        while bits >= 5 {
            bits -= 5;
            let index = ((acc >> bits) & 0x1f) as usize;
            let _ = out.push(ALPHABET[index] as char);
        }
    }

    // The trailing bit, left-aligned in its group -- the mirror of the padding check in
    // `decode_key`.
    if bits > 0 {
        let index = ((acc << (5 - bits)) & 0x1f) as usize;
        let _ = out.push(ALPHABET[index] as char);
    }

    let _ = out.push(CHECK_ALPHABET[check_symbol_value(key) as usize] as char);
    out
}

/// The key as a 256-bit big-endian integer, modulo 37.
///
/// Done a byte at a time so there is no bignum: `rem * 256 + byte` never exceeds 37 * 256 +
/// 255, which fits a `u16` several times over.
fn check_symbol_value(key: &[u8; KEY_LEN]) -> u8 {
    let mut rem = 0u16;
    for &byte in key.iter() {
        rem = (rem * 256 + u16::from(byte)) % 37;
    }
    rem as u8
}

/// A data symbol's value, applying Crockford's decode aliases.
fn data_value(symbol: u8) -> Result<u8, CrockfordError> {
    let lower = symbol.to_ascii_lowercase();
    match lower {
        b'o' => Ok(0),
        b'i' | b'l' => Ok(1),
        _ => match ALPHABET.iter().position(|&c| c == lower) {
            Some(index) => Ok(index as u8),
            None => Err(CrockfordError::BadChar),
        },
    }
}

/// As [`data_value`], but the five check-only symbols are legal too.
fn data_or_check_value(symbol: u8) -> Result<u8, CrockfordError> {
    let lower = symbol.to_ascii_lowercase();
    match CHECK_ALPHABET.iter().position(|&c| c == lower) {
        Some(index) => Ok(index as u8),
        // `o`, `i` and `l` are still aliases in the check position.
        None => data_value(symbol),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Not a real key -- a fixed pattern, so the expected string below is stable and a
    /// change to the packing shows up as a diff rather than as a round-trip that still
    /// passes. This exact vector is asserted by the TypeScript decoder too.
    const VECTOR: [u8; KEY_LEN] = [
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,
        0x0e, 0x0f, 0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb,
        0xfc, 0xfd, 0xfe, 0xff,
    ];

    #[test]
    fn a_key_round_trips() {
        let text = encode_key(&VECTOR);
        assert_eq!(text.len(), ENCODED_LEN);
        assert_eq!(decode_key(&text).unwrap(), VECTOR);
    }

    #[test]
    fn every_byte_pattern_round_trips() {
        // Walks the carry through every bit position, which is where a shift-by-one bug in
        // the 5/8 repacking hides.
        for seed in 0u8..=255 {
            let mut key = [0u8; KEY_LEN];
            for (i, slot) in key.iter_mut().enumerate() {
                *slot = seed.wrapping_add(i as u8).wrapping_mul(31);
            }
            let text = encode_key(&key);
            assert_eq!(decode_key(&text).unwrap(), key, "seed {seed}");
        }
    }

    #[test]
    fn the_encoding_uses_only_the_alphabet() {
        let text = encode_key(&VECTOR);
        for (i, c) in text.bytes().enumerate() {
            let legal = if i == ENCODED_LEN - 1 {
                CHECK_ALPHABET.contains(&c)
            } else {
                ALPHABET.contains(&c)
            };
            assert!(legal, "symbol {c:?} at {i} is outside the alphabet");
        }
    }

    #[test]
    fn decoding_is_case_insensitive() {
        let text = encode_key(&VECTOR);
        let mut upper = heapless::String::<ENCODED_LEN>::new();
        for c in text.chars() {
            upper.push(c.to_ascii_uppercase()).unwrap();
        }
        assert_eq!(decode_key(&upper).unwrap(), VECTOR);
    }

    #[test]
    fn hyphens_and_whitespace_are_ignored() {
        let text = encode_key(&VECTOR);
        let mut spaced = alloc::string::String::new();
        for (i, c) in text.chars().enumerate() {
            if i > 0 && i % 4 == 0 {
                spaced.push('-');
            }
            spaced.push(c);
        }
        assert_eq!(decode_key(&spaced).unwrap(), VECTOR);
        assert_eq!(decode_key(&alloc::format!("  {text}  ")).unwrap(), VECTOR);
    }

    #[test]
    fn the_confusable_letters_are_aliases() {
        // Build a string whose payload uses `0` and `1`, then retype it the way a person
        // would after reading it off a screen.
        let key = [0u8; KEY_LEN];
        let text = encode_key(&key);
        assert!(text.starts_with('0'), "expected a leading zero to substitute into");
        let retyped: alloc::string::String = text
            .chars()
            .map(|c| match c {
                '0' => 'O',
                '1' => 'l',
                other => other,
            })
            .collect();
        assert_eq!(decode_key(&retyped).unwrap(), key);
    }

    #[test]
    fn u_is_not_a_data_symbol() {
        let text = encode_key(&VECTOR);
        let mut broken: alloc::string::String = text.chars().collect();
        broken.replace_range(0..1, "u");
        // Either it is refused as a bad character or the check symbol catches it; both are
        // rejections, and the point is that `u` never silently means something.
        assert!(matches!(
            decode_key(&broken),
            Err(CrockfordError::BadChar) | Err(CrockfordError::CheckMismatch)
        ));
    }

    #[test]
    fn a_string_of_the_wrong_length_is_refused() {
        let text = encode_key(&VECTOR);
        assert_eq!(decode_key(&text[..ENCODED_LEN - 1]), Err(CrockfordError::WrongLength));
        assert_eq!(
            decode_key(&alloc::format!("{text}0")),
            Err(CrockfordError::WrongLength)
        );
        assert_eq!(decode_key(""), Err(CrockfordError::WrongLength));
    }

    #[test]
    fn non_zero_padding_is_refused() {
        let text = encode_key(&VECTOR);
        let mut chars: alloc::vec::Vec<char> = text.chars().collect();
        // The 52nd symbol carries one significant bit and four of padding. Anything that
        // sets a padding bit must be refused, or sixteen strings mean one key.
        let last_data = data_value(chars[ENCODED_LEN - 2] as u8).unwrap();
        chars[ENCODED_LEN - 2] = ALPHABET[(last_data | 0x01) as usize] as char;
        let broken: alloc::string::String = chars.into_iter().collect();
        assert_eq!(decode_key(&broken), Err(CrockfordError::PaddingNotZero));
    }

    #[test]
    fn a_single_character_typo_is_caught() {
        // The reason the check symbol earns its character. Every single-symbol substitution
        // in the payload must be rejected rather than decoding to a different valid key.
        let text = encode_key(&VECTOR);
        let original: alloc::vec::Vec<char> = text.chars().collect();

        let mut checked = 0;
        for position in 0..ENCODED_LEN - 1 {
            for &replacement in ALPHABET.iter() {
                if original[position] == replacement as char {
                    continue;
                }
                let mut typo = original.clone();
                typo[position] = replacement as char;
                let text: alloc::string::String = typo.into_iter().collect();
                assert!(
                    decode_key(&text).is_err(),
                    "a typo at {position} decoded to a key"
                );
                checked += 1;
            }
        }
        assert!(checked > 1000, "expected to have exercised every substitution");
    }

    #[test]
    fn a_corrupted_check_symbol_is_caught() {
        let text = encode_key(&VECTOR);
        let mut chars: alloc::vec::Vec<char> = text.chars().collect();
        let wrong = (check_symbol_value(&VECTOR) + 1) % 37;
        chars[ENCODED_LEN - 1] = CHECK_ALPHABET[wrong as usize] as char;
        let broken: alloc::string::String = chars.into_iter().collect();
        assert_eq!(decode_key(&broken), Err(CrockfordError::CheckMismatch));
    }

    #[test]
    fn the_check_symbol_is_the_key_modulo_37() {
        // Derived by hand rather than by running the function, so this is a check on the
        // implementation and not a restatement of it.
        assert_eq!(check_symbol_value(&[0u8; KEY_LEN]), 0);

        let mut one = [0u8; KEY_LEN];
        one[KEY_LEN - 1] = 1;
        assert_eq!(check_symbol_value(&one), 1);

        // The all-ones key is 2^256 - 1. 37 is prime, so by Fermat 2^36 = 1 (mod 37);
        // 256 = 7*36 + 4, so 2^256 = 2^4 = 16, and 2^256 - 1 = 15 (mod 37).
        assert_eq!(check_symbol_value(&[0xffu8; KEY_LEN]), 15);
    }
}
