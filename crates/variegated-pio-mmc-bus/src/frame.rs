//! Command framing, response validation, and the arithmetic of a data transfer.
//!
//! Pure functions, kept free of `embassy-rp` so they can be tested on a host. These are
//! the parts a logic analyser is worst at checking: a response packed into the wrong word
//! order still looks like a perfectly good bus trace.

use sdio::{BusWidth, MmcError, ResponseLen};

use crate::crc::crc7;

/// Bits to shift out for one command, minus one -- the `X` preload for `cmd_rsp`.
///
/// One `0xFF` preamble byte plus the six frame bytes, eight bits each: `7 * 8 - 1`. From
/// SdFat, where it appears as the literal 55.
pub const CMD_X_REGISTER: u32 = 55;

/// The idle byte clocked out ahead of every command frame.
pub const PREAMBLE: u8 = 0xFF;

/// Nibbles of idle-then-start that open a write block: seven `0xF` then one `0x0`.
pub const WRITE_START_TOKEN: u32 = 0xFFFF_FFF0;

/// Trailing all-ones word; `wr_data` consumes exactly one nibble of it as the stop bit.
pub const WRITE_END_TOKEN: u32 = 0xFFFF_FFFF;

/// The card accepted the block: start bit 0, status `010`, end bit 1.
///
/// Grouped by the token's own fields rather than by nibble; even groups would hide the
/// three-bit status that is the only part worth reading.
#[allow(clippy::unusual_byte_groupings)]
pub const WRITE_ACCEPTED: u8 = 0b0_010_1;
/// Mask selecting the five meaningful bits of a CRC status token.
pub const WRITE_STATUS_MASK: u8 = 0x1F;

/// The all-ones reserved byte that opens a response with no command index to echo.
const RESERVED_HEADER: u8 = 0x3F;

/// The six bytes of a command frame, exactly as they go onto CMD.
///
/// Byte for byte the same frame the `sdio` crate's SPI transport builds. The two
/// transports differ in how the frame is clocked, not in what the frame is, and keeping
/// them identical means a CRC-7 bug cannot hide in one of them.
pub fn command_frame(index: u8, arg: u32) -> [u8; 6] {
    let mut buf = [
        0x40 | (index & 0x3F),
        (arg >> 24) as u8,
        (arg >> 16) as u8,
        (arg >> 8) as u8,
        arg as u8,
        0,
    ];
    buf[5] = (crc7(&buf[..5]) << 1) | 1;
    buf
}

/// Bytes of response to clock in, from the response's length on the native bus.
///
/// Note this is the *native* framing, not SPI's: a 48-bit response is six bytes here
/// (header, four payload bytes, CRC) where SPI mode strips it to a bare status byte plus
/// an optional payload.
pub const fn response_bytes(len: ResponseLen) -> usize {
    match len {
        ResponseLen::Zero => 0,
        ResponseLen::R48 => 6,
        ResponseLen::R136 => 17,
    }
}

/// Bits of response to clock in, minus one -- the `Y` preload for `cmd_rsp`.
///
/// Zero for a command with no response, which is what makes `cmd_rsp`'s
/// `jmp !Y cmd_begin` take the early exit instead of hunting for a start bit that will
/// never come.
pub const fn y_register(n_rsp: usize) -> u32 {
    if n_rsp == 0 {
        0
    } else {
        (8 * n_rsp - 1) as u32
    }
}

/// Validate a response as received and pack it into the words `sdio` expects.
///
/// `has_crc` is the response type's [`sdio::Response::CRC`]. It is false for exactly the
/// responses that carry an OCR -- R3 and R4 -- which the specification sends without a
/// checksum because they are exchanged before the card has agreed on anything to
/// checksum with.
///
/// # Word order
///
/// A 48-bit response puts its 32-bit payload in `words[0]` and nothing else.
///
/// A 136-bit response is the one to get right: `sdio` reassembles CID and CSD as
/// `words[3] << 96 | words[2] << 64 | words[1] << 32 | words[0]`, so **`words[3]` holds
/// the most significant bits** -- the ones the card sent first. Reversing it is silent
/// rather than loud: `CSD::version()` reads bits [127:126], which live in `words[3]`, so
/// a reversed packing reads the register's trailing CRC-7 byte instead and decodes a
/// modern SDHC card as a tiny version-0 one with an absurd block count. This is not
/// hypothetical -- it is the same class of bug the SPI transport in this workspace
/// already shipped once. `r136_word_order_survives_a_round_trip_through_sdio` pins it
/// against the real `sdio` types rather than against this function's own idea of them.
pub fn pack_response(rtn: &[u8], has_crc: bool) -> Result<[u32; 4], MmcError> {
    let mut words = [0u32; 4];

    match rtn.len() {
        0 => {}
        6 => {
            if has_crc {
                if crc7(&rtn[..5]) != rtn[5] >> 1 {
                    return Err(MmcError::Crc);
                }
            } else {
                // No CRC to check, so the framing is checked instead. Both of these bytes
                // are all-ones reserved fields in an OCR response, so either being wrong
                // means the bits were sampled at the wrong offset -- a failure mode a CRC
                // would have caught and which nothing else here would.
                if rtn[0] != RESERVED_HEADER || rtn[5] != 0xFF {
                    return Err(MmcError::Crc);
                }
            }
            words[0] = u32::from_be_bytes([rtn[1], rtn[2], rtn[3], rtn[4]]);
        }
        17 => {
            if rtn[0] != RESERVED_HEADER {
                return Err(MmcError::Crc);
            }
            // The CRC-7 covers the register contents only. It is the checksum stored
            // *inside* the CID/CSD, not one the card computes over the response, so it
            // starts after the header byte and ends before itself.
            if crc7(&rtn[1..16]) != rtn[16] >> 1 {
                return Err(MmcError::Crc);
            }
            words[3] = u32::from_be_bytes([rtn[1], rtn[2], rtn[3], rtn[4]]);
            words[2] = u32::from_be_bytes([rtn[5], rtn[6], rtn[7], rtn[8]]);
            words[1] = u32::from_be_bytes([rtn[9], rtn[10], rtn[11], rtn[12]]);
            words[0] = u32::from_be_bytes([rtn[13], rtn[14], rtn[15], rtn[16]]);
        }
        _ => return Err(MmcError::Other),
    }

    Ok(words)
}

/// The command index a 48-bit response echoes back, for tracing.
///
/// Not validated: the CRC-7 already covers this byte, so checking it separately adds
/// nothing against corruption, and the responses that carry no CRC carry no index either.
/// It is worth *logging*, because a stream that has slipped by a whole response shows up
/// here far more legibly than as a checksum failure.
pub fn echoed_index(rtn: &[u8]) -> u8 {
    if rtn.is_empty() { 0 } else { rtn[0] & 0x3F }
}

/// RX words one data block occupies, including its two words of CRC.
///
/// Both widths come out as "one word per eight SD clocks", which is not a coincidence:
/// `rd_data` autopushes at 32 bits and takes four bits per clock, while `rd_data_1bit`
/// autopushes at **8** bits and takes one. Choosing that threshold instead of 32 is what
/// makes the token accounting in [`crate::bus`] width-independent, and what keeps every
/// block size a whole number of pushes -- at a 32-bit threshold an 8-byte SCR read ends
/// with a 16-bit remainder that never autopushes at all.
pub const fn read_words(block_size: usize, width: BusWidth) -> usize {
    match width {
        // 2B data nibbles + 16 CRC nibbles, 8 nibbles to a word.
        BusWidth::W4 => block_size / 4 + 2,
        // B data bytes + 2 CRC bytes, one byte to a word.
        _ => block_size + 2,
    }
}

/// One received 4-bit data word, in the byte order the destination buffer wants.
///
/// `rd_data` shifts its ISR **left**, so the first nibble to arrive ends up at bits
/// [31:28] of the pushed word. Writing that word most-significant byte first therefore
/// puts nibbles 0 and 1 into byte 0, which is the wire order the whole crate works in.
///
/// Spelled as `to_be_bytes` rather than `swap_bytes().to_ne_bytes()` -- the two coincide
/// only on a little-endian host, and this is the conversion where getting it wrong
/// produces a buffer that is subtly, byte-reversibly wrong rather than obviously broken.
/// The DMA path gets the identical result from the hardware byte swap.
pub const fn rx_word_to_wire(word: u32) -> [u8; 4] {
    word.to_be_bytes()
}

/// Four wire-order bytes as a word for `wr_data`, the exact inverse of
/// [`rx_word_to_wire`].
///
/// `wr_data` shifts its OSR left from a 32-bit threshold, so the nibble it clocks out
/// first is bits [31:28] -- which must be the high nibble of the first wire byte.
pub const fn wire_to_tx_word(bytes: [u8; 4]) -> u32 {
    u32::from_be_bytes(bytes)
}

/// The `X` preload for `wr_data`: nibbles to clock out, minus one.
///
/// `8` for the start-token word, `2 * B` for the payload, `16` for the four lines' CRCs
/// and `1` for the stop bit. For a 512-byte block that is 1048, which is the literal
/// SdFat uses.
pub const fn write_x_register(block_size: usize) -> u32 {
    (8 + 2 * block_size + 16 + 1 - 1) as u32
}

/// Words to push to `wr_data`: the start token, the payload, two CRC words, the end token.
///
/// The state machine stops after `write_x_register + 1` nibbles and abandons the last
/// seven nibbles of the end-token word; the next block's FIFO clear discards them.
pub const fn write_words(block_size: usize) -> usize {
    block_size / 4 + 4
}

#[cfg(test)]
mod tests {
    use super::*;
    use sdio::{R2, Response};

    /// The frame must match what the SPI transport sends, since it is the same frame.
    #[test]
    fn command_frames_match_the_published_examples() {
        assert_eq!(command_frame(0, 0), [0x40, 0x00, 0x00, 0x00, 0x00, 0x95]);
        assert_eq!(command_frame(8, 0x1AA), [0x48, 0x00, 0x00, 0x01, 0xAA, 0x87]);
        assert_eq!(command_frame(17, 0), [0x51, 0x00, 0x00, 0x00, 0x00, 0x55]);
    }

    #[test]
    fn the_command_index_is_masked_into_six_bits() {
        // CMD55 with the transmission bit already set by a careless caller.
        assert_eq!(command_frame(55 | 0x40, 0)[0], 0x40 | 55);
    }

    /// Build a well-formed 48-bit response around a payload.
    fn r48(index: u8, payload: u32) -> [u8; 6] {
        let p = payload.to_be_bytes();
        let mut r = [index & 0x3F, p[0], p[1], p[2], p[3], 0];
        r[5] = (crc7(&r[..5]) << 1) | 1;
        r
    }

    #[test]
    fn a_48_bit_payload_lands_in_word_zero() {
        let words = pack_response(&r48(17, 0xDEAD_BEEF), true).unwrap();
        assert_eq!(words, [0xDEAD_BEEF, 0, 0, 0]);
    }

    #[test]
    fn a_corrupt_48_bit_response_is_rejected() {
        let mut r = r48(8, 0x0000_01AA);
        r[3] ^= 0x08;
        assert!(matches!(pack_response(&r, true), Err(MmcError::Crc)));
    }

    /// R3 and R4 carry no CRC, so the reserved bytes are the only framing check there is.
    #[test]
    fn an_ocr_response_is_checked_by_its_reserved_bytes() {
        let good = [0x3F, 0xC0, 0xFF, 0x80, 0x00, 0xFF];
        assert_eq!(pack_response(&good, false).unwrap()[0], 0xC0FF_8000);

        let mut bad_header = good;
        bad_header[0] = 0x3E;
        assert!(matches!(pack_response(&bad_header, false), Err(MmcError::Crc)));

        let mut bad_tail = good;
        bad_tail[5] = 0xFE;
        assert!(matches!(pack_response(&bad_tail, false), Err(MmcError::Crc)));
    }

    /// Build a well-formed 136-bit response around 15 bytes of register content.
    fn r136(payload: [u8; 15]) -> [u8; 17] {
        let mut r = [0u8; 17];
        r[0] = 0x3F;
        r[1..16].copy_from_slice(&payload);
        r[16] = (crc7(&payload) << 1) | 1;
        r
    }

    /// The test that would have caught the packing bug this workspace already shipped once.
    ///
    /// Goes all the way through the real `sdio` types rather than asserting on `words`
    /// directly, because the contract being pinned is `sdio`'s reassembly, not ours: a
    /// CSD whose first transmitted byte says "version 2" must decode as version 2.
    #[test]
    fn r136_word_order_survives_a_round_trip_through_sdio() {
        // CSD_STRUCTURE = 0b01 (v2, SDHC/SDXC) in the top two bits of the first byte.
        let mut payload = [0u8; 15];
        payload[0] = 0b0100_0000;
        // A recognisable value further down, to catch a rotation as well as a reversal.
        payload[7] = 0xA5;

        let words = pack_response(&r136(payload), true).unwrap();
        let csd = sdio::common::CSD::<()>::from(R2::from_words(&words));
        assert_eq!(csd.version(), 1, "words {words:08x?}");

        // And the reverse packing must *not* decode as v2, or the assertion above is
        // satisfied by accident rather than by the word order.
        let reversed = [words[3], words[2], words[1], words[0]];
        let wrong = sdio::common::CSD::<()>::from(R2::from_words(&reversed));
        assert_ne!(wrong.version(), 1);
    }

    #[test]
    fn a_136_bit_response_is_rejected_on_a_bad_header_or_crc() {
        let good = r136([0x40; 15]);

        let mut bad_header = good;
        bad_header[0] = 0x00;
        assert!(matches!(pack_response(&bad_header, true), Err(MmcError::Crc)));

        let mut bad_crc = good;
        bad_crc[8] ^= 0x01;
        assert!(matches!(pack_response(&bad_crc, true), Err(MmcError::Crc)));
    }

    #[test]
    fn a_command_with_no_response_packs_to_zeros() {
        assert_eq!(pack_response(&[], true).unwrap(), [0; 4]);
    }

    #[test]
    fn response_lengths_are_the_native_bus_lengths() {
        assert_eq!(response_bytes(ResponseLen::Zero), 0);
        assert_eq!(response_bytes(ResponseLen::R48), 6);
        assert_eq!(response_bytes(ResponseLen::R136), 17);

        assert_eq!(y_register(0), 0);
        assert_eq!(y_register(6), 47);
        assert_eq!(y_register(17), 135);
    }

    /// The conversion the whole 4-bit data path hinges on.
    ///
    /// Nibble 0 -- the first thing on the wire -- must survive as the *high* nibble of
    /// byte 0. Anything else and the CRC transposition reads the lines in the wrong order
    /// while the data itself still looks plausible.
    #[test]
    fn a_received_word_lands_in_the_buffer_in_wire_order() {
        // Nibbles 0..7 are 1,2,3,4,5,6,7,8 in the order they arrived.
        assert_eq!(rx_word_to_wire(0x1234_5678), [0x12, 0x34, 0x56, 0x78]);
        // The first nibble alone: it must reach byte 0's high half, not byte 3's low half.
        assert_eq!(rx_word_to_wire(0xF000_0000), [0xF0, 0x00, 0x00, 0x00]);
        // ...and DAT0's bit of the first nibble specifically.
        assert_eq!(rx_word_to_wire(0x1000_0000), [0x10, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn the_transmit_word_is_the_exact_inverse() {
        for w in [0u32, 0x1234_5678, 0xFFFF_FFF0, 0xDEAD_BEEF, u32::MAX] {
            assert_eq!(wire_to_tx_word(rx_word_to_wire(w)), w, "{w:#010x}");
        }
        // The start token has to clock out as seven idle nibbles then a zero start bit,
        // so its wire bytes are FF FF FF F0 and not the reverse.
        assert_eq!(rx_word_to_wire(WRITE_START_TOKEN), [0xFF, 0xFF, 0xFF, 0xF0]);
    }

    /// Every block size the `sdio` crate actually asks this bus for.
    #[test]
    fn transfer_arithmetic_covers_every_block_size_in_use() {
        // 512-byte sector: SdFat's own figure of 130 words.
        assert_eq!(read_words(512, BusWidth::W4), 130);
        // 64-byte SD Status / CMD6 switch response.
        assert_eq!(read_words(64, BusWidth::W4), 18);
        // 8-byte SCR -- and the 1-bit form, which is the one ACMD51 actually uses,
        // because `Card::acquire` reads the SCR before it sets 4-bit width.
        assert_eq!(read_words(8, BusWidth::W4), 4);
        assert_eq!(read_words(8, BusWidth::W1), 10);

        // The literal SdFat writes into X for a 512-byte block.
        assert_eq!(write_x_register(512), 1048);
        assert_eq!(write_words(512), 132);
    }

    /// The nibble count and the word count have to describe the same transfer, or the
    /// state machine stops mid-block or waits for a word that never comes.
    #[test]
    fn the_write_nibble_count_matches_the_words_pushed() {
        for block in [8usize, 64, 512] {
            let nibbles = write_x_register(block) as usize + 1;
            // 8 start + 2B payload + 16 CRC + 1 stop.
            assert_eq!(nibbles, 8 + 2 * block + 16 + 1);
            // The words carry at least those nibbles, with the end token part-consumed.
            let pushed_nibbles = write_words(block) * 8;
            assert!(pushed_nibbles >= nibbles, "block {block}");
            assert!(pushed_nibbles - nibbles < 8, "block {block}: over-pushed a word");
        }
    }
}
