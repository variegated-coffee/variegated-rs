/**
 * CRC-32/ISCSI, also called CRC-32C or Castagnoli.
 *
 * This is the checksum the machine writes. A routine — in flash, and on the wire when the
 * machine serves a definition — is `postcard::to_slice_crc32(&Routine, ...)`: the postcard
 * encoding followed by the checksum as four little-endian bytes. There is no header, no
 * magic and no container; the trailer is the entire framing.
 *
 * Parameters, since "CRC-32" alone names a family rather than an algorithm: reflected
 * polynomial `0x82F63B78`, initial value and final XOR both `0xFFFFFFFF`, input and output
 * reflected. Getting any one of those wrong yields a checksum that is stable, plausible, and
 * never matches.
 *
 * Duplicated from `@variegated-coffee/shot-log`'s copy rather than shared, because this
 * bundle is compressed into the firmware image and depends only on `serde-postcard-ts`, the
 * design system and preact. Thirty lines is the cheaper side of that trade.
 */
const TABLE = buildTable();

function buildTable(): Uint32Array {
  const table = new Uint32Array(256);
  for (let i = 0; i < 256; i++) {
    let c = i;
    for (let bit = 0; bit < 8; bit++) {
      c = c & 1 ? (c >>> 1) ^ 0x82f63b78 : c >>> 1;
    }
    table[i] = c >>> 0;
  }
  return table;
}

/** The CRC-32C of `bytes`, as an unsigned 32-bit number. */
export function crc32c(bytes: Uint8Array): number {
  let crc = 0xffffffff;
  for (const byte of bytes) {
    crc = TABLE[(crc ^ byte) & 0xff] ^ (crc >>> 8);
  }
  return (crc ^ 0xffffffff) >>> 0;
}

/** Read a little-endian `u32` at `offset`. */
export function readU32LE(bytes: Uint8Array, offset: number): number {
  return (
    ((bytes[offset] | (bytes[offset + 1] << 8) | (bytes[offset + 2] << 16)) |
      (bytes[offset + 3] << 24)) >>>
    0
  );
}

/** Length of the CRC-32C trailer a routine carries. */
export const CRC_TRAILER_BYTES = 4;
