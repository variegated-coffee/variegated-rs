//! Bytes that arrived ahead of the frames they belong to.
//!
//! A WebSocket upgrade is one TCP stream carrying two things in sequence: an HTTP response head,
//! and then frames. Nothing makes the server's first frame arrive in a *later* segment than the
//! head — and Plantlet drains any queued routine push from an alarm armed the moment the socket
//! opens, so the first frame is often sent immediately after the 101 and coalesced with it.
//!
//! A reader that finds the end of the head and then goes back to the socket for frames therefore
//! discards whatever followed the head in that same read. The frame is simply gone: the push is
//! recorded as sent on the server and never applied on the machine.
//!
//! This is the buffer that stops that happening. It doubles as the head buffer, so the leftover
//! never has to be copied anywhere: the reader fills it, says where the head ended, and what
//! remains is handed out ahead of anything read from the socket afterwards.
//!
//! It lives here rather than beside the socket code for the reason the Noise session does: the
//! comms firmware sets `harness = false` and runs no tests, so a buffer with an index in it
//! would be permanently unverified there. Everything here is pure.

/// Room for one response head.
///
/// Sized for a 101 that has been through a CDN rather than one the origin would send alone.
/// Cloudflare adds `Report-To` — around 330 bytes by itself — plus `Nel`, `CF-RAY`, `Server`,
/// `alt-svc` and `Date`, on top of the upgrade headers and the Noise handshake header. Measured
/// at roughly 690 bytes against a deployed Worker. An earlier 512 fit a direct connection and
/// nothing else, and a machine that could not fit the head never completed a handshake at all.
pub const HEAD_LEN: usize = 2048;

/// A buffer holding what has been read but not yet consumed.
pub struct Pending {
    buf: [u8; HEAD_LEN],
    head: usize,
    tail: usize,
}

impl Default for Pending {
    fn default() -> Self {
        Self::new()
    }
}

impl Pending {
    pub const fn new() -> Self {
        Self {
            buf: [0; HEAD_LEN],
            head: 0,
            tail: 0,
        }
    }

    /// The whole buffer, to be filled by a reader.
    pub fn spare(&mut self) -> &mut [u8; HEAD_LEN] {
        &mut self.buf
    }

    /// What has been read so far, for a reader looking for the end of a head.
    pub fn filled(&self, to: usize) -> &[u8] {
        &self.buf[..to]
    }

    /// Keep `filled - from` bytes, starting at `from`, and discard everything before it.
    ///
    /// Called once the head has been parsed: `from` is where the head ended and `filled` is how
    /// much had been read, so what is kept is exactly the frame bytes that shared the head's
    /// last segment.
    pub fn keep(&mut self, from: usize, filled: usize) {
        debug_assert!(from <= filled && filled <= HEAD_LEN);
        self.head = from;
        self.tail = filled;
    }

    /// Whether anything is waiting to be consumed before the socket is read again.
    pub fn is_empty(&self) -> bool {
        self.head >= self.tail
    }

    /// How many bytes are waiting.
    pub fn len(&self) -> usize {
        self.tail.saturating_sub(self.head)
    }

    /// Fill as much of `out` as is waiting, and report how much that was.
    ///
    /// Short reads are the normal case rather than an error: the leftover is whatever happened to
    /// share a segment with the head, so it can be a whole frame, part of one, or nothing. The
    /// caller reads the balance from the socket.
    pub fn take(&mut self, out: &mut [u8]) -> usize {
        let n = out.len().min(self.len());
        out[..n].copy_from_slice(&self.buf[self.head..self.head + n]);
        self.head += n;
        n
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The head is dropped and only what followed it is kept.
    #[test]
    fn keeps_what_followed_the_head() {
        let mut pending = Pending::new();
        pending.spare()[..8].copy_from_slice(b"HEAD\x82\x01\xff\x00");
        pending.keep(4, 8);

        assert_eq!(pending.len(), 4);
        let mut out = [0u8; 4];
        assert_eq!(pending.take(&mut out), 4);
        assert_eq!(&out, b"\x82\x01\xff\x00");
        assert!(pending.is_empty());
    }

    /// A head that used the whole read leaves nothing behind, and that is not an error.
    #[test]
    fn a_head_with_nothing_after_it_leaves_nothing() {
        let mut pending = Pending::new();
        pending.keep(690, 690);

        assert!(pending.is_empty());
        assert_eq!(pending.take(&mut [0u8; 4]), 0);
    }

    /// The reason this type exists: a caller asking for more than is waiting gets a short answer
    /// rather than a wrong one, and knows to read the balance from the socket.
    #[test]
    fn a_partial_frame_is_reported_short_rather_than_padded() {
        let mut pending = Pending::new();
        pending.spare()[..6].copy_from_slice(b"..\x01\x02\x03\x04");
        pending.keep(2, 6);

        let mut out = [0xAAu8; 8];
        assert_eq!(pending.take(&mut out), 4);
        assert_eq!(&out[..4], b"\x01\x02\x03\x04");
        // Untouched, so the caller can read straight into the tail.
        assert_eq!(&out[4..], &[0xAA; 4]);
        assert!(pending.is_empty());
    }

    /// Consumed across several reads, the way a frame header and then its payload would be.
    #[test]
    fn drains_across_successive_takes() {
        let mut pending = Pending::new();
        pending.spare()[..5].copy_from_slice(b"\x82\x03abc");
        pending.keep(0, 5);

        let mut header = [0u8; 2];
        assert_eq!(pending.take(&mut header), 2);
        assert_eq!(&header, b"\x82\x03");

        let mut payload = [0u8; 3];
        assert_eq!(pending.take(&mut payload), 3);
        assert_eq!(&payload, b"abc");

        assert!(pending.is_empty());
        assert_eq!(pending.len(), 0);
    }

    /// A fresh buffer has nothing in it, so the first read goes to the socket.
    #[test]
    fn starts_empty() {
        let mut pending = Pending::new();
        assert!(pending.is_empty());
        assert_eq!(pending.len(), 0);
        assert_eq!(pending.take(&mut [0u8; 1]), 0);
    }
}
