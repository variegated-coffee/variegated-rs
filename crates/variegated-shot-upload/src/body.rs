//! Framing the upload request, and streaming the shot into it.
//!
//! # The promise this module has to keep
//!
//! The request declares an exact `Content-Length` taken from the first chunk, and after that
//! it is committed: the server will read exactly that many bytes and wait for them. Every
//! invariant here exists to make sure the body matches the promise, or that the attempt
//! fails in a way the caller can turn into a connection reset rather than a half-written
//! upload the server is still waiting on.
//!
//! Nothing here buffers the shot. A shot is tens of kilobytes and the device that sends it
//! has neither the RAM nor the storage -- chunks arrive from a [`ChunkSource`] and go
//! straight out.

use alloc::vec::Vec;
use core::fmt::Write as _;

use embedded_io_async::Write;
use variegated_controller_types::shot_log::ShotLogId;

/// One slice of a shot, as the storage side hands it over.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Chunk {
    /// Which shot this is a slice of, echoed back so the caller can detect a crossed reply.
    pub id: ShotLogId,
    /// Where this slice starts. Echoed for the same reason.
    pub offset: u32,
    /// The full encoded length of the shot. Constant across chunks; the first one is what
    /// becomes `Content-Length`.
    pub total: u32,
    /// Whether this is the final slice.
    pub last: bool,
    pub bytes: Vec<u8>,
}

/// Somewhere shot bytes come from, one slice at a time.
///
/// A trait rather than a concrete type because the storage is on the *other* processor: the
/// firmware's implementation is a request/reply round trip over a UART link. It is also what
/// makes the framing below testable without any of that.
#[allow(async_fn_in_trait)]
pub trait ChunkSource {
    /// Fetch the slice starting at `offset`.
    async fn chunk(&mut self, id: ShotLogId, offset: u32) -> Option<Chunk>;
}

/// Why the body could not be written as promised.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BodyError {
    /// The source stopped answering, or answered with something other than what was asked
    /// for. **The caller must reset the connection, not close it gracefully**: the request
    /// has an exact `Content-Length` on the wire, and a clean close after a short body
    /// leaves the server waiting for bytes that are never coming.
    Source,
    /// The stream would not take the bytes.
    Write,
}

/// The request head, up to and including the blank line.
///
/// Separate from the send so it can be asserted on directly -- the `Authorization` header
/// and the exact `Content-Length` are the two things most worth pinning.
pub fn request_head(
    path: &str,
    host: &str,
    token: &str,
    content_length: u32,
) -> heapless::String<512> {
    let mut head = heapless::String::new();
    // `Connection: close` because we upload one shot per connection and never read the
    // response body -- which is what makes draining it unnecessary.
    let _ = write!(
        head,
        "POST {path} HTTP/1.1\r\n\
         Host: {host}\r\n\
         Authorization: Bearer {token}\r\n\
         Content-Type: application/octet-stream\r\n\
         Content-Length: {content_length}\r\n\
         Connection: close\r\n\
         \r\n"
    );
    head
}

/// Write the head, then pump the shot into `stream` chunk by chunk.
///
/// `first` must be the chunk at offset 0, already fetched: its `total` is the
/// `Content-Length`, and fetching it **before** anything is written is what lets a missing
/// shot be a clean failure rather than a half-sent request. The caller does that fetch so it
/// can distinguish "no such shot" from "the link died", which are different outcomes.
pub async fn send<S, W>(
    stream: &mut W,
    source: &mut S,
    id: ShotLogId,
    first: Chunk,
    head: &str,
) -> Result<(), BodyError>
where
    S: ChunkSource,
    W: Write,
{
    stream.write_all(head.as_bytes()).await.map_err(|_| BodyError::Write)?;

    let total = first.total;
    let mut written = first.bytes.len() as u32;
    let mut last = first.last;
    stream.write_all(&first.bytes).await.map_err(|_| BodyError::Write)?;

    while !last {
        let chunk = source.chunk(id, written).await.ok_or(BodyError::Source)?;

        // The echo check. The link this runs over has no correlation id -- a lock keeps one
        // request in flight -- so this should be impossible; it is checked anyway because
        // splicing another shot's bytes into this upload is a corruption no consumer could
        // detect. The CRC would fail on a file that is structurally perfect, and nothing
        // would point at where the bytes came from.
        if chunk.id != id || chunk.offset != written {
            return Err(BodyError::Source);
        }
        // An empty chunk before `last` would loop forever at the same offset.
        if chunk.bytes.is_empty() {
            return Err(BodyError::Source);
        }
        // Past `Content-Length` means the shot grew under us or the far side is confused.
        // Either way the promise already on the wire cannot be kept.
        if written as u64 + chunk.bytes.len() as u64 > total as u64 {
            return Err(BodyError::Source);
        }

        stream.write_all(&chunk.bytes).await.map_err(|_| BodyError::Write)?;
        written += chunk.bytes.len() as u32;
        last = chunk.last;
    }

    // `last` arriving early is the mirror of the overrun check above.
    if written != total {
        return Err(BodyError::Source);
    }

    stream.flush().await.map_err(|_| BodyError::Write)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use embassy_futures::block_on;

    fn id() -> ShotLogId {
        ShotLogId { day: Some(20260816), time: 13542656 }
    }

    /// Collects everything written, so a test can assert on the exact bytes on the wire.
    #[derive(Default)]
    struct Sink {
        written: Vec<u8>,
        flushed: bool,
    }

    impl embedded_io_async::ErrorType for Sink {
        type Error = core::convert::Infallible;
    }

    impl Write for Sink {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.written.extend_from_slice(buf);
            Ok(buf.len())
        }
        async fn flush(&mut self) -> Result<(), Self::Error> {
            self.flushed = true;
            Ok(())
        }
    }

    /// Serves a shot out of one buffer, `chunk_len` at a time.
    struct Source {
        data: Vec<u8>,
        chunk_len: usize,
        /// Corrupts the offset echoed on the Nth call, to exercise the mismatch check.
        lie_about_offset_on: Option<u32>,
        /// Stops answering after this many calls.
        die_after: Option<u32>,
        calls: u32,
    }

    impl Source {
        fn new(len: usize, chunk_len: usize) -> Self {
            Self {
                data: (0..len).map(|i| (i % 251) as u8).collect(),
                chunk_len,
                lie_about_offset_on: None,
                die_after: None,
                calls: 0,
            }
        }

        fn first(&mut self) -> Chunk {
            block_on(self.chunk(id(), 0)).expect("chunk zero always exists")
        }
    }

    impl ChunkSource for Source {
        async fn chunk(&mut self, id: ShotLogId, offset: u32) -> Option<Chunk> {
            self.calls += 1;
            if let Some(n) = self.die_after
                && self.calls > n
            {
                return None;
            }

            let start = offset as usize;
            let end = (start + self.chunk_len).min(self.data.len());
            let echoed = if self.lie_about_offset_on == Some(offset) { offset + 1 } else { offset };

            Some(Chunk {
                id,
                offset: echoed,
                total: self.data.len() as u32,
                last: end >= self.data.len(),
                bytes: self.data[start..end].to_vec(),
            })
        }
    }

    fn run(mut source: Source) -> (Result<(), BodyError>, Sink, Vec<u8>) {
        let first = source.first();
        let expected = source.data.clone();
        let head = request_head("/api/shots", "h.example", "tok", first.total);
        let mut sink = Sink::default();
        let result = block_on(send(&mut sink, &mut source, id(), first, &head));
        (result, sink, expected)
    }

    #[test]
    fn the_head_declares_the_exact_length_and_carries_the_token() {
        let head = request_head("/api/shots?src=m1", "h.example", "sekrit", 51291);
        assert!(head.contains("POST /api/shots?src=m1 HTTP/1.1\r\n"));
        assert!(head.contains("Host: h.example\r\n"));
        assert!(head.contains("Authorization: Bearer sekrit\r\n"));
        assert!(head.contains("Content-Length: 51291\r\n"));
        assert!(head.contains("Connection: close\r\n"));
        // The blank line, without which the server keeps reading headers forever.
        assert!(head.ends_with("\r\n\r\n"));
    }

    #[test]
    fn the_body_is_exactly_the_shot_and_exactly_content_length() {
        // The whole contract in one assertion: what the head promised is what arrived, and
        // the bytes are the shot's, in order, unaltered.
        let (result, sink, expected) = run(Source::new(51291, 1024));
        assert_eq!(result, Ok(()));

        let split = sink.written.windows(4).position(|w| w == b"\r\n\r\n").unwrap() + 4;
        let (head, body) = sink.written.split_at(split);

        assert!(core::str::from_utf8(head).unwrap().contains("Content-Length: 51291\r\n"));
        assert_eq!(body.len(), 51291);
        assert_eq!(body, &expected[..]);
        assert!(sink.flushed);
    }

    #[test]
    fn a_shot_that_fits_in_one_chunk_still_works() {
        // The `last` flag on the very first chunk, which skips the loop entirely.
        let (result, sink, expected) = run(Source::new(600, 1024));
        assert_eq!(result, Ok(()));
        assert!(sink.written.ends_with(&expected));
    }

    #[test]
    fn a_shot_whose_length_is_a_chunk_multiple_works() {
        // The off-by-one that a length not divisible by the chunk size would hide.
        let (result, _, expected) = run(Source::new(4096, 1024));
        assert_eq!(result, Ok(()));
        assert_eq!(expected.len(), 4096);
    }

    #[test]
    fn a_link_that_stops_answering_is_an_error_not_a_short_body() {
        // The case that must never be reported as success: the server would be left waiting
        // for the rest of a Content-Length it was promised.
        let mut source = Source::new(51291, 1024);
        source.die_after = Some(3);
        let (result, _, _) = run(source);
        assert_eq!(result, Err(BodyError::Source));
    }

    #[test]
    fn a_chunk_for_the_wrong_offset_is_refused() {
        let mut source = Source::new(51291, 1024);
        source.lie_about_offset_on = Some(1024);
        let (result, _, _) = run(source);
        assert_eq!(result, Err(BodyError::Source));
    }

    #[test]
    fn a_stream_that_refuses_bytes_is_a_write_error() {
        // Distinct from `Source` so the caller can tell a dead network from a dead link;
        // they retry the same way but read very differently in a log.
        struct Broken;
        impl embedded_io_async::ErrorType for Broken {
            type Error = embedded_io_async::ErrorKind;
        }
        impl Write for Broken {
            async fn write(&mut self, _: &[u8]) -> Result<usize, Self::Error> {
                Err(embedded_io_async::ErrorKind::BrokenPipe)
            }
            async fn flush(&mut self) -> Result<(), Self::Error> {
                Ok(())
            }
        }

        let mut source = Source::new(4096, 1024);
        let first = source.first();
        let head = request_head("/", "h", "t", first.total);
        let result = block_on(send(&mut Broken, &mut source, id(), first, &head));
        assert_eq!(result, Err(BodyError::Write));
    }
}
