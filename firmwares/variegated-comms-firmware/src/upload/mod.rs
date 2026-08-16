//! Uploading finished shot logs to a remote HTTPS endpoint.
//!
//! The application processor holds the endpoint and token and pushes them over the link
//! (`channels::SHOT_UPLOAD_CONFIG`); this side does the network.
//!
//! # Status
//!
//! The TLS half is here and links. The streaming upload -- pulling a shot 1 kB at a time
//! over the UART link and writing it into the session with an exact `Content-Length` -- is
//! not yet written. See the plan.

pub mod roots;
pub mod tls;

