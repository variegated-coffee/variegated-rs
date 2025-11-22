//! HTTP server functionality

use core::fmt::{Debug, Display};
use core::net::SocketAddr;

use defmt::{error, info};
use edge_http::io::server::{Connection as ServerConnection, DefaultServer, Handler};
use edge_http::io::Error;
use edge_http::Method;
use edge_nal::TcpBind;
use edge_nal_embassy::Tcp;
use embedded_io_async::{Read, Write};

/// HTTP request handler
pub struct HttpHandler;

impl Handler for HttpHandler {
    type Error<E> = Error<E>
    where
        E: Debug;

    async fn handle<T, const N: usize>(
        &self,
        _task_id: impl Display + Copy,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Self::Error<T::Error>>
    where
        T: Read + Write,
    {
        let headers = conn.headers()?;
        let is_root = headers.method == Method::Get && headers.path == "/";

        if is_root {
            // Respond with "Hello World"
            conn.initiate_response(200, Some("OK"), &[("Content-Type", "text/plain")])
                .await?;
            conn.write_all(b"Hello World").await?;
            info!("Served Hello World to client");
        } else {
            // Return 404 for other paths
            conn.initiate_response(404, Some("Not Found"), &[]).await?;
            info!("Returned 404 for non-root path");
        }

        Ok(())
    }
}

/// HTTP server task
#[embassy_executor::task]
pub async fn http_server_task(tcp_stack: &'static Tcp<'static, 8, 1024, 1024>) {
    info!("Starting HTTP server on port 80...");

    let mut server = DefaultServer::new();

    let bind_addr = SocketAddr::from(([0, 0, 0, 0], 80));

    match tcp_stack.bind(bind_addr).await {
        Ok(acceptor) => {
            info!("HTTP server bound to port 80");
            if let Err(e) = server.run(None, acceptor, HttpHandler).await {
                error!("HTTP server error: {:?}", e);
            }
        }
        Err(e) => {
            error!("Failed to bind HTTP server to port 80: {:?}", e);
        }
    }
}
