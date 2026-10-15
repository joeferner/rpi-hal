//! A TLS stream over an `embassy-net` TCP socket.
//!
//! This is the bridge between two worlds that do not natively meet: `rustls`
//! is sans-IO and synchronous, and `embassy-net` is async. `rustls`'s usual
//! `Connection`/`Reader`/`Writer` types are `std`-only, so the connection is
//! driven through the `unbuffered` API instead — a state machine that hands
//! back "encode these bytes", "transmit what you have", or "I need more
//! input", and leaves every byte of actual IO to the caller.
//!
//! Driving that state machine against an async socket is the whole of this
//! module.
//!
//! The result implements `embedded-io-async`'s `Read` and `Write`, so
//! anything written against those traits — an HTTP client, for instance —
//! runs over TLS without knowing it.

use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;

use embassy_net::tcp::TcpSocket;
use embedded_io_async::{ErrorType, Read, Write};
use rustls::client::UnbufferedClientConnection;
use rustls::pki_types::ServerName;
use rustls::unbuffered::{ConnectionState, EncodeError, EncryptError, UnbufferedStatus};
use rustls::{ClientConfig, ProtocolVersion, SupportedCipherSuite};

/// Starting size for the record buffers.
///
/// A TLS 1.3 record tops out around 16 KiB plus framing. Both buffers grow
/// if `rustls` asks for more, so this is a starting point rather than a
/// limit — but it is sized to cover the common case without a reallocation,
/// since the certificate chain arriving during a handshake is the largest
/// thing either buffer normally sees.
const BUFFER_SIZE: usize = 18 * 1024;

/// What went wrong on a TLS stream.
#[derive(Debug)]
pub enum Error {
    /// The peer name was not a valid DNS name or IP address.
    InvalidServerName,
    /// `rustls` rejected the connection. Certificate verification failures
    /// arrive here, which is the point of the exercise: a handshake against
    /// an untrusted, expired or wrongly-named certificate must fail rather
    /// than proceed.
    Tls(rustls::Error),
    /// The TCP socket failed underneath.
    Socket,
    /// The peer closed the connection in the middle of the handshake.
    UnexpectedClose,
}

/// One line, for the console.
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::InvalidServerName => write!(f, "invalid server name"),
            Error::Tls(e) => write!(f, "TLS error: {e}"),
            Error::Socket => write!(f, "TCP socket error"),
            Error::UnexpectedClose => write!(f, "peer closed unexpectedly"),
        }
    }
}

// `embedded-io` 0.7 requires its error types to be `core::error::Error`,
// which in turn needs `Display` above.
impl core::error::Error for Error {}

/// Every failure is `Other`: none of them maps onto an `std::io` kind a
/// caller could act on differently.
impl embedded_io_async::Error for Error {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        embedded_io_async::ErrorKind::Other
    }
}

/// A TLS connection, wrapping a connected TCP socket.
pub struct TlsStream<'s> {
    /// The `rustls` state machine.
    connection: UnbufferedClientConnection,
    /// The transport. Already connected before the handshake starts.
    socket: TcpSocket<'s>,
    /// TLS records received but not yet consumed by `rustls`.
    incoming: Vec<u8>,
    /// How much of `incoming` holds real data.
    incoming_used: usize,
    /// TLS records to hand to the socket.
    outgoing: Vec<u8>,
    /// How much of `outgoing` holds real data.
    outgoing_used: usize,
    /// Decrypted application data not yet returned to the caller.
    plaintext: Vec<u8>,
    /// Read cursor into `plaintext`.
    plaintext_read: usize,
    /// Set once the peer has closed its side.
    peer_closed: bool,
}

impl<'s> TlsStream<'s> {
    /// Performs the TLS handshake over an already-connected socket.
    ///
    /// Returns once the connection is ready to carry application data,
    /// which means the server's certificate chain has been verified against
    /// the configured trust anchors, its name against `server_name`, and its
    /// validity against the clock.
    pub async fn connect(
        socket: TcpSocket<'s>,
        config: Arc<ClientConfig>,
        server_name: &str,
    ) -> Result<Self, Error> {
        let name = ServerName::try_from(server_name)
            .map_err(|_| Error::InvalidServerName)?
            .to_owned();
        let connection = UnbufferedClientConnection::new(config, name).map_err(Error::Tls)?;

        let mut stream = Self {
            connection,
            socket,
            incoming: vec![0; BUFFER_SIZE],
            incoming_used: 0,
            outgoing: vec![0; BUFFER_SIZE],
            outgoing_used: 0,
            plaintext: Vec::new(),
            plaintext_read: 0,
            peer_closed: false,
        };

        stream.drive(false).await?;
        Ok(stream)
    }

    /// The protocol version the handshake settled on.
    pub fn protocol_version(&self) -> Option<ProtocolVersion> {
        self.connection.protocol_version()
    }

    /// The cipher suite the handshake settled on.
    pub fn cipher_suite(&self) -> Option<SupportedCipherSuite> {
        self.connection.negotiated_cipher_suite()
    }

    /// Hands back the TCP socket, for a caller that wants to close it
    /// itself. The TLS session is abandoned without a `close_notify`.
    pub fn into_socket(self) -> TcpSocket<'s> {
        self.socket
    }

    /// Runs the state machine until it is blocked on the caller.
    ///
    /// With `want_data` false this returns as soon as the connection can
    /// carry application data — which during [`connect`](Self::connect)
    /// means the handshake is complete. With it true, it additionally keeps
    /// going until at least one application record has been decrypted or
    /// the peer has closed.
    ///
    /// The loop is the heart of the module. Each pass asks `rustls` what it
    /// wants next and does exactly that one thing, because every branch can
    /// change what the next answer will be.
    async fn drive(&mut self, want_data: bool) -> Result<(), Error> {
        loop {
            // Disjoint field borrows: `state` borrows `connection` and
            // `incoming`, which leaves `socket` and `outgoing` free to use
            // inside the match. Borrowing `self` as a whole would not.
            let Self {
                connection,
                socket,
                incoming,
                incoming_used,
                outgoing,
                outgoing_used,
                plaintext,
                peer_closed,
                ..
            } = self;

            let UnbufferedStatus { discard, state } =
                connection.process_tls_records(&mut incoming[..*incoming_used]);

            let mut needs_input = false;
            let mut done = false;

            match state.map_err(Error::Tls)? {
                // `rustls` has bytes to send. Append them to the outgoing
                // buffer; they go out on the next Transmit.
                ConnectionState::EncodeTlsData(mut encoder) => loop {
                    match encoder.encode(&mut outgoing[*outgoing_used..]) {
                        Ok(written) => {
                            *outgoing_used += written;
                            break;
                        }
                        Err(EncodeError::InsufficientSize(required)) => {
                            // Grow and retry rather than failing: the
                            // buffer is a starting guess, not a contract.
                            outgoing.resize(*outgoing_used + required.required_size, 0);
                        }
                        Err(e) => {
                            return Err(Error::Tls(rustls::Error::General(alloc::format!(
                                "TLS encode failed: {e:?}"
                            ))));
                        }
                    }
                },

                // Everything buffered has to reach the peer before the
                // handshake can move on.
                ConnectionState::TransmitTlsData(transmit) => {
                    socket
                        .write_all(&outgoing[..*outgoing_used])
                        .await
                        .map_err(|_| Error::Socket)?;
                    *outgoing_used = 0;
                    transmit.done();
                }

                // `rustls` needs more bytes from the peer.
                ConnectionState::BlockedHandshake => needs_input = true,

                // Decrypted application data. Copied out because the record
                // borrows the incoming buffer, which is about to be shifted.
                ConnectionState::ReadTraffic(mut traffic) => {
                    while let Some(record) = traffic.next_record() {
                        let record = record.map_err(Error::Tls)?;
                        plaintext.extend_from_slice(record.payload);
                    }
                    done = true;
                }

                // Handshake complete and no data pending. For `connect` that
                // is the finish line; for a read it means waiting.
                ConnectionState::WriteTraffic(_) => {
                    if want_data {
                        needs_input = true;
                    } else {
                        done = true;
                    }
                }

                ConnectionState::PeerClosed | ConnectionState::Closed => {
                    *peer_closed = true;
                    done = true;
                }

                // Early data is never sent, so the peer cannot be reading it.
                other => {
                    return Err(Error::Tls(rustls::Error::General(alloc::format!(
                        "unexpected TLS state: {other:?}"
                    ))));
                }
            }

            // Drop what `rustls` consumed, keeping any partial record.
            if discard != 0 {
                incoming.copy_within(discard..*incoming_used, 0);
                *incoming_used -= discard;
            }

            if needs_input {
                if *incoming_used == incoming.len() {
                    // A record larger than the buffer: make room rather than
                    // deadlocking on a full buffer that `rustls` cannot yet
                    // parse.
                    incoming.resize(incoming.len() * 2, 0);
                }
                let read = socket
                    .read(&mut incoming[*incoming_used..])
                    .await
                    .map_err(|_| Error::Socket)?;
                if read == 0 {
                    if want_data {
                        *peer_closed = true;
                        return Ok(());
                    }
                    return Err(Error::UnexpectedClose);
                }
                *incoming_used += read;
                continue;
            }

            if done {
                return Ok(());
            }
        }
    }

    /// Sends everything buffered for transmission.
    async fn flush_outgoing(&mut self) -> Result<(), Error> {
        if self.outgoing_used == 0 {
            return Ok(());
        }
        self.socket
            .write_all(&self.outgoing[..self.outgoing_used])
            .await
            .map_err(|_| Error::Socket)?;
        self.outgoing_used = 0;
        Ok(())
    }
}

/// [`Error`], for every operation.
impl ErrorType for TlsStream<'_> {
    type Error = Error;
}

/// Decrypted application data, as the peer sent it.
impl Read for TlsStream<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        // Serve from what has already been decrypted before touching the
        // socket again.
        if self.plaintext_read == self.plaintext.len() {
            self.plaintext.clear();
            self.plaintext_read = 0;

            if self.peer_closed {
                return Ok(0);
            }
            self.drive(true).await?;

            if self.plaintext.is_empty() {
                // Closed with nothing further to give.
                return Ok(0);
            }
        }

        let available = &self.plaintext[self.plaintext_read..];
        let take = available.len().min(buf.len());
        buf[..take].copy_from_slice(&available[..take]);
        self.plaintext_read += take;
        Ok(take)
    }
}

/// Application data, encrypted and sent as one record per call.
impl Write for TlsStream<'_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Error> {
        {
            let Self {
                connection,
                incoming,
                incoming_used,
                outgoing,
                outgoing_used,
                ..
            } = self;

            let UnbufferedStatus { discard, state } =
                connection.process_tls_records(&mut incoming[..*incoming_used]);

            match state.map_err(Error::Tls)? {
                ConnectionState::WriteTraffic(mut writer) => loop {
                    match writer.encrypt(buf, &mut outgoing[*outgoing_used..]) {
                        Ok(written) => {
                            *outgoing_used += written;
                            break;
                        }
                        Err(EncryptError::InsufficientSize(required)) => {
                            outgoing.resize(*outgoing_used + required.required_size, 0);
                        }
                        Err(e) => {
                            return Err(Error::Tls(rustls::Error::General(alloc::format!(
                                "TLS encrypt failed: {e:?}"
                            ))));
                        }
                    }
                },
                ConnectionState::PeerClosed | ConnectionState::Closed => {
                    return Err(Error::UnexpectedClose);
                }
                other => {
                    return Err(Error::Tls(rustls::Error::General(alloc::format!(
                        "not ready to write, in state: {other:?}"
                    ))));
                }
            }

            if discard != 0 {
                incoming.copy_within(discard..*incoming_used, 0);
                *incoming_used -= discard;
            }
        }

        self.flush_outgoing().await?;
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Error> {
        self.flush_outgoing().await
    }
}
