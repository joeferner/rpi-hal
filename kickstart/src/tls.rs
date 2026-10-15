//! TLS client connections: `rustls` with real certificate-chain
//! verification, over an `embassy-net` TCP socket.
//!
//! A cut-down embedded TLS stack would encrypt the connection without
//! meaningfully authenticating the peer, which for a board that sends
//! alerts or fetches data means anybody on the path can impersonate the
//! other end. This is `rustls` in `no_std` + `alloc` mode instead, checking
//! the server's chain against trust anchors and its validity against the
//! clock.
//!
//! ```ignore
//! let config = tls::client_config();           // once, and share the Arc
//! let mut socket = TcpSocket::new(stack, &mut rx, &mut tx);
//! socket.connect((address, 443)).await?;
//! let mut stream = TlsStream::connect(socket, config.clone(), "example.com").await?;
//! stream.write_all(b"GET / HTTP/1.1\r\nHost: example.com\r\n\r\n").await?;
//! ```
//!
//! # What a Pi has to supply
//!
//! Three things a hosted platform provides for free, each a place this
//! could silently go wrong:
//!
//! * **A crypto provider.** `ring` and `aws-lc-rs` both need a C toolchain,
//!   so `rustls-rustcrypto` is the one that builds for a bare-metal target.
//!   **It is pre-release and unaudited.** The protocol handling and the
//!   chain verification are real `rustls`; the primitives underneath are not
//!   audited ones. Weigh that against what the connection protects.
//! * **A clock.** Certificate validity is checked against
//!   [`crate::clock`], through [`clock::rustls_time_provider`]. Until
//!   something has set the clock — `sntp`, on a board with no RTC — every
//!   handshake fails closed rather than validating against 1970.
//! * **Entropy.** The primitives reach randomness through `getrandom`, and
//!   a bare-metal target has no backend for it. The `entropy` feature
//!   registers the SoC's hardware generator as one; without it, or another,
//!   the image fails to link on `__getrandom_custom`.
//!
//! [`clock::rustls_time_provider`]: crate::clock::rustls_time_provider
//!
//! # Trust anchors
//!
//! [`client_config_with`] takes a `RootCertStore`, so a board talking to
//! one private server carries that server's anchor and nothing else. With
//! the `webpki-roots` feature, [`client_config`] builds one from Mozilla's
//! set, compiled in. There is no system trust store on a board and nothing
//! to update one, so a root rotation is a firmware update — the same trade
//! every other constant in the image makes, and better than anchors read
//! off a card anybody can write.
//!
//! [`client_config_with`]: crate::tls::client_config_with
//! [`client_config`]: crate::tls::client_config

use alloc::sync::Arc;

use rustls::{ClientConfig, RootCertStore};

mod stream;
pub use stream::{Error, TlsStream};

/// Builds a client configuration trusting `roots`.
///
/// **TLS 1.3 only.** Dropping 1.2 removes a meaningful amount of protocol
/// surface from an unaudited crypto provider, and every service these
/// boards talk to speaks 1.3. A board that must reach a 1.2-only server
/// builds its own `ClientConfig` from the same pieces: the provider from
/// `rustls_rustcrypto::provider()` and the clock from
/// [`clock::rustls_time_provider`](crate::clock::rustls_time_provider).
///
/// Allocates, and the trust anchor set is the largest single allocation a
/// board is likely to make outside the filesystem: build it once and share
/// the `Arc`, rather than once per connection.
pub fn client_config_with(roots: RootCertStore) -> Arc<ClientConfig> {
    let config = ClientConfig::builder_with_details(
        Arc::new(rustls_rustcrypto::provider()),
        Arc::new(crate::clock::rustls_time_provider()),
    )
    .with_protocol_versions(&[&rustls::version::TLS13])
    // Fails only if the provider supports none of the requested versions.
    // `rustls-rustcrypto` supports 1.3, so this is a build-time mistake
    // rather than a runtime condition.
    .expect("crypto provider does not support TLS 1.3")
    .with_root_certificates(roots)
    .with_no_client_auth();
    Arc::new(config)
}

/// Builds a client configuration trusting Mozilla's root set, compiled in.
/// See [`client_config_with`].
#[cfg(feature = "webpki-roots")]
pub fn client_config() -> Arc<ClientConfig> {
    client_config_with(RootCertStore {
        roots: webpki_roots::TLS_SERVER_ROOTS.to_vec(),
    })
}

/// How many trust anchors [`client_config`] compiles in, for a bring-up
/// line.
#[cfg(feature = "webpki-roots")]
pub fn trust_anchor_count() -> usize {
    webpki_roots::TLS_SERVER_ROOTS.len()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Building the configuration is where a provider that cannot do what
    /// is asked of it fails — the `expect` in `client_config_with`.
    #[test]
    fn the_configuration_builds() {
        let config = client_config_with(RootCertStore::empty());
        assert!(
            config
                .crypto_provider()
                .cipher_suites
                .iter()
                .any(|suite| { suite.version() == &rustls::version::TLS13 })
        );
    }

    #[cfg(feature = "webpki-roots")]
    #[test]
    fn mozillas_roots_are_compiled_in() {
        assert!(trust_anchor_count() > 100, "{}", trust_anchor_count());
        let _ = client_config();
    }
}
