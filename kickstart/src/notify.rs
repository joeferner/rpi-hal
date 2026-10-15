//! Outbound notifications: the message, and the channels that deliver it.
//!
//! Two channels ship, both an HTTPS POST over [`crate::https`]:
//!
//! * [`Ntfy`](crate::notify::Ntfy), push to an [ntfy](https://ntfy.sh)
//!   topic. The message body is the request body and everything else —
//!   title, priority, tags — travels in headers. That is the whole protocol.
//! * [`Resend`](crate::notify::Resend), email through
//!   [Resend](https://resend.com)'s API: a JSON body.
//!
//! Both are [`Channel`](crate::notify::Channel)s, which is the plug point:
//! a board wanting a webhook, Matrix, or an MQTT publish implements the
//! trait for its own type and sends through it the same way.
//!
//! ```ignore
//! let ntfy = notify::Ntfy { host, port: 443, base_path: "", topic, token };
//! let message = notify::Notification {
//!     title: "Water detected",
//!     body: "Zone 2 (water heater) is wet.",
//!     priority: notify::Priority::Max,
//!     tags: "rotating_light",
//! };
//! ntfy.send(&CLIENT, stack, tls.clone(), &message).await?;
//! ```
//!
//! # What stays the board's
//!
//! Which channels a message goes to, what is said, and what a failure means.
//! A board with two channels should try both and report each — the point of
//! two is that one provider's outage is not the alarm's outage — and that
//! policy, the report it produces, and any counting of failures are the
//! board's to shape.
//!
//! So is the clock. With no wall clock `rustls` fails closed and every
//! certificate looks unusable, which reads as a verification failure and
//! sends the reader to the wrong layer. A board that checks
//! [`crate::clock`] before sending can say so as
//! [`Error::NoClock`](crate::notify::Error::NoClock) instead; the channels
//! here do not check, because a board may have its own reason to try
//! anyway.
//!
//! # Logging
//!
//! Each send logs its outcome on the console, a refusal with the start of
//! the server's answer: ntfy and Resend both explain a refusal in the body,
//! and without it every refusal looks the same. Never a token.

use alloc::string::String;
use alloc::sync::Arc;
use core::fmt::{self, Write as _};

use embassy_net::Stack;
use rustls::ClientConfig;

use crate::https::{self, Client, Request};
use crate::logln;

/// How loudly a notification should arrive.
///
/// These are ntfy's 1–5 priority levels, which is the point of the enum: the
/// levels are chosen so a phone's own notification rules can tell an alarm
/// apart from a heartbeat without reading the text. Resend has no such
/// notion and ignores it.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Priority {
    /// Arrives silently. A heartbeat, which exists to be noticed when it
    /// stops rather than read when it arrives.
    Min,
    /// An ordinary message: booted, update applied, test.
    Default,
    /// Something is wrong but it is not the emergency: a cut sensor loop, a
    /// clock that will not sync.
    High,
    /// The emergency. Meant to break through a silenced phone.
    Max,
}

impl Priority {
    /// The value for ntfy's `Priority` header.
    pub fn header(self) -> &'static str {
        match self {
            Priority::Min => "1",
            Priority::Default => "3",
            Priority::High => "4",
            Priority::Max => "5",
        }
    }
}

/// The level's name, for the console.
impl fmt::Display for Priority {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Priority::Min => "min",
            Priority::Default => "default",
            Priority::High => "high",
            Priority::Max => "max",
        })
    }
}

/// One message.
///
/// Borrowed rather than owned so a caller can format a body into a local
/// `String` and pass it straight in; nothing here outlives the send.
#[derive(Clone, Copy, Debug)]
pub struct Notification<'a> {
    /// One-line summary: ntfy's `Title` header, and the email subject. Line
    /// breaks are replaced before it goes into a header — see
    /// [`single_line`].
    pub title: &'a str,
    /// The message itself. Multi-line is fine.
    pub body: &'a str,
    /// How loudly it arrives.
    pub priority: Priority,
    /// Comma-separated ntfy tag names, which render as emoji — for example
    /// `rotating_light`. Empty for none. Email ignores them.
    pub tags: &'a str,
}

/// Why a notification did not go out on a channel.
#[derive(Debug)]
pub enum Error {
    /// The clock has not been set, so no certificate can be verified. Never
    /// produced by a channel here — see the module documentation — but the
    /// failure a board checking first reports.
    NoClock,
    /// The request itself failed: DNS, TCP, TLS, or the response.
    Http(https::Error),
    /// The server answered something other than a 2xx. Carries the code.
    Status(u16),
}

/// One line, for the console or a response body.
impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::NoClock => f.write_str("the clock is not set, so no certificate can be checked"),
            Error::Http(e) => write!(f, "{e}"),
            Error::Status(status) => write!(f, "refused with {status}"),
        }
    }
}

impl core::error::Error for Error {}

/// Something a [`Notification`] can be delivered through.
///
/// The plug point: [`Ntfy`] and [`Resend`] implement it, and a board adds
/// another transport by implementing it for its own type.
// `async fn` without a `Send` bound, as picoserve's and `ota::Hooks` are:
// the caller runs on a single-core executor, and nothing here moves a
// future between threads.
#[allow(async_fn_in_trait)]
pub trait Channel {
    /// Delivers `notification` through `client` and the TLS configuration
    /// `tls`, over `stack`. `Ok` means the far end accepted it.
    async fn send(
        &self,
        client: &Client,
        stack: Stack<'_>,
        tls: Arc<ClientConfig>,
        notification: &Notification<'_>,
    ) -> Result<(), Error>;
}

/// An ntfy topic on a self-hosted (or the public) instance: where to POST,
/// and what to authenticate with.
#[derive(Clone, Copy, Debug)]
pub struct Ntfy<'a> {
    /// The instance's host name. Used for DNS, the `Host` header, and TLS
    /// server-name verification.
    pub host: &'a str,
    /// The instance's port, usually 443.
    pub port: u16,
    /// Any path prefix the instance is served under, without a trailing
    /// slash. Empty for one at the root.
    pub base_path: &'a str,
    /// The topic to publish to. Treat it as a secret: on any instance
    /// whoever knows the topic name can read the messages and post fake
    /// ones, so it should be long and unguessable even with a token in
    /// force.
    pub topic: &'a str,
    /// Bearer token for the `Authorization` header. Never logged.
    pub token: &'a str,
}

impl Ntfy<'_> {
    /// The instance URL, `https://host[:port][/prefix]` — the form a settings
    /// file holds and `config::value::url` takes apart.
    pub fn url(&self) -> String {
        let mut url = alloc::format!("https://{}", self.host);
        if self.port != 443 {
            let _ = write!(url, ":{}", self.port);
        }
        url.push_str(self.base_path);
        url
    }

    /// The request target: the topic under any prefix.
    fn path(&self) -> String {
        alloc::format!("{}/{}", self.base_path, self.topic)
    }
}

impl Channel for Ntfy<'_> {
    /// Publishes to the topic. The title is passed through [`single_line`]
    /// before it becomes a header.
    async fn send(
        &self,
        client: &Client,
        stack: Stack<'_>,
        tls: Arc<ClientConfig>,
        notification: &Notification<'_>,
    ) -> Result<(), Error> {
        let title = single_line(notification.title);
        let authorization = alloc::format!("Bearer {}", self.token);
        let path = self.path();
        let mut headers = alloc::vec![
            ("Authorization", authorization.as_str()),
            ("Title", title.as_str()),
            ("Priority", notification.priority.header()),
        ];
        if !notification.tags.is_empty() {
            headers.push(("Tags", notification.tags));
        }

        logln!("notify[{}]: {}", notification.priority, title);
        let response = client
            .send(
                stack,
                tls,
                &Request {
                    method: "POST",
                    host: self.host,
                    port: self.port,
                    path: &path,
                    headers: &headers,
                    body: notification.body.as_bytes(),
                },
            )
            .await;

        match response {
            Ok(response) if (200..300).contains(&response.status) => {
                logln!("notify: delivered to {}/{}", self.host, self.topic);
                Ok(())
            }
            Ok(response) => {
                // The request target goes out too: a wrong topic or a stray
                // path prefix is the likeliest cause, and invisible otherwise.
                logln!(
                    "notify: {}{path} refused it: {} -- {}",
                    self.host,
                    response.status_line,
                    response.preview()
                );
                Err(Error::Status(response.status))
            }
            Err(e) => {
                logln!("notify: not delivered: {e}");
                Err(Error::Http(e))
            }
        }
    }
}

/// Email through Resend's API.
///
/// A transactional email API rather than SMTP: one more HTTPS POST over the
/// stack that already exists, against a second protocol to implement and
/// debug. The token is static and send-scoped, so there is no refresh path
/// to fail unnoticed on a device meant to run untouched for years.
#[derive(Clone, Copy, Debug)]
pub struct Resend<'a> {
    /// Resend API key. Never logged.
    pub token: &'a str,
    /// Sender address. Resend accepts only a domain verified on the account,
    /// so this is not free-form in practice — an unverified one is a `403`.
    pub from: &'a str,
    /// Recipients, comma-separated.
    pub to: &'a str,
}

/// Resend's API host. Fixed rather than configurable: the body is Resend's
/// own shape, so a different host would need different code, not a
/// different setting.
const RESEND_HOST: &str = "api.resend.com";

/// Resend's send endpoint.
const RESEND_PATH: &str = "/emails";

impl Resend<'_> {
    /// The request body: `{"from", "to": [...], "subject", "text"}`.
    ///
    /// The title becomes the subject and the body the plain-text message. No
    /// HTML part: a few lines of state are a bad place to introduce a second
    /// rendering of the same facts that could disagree with the first.
    fn body(&self, notification: &Notification<'_>) -> String {
        let mut body = String::from("{\"from\":");
        json_string(self.from, &mut body);
        body.push_str(",\"to\":[");
        for (index, address) in self.to.split(',').map(str::trim).enumerate() {
            if index > 0 {
                body.push(',');
            }
            json_string(address, &mut body);
        }
        body.push_str("],\"subject\":");
        json_string(notification.title, &mut body);
        body.push_str(",\"text\":");
        json_string(notification.body, &mut body);
        body.push('}');
        body
    }
}

impl Channel for Resend<'_> {
    /// Sends it as an email to every recipient in [`Resend::to`].
    async fn send(
        &self,
        client: &Client,
        stack: Stack<'_>,
        tls: Arc<ClientConfig>,
        notification: &Notification<'_>,
    ) -> Result<(), Error> {
        let body = self.body(notification);
        let authorization = alloc::format!("Bearer {}", self.token);
        let headers = [
            ("Authorization", authorization.as_str()),
            ("Content-Type", "application/json"),
        ];

        let response = client
            .send(
                stack,
                tls,
                &Request {
                    method: "POST",
                    host: RESEND_HOST,
                    port: 443,
                    path: RESEND_PATH,
                    headers: &headers,
                    body: body.as_bytes(),
                },
            )
            .await;

        match response {
            Ok(response) if (200..300).contains(&response.status) => {
                logln!("notify: emailed {}", self.to);
                Ok(())
            }
            Ok(response) => {
                // The common ones are worth recognising on sight: 401/403 is
                // the API key, and 403 with a domain complaint is a `from`
                // address on a domain the account has not verified.
                logln!(
                    "notify: email refused: {} -- {}",
                    response.status_line,
                    response.preview()
                );
                Err(Error::Status(response.status))
            }
            Err(e) => {
                logln!("notify: email not delivered: {e}");
                Err(Error::Http(e))
            }
        }
    }
}

/// `text` with every control character replaced by a space, so it can be
/// carried in an HTTP header.
///
/// Not cosmetic: a line break in a header value ends that header and starts
/// a new one, so a title formatted from a name somebody typed could
/// otherwise add headers of its own.
pub fn single_line(text: &str) -> String {
    text.chars()
        .map(|c| if c.is_control() { ' ' } else { c })
        .collect()
}

/// Appends `text` to `out` as a quoted JSON string.
///
/// Hand-rolled because a channel's body is one flat object and a serializer
/// would be a dependency for five escape rules. The escaping itself is not
/// optional: an unescaped quote or newline in a message produces a request
/// the far end rejects for reasons that look nothing like the cause.
pub fn json_string(text: &str, out: &mut String) {
    out.push('"');
    for c in text.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
}

#[cfg(test)]
mod tests {
    use super::*;

    const MESSAGE: Notification<'static> = Notification {
        title: "Water \"heater\"\nzone",
        body: "line one\nline\ttwo \\ end",
        priority: Priority::Max,
        tags: "rotating_light",
    };

    #[test]
    fn json_strings_are_escaped() {
        let mut out = String::new();
        json_string("a\"b\\c\nd\re\tf\u{1}g é", &mut out);
        assert_eq!(out, r#""a\"b\\c\nd\re\tf\u0001g é""#);
    }

    #[test]
    fn a_header_value_is_one_line() {
        assert_eq!(single_line("a\r\nb\tc"), "a  b c");
        assert_eq!(single_line("plain"), "plain");
    }

    #[test]
    fn the_resend_body_is_the_json_resend_expects() {
        let resend = Resend {
            token: "re_x",
            from: "alerts@example.com",
            to: "a@example.com, b@example.com",
        };
        assert_eq!(
            resend.body(&MESSAGE),
            r#"{"from":"alerts@example.com","to":["a@example.com","b@example.com"],"subject":"Water \"heater\"\nzone","text":"line one\nline\ttwo \\ end"}"#
        );
    }

    #[test]
    fn ntfy_publishes_under_its_prefix() {
        let mut ntfy = Ntfy {
            host: "ntfy.example.com",
            port: 443,
            base_path: "",
            topic: "alerts",
            token: "tk",
        };
        assert_eq!(ntfy.path(), "/alerts");
        assert_eq!(ntfy.url(), "https://ntfy.example.com");
        ntfy.base_path = "/sub";
        ntfy.port = 8443;
        assert_eq!(ntfy.path(), "/sub/alerts");
        assert_eq!(ntfy.url(), "https://ntfy.example.com:8443/sub");
    }

    #[test]
    fn priorities_are_ntfys_levels() {
        let levels = [
            Priority::Min,
            Priority::Default,
            Priority::High,
            Priority::Max,
        ]
        .map(Priority::header);
        assert_eq!(levels, ["1", "3", "4", "5"]);
        assert_eq!(alloc::format!("{}", Priority::High), "high");
    }
}
