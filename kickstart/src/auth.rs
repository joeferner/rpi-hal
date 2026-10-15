//! Password login for a board's web interface: one shared password, a
//! PBKDF2 verifier stored in its settings, and a session cookie handed out
//! in exchange for it.
//!
//! ```ignore
//! static SESSIONS: auth::Sessions<4> = auth::Sessions::new(auth::LIFETIME);
//!
//! let app = Router::from_service(web::SiteFiles(site))
//!     .route("/api/v1/config", get(settings).post(save_settings))
//!     // ...every route, then the layer over all of them:
//!     .layer(auth::RequireLogin::new(&SESSIONS, required, is_public));
//! ```
//!
//! What stays the board's: where the verifier is stored (its settings, set
//! from [`store`](crate::auth::store) and checked with
//! [`verify`](crate::auth::verify)), the routes that log in and out — their
//! JSON is the board's page's — and the whitelist of what is reachable
//! without a session, which [`RequireLogin`](crate::auth::RequireLogin) asks
//! rather than knows.
//!
//! # What this is worth, and what it is not
//!
//! The server speaks plain HTTP, so **the password and the cookie that comes
//! back both cross the network in the clear**, and anyone able to watch the
//! traffic or replay a captured cookie is in. What this stops is a browser
//! on the same LAN reaching the settings and the firmware upload by typing an
//! address.
//!
//! That is the honest scope: it makes the interface not-open, not private.
//! The PBKDF2 verifier is aimed at a different attacker to the sniffer —
//! someone who takes the SD card out. A card carries notification tokens
//! already, so it is a secret either way; what the KDF adds is that the
//! *password* does not come off it, which matters because people reuse them.
//!
//! # No password means no login
//!
//! [`RequireLogin`](crate::auth::RequireLogin) asks the board whether a
//! login is required on every
//! request, and a board with no verifier stored should answer no and serve
//! everything. A fresh board has to be reachable to be configured, and a
//! login screen with no credential that can satisfy it is a brick. The page
//! should say which state it is in, so an open board is not mistaken for a
//! protected one.
//!
//! # Randomness
//!
//! Salts and session tokens come from `getrandom` — on a Pi, the hardware
//! RNG through the `entropy` feature's backend, which the board turns
//! on with that feature. Through `getrandom` rather than `entropy` directly
//! so this module needs nothing of `rpi-hal`, and runs its tests on the host.

use alloc::string::String;
use core::cell::RefCell;
use core::fmt::Write as _;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant};
use picoserve::ResponseSent;
use picoserve::io::Read;
use picoserve::request::{Path, RequestParts};
use picoserve::response::{IntoResponse, ResponseWriter, StatusCode};
use picoserve::routing::{Layer, Next};
use subtle::ConstantTimeEq;

use crate::web::TextBody;

/// Name of the session cookie.
pub const COOKIE: &str = "session";

/// How long a session lasts before the password is needed again, in
/// seconds: twelve hours.
///
/// Absolute, not idle-based: a sliding expiry would mean a cookie captured
/// off the wire stays valid for as long as somebody keeps the page open,
/// and the wire is the weak point here.
///
/// Public because the cookie's `Max-Age` has to say the same thing — see
/// [`set_cookie`] — and a browser that kept a cookie the board had already
/// forgotten would show a signed-in page whose every request fails.
pub const LIFETIME_SECONDS: u64 = 12 * 60 * 60;

/// [`LIFETIME_SECONDS`] as the duration [`Sessions::new`] takes.
pub const LIFETIME: Duration = Duration::from_secs(LIFETIME_SECONDS);

/// Shortest password worth storing, in characters.
///
/// A floor rather than a policy: there is no lockout and nothing that
/// notices a board being guessed at, so the only thing standing between a
/// four-character password and someone on the LAN with a script is how long
/// [`ITERATIONS`] makes each attempt take. Eight is the point at which that
/// arithmetic stops being the interesting part.
///
/// The board enforces it where the password is set — the one place it
/// arrives in the clear — rather than [`store`]: a verifier already on the
/// card is accepted whatever it was made from.
pub const PASSWORD_MIN: usize = 8;

/// PBKDF2 iterations [`store`] uses for a newly set password.
///
/// The count is stored *in* the verifier rather than only here, so raising
/// this does not invalidate a password already on a card: an old value
/// keeps verifying at the count it was made with, and gets the new one the
/// next time it is set.
///
/// 100,000 costs roughly half a second of Cortex-A7 at 900 MHz, and it is
/// spent inside a web task — so it also stalls whatever else that task would
/// have served for that long. Acceptable for something a person does twice
/// a day: a cheap count buys nothing here, because the password is on the
/// wire in the clear anyway, and the card is the only attacker this number
/// is aimed at.
pub const ITERATIONS: u32 = 100_000;

/// Bytes of randomness in a session token. Rendered as hex, so the cookie
/// is twice this many characters.
const TOKEN_BYTES: usize = 32;

/// Salt bytes in a stored verifier.
const SALT_BYTES: usize = 16;

/// Digest bytes in a stored verifier: SHA-256's own output length.
const HASH_BYTES: usize = 32;

/// The prefix identifying the stored format.
const SCHEME: &str = "pbkdf2-sha256";

/// Builds the verifier to store for `password`:
/// `pbkdf2-sha256$<iterations>$<salt>$<hash>`.
///
/// The salt is fresh on every call, so setting the same password twice
/// produces different text — which is what stops the settings file from
/// saying whether a password was actually changed.
///
/// Shaped like the PHC strings other tools use without claiming to be one:
/// the fields are hex rather than PHC's unpadded base64, because hex is the
/// encoding that cannot grow a `$` or a `=` and confuse this format's own
/// separator, or anything a settings file wraps it in.
///
/// Takes [`ITERATIONS`] rounds of PBKDF2 — see there for what that costs.
pub fn store(password: &str) -> String {
    let mut salt = [0u8; SALT_BYTES];
    fill(&mut salt);
    store_with(password, &salt, ITERATIONS)
}

/// [`store`] with the salt and count given rather than chosen, for tests.
fn store_with(password: &str, salt: &[u8; SALT_BYTES], iterations: u32) -> String {
    let hash = derive(password, salt, iterations);
    let mut text = String::new();
    let _ = write!(text, "{SCHEME}${iterations}$");
    push_hex(&mut text, salt);
    text.push('$');
    push_hex(&mut text, &hash);
    text
}

/// Whether `password` matches the verifier `stored`.
///
/// A malformed `stored` is a failed verification rather than an error. The
/// value comes off a card that somebody may have edited by hand, and a
/// login has exactly one useful answer for "this verifier is not readable":
/// no.
pub fn verify(password: &str, stored: &str) -> bool {
    let Some((salt, expected, iterations)) = parse_stored(stored) else {
        return false;
    };
    let computed = derive(password, &salt, iterations);
    bool::from(computed.ct_eq(&expected))
}

/// Splits a stored verifier into salt, digest and iteration count.
fn parse_stored(stored: &str) -> Option<([u8; SALT_BYTES], [u8; HASH_BYTES], u32)> {
    let mut fields = stored.split('$');
    if fields.next()? != SCHEME {
        return None;
    }
    let iterations: u32 = fields.next()?.parse().ok()?;
    // Zero would make the derivation a no-op in some implementations and is
    // never something `store` wrote.
    if iterations == 0 {
        return None;
    }
    let salt = decode_hex::<SALT_BYTES>(fields.next()?)?;
    let hash = decode_hex::<HASH_BYTES>(fields.next()?)?;
    // A fifth field means some other format that happens to share the first
    // four, and guessing at it is worse than refusing it.
    if fields.next().is_some() {
        return None;
    }
    Some((salt, hash, iterations))
}

/// PBKDF2-HMAC-SHA256.
fn derive(password: &str, salt: &[u8], iterations: u32) -> [u8; HASH_BYTES] {
    let mut out = [0u8; HASH_BYTES];
    pbkdf2::pbkdf2_hmac::<sha2::Sha256>(password.as_bytes(), salt, iterations, &mut out);
    out
}

/// The live sessions: at most `N`, each good for the lifetime it was made
/// with. Declared by the board as a `static`, the way
/// [`storage::Shared`](crate::storage::Shared) is.
///
/// Small on purpose. A device with one password has, realistically, one or
/// two people who ever open its page; the limit exists so an attacker
/// cannot make the board allocate by logging in repeatedly. Once it is full
/// the oldest login is evicted rather than the new one refused — a full
/// table locking out the person with the correct password would be a
/// denial of service anyone on the LAN could trigger.
pub struct Sessions<const N: usize> {
    /// The table. A blocking mutex rather than an async one: every
    /// operation is a comparison over at most `N` fixed-size entries, so
    /// the critical section is bounded and short, and an `.await` inside a
    /// lookup would let the table change under a caller that had already
    /// decided it was allowed.
    table: Mutex<CriticalSectionRawMutex, RefCell<[Option<Session>; N]>>,
    /// Counter behind [`Session::serial`].
    next_serial: AtomicU32,
    /// How long a session is accepted for.
    lifetime: Duration,
}

/// One live session.
#[derive(Clone, Copy)]
struct Session {
    /// The raw token. Compared in constant time; never logged.
    token: [u8; TOKEN_BYTES],
    /// When it stops being accepted.
    expires: Instant,
    /// Which login this was, so the oldest can be evicted without keeping
    /// the table sorted.
    serial: u32,
}

impl<const N: usize> Sessions<N> {
    /// An empty table whose sessions last `lifetime` — usually
    /// [`LIFETIME`], which is what [`set_cookie`]'s `Max-Age` says.
    pub const fn new(lifetime: Duration) -> Self {
        Sessions {
            table: Mutex::new(RefCell::new([None; N])),
            next_serial: AtomicU32::new(0),
            lifetime,
        }
    }

    /// Records a new session and returns its token, as hex — the value of
    /// the cookie [`set_cookie`] builds.
    ///
    /// Call only once the password has been [`verify`]ed.
    pub fn open(&self) -> String {
        let mut token = [0u8; TOKEN_BYTES];
        fill(&mut token);
        let now = Instant::now();
        let session = Session {
            token,
            expires: now + self.lifetime,
            serial: self.next_serial.fetch_add(1, Ordering::Relaxed),
        };

        self.table.lock(|table| {
            let mut table = table.borrow_mut();
            // An expired entry is a free slot, so a board nobody has logged
            // into for a day does not evict a live session to make room.
            if let Some(slot) = table
                .iter_mut()
                .find(|slot| slot.is_none_or(|held| held.expires <= now))
            {
                *slot = Some(session);
                return;
            }
            // Full and all live: evict the oldest login. See the type's
            // documentation for why this is not a refusal.
            let oldest = table
                .iter()
                .enumerate()
                .filter_map(|(index, slot)| slot.map(|held| (index, held.serial)))
                .min_by_key(|&(_, serial)| serial)
                .map_or(0, |(index, _)| index);
            table[oldest] = Some(session);
        });

        let mut hex = String::new();
        push_hex(&mut hex, &token);
        hex
    }

    /// Whether `token` names a live session. An expired one is dropped as
    /// it is found.
    pub fn accepts(&self, token: &str) -> bool {
        let Some(raw) = decode_hex::<TOKEN_BYTES>(token) else {
            return false;
        };
        let now = Instant::now();
        self.table.lock(|table| {
            let mut table = table.borrow_mut();
            for slot in table.iter_mut() {
                let Some(session) = slot else { continue };
                // Constant time, even though a token is not a password: the
                // comparison is against a value an attacker supplies and can
                // vary, which is the shape a timing attack needs.
                if !bool::from(session.token.ct_eq(&raw)) {
                    continue;
                }
                if session.expires <= now {
                    *slot = None;
                    return false;
                }
                return true;
            }
            false
        })
    }

    /// Ends the session named by `token`, if it names one.
    ///
    /// Idempotent, and says nothing about whether it matched: a caller
    /// asking to be logged out wants the state it gets either way.
    pub fn close(&self, token: &str) {
        let Some(raw) = decode_hex::<TOKEN_BYTES>(token) else {
            return;
        };
        self.table.lock(|table| {
            for slot in table.borrow_mut().iter_mut() {
                if slot.is_some_and(|held| bool::from(held.token.ct_eq(&raw))) {
                    *slot = None;
                }
            }
        });
    }

    /// Ends every session.
    ///
    /// For a password change. A new password that left the old sessions
    /// working would not have taken effect the way whoever changed it
    /// expects — the usual reason to change one is that somebody should
    /// stop having access.
    pub fn close_all(&self) {
        self.table.lock(|table| *table.borrow_mut() = [None; N]);
    }
}

/// The session token in a `Cookie:` header, if it carries one.
///
/// A header holds any number of `name=value` pairs separated by `;`, and
/// the session may be any of them — a browser sends every cookie it holds
/// for the origin.
pub fn session_cookie(header: &str) -> Option<&str> {
    header.split(';').find_map(|pair| {
        let (name, value) = pair.split_once('=')?;
        (name.trim() == COOKIE).then(|| value.trim())
    })
}

/// The `Set-Cookie` value that hands `token` to the browser.
///
/// `HttpOnly` so a cross-site script cannot read it, `SameSite=Strict` so
/// another site cannot make a browser use it, `Path=/` so it covers the API
/// as well as the pages, and `Max-Age` of [`LIFETIME_SECONDS`]. No `Secure`,
/// and it would be a bug to add one: the server speaks plain HTTP, and a
/// `Secure` cookie would never be sent back.
pub fn set_cookie(token: &str) -> String {
    alloc::format!(
        "{COOKIE}={token}; Path=/; HttpOnly; SameSite=Strict; Max-Age={LIFETIME_SECONDS}"
    )
}

/// The `Set-Cookie` value that makes the browser drop the session cookie.
pub fn clear_cookie() -> String {
    alloc::format!("{COOKIE}=; Path=/; HttpOnly; SameSite=Strict; Max-Age=0")
}

/// Longest decoded request path [`decoded_path`] will assemble, in bytes.
///
/// A path that does not fit was not going to match a route either.
pub const PATH_MAX: usize = 192;

/// The request path with its percent-escapes resolved, as `picoserve` will
/// match it — or `None` if it does not fit [`PATH_MAX`] or does not decode.
///
/// This exists because a gate and the router have to be looking at the
/// same string. `picoserve` compares route patterns against the decoded
/// path, so `/%61pi/v1/config` routes to `/api/v1/config`; a whitelist
/// reading the raw path would see something it does not recognise and,
/// were it written the other way round, let it through. Segment
/// separators are the one thing not taken from the decoding — `picoserve`
/// matches a `/` in a pattern only against a literal `/`, so an encoded
/// `%2f` cannot manufacture one, and rebuilding the path from
/// [`Path::segments`] keeps that property.
pub fn decoded_path(path: Path<'_>) -> Option<heapless::String<PATH_MAX>> {
    let mut decoded = heapless::String::new();
    for segment in path.segments() {
        decoded.push('/').ok()?;
        for character in segment.chars() {
            decoded.push(character.ok()?.into_char()).ok()?;
        }
    }
    Some(decoded)
}

/// A `picoserve` layer that refuses a request with a `401` unless the
/// board needs no login, the request is on its whitelist, or it carries a
/// live session.
///
/// Put it outermost, over every route rather than over a chosen few, so a
/// route added to the router is protected by having been added.
///
/// The refusal is `{"ok":false,"error":"a session is required"}` with a
/// `401` rather than a `403`: the client can do something about it, and
/// what it should do is show the login screen. A page keying off that
/// status handles an expired session on every route the same way.
pub struct RequireLogin<'a, const N: usize> {
    sessions: &'a Sessions<N>,
    required: fn() -> bool,
    public: fn(&str, &str) -> bool,
}

impl<'a, const N: usize> RequireLogin<'a, N> {
    /// Checks requests against `sessions`.
    ///
    /// `required` answers whether a login is demanded at all — false when
    /// the board has no verifier stored, in which case every request is
    /// served. It is asked per request, so setting or clearing a password
    /// takes effect at once.
    ///
    /// `public(path, method)` is the whitelist: whether that request is
    /// served without a session. `path` is the **decoded** path
    /// ([`decoded_path`]), so a comparison in it cannot be dodged by
    /// percent-encoding a letter. **Write it as a whitelist** — every arm
    /// naming something public and the fall-through `false` — so a route
    /// added later is protected rather than open.
    pub fn new(
        sessions: &'a Sessions<N>,
        required: fn() -> bool,
        public: fn(&str, &str) -> bool,
    ) -> Self {
        RequireLogin {
            sessions,
            required,
            public,
        }
    }

    /// Whether a request may proceed: its decoded path (`None` if it would
    /// not decode), its method, and its raw `Cookie:` header.
    fn allows(&self, path: Option<&str>, method: &str, cookie: Option<&str>) -> bool {
        // First: on a board with no password there is nothing to protect,
        // and a path that would not decode should not be refused on a device
        // that is open anyway.
        if !(self.required)() {
            return true;
        }
        if path.is_some_and(|path| (self.public)(path, method)) {
            return true;
        }
        cookie
            .and_then(session_cookie)
            .is_some_and(|token| self.sessions.accepts(token))
    }
}

impl<const N: usize, State, PathParameters> Layer<State, PathParameters> for RequireLogin<'_, N> {
    type NextState = State;
    type NextPathParameters = PathParameters;

    async fn call_layer<
        'a,
        R: Read + 'a,
        NextLayer: Next<'a, R, Self::NextState, Self::NextPathParameters>,
        W: ResponseWriter<Error = R::Error>,
    >(
        &self,
        next: NextLayer,
        state: &State,
        path_parameters: PathParameters,
        request_parts: RequestParts<'_>,
        response_writer: W,
    ) -> Result<ResponseSent, W::Error> {
        // A header that is not UTF-8 is no cookie at all: a session token
        // is hex, so what reaches here is something else's cookie or a
        // deliberately malformed one, and neither carries a session.
        let cookie = request_parts
            .headers()
            .get("cookie")
            .and_then(|value| value.as_str().ok());
        let path = decoded_path(request_parts.path());
        if self.allows(path.as_deref(), request_parts.method(), cookie) {
            return next.run(state, path_parameters, response_writer).await;
        }
        (
            StatusCode::new(401),
            TextBody {
                body: String::from(r#"{"ok":false,"error":"a session is required"}"#),
                content_type: "application/json",
            },
        )
            .write_to(next.into_connection().await?, response_writer)
            .await
    }
}

/// Fills `out` from `getrandom`. Infallible on every target this runs on:
/// a Pi's backend is the hardware RNG, which can only make a caller wait.
fn fill(out: &mut [u8]) {
    getrandom::getrandom(out).expect("no randomness for a salt or a session token");
}

/// Appends `bytes` to `out` as lower-case hex.
fn push_hex(out: &mut String, bytes: &[u8]) {
    for byte in bytes {
        let _ = write!(out, "{byte:02x}");
    }
}

/// Decodes exactly `N` bytes of hex, either case, rejecting anything else.
///
/// Length and alphabet are both checked, which lets a cookie or a stored
/// field holding something else be a plain "no" rather than a value to
/// compare against.
fn decode_hex<const N: usize>(text: &str) -> Option<[u8; N]> {
    if text.len() != N * 2 {
        return None;
    }
    let digit = |byte: u8| match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        b'A'..=b'F' => Some(byte - b'A' + 10),
        _ => None,
    };
    let mut raw = [0u8; N];
    // The length check above means the remainder is empty and every byte of
    // `raw` is covered, so the pairs and the output line up exactly.
    let (pairs, _) = text.as_bytes().as_chunks::<2>();
    for (byte, pair) in raw.iter_mut().zip(pairs) {
        *byte = (digit(pair[0])? << 4) | digit(pair[1])?;
    }
    Some(raw)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Fewer rounds than a board uses: the format and the comparison are
    /// what is under test, not the key derivation's cost.
    const TEST_ROUNDS: u32 = 1000;

    #[test]
    fn a_stored_password_verifies_and_another_does_not() {
        let stored = store_with("correct horse", &[7; SALT_BYTES], TEST_ROUNDS);
        assert!(verify("correct horse", &stored));
        assert!(!verify("correct hors", &stored));
        assert!(!verify("", &stored));
    }

    #[test]
    fn the_stored_form_is_scheme_count_salt_hash() {
        let stored = store_with("pw", &[0xab; SALT_BYTES], 42);
        let fields: alloc::vec::Vec<&str> = stored.split('$').collect();
        assert_eq!(fields[0], "pbkdf2-sha256");
        assert_eq!(fields[1], "42");
        assert_eq!(fields[2], "ab".repeat(SALT_BYTES));
        assert_eq!(fields[3].len(), HASH_BYTES * 2);
        assert_eq!(fields.len(), 4);
    }

    #[test]
    fn store_salts_freshly_each_time() {
        // At the real count, once each: this is the one test that runs it.
        let first = store("same password");
        let second = store("same password");
        assert_ne!(first, second);
        assert!(verify("same password", &first));
    }

    #[test]
    fn a_malformed_verifier_is_a_no() {
        let good = store_with("pw", &[1; SALT_BYTES], TEST_ROUNDS);
        let (head, hash) = good.rsplit_once('$').unwrap();
        for stored in [
            "",
            "pw",
            &good.replace("pbkdf2-sha256", "pbkdf2-sha1"),
            &good.replacen(&alloc::format!("${TEST_ROUNDS}$"), "$0$", 1),
            &alloc::format!("{good}$extra"),
            &alloc::format!("{head}${}", &hash[1..]),
            &alloc::format!("{head}${}g", &hash[1..]),
        ] {
            assert!(!verify("pw", stored), "accepted {stored:?}");
        }
    }

    #[test]
    fn an_upper_case_verifier_still_reads() {
        let stored = store_with("pw", &[0xcd; SALT_BYTES], TEST_ROUNDS);
        assert!(verify(
            "pw",
            &stored.to_uppercase().replace("PBKDF2-SHA256", SCHEME)
        ));
    }

    #[test]
    fn the_session_cookie_is_found_among_others() {
        assert_eq!(session_cookie("session=abc"), Some("abc"));
        assert_eq!(session_cookie("theme=dark; session=abc; x=1"), Some("abc"));
        assert_eq!(session_cookie(" session = abc "), Some("abc"));
        assert_eq!(session_cookie("sessionx=abc; xsession=def"), None);
        assert_eq!(session_cookie(""), None);
    }

    #[test]
    fn the_cookies_say_what_they_should() {
        assert_eq!(
            set_cookie("ab12"),
            "session=ab12; Path=/; HttpOnly; SameSite=Strict; Max-Age=43200"
        );
        assert!(clear_cookie().ends_with("Max-Age=0"));
        assert!(!set_cookie("t").contains("Secure"));
    }

    #[test]
    fn a_session_is_accepted_until_it_expires_or_closes() {
        let sessions: Sessions<2> = Sessions::new(LIFETIME);
        let token = sessions.open();
        assert_eq!(token.len(), TOKEN_BYTES * 2);
        assert!(sessions.accepts(&token));
        assert!(sessions.accepts(&token.to_uppercase()));
        assert!(!sessions.accepts("not hex"));
        assert!(!sessions.accepts(&"0".repeat(TOKEN_BYTES * 2)));

        sessions.close(&token);
        assert!(!sessions.accepts(&token));

        // Expiry by a lifetime of nothing rather than by advancing the mock
        // clock: that clock is one counter shared by every test in the crate,
        // and moving it under the `clock` tests running alongside breaks them.
        let expired: Sessions<2> = Sessions::new(Duration::from_ticks(0));
        let token = expired.open();
        assert!(!expired.accepts(&token));
        // And the expired entry is a free slot, not a live one to evict.
        let first = expired.open();
        let _second = expired.open();
        assert!(!expired.accepts(&first));
    }

    #[test]
    fn a_full_table_evicts_the_oldest_login() {
        let sessions: Sessions<2> = Sessions::new(LIFETIME);
        let first = sessions.open();
        let second = sessions.open();
        let third = sessions.open();
        assert!(!sessions.accepts(&first));
        assert!(sessions.accepts(&second));
        assert!(sessions.accepts(&third));

        sessions.close_all();
        assert!(!sessions.accepts(&second));
        assert!(!sessions.accepts(&third));
    }

    #[test]
    fn the_gate_lets_through_what_it_should() {
        static SESSIONS: Sessions<2> = Sessions::new(LIFETIME);
        fn required() -> bool {
            true
        }
        fn open() -> bool {
            false
        }
        fn public(path: &str, method: &str) -> bool {
            matches!((method, path), ("POST", "/api/v1/login"))
        }

        let gate = RequireLogin::new(&SESSIONS, required, public);
        assert!(gate.allows(Some("/api/v1/login"), "POST", None));
        assert!(!gate.allows(Some("/api/v1/login"), "GET", None));
        assert!(!gate.allows(Some("/api/v1/config"), "GET", None));
        assert!(!gate.allows(None, "POST", None));

        let token = SESSIONS.open();
        let cookie = alloc::format!("theme=dark; session={token}");
        assert!(gate.allows(Some("/api/v1/config"), "GET", Some(&cookie)));
        assert!(!gate.allows(Some("/api/v1/config"), "GET", Some("session=00")));

        let open_board = RequireLogin::new(&SESSIONS, open, public);
        assert!(open_board.allows(None, "POST", None));
    }
}
