//! LE Security Manager Protocol (SMP): LE Legacy "Just Works" pairing that
//! brings a connection up to an encrypted link, in either role.
//!
//! [`Smp`] is the responder (peripheral) pairing state machine — it answers
//! the central's Pairing Request, runs the confirm/random exchange, derives
//! the Short Term Key, and hands it to the controller to encrypt the link.
//! [`Initiator`] is the mirror for the central role: it *sends* the Pairing
//! Request, drives the same confirm/random exchange from the other side, and
//! then commands the controller to encrypt with the derived STK
//! ([`Bluetooth::le_start_encryption`]) — the path a host uses to pair with a
//! HID device (a mouse, a gamepad) before it will hand over input reports.
//! Both are built on the pairing key functions `c1` (the confirm-value
//! function: both sides commit to a random before revealing it) and `s1`
//! (which derives the STK), defined by the Bluetooth spec (Vol 3, Part H,
//! 2.2.3–2.2.4) in terms of AES-128 — which [`Bluetooth::le_encrypt`]
//! performs in the controller, so there's no software AES here.
//!
//! This is Just Works only: IO capability NoInputNoOutput, no MITM
//! protection. Bonding is optional (see the `bonding` flags): with it, keys
//! are distributed and persisted ([`Bond`]) so a reconnect re-encrypts
//! without re-pairing; without it the link is encrypted for the session only.
//! Just Works is the minimum a HID-over-GATT link requires to carry input.
//!
//! # Byte order, and why there's a self-test
//!
//! SMP values travel little-endian on the wire, but the spec's crypto is
//! defined on big-endian 128-bit values, and HCI `LE_Encrypt`'s own octet
//! order isn't something to assume. A single wrong-endian buffer produces a
//! confirm-value mismatch that looks like a protocol bug but is really
//! crypto — the worst kind to debug on hardware.
//!
//! So rather than trust an assumption, [`self_test`] pins it down against
//! published vectors, on the actual controller, before any pairing:
//!
//! 1. It runs the spec's sample AES block through `LE_Encrypt` and, if the
//!    result doesn't match, retries with the bytes reversed — **detecting**
//!    the controller's octet convention from a known answer rather than
//!    guessing it.
//! 2. It then runs the spec's `c1` sample vector end to end and checks the
//!    output.
//!
//! If both pass, the whole chain (AES primitive, byte handling, `c1`
//! assembly) is proven correct on that hardware. `Crypto` captures the
//! detected convention so its `c1`/`s1` use it thereafter.

use super::{Bluetooth, Error};
use crate::timer::Timer;

/// Known AES-128 test vector (Bluetooth Core Spec sample for the `e`
/// function): `key`, `plaintext`, and the expected `ciphertext`, as
/// big-endian byte arrays. Used to detect the controller's `LE_Encrypt`
/// octet order (see [`self_test`]).
const AES_KEY: [u8; 16] = [
    0x4c, 0x68, 0x38, 0x41, 0x39, 0xf5, 0x74, 0xd8, 0x36, 0xbc, 0xf3, 0x4e, 0x9d, 0xfb, 0x01, 0xbf,
];
/// Plaintext of the AES test vector. See [`AES_KEY`].
const AES_PLAINTEXT: [u8; 16] = [
    0x02, 0x13, 0x24, 0x35, 0x46, 0x57, 0x68, 0x79, 0xac, 0xbd, 0xce, 0xdf, 0xe0, 0xf1, 0x02, 0x13,
];
/// Expected ciphertext of the AES test vector. See [`AES_KEY`].
const AES_CIPHERTEXT: [u8; 16] = [
    0x99, 0xad, 0x1b, 0x52, 0x26, 0xa3, 0x7e, 0x3e, 0x05, 0x8e, 0x3b, 0x8e, 0x27, 0xc2, 0xc6, 0x66,
];

// The spec's `c1` sample vector (Vol 3, Part H, D.1), converted to the
// wire/little-endian order this module's `c1` takes as input. The spec
// states these as big-endian 128-bit values; each is byte-reversed here so
// that `Crypto::c1`'s own construction reproduces the spec's `p1`/`p2` and
// the expected confirm value.

/// `c1` sample: the random `r` (spec `0x5783D5…2EE0`), wire order.
const C1_R: [u8; 16] = [
    0xe0, 0x2e, 0x70, 0xc6, 0x4e, 0x27, 0x88, 0x63, 0x0e, 0x6f, 0xad, 0x56, 0x21, 0xd5, 0x83, 0x57,
];
/// `c1` sample: the Pairing Request PDU bytes, wire order.
const C1_PREQ: [u8; 7] = [0x01, 0x01, 0x00, 0x00, 0x10, 0x07, 0x07];
/// `c1` sample: the Pairing Response PDU bytes, wire order.
const C1_PRES: [u8; 7] = [0x02, 0x03, 0x00, 0x00, 0x08, 0x00, 0x05];
/// `c1` sample: initiator address (spec `ia = 0xA1A2A3A4A5A6`), wire order.
const C1_IA: [u8; 6] = [0xa6, 0xa5, 0xa4, 0xa3, 0xa2, 0xa1];
/// `c1` sample: responder address (spec `ra = 0xB1B2B3B4B5B6`), wire order.
const C1_RA: [u8; 6] = [0xb6, 0xb5, 0xb4, 0xb3, 0xb2, 0xb1];
/// `c1` sample: initiator address type (`iat = 1`).
const C1_IAT: u8 = 0x01;
/// `c1` sample: responder address type (`rat = 0`).
const C1_RAT: u8 = 0x00;
/// `c1` sample: expected confirm value (spec `0x1E1E3F…3B86`), wire order.
const C1_EXPECTED: [u8; 16] = [
    0x86, 0x3b, 0xf1, 0xbe, 0xc5, 0x4d, 0xa7, 0xd2, 0xea, 0x88, 0x89, 0x87, 0xef, 0x3f, 0x1e, 0x1e,
];

/// Reverses a 16-byte block (little-endian ⇄ big-endian).
fn reverse16(x: &[u8; 16]) -> [u8; 16] {
    let mut out = [0u8; 16];
    for (i, b) in x.iter().rev().enumerate() {
        out[i] = *b;
    }
    out
}

/// XORs two 16-byte blocks.
fn xor16(a: &[u8; 16], b: &[u8; 16]) -> [u8; 16] {
    let mut out = [0u8; 16];
    for i in 0..16 {
        out[i] = a[i] ^ b[i];
    }
    out
}

/// Detects the controller's `LE_Encrypt` octet convention by running the
/// [`AES_KEY`]/[`AES_PLAINTEXT`] vector: `Ok(Some(false))` if the controller
/// takes the bytes as given (big-endian), `Ok(Some(true))` if the operands
/// must be byte-reversed, or `Ok(None)` if neither reproduces the known
/// answer (the crypto can't be trusted).
fn detect(bt: &mut Bluetooth, timer: &Timer) -> Result<Option<bool>, Error> {
    let direct = bt.le_encrypt(&AES_KEY, &AES_PLAINTEXT, timer)?;
    if direct == AES_CIPHERTEXT {
        return Ok(Some(false));
    }
    let swapped =
        reverse16(&bt.le_encrypt(&reverse16(&AES_KEY), &reverse16(&AES_PLAINTEXT), timer)?);
    if swapped == AES_CIPHERTEXT {
        return Ok(Some(true));
    }
    Ok(None)
}

/// The fixed inputs to [`Crypto::c1`] for a pairing session: the exchanged
/// Pairing Request/Response PDUs and the two device addresses (with their
/// types), all in wire order.
pub(crate) struct PairingContext {
    /// The Pairing Request PDU bytes (initiator → responder), as sent.
    pub preq: [u8; 7],
    /// The Pairing Response PDU bytes (responder → initiator), as sent.
    pub pres: [u8; 7],
    /// Initiator (central) device address, wire order (LSB first).
    pub ia: [u8; 6],
    /// Initiator address type: `0` public, `1` random.
    pub iat: u8,
    /// Responder (our) device address, wire order (LSB first).
    pub ra: [u8; 6],
    /// Responder address type: `0` public, `1` random.
    pub rat: u8,
}

/// The SMP pairing crypto functions, bound to a controller and the octet
/// convention detected for its `LE_Encrypt`.
pub(crate) struct Crypto {
    /// Whether `LE_Encrypt` operands/results must be byte-reversed to get a
    /// standard (big-endian) AES-128, as determined by [`detect`].
    swap: bool,
}

impl Crypto {
    /// Builds a [`Crypto`] after detecting the controller's `LE_Encrypt`
    /// octet convention. Returns [`Error::CryptoSelfTest`] if the controller
    /// doesn't reproduce the known AES vector under either convention.
    pub(crate) fn new(bt: &mut Bluetooth, timer: &Timer) -> Result<Self, Error> {
        match detect(bt, timer)? {
            Some(swap) => Ok(Self { swap }),
            None => Err(Error::CryptoSelfTest),
        }
    }

    /// Standard AES-128 on big-endian 16-byte blocks, applying the detected
    /// byte-order convention around the controller's `LE_Encrypt`.
    fn aes(
        &self,
        bt: &mut Bluetooth,
        timer: &Timer,
        key: &[u8; 16],
        block: &[u8; 16],
    ) -> Result<[u8; 16], Error> {
        if self.swap {
            let out = bt.le_encrypt(&reverse16(key), &reverse16(block), timer)?;
            Ok(reverse16(&out))
        } else {
            bt.le_encrypt(key, block, timer)
        }
    }

    /// The SMP `e` function on wire-order (little-endian) operands: the
    /// spec's crypto is big-endian, so the operands are reversed around the
    /// big-endian [`Self::aes`]. This mirrors a conventional
    /// `bt_encrypt_le`.
    fn e(
        &self,
        bt: &mut Bluetooth,
        timer: &Timer,
        k: &[u8; 16],
        data: &[u8; 16],
    ) -> Result<[u8; 16], Error> {
        let out = self.aes(bt, timer, &reverse16(k), &reverse16(data))?;
        Ok(reverse16(&out))
    }

    /// The `c1` confirm-value function (Vol 3, Part H, 2.2.3):
    /// `c1 = e(k, e(k, r ⊕ p1) ⊕ p2)`, where `p1 = iat ‖ rat ‖ preq ‖ pres`
    /// and `p2 = ra ‖ ia ‖ padding`, from the [`PairingContext`]. `k` is the
    /// Temporary Key and `r` the confirming random, both in wire order.
    pub(crate) fn c1(
        &self,
        bt: &mut Bluetooth,
        timer: &Timer,
        k: &[u8; 16],
        r: &[u8; 16],
        ctx: &PairingContext,
    ) -> Result<[u8; 16], Error> {
        let mut p1 = [0u8; 16];
        p1[0] = ctx.iat;
        p1[1] = ctx.rat;
        p1[2..9].copy_from_slice(&ctx.preq);
        p1[9..16].copy_from_slice(&ctx.pres);

        let step1 = self.e(bt, timer, k, &xor16(r, &p1))?;

        let mut p2 = [0u8; 16];
        p2[0..6].copy_from_slice(&ctx.ra);
        p2[6..12].copy_from_slice(&ctx.ia);
        // p2[12..16] stays zero (padding).

        self.e(bt, timer, k, &xor16(&step1, &p2))
    }

    /// The `s1` key-generation function (Vol 3, Part H, 2.2.4):
    /// `s1(k, r1, r2) = e(k, r2[0..8] ‖ r1[0..8])`. The Short Term Key for a
    /// responder is `s1(TK, Srand, Mrand)` — `r1` the local random `Srand`,
    /// `r2` the remote random `Mrand`.
    pub(crate) fn s1(
        &self,
        bt: &mut Bluetooth,
        timer: &Timer,
        k: &[u8; 16],
        r1: &[u8; 16],
        r2: &[u8; 16],
    ) -> Result<[u8; 16], Error> {
        let mut data = [0u8; 16];
        data[0..8].copy_from_slice(&r2[0..8]);
        data[8..16].copy_from_slice(&r1[0..8]);
        self.e(bt, timer, k, &data)
    }
}

/// The outcome of the SMP crypto [`self_test`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SelfTest {
    /// The detected `LE_Encrypt` byte convention: `true` if operands must be
    /// reversed for a standard AES, `false` if taken as given. Only
    /// meaningful when [`Self::aes_ok`] is set.
    pub swapped: bool,
    /// Whether the controller reproduced the known AES-128 vector (under one
    /// of the two conventions).
    pub aes_ok: bool,
    /// Whether the spec's `c1` sample vector produced the expected confirm
    /// value.
    pub c1_ok: bool,
}

impl SelfTest {
    /// `true` only if every check passed — the pairing crypto is trustworthy
    /// on this hardware.
    pub fn passed(&self) -> bool {
        self.aes_ok && self.c1_ok
    }
}

/// Runs the SMP crypto self-test against the controller: detects the
/// `LE_Encrypt` byte convention from a known AES vector, then verifies the
/// spec's `c1` sample vector end to end. Call once after the controller is
/// up; a [`SelfTest`] with [`SelfTest::passed`] set means pairing crypto can
/// be trusted on this hardware.
pub fn self_test(bt: &mut Bluetooth, timer: &Timer) -> Result<SelfTest, Error> {
    let Some(swap) = detect(bt, timer)? else {
        return Ok(SelfTest {
            swapped: false,
            aes_ok: false,
            c1_ok: false,
        });
    };
    let crypto = Crypto { swap };
    let ctx = PairingContext {
        preq: C1_PREQ,
        pres: C1_PRES,
        ia: C1_IA,
        iat: C1_IAT,
        ra: C1_RA,
        rat: C1_RAT,
    };
    let confirm = crypto.c1(bt, timer, &[0u8; 16], &C1_R, &ctx)?;
    Ok(SelfTest {
        swapped: swap,
        aes_ok: true,
        c1_ok: confirm == C1_EXPECTED,
    })
}

/// SMP PDU code `Pairing Request`.
const SMP_PAIRING_REQUEST: u8 = 0x01;
/// SMP PDU code `Pairing Response`.
const SMP_PAIRING_RESPONSE: u8 = 0x02;
/// SMP PDU code `Pairing Confirm`.
const SMP_PAIRING_CONFIRM: u8 = 0x03;
/// SMP PDU code `Pairing Random`.
const SMP_PAIRING_RANDOM: u8 = 0x04;
/// SMP PDU code `Pairing Failed`.
const SMP_PAIRING_FAILED: u8 = 0x05;
/// SMP PDU code `Encryption Information` — distributes the Long Term Key
/// during bonding.
const SMP_ENCRYPTION_INFORMATION: u8 = 0x06;
/// SMP PDU code `Master Identification` — distributes the EDIV and Rand that
/// identify a distributed LTK on reconnect.
const SMP_MASTER_IDENTIFICATION: u8 = 0x07;

/// SMP `Pairing Failed` reason `Confirm Value Failed` — the peer's revealed
/// random didn't match the confirm it committed to.
const SMP_ERR_CONFIRM_FAILED: u8 = 0x04;
/// SMP `Pairing Failed` reason `Unspecified Reason`.
const SMP_ERR_UNSPECIFIED: u8 = 0x08;

/// IO Capability `NoInputNoOutput` — selects the Just Works association
/// model (no passkey, no numeric comparison).
const IO_CAP_NO_INPUT_NO_OUTPUT: u8 = 0x03;
/// AuthReq with no bonding: session encryption only, no MITM, no Secure
/// Connections, no keypress. Clearing the SC bit forces LE Legacy.
const AUTH_REQ_NO_BONDING: u8 = 0x00;
/// AuthReq requesting bonding: the Bonding flag set (bits 1-0 = `01`), still
/// no MITM / no Secure Connections. Used when persisting keys so a host can
/// keep the device (e.g. a keyboard) paired across reconnects.
const AUTH_REQ_BONDING: u8 = 0x01;
/// Key Distribution flag `EncKey` — the Long Term Key. Set in the Pairing
/// Response's Responder Key Distribution when bonding, so we distribute our
/// LTK.
const KEY_DIST_ENC_KEY: u8 = 0x01;
/// Maximum encryption key size we advertise/accept (bytes).
const MAX_ENC_KEY_SIZE: u8 = 16;
/// The Temporary Key for Just Works pairing: all zeros.
const TK_JUST_WORKS: [u8; 16] = [0u8; 16];

/// Where the responder pairing state machine is in the LE Legacy exchange.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    /// No pairing in progress (freshly connected, or finished/failed).
    Idle,
    /// Sent Pairing Response; awaiting the initiator's Pairing Confirm.
    WaitConfirm,
    /// Sent our Pairing Confirm; awaiting the initiator's Pairing Random.
    WaitRandom,
    /// Verified the initiator's random and derived the STK; awaiting the
    /// controller's Long Term Key Request to hand it over.
    WaitLtk,
}

/// The LE Legacy "Just Works" pairing responder — drives a connection from
/// the central's Pairing Request to an encrypted link.
///
/// Lifecycle per connection: [`Smp::begin`] with the two addresses when the
/// link comes up, feed every SMP PDU (L2CAP CID `0x0006`) to
/// [`Smp::handle`] and send back what it returns, then on the controller's
/// [`Event::LongTermKeyRequest`](super::Event::LongTermKeyRequest) look up
/// [`Smp::long_term_key`] and hand it to
/// [`Bluetooth::le_ltk_request_reply`](super::Bluetooth::le_ltk_request_reply).
/// An [`Event::EncryptionChange`](super::Event::EncryptionChange) with
/// `enabled` set confirms the link is encrypted; if bonding, call
/// [`Smp::distribute_keys`] then and send the returned PDUs. Call
/// [`Smp::reset`] on disconnect before reuse.
pub struct Smp {
    /// The pairing crypto bound to this controller.
    crypto: Crypto,
    /// Whether to bond: request the Bonding flag and distribute an LTK so
    /// the host keeps the device paired across reconnects. When `false`,
    /// pairing only encrypts the current session.
    bonding: bool,
    /// Progress through the pairing exchange.
    state: State,
    /// Pairing Request/Response PDUs and the two addresses — the fixed
    /// inputs to `c1`. Addresses are set by [`Self::begin`]; the PDUs are
    /// filled as they're exchanged.
    ctx: PairingContext,
    /// Our random value (`Srand`), generated for the confirm exchange.
    prnd: [u8; 16],
    /// The initiator's committed confirm value (`Mconfirm`).
    mconfirm: [u8; 16],
    /// The Short Term Key derived once the random exchange verifies — used
    /// to encrypt the initial pairing session.
    stk: [u8; 16],
    /// Whether `stk` holds a usable key for this pairing yet.
    have_stk: bool,
    /// The bonded Long Term Key distributed to the host — used to encrypt
    /// reconnections. Persists across [`Self::begin`]/[`Self::reset`] (in RAM
    /// only; a reboot loses the bond).
    bonded_ltk: [u8; 16],
    /// The EDIV identifying `bonded_ltk` on a reconnect's LTK request.
    bonded_ediv: u16,
    /// The Rand identifying `bonded_ltk` on a reconnect's LTK request.
    bonded_rand: [u8; 8],
    /// Whether a bond (LTK) has been established and stored.
    have_bond: bool,
    /// Whether keys have been distributed for the current pairing (so
    /// [`Self::distribute_keys`] runs once per pairing).
    distributed: bool,
}

/// A stored bond: the Long Term Key and the EDIV/Rand that identify it on a
/// reconnect. Export it with [`Smp::bond`] to persist across reboots, and
/// reload it with [`Smp::restore_bond`] at startup so a returning central
/// re-encrypts without re-pairing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Bond {
    /// The Long Term Key distributed to the central.
    pub ltk: [u8; 16],
    /// The EDIV identifying the LTK on a reconnect's LTK request.
    pub ediv: u16,
    /// The Rand identifying the LTK on a reconnect's LTK request.
    pub rand: [u8; 8],
}

/// The keys a bonding responder distributes after the link is first
/// encrypted, from [`Smp::distribute_keys`]. Each field is a complete SMP
/// PDU to send on the SMP channel (L2CAP CID `0x0006`), in order.
pub struct KeyDistribution {
    /// `Encryption Information` PDU: the Long Term Key.
    pub encryption_information: [u8; 17],
    /// `Master Identification` PDU: the EDIV and Rand identifying the LTK.
    pub master_identification: [u8; 11],
}

impl Smp {
    /// Creates a pairing responder, detecting the controller's `LE_Encrypt`
    /// convention (as [`self_test`] does). Returns [`Error::CryptoSelfTest`]
    /// if the crypto can't be trusted. Build one after the controller is up;
    /// reuse it across connections via [`Self::begin`]/[`Self::reset`].
    ///
    /// `bonding` selects whether to persist keys: `true` requests bonding and
    /// distributes an LTK (via [`Self::distribute_keys`]) so a host keeps the
    /// device paired across reconnects — needed for a real OS to accept a HID
    /// keyboard; `false` only encrypts the current session.
    pub fn new(bt: &mut Bluetooth, timer: &Timer, bonding: bool) -> Result<Self, Error> {
        Ok(Self {
            crypto: Crypto::new(bt, timer)?,
            bonding,
            state: State::Idle,
            ctx: PairingContext {
                preq: [0; 7],
                pres: [0; 7],
                ia: [0; 6],
                iat: 0,
                ra: [0; 6],
                rat: 0,
            },
            prnd: [0; 16],
            mconfirm: [0; 16],
            stk: [0; 16],
            have_stk: false,
            bonded_ltk: [0; 16],
            bonded_ediv: 0,
            bonded_rand: [0; 8],
            have_bond: false,
            distributed: false,
        })
    }

    /// Prepares for pairing on a new connection: records the initiator
    /// (central/peer) and responder (our) addresses and types — the `c1`
    /// inputs — and clears any prior pairing progress. `peer_addr`/`own_addr`
    /// are in HCI wire order (LSB first). A stored bond is preserved, so a
    /// reconnect can encrypt with the bonded LTK.
    pub fn begin(
        &mut self,
        peer_addr: [u8; 6],
        peer_addr_type: u8,
        own_addr: [u8; 6],
        own_addr_type: u8,
    ) {
        self.ctx.ia = peer_addr;
        self.ctx.iat = peer_addr_type;
        self.ctx.ra = own_addr;
        self.ctx.rat = own_addr_type;
        self.state = State::Idle;
        self.have_stk = false;
        self.distributed = false;
    }

    /// Clears the current pairing's state — call on disconnect before reuse.
    /// A stored bond (LTK) is kept, so the same device can reconnect
    /// encrypted without re-pairing (until a reboot loses it).
    pub fn reset(&mut self) {
        self.state = State::Idle;
        self.have_stk = false;
        self.distributed = false;
    }

    /// The key to answer a [`Long Term Key
    /// Request`](super::Event::LongTermKeyRequest): the freshly-derived STK
    /// for an in-progress pairing (`ediv`/`rand` both zero), or the stored
    /// bonded LTK when `ediv`/`rand` match those distributed earlier (a
    /// reconnect). `None` if neither applies — reject with
    /// [`Bluetooth::le_ltk_request_negative_reply`](super::Bluetooth::le_ltk_request_negative_reply).
    pub fn long_term_key(&self, ediv: u16, rand: [u8; 8]) -> Option<[u8; 16]> {
        if ediv == 0 && rand == [0u8; 8] && self.have_stk {
            return Some(self.stk);
        }
        if self.have_bond && ediv == self.bonded_ediv && rand == self.bonded_rand {
            return Some(self.bonded_ltk);
        }
        None
    }

    /// The current stored bond, if one has been established (by pairing or
    /// [`Self::restore_bond`]) — for persisting it so it survives a reboot.
    pub fn bond(&self) -> Option<Bond> {
        self.have_bond.then_some(Bond {
            ltk: self.bonded_ltk,
            ediv: self.bonded_ediv,
            rand: self.bonded_rand,
        })
    }

    /// Loads a previously-persisted [`Bond`] so a returning central can
    /// re-encrypt with the stored LTK without re-pairing. Call once at
    /// startup, before advertising, when a saved bond is available.
    pub fn restore_bond(&mut self, bond: &Bond) {
        self.bonded_ltk = bond.ltk;
        self.bonded_ediv = bond.ediv;
        self.bonded_rand = bond.rand;
        self.have_bond = true;
    }

    /// After the link is first encrypted (an
    /// [`Event::EncryptionChange`](super::Event::EncryptionChange) with
    /// `enabled`), distributes the bond keys: generates a fresh LTK and its
    /// EDIV/Rand, stores them, and returns the SMP PDUs to send so the host
    /// bonds. Returns `None` when not bonding, or already distributed this
    /// pairing, or no pairing just completed (a reconnect). Call once per
    /// [`Event::EncryptionChange`](super::Event::EncryptionChange).
    pub fn distribute_keys(
        &mut self,
        bt: &mut Bluetooth,
        timer: &Timer,
    ) -> Result<Option<KeyDistribution>, Error> {
        if !self.bonding || self.distributed || !self.have_stk {
            return Ok(None);
        }

        // Fresh LTK (two 8-byte draws), Rand (8 bytes), EDIV (2 bytes, forced
        // non-zero so it never collides with the STK request's zero EDIV).
        let mut ltk = [0u8; 16];
        ltk[0..8].copy_from_slice(&bt.le_rand(timer)?);
        ltk[8..16].copy_from_slice(&bt.le_rand(timer)?);
        let rand = bt.le_rand(timer)?;
        let ediv_bytes = bt.le_rand(timer)?;
        let mut ediv = u16::from_le_bytes([ediv_bytes[0], ediv_bytes[1]]);
        if ediv == 0 {
            ediv = 1;
        }

        self.bonded_ltk = ltk;
        self.bonded_rand = rand;
        self.bonded_ediv = ediv;
        self.have_bond = true;
        self.distributed = true;

        let mut encryption_information = [0u8; 17];
        encryption_information[0] = SMP_ENCRYPTION_INFORMATION;
        encryption_information[1..17].copy_from_slice(&ltk);

        let mut master_identification = [0u8; 11];
        master_identification[0] = SMP_MASTER_IDENTIFICATION;
        master_identification[1..3].copy_from_slice(&ediv.to_le_bytes());
        master_identification[3..11].copy_from_slice(&rand);

        Ok(Some(KeyDistribution {
            encryption_information,
            master_identification,
        }))
    }

    /// Handles one inbound SMP PDU, writing any response PDU into `out` and
    /// returning its length, or `None` if nothing should be sent. Drives the
    /// crypto through the controller, so it needs `bt`/`timer`.
    ///
    /// `out` need only be small (the largest response, a Pairing
    /// Confirm/Random, is 17 bytes). Send the returned bytes on the SMP
    /// channel (L2CAP CID `0x0006`).
    pub fn handle(
        &mut self,
        bt: &mut Bluetooth,
        timer: &Timer,
        pdu: &[u8],
        out: &mut [u8],
    ) -> Result<Option<usize>, Error> {
        let Some(&code) = pdu.first() else {
            return Ok(None);
        };
        match code {
            SMP_PAIRING_REQUEST => self.on_pairing_request(bt, timer, pdu, out),
            SMP_PAIRING_CONFIRM => self.on_pairing_confirm(bt, timer, pdu, out),
            SMP_PAIRING_RANDOM => self.on_pairing_random(bt, timer, pdu, out),
            SMP_PAIRING_FAILED => {
                // The peer aborted; return to idle without replying.
                self.state = State::Idle;
                Ok(None)
            }
            // Anything else (e.g. a Security Request, which a peripheral
            // sends rather than receives) isn't part of this responder flow.
            _ => Ok(None),
        }
    }

    /// Pairing Request → build and return our Pairing Response, and generate
    /// our random for the coming confirm exchange.
    fn on_pairing_request(
        &mut self,
        bt: &mut Bluetooth,
        timer: &Timer,
        pdu: &[u8],
        out: &mut [u8],
    ) -> Result<Option<usize>, Error> {
        if pdu.len() < 7 {
            return Ok(Some(fail(out, SMP_ERR_UNSPECIFIED)));
        }
        // Remember the request PDU verbatim (c1 uses its 7 bytes).
        self.ctx.preq.copy_from_slice(&pdu[0..7]);

        // Our random, from the controller's RNG (two 8-byte draws).
        self.prnd[0..8].copy_from_slice(&bt.le_rand(timer)?);
        self.prnd[8..16].copy_from_slice(&bt.le_rand(timer)?);

        // Pairing Response: code, IO cap, OOB (none), AuthReq, max key size,
        // initiator key distribution, responder key distribution. When
        // bonding we request the Bonding flag and advertise that we'll
        // distribute our LTK (responder EncKey); we request nothing from the
        // initiator (initiator key distribution = 0).
        let (authreq, rkd) = if self.bonding {
            (AUTH_REQ_BONDING, KEY_DIST_ENC_KEY)
        } else {
            (AUTH_REQ_NO_BONDING, 0x00)
        };
        let pres = [
            SMP_PAIRING_RESPONSE,
            IO_CAP_NO_INPUT_NO_OUTPUT,
            0x00,
            authreq,
            MAX_ENC_KEY_SIZE,
            0x00,
            rkd,
        ];
        self.ctx.pres.copy_from_slice(&pres);
        self.state = State::WaitConfirm;

        out[0..7].copy_from_slice(&pres);
        Ok(Some(7))
    }

    /// Pairing Confirm → store the initiator's confirm, compute and return
    /// ours (`c1(TK, Srand, …)`).
    fn on_pairing_confirm(
        &mut self,
        bt: &mut Bluetooth,
        timer: &Timer,
        pdu: &[u8],
        out: &mut [u8],
    ) -> Result<Option<usize>, Error> {
        if self.state != State::WaitConfirm || pdu.len() < 17 {
            return Ok(Some(fail(out, SMP_ERR_UNSPECIFIED)));
        }
        self.mconfirm.copy_from_slice(&pdu[1..17]);

        let confirm = self
            .crypto
            .c1(bt, timer, &TK_JUST_WORKS, &self.prnd, &self.ctx)?;
        self.state = State::WaitRandom;

        out[0] = SMP_PAIRING_CONFIRM;
        out[1..17].copy_from_slice(&confirm);
        Ok(Some(17))
    }

    /// Pairing Random → verify the initiator's random against its earlier
    /// confirm; on success derive the STK and return our random, else fail.
    fn on_pairing_random(
        &mut self,
        bt: &mut Bluetooth,
        timer: &Timer,
        pdu: &[u8],
        out: &mut [u8],
    ) -> Result<Option<usize>, Error> {
        if self.state != State::WaitRandom || pdu.len() < 17 {
            return Ok(Some(fail(out, SMP_ERR_UNSPECIFIED)));
        }
        let mut mrand = [0u8; 16];
        mrand.copy_from_slice(&pdu[1..17]);

        // The initiator's random must reproduce the confirm it committed to.
        let check = self
            .crypto
            .c1(bt, timer, &TK_JUST_WORKS, &mrand, &self.ctx)?;
        if check != self.mconfirm {
            self.state = State::Idle;
            return Ok(Some(fail(out, SMP_ERR_CONFIRM_FAILED)));
        }

        // STK = s1(TK, Srand, Mrand); the controller will ask for it next.
        self.stk = self
            .crypto
            .s1(bt, timer, &TK_JUST_WORKS, &self.prnd, &mrand)?;
        self.have_stk = true;
        self.state = State::WaitLtk;

        out[0] = SMP_PAIRING_RANDOM;
        out[1..17].copy_from_slice(&self.prnd);
        Ok(Some(17))
    }
}

/// What an [`Initiator::handle`] wants the caller to do next.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    /// Nothing to send or do for this PDU.
    Idle,
    /// Send the first `len` bytes of the caller's `out` buffer on the SMP
    /// channel (L2CAP CID `0x0006`).
    Send(usize),
    /// Phase 2 finished: the Short Term Key is ready. Command the controller
    /// to encrypt the link with
    /// [`Bluetooth::le_start_encryption`](super::Bluetooth::le_start_encryption)
    /// passing [`Initiator::short_term_key`], with `ediv` and `rand` both
    /// zero. No SMP PDU is sent for this step.
    StartEncryption,
    /// Pairing failed and a `Pairing Failed` PDU (the first `len` bytes of
    /// `out`) should be sent; the exchange is over.
    Failed(usize),
}

/// Where the initiator pairing state machine is in the LE Legacy exchange.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum InitState {
    /// No pairing started, or finished/failed.
    Idle,
    /// Sent Pairing Request; awaiting the responder's Pairing Response.
    WaitResponse,
    /// Sent our Pairing Confirm; awaiting the responder's Pairing Confirm.
    WaitConfirm,
    /// Sent our Pairing Random; awaiting the responder's Pairing Random.
    WaitRandom,
    /// Verified the responder and derived the STK; the caller has been told to
    /// start encryption, and (when bonding) the responder's key distribution
    /// is awaited on the SMP channel.
    WaitEncryption,
}

/// The LE Legacy "Just Works" pairing **initiator** (central) — drives a
/// connection from our Pairing Request to an encrypted link.
///
/// Lifecycle per connection: [`Initiator::begin`] with the two addresses when
/// the link comes up, [`Initiator::start_pairing`] to build the Pairing
/// Request and send it, then feed every inbound SMP PDU (L2CAP CID `0x0006`)
/// to [`Initiator::handle`] and act on the [`Action`] it returns — sending the
/// PDU it produced, or, on [`Action::StartEncryption`], calling
/// [`Bluetooth::le_start_encryption`](super::Bluetooth::le_start_encryption)
/// with [`Initiator::short_term_key`]. An
/// [`Event::EncryptionChange`](super::Event::EncryptionChange) with `enabled`
/// confirms the link is encrypted; when bonding, the responder then
/// distributes its LTK over SMP, which [`Initiator::handle`] stores into a
/// [`Bond`] (see [`Initiator::bond`]). Call [`Initiator::reset`] on disconnect
/// before reuse.
pub struct Initiator {
    /// The pairing crypto bound to this controller.
    crypto: Crypto,
    /// Whether to bond: request the responder distribute its LTK and persist
    /// it so a reconnect re-encrypts without re-pairing.
    bonding: bool,
    /// Progress through the pairing exchange.
    state: InitState,
    /// Pairing Request/Response PDUs and the two addresses — the fixed inputs
    /// to `c1`. For the initiator, `ia`/`iat` are *our* address (we initiate)
    /// and `ra`/`rat` the peer's.
    ctx: PairingContext,
    /// Our random value (`Mrand`), generated for the confirm exchange.
    mrand: [u8; 16],
    /// The responder's committed confirm value (`Sconfirm`).
    sconfirm: [u8; 16],
    /// The Short Term Key derived once the random exchange verifies.
    stk: [u8; 16],
    /// Whether `stk` holds a usable key for this pairing yet.
    have_stk: bool,
    /// The responder's distributed Long Term Key (bonding) — used to
    /// re-encrypt a reconnection.
    peer_ltk: [u8; 16],
    /// The EDIV identifying `peer_ltk` on a reconnect.
    peer_ediv: u16,
    /// The Rand identifying `peer_ltk` on a reconnect.
    peer_rand: [u8; 8],
    /// Whether a bond (the peer's LTK) has been received and stored.
    have_bond: bool,
}

impl Initiator {
    /// Creates a pairing initiator, detecting the controller's `LE_Encrypt`
    /// convention (as [`self_test`] does). Returns [`Error::CryptoSelfTest`]
    /// if the crypto can't be trusted. Build one after the controller is up;
    /// reuse it across connections via [`Self::begin`]/[`Self::reset`].
    ///
    /// `bonding` selects whether to persist keys: `true` asks the responder to
    /// distribute its LTK and stores it (a [`Bond`]) so a reconnect
    /// re-encrypts without re-pairing; `false` only encrypts the current
    /// session.
    pub fn new(bt: &mut Bluetooth, timer: &Timer, bonding: bool) -> Result<Self, Error> {
        Ok(Self {
            crypto: Crypto::new(bt, timer)?,
            bonding,
            state: InitState::Idle,
            ctx: PairingContext {
                preq: [0; 7],
                pres: [0; 7],
                ia: [0; 6],
                iat: 0,
                ra: [0; 6],
                rat: 0,
            },
            mrand: [0; 16],
            sconfirm: [0; 16],
            stk: [0; 16],
            have_stk: false,
            peer_ltk: [0; 16],
            peer_ediv: 0,
            peer_rand: [0; 8],
            have_bond: false,
        })
    }

    /// Prepares for pairing on a new connection: records our (initiator) and
    /// the peer's (responder) addresses and types — the `c1` inputs — and
    /// clears any prior pairing progress. Addresses are in HCI wire order (LSB
    /// first). A stored bond is preserved, so a reconnect can encrypt with it.
    pub fn begin(
        &mut self,
        own_addr: [u8; 6],
        own_addr_type: u8,
        peer_addr: [u8; 6],
        peer_addr_type: u8,
    ) {
        self.ctx.ia = own_addr;
        self.ctx.iat = own_addr_type;
        self.ctx.ra = peer_addr;
        self.ctx.rat = peer_addr_type;
        self.state = InitState::Idle;
        self.have_stk = false;
    }

    /// Clears the current pairing's state — call on disconnect before reuse.
    /// A stored bond is kept, so the same device can reconnect encrypted
    /// without re-pairing (until a reboot loses it).
    pub fn reset(&mut self) {
        self.state = InitState::Idle;
        self.have_stk = false;
    }

    /// Builds our Pairing Request into `out` and returns its length (7). Send
    /// the bytes on the SMP channel to open the exchange, then feed the
    /// responder's replies to [`Self::handle`].
    ///
    /// When bonding, the request asks the responder to distribute its LTK
    /// (Responder Key Distribution = `EncKey`) and sets the Bonding flag.
    pub fn start_pairing(&mut self, out: &mut [u8]) -> usize {
        let (authreq, rkd) = if self.bonding {
            (AUTH_REQ_BONDING, KEY_DIST_ENC_KEY)
        } else {
            (AUTH_REQ_NO_BONDING, 0x00)
        };
        // Pairing Request: code, IO cap, OOB (none), AuthReq, max key size,
        // initiator key distribution (none), responder key distribution.
        let preq = [
            SMP_PAIRING_REQUEST,
            IO_CAP_NO_INPUT_NO_OUTPUT,
            0x00,
            authreq,
            MAX_ENC_KEY_SIZE,
            0x00,
            rkd,
        ];
        self.ctx.preq.copy_from_slice(&preq);
        self.state = InitState::WaitResponse;
        out[0..7].copy_from_slice(&preq);
        7
    }

    /// The Short Term Key derived by pairing — pass it to
    /// [`Bluetooth::le_start_encryption`](super::Bluetooth::le_start_encryption)
    /// (with `ediv`/`rand` zero) when [`Self::handle`] returns
    /// [`Action::StartEncryption`]. All zeros until then.
    pub fn short_term_key(&self) -> [u8; 16] {
        self.stk
    }

    /// The bond received from the responder, if bonding completed — the peer's
    /// LTK and the EDIV/Rand that identify it. Persist it (see [`Bond`]) to
    /// survive a reboot; on a reconnect pass its fields to
    /// [`Bluetooth::le_start_encryption`](super::Bluetooth::le_start_encryption)
    /// to re-encrypt without re-pairing.
    pub fn bond(&self) -> Option<Bond> {
        self.have_bond.then_some(Bond {
            ltk: self.peer_ltk,
            ediv: self.peer_ediv,
            rand: self.peer_rand,
        })
    }

    /// Loads a previously-persisted [`Bond`] (the peer's LTK) so a reconnect
    /// can re-encrypt without re-pairing. Call at startup when a saved bond is
    /// available; then, on reconnect, encrypt with its fields rather than
    /// running [`Self::start_pairing`].
    pub fn restore_bond(&mut self, bond: &Bond) {
        self.peer_ltk = bond.ltk;
        self.peer_ediv = bond.ediv;
        self.peer_rand = bond.rand;
        self.have_bond = true;
    }

    /// Handles one inbound SMP PDU, driving the exchange forward. Writes any
    /// outgoing PDU into `out` and returns an [`Action`] telling the caller
    /// what to do (send it, start encryption, or nothing). Drives the crypto
    /// through the controller, so it needs `bt`/`timer`.
    ///
    /// `out` need only be small (the largest PDU produced, a Pairing
    /// Confirm/Random, is 17 bytes).
    pub fn handle(
        &mut self,
        bt: &mut Bluetooth,
        timer: &Timer,
        pdu: &[u8],
        out: &mut [u8],
    ) -> Result<Action, Error> {
        let Some(&code) = pdu.first() else {
            return Ok(Action::Idle);
        };
        match code {
            SMP_PAIRING_RESPONSE => self.on_pairing_response(bt, timer, pdu, out),
            SMP_PAIRING_CONFIRM => self.on_pairing_confirm(pdu, out),
            SMP_PAIRING_RANDOM => self.on_pairing_random(bt, timer, pdu, out),
            SMP_ENCRYPTION_INFORMATION => {
                // The responder's LTK (bonding). Store it; its EDIV/Rand
                // follow in a Master Identification.
                if pdu.len() >= 17 {
                    self.peer_ltk.copy_from_slice(&pdu[1..17]);
                }
                Ok(Action::Idle)
            }
            SMP_MASTER_IDENTIFICATION => {
                // The EDIV/Rand identifying the LTK just distributed — the
                // bond is now complete.
                if pdu.len() >= 11 {
                    self.peer_ediv = u16::from_le_bytes([pdu[1], pdu[2]]);
                    self.peer_rand.copy_from_slice(&pdu[3..11]);
                    self.have_bond = true;
                }
                Ok(Action::Idle)
            }
            SMP_PAIRING_FAILED => {
                self.state = InitState::Idle;
                Ok(Action::Idle)
            }
            // Anything else (e.g. a Security Request from the peripheral, which
            // we don't need since we initiate pairing ourselves) is ignored.
            _ => Ok(Action::Idle),
        }
    }

    /// Pairing Response → remember it, generate our random, and return our
    /// Pairing Confirm (`c1(TK, Mrand, …)`). The initiator commits first.
    fn on_pairing_response(
        &mut self,
        bt: &mut Bluetooth,
        timer: &Timer,
        pdu: &[u8],
        out: &mut [u8],
    ) -> Result<Action, Error> {
        if self.state != InitState::WaitResponse || pdu.len() < 7 {
            self.state = InitState::Idle;
            return Ok(Action::Failed(fail(out, SMP_ERR_UNSPECIFIED)));
        }
        // Remember the response PDU verbatim (c1 uses its 7 bytes).
        self.ctx.pres.copy_from_slice(&pdu[0..7]);

        // Our random, from the controller's RNG (two 8-byte draws).
        self.mrand[0..8].copy_from_slice(&bt.le_rand(timer)?);
        self.mrand[8..16].copy_from_slice(&bt.le_rand(timer)?);

        let confirm = self
            .crypto
            .c1(bt, timer, &TK_JUST_WORKS, &self.mrand, &self.ctx)?;
        self.state = InitState::WaitConfirm;

        out[0] = SMP_PAIRING_CONFIRM;
        out[1..17].copy_from_slice(&confirm);
        Ok(Action::Send(17))
    }

    /// Pairing Confirm (the responder's `Sconfirm`) → store it and reveal our
    /// random with a Pairing Random.
    fn on_pairing_confirm(&mut self, pdu: &[u8], out: &mut [u8]) -> Result<Action, Error> {
        if self.state != InitState::WaitConfirm || pdu.len() < 17 {
            self.state = InitState::Idle;
            return Ok(Action::Failed(fail(out, SMP_ERR_UNSPECIFIED)));
        }
        self.sconfirm.copy_from_slice(&pdu[1..17]);
        self.state = InitState::WaitRandom;

        out[0] = SMP_PAIRING_RANDOM;
        out[1..17].copy_from_slice(&self.mrand);
        Ok(Action::Send(17))
    }

    /// Pairing Random (the responder's `Srand`) → verify it reproduces the
    /// confirm the responder committed to; on success derive the STK and tell
    /// the caller to start encryption, else return a Pairing Failed to send.
    fn on_pairing_random(
        &mut self,
        bt: &mut Bluetooth,
        timer: &Timer,
        pdu: &[u8],
        out: &mut [u8],
    ) -> Result<Action, Error> {
        if self.state != InitState::WaitRandom || pdu.len() < 17 {
            self.state = InitState::Idle;
            return Ok(Action::Failed(fail(out, SMP_ERR_UNSPECIFIED)));
        }
        let mut srand = [0u8; 16];
        srand.copy_from_slice(&pdu[1..17]);

        // The responder's random must reproduce the confirm it committed to.
        let check = self
            .crypto
            .c1(bt, timer, &TK_JUST_WORKS, &srand, &self.ctx)?;
        if check != self.sconfirm {
            self.state = InitState::Idle;
            return Ok(Action::Failed(fail(out, SMP_ERR_CONFIRM_FAILED)));
        }

        // STK = s1(TK, Srand, Mrand); command the controller to encrypt with it.
        self.stk = self
            .crypto
            .s1(bt, timer, &TK_JUST_WORKS, &srand, &self.mrand)?;
        self.have_stk = true;
        self.state = InitState::WaitEncryption;
        Ok(Action::StartEncryption)
    }
}

/// Writes a `Pairing Failed` PDU with `reason` into `out`, returning its
/// length (2).
fn fail(out: &mut [u8], reason: u8) -> usize {
    out[0] = SMP_PAIRING_FAILED;
    out[1] = reason;
    2
}
