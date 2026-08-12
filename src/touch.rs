//! Driver for the DSI touchscreen's touch input, read from a shared
//! touch buffer the VideoCore firmware keeps updated, rather than over
//! I2C.
//!
//! On the official Raspberry Pi 7" touchscreen (and its many
//! electrically identical clones — see [`crate::mailbox`]'s doc comment
//! on how a framebuffer ends up on this same display), the FT5406
//! capacitive touch controller is wired to a VideoCore-only I2C bus this
//! crate's own `i2c.rs` (BSC1, on the 40-pin header) can't reach at
//! all. Instead, the firmware itself polls the controller and needs a
//! buffer to continuously rewrite with the current touch points. This
//! driver supplies its own (a fixed `static`, handed to the firmware
//! once via
//! [`Mailbox::set_touch_buffer_address`](crate::mailbox::Mailbox::set_touch_buffer_address),
//! "Set Touchbuffer") rather than asking the firmware for one of *its*
//! choosing ("Get Touchbuffer") — see that method's doc comment for
//! why: on real hardware, `GET_TOUCHBUF`'s returned address landed
//! outside RAM entirely and hung the core on the first read.
//!
//! Register layout, the tag ids, and the event-type/id bit-packing
//! below all follow the Linux kernel's `rpi-ft5406` driver
//! (`drivers/input/touchscreen/rpi-ft5406.c`), the authoritative
//! reference for this firmware-mediated buffer's format — including
//! that same driver's own history of moving from `GET_TOUCHBUF` to
//! `SET_TOUCHBUF` as its primary mechanism.
//!
//! ## What this driver deliberately doesn't do
//!
//! The Linux driver overwrites the buffer's `num_points` byte with a
//! sentinel (`99`) after each read, so it can tell "the firmware hasn't
//! refreshed since I last looked" apart from "genuinely no touches" and
//! skip re-reporting an unchanged frame to the input subsystem. This
//! driver never writes to the buffer at all — it stays purely a reader
//! of firmware-owned shared memory — and re-reads the same unchanged
//! frame harmlessly if nothing changed:
//! [`poll`](crate::touch::TouchScreen::poll)'s edge-triggered
//! [`TouchEvent`](crate::touch::TouchEvent)s are computed by diffing
//! this poll's ids against the previous poll's, so an unchanged frame
//! just produces no events, not incorrect ones.
//!
//! ## Torn reads
//!
//! The firmware updates this buffer with no locking or handshake this
//! driver can observe, so a read here can race a firmware write and
//! see a partially-updated frame (as the Linux driver's own comments
//! acknowledge). Not defended against here either — the same
//! inherent limitation, not a bug specific to this implementation.

use crate::cache::invalidate_range;
use crate::mailbox::Mailbox;

/// Maximum simultaneous touch points the firmware's buffer tracks —
/// fixed by the FT5406 controller itself.
const MAX_TOUCHES: usize = 10;

/// Byte size of the firmware's touch buffer: a 3-byte header
/// (`device_mode`, `gesture_id`, `num_points`) followed by
/// [`MAX_TOUCHES`] fixed 6-byte point entries.
const BUFFER_SIZE: usize = 3 + MAX_TOUCHES * 6;

/// A point's `event_type` field: this is the first frame this point
/// touched down.
const EVENT_TOUCH_DOWN: u8 = 0;
/// A point's `event_type` field: the point is still down, possibly at
/// a new position. The third value, `1`, is `EVENT_TOUCH_UP` (the last
/// frame before the point is removed from the buffer entirely) — not
/// checked for by name here, since [`TouchScreen::poll`] only branches
/// on "should this point's position be reported" (`DOWN`/`CONTACT`);
/// an id's actual release is detected separately, by the id dropping
/// out of the buffer on a later poll (see [`TouchEvent::Released`]).
const EVENT_TOUCH_CONTACT: u8 = 2;

/// The firmware's "nothing new since your last poll" sentinel value for
/// `num_points` — confirmed on real hardware (see [`TouchScreen::poll`]).
/// Not a real point count, and specifically *not* `0..=MAX_TOUCHES`
/// clamped down to [`MAX_TOUCHES`]: doing that misreads "nothing new"
/// as "ten points," decoding whatever stale bytes happen to sit in
/// those slots (old data from an earlier, genuinely larger touch) as
/// phantom touches.
const NO_NEW_DATA: u8 = 99;

/// One active touch point.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Touch {
    /// The controller's own touch-slot id (`0..10`) — stable for as
    /// long as this physical touch stays down, then free to be reused
    /// by a later, unrelated touch.
    pub id: u8,
    /// X position, 0..4095 (12-bit).
    pub x: u16,
    /// Y position, 0..4095 (12-bit).
    pub y: u16,
}

/// The active touch points read by one [`TouchScreen::poll`], indexed
/// by touch id.
#[derive(Clone, Copy)]
pub struct TouchReport {
    touches: [Option<Touch>; MAX_TOUCHES],
}

impl TouchReport {
    /// Iterates the touch points active this poll.
    pub fn touches(&self) -> impl Iterator<Item = Touch> + '_ {
        self.touches.iter().filter_map(|touch| *touch)
    }

    /// The active touch point with the given controller id, if any.
    pub fn touch(&self, id: u8) -> Option<Touch> {
        self.touches.get(id as usize).copied().flatten()
    }
}

/// A touch id's transition between two consecutive polls.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TouchEvent {
    /// A touch id newly appeared this poll (wasn't active last poll).
    Pressed(Touch),
    /// A touch id present last poll is no longer in the buffer at all
    /// this poll.
    Released {
        /// The touch id that was released.
        id: u8,
    },
}

/// A newly-read [`TouchReport`] together with the id transitions since
/// the previous poll, produced by [`TouchScreen::poll`]. Read current
/// positions from [`Self::report`], and iterate [`Self::events`] for
/// press/release transitions.
pub struct TouchUpdate {
    previous_ids: u16,
    current_ids: u16,
    report: TouchReport,
}

impl TouchUpdate {
    /// The touch points active as of this poll.
    pub fn report(&self) -> &TouchReport {
        &self.report
    }

    /// Iterates the press/release events between the previous poll and
    /// this one.
    pub fn events(&self) -> TouchEventIter<'_> {
        TouchEventIter {
            update: self,
            id: 0,
        }
    }
}

/// Iterator over the [`TouchEvent`]s in a [`TouchUpdate`] — see
/// [`TouchUpdate::events`].
pub struct TouchEventIter<'a> {
    update: &'a TouchUpdate,
    id: u8,
}

impl Iterator for TouchEventIter<'_> {
    type Item = TouchEvent;

    fn next(&mut self) -> Option<TouchEvent> {
        while (self.id as usize) < MAX_TOUCHES {
            let mask = 1u16 << self.id;
            let id = self.id;
            self.id += 1;

            let was_active = self.update.previous_ids & mask != 0;
            let is_active = self.update.current_ids & mask != 0;
            if is_active && !was_active {
                // `poll` only ever sets a bit in `current_ids` alongside
                // storing the matching `Touch`, so this should always be
                // `Some` -- but if it somehow isn't, skip rather than
                // unwrap: there's nothing to hand out, not a fatal error.
                if let Some(touch) = self.update.report.touch(id) {
                    return Some(TouchEvent::Pressed(touch));
                }
                continue;
            }
            if was_active && !is_active {
                return Some(TouchEvent::Released { id });
            }
        }
        None
    }
}

/// The buffer handed to the firmware via
/// [`Mailbox::set_touch_buffer_address`](crate::mailbox::Mailbox::set_touch_buffer_address).
/// A `static`, not a [`TouchScreen`] field: once the firmware has this
/// address it writes there indefinitely, so the memory must never
/// move — unlike a struct field, which moves whenever its owning value
/// does (e.g. `TouchScreen::new`'s return value moving into the
/// caller's variable). Cache-line aligned so [`invalidate_range`]'s
/// line-granularity invalidation never touches a neighboring, unrelated
/// static.
#[repr(C, align(64))]
struct TouchBuffer([u8; BUFFER_SIZE]);

static mut TOUCH_BUFFER: TouchBuffer = TouchBuffer([0; BUFFER_SIZE]);

/// Reads the DSI touchscreen's current touch points, kept updated by
/// the VideoCore firmware in a buffer this driver owns and supplies to
/// it (see [`Mailbox::set_touch_buffer_address`](crate::mailbox::Mailbox::set_touch_buffer_address)).
///
/// Build one with [`Self::new`], then call [`Self::poll`] repeatedly.
/// Unlike a USB HID device, there's no transfer to pace — this just
/// re-reads memory the firmware writes on its own, so poll as often as
/// a caller finds useful (e.g. once per display frame). Only ever
/// construct one at a time — every instance shares the same underlying
/// `TOUCH_BUFFER` static, so a second, concurrently-used instance
/// would race the first.
pub struct TouchScreen {
    address: u32,
    known_ids: u16,
    /// The touch points as of the last poll that actually carried new
    /// data — reused as-is when a later poll catches the firmware's
    /// [`NO_NEW_DATA`] sentinel, so a "nothing changed" poll still
    /// reports the current, unchanged positions instead of dropping
    /// them.
    last_touches: [Option<Touch>; MAX_TOUCHES],
}

impl TouchScreen {
    /// Hands the firmware this driver's own buffer address (tag
    /// `0x0004_801f`, "Set Touchbuffer") so it starts writing touch
    /// points there. That address never needs re-sending — it's a fixed
    /// `static` — so this is the only mailbox call this driver ever
    /// makes; [`Self::poll`] reads the buffer directly from then on.
    pub fn new(mailbox: &mut Mailbox) -> Result<Self, crate::mailbox::Error> {
        // `addr_of!` only takes the static's address -- doesn't need
        // `unsafe`, unlike forming a `&`/`&mut` reference to it (which
        // could alias with whatever the firmware does to its contents
        // over the bus).
        let address = core::ptr::addr_of!(TOUCH_BUFFER) as u32;
        mailbox.set_touch_buffer_address(address)?;
        Ok(Self {
            address,
            known_ids: 0,
            last_touches: [None; MAX_TOUCHES],
        })
    }

    /// This driver's touch buffer address, as handed to the firmware —
    /// useful for confirming [`Self::new`] ran and computed a plausible
    /// address while diagnosing a display that isn't producing touch
    /// data.
    pub fn address(&self) -> u32 {
        self.address
    }

    /// Reads the current touch points, returning them together with the
    /// press/release transitions since the previous poll (see
    /// [`TouchUpdate`]).
    ///
    /// Confirmed on real hardware: the firmware writes `NO_NEW_DATA`
    /// (`99`) into `num_points` between real updates, not a real point
    /// count — this must be recognized and treated as "nothing changed,
    /// don't touch state," not clamped down to `MAX_TOUCHES` like an
    /// out-of-range real count would be. Clamping it was the actual bug
    /// behind an early version of this driver appearing to read "stale
    /// data": every `NO_NEW_DATA` poll was misread as ten real points,
    /// decoding whatever old bytes happened to still sit in those slots
    /// (positions from an earlier, genuinely larger touch) as phantom
    /// touches.
    pub fn poll(&mut self) -> TouchUpdate {
        let buffer = self.read_buffer();

        if buffer[2] == NO_NEW_DATA {
            return TouchUpdate {
                previous_ids: self.known_ids,
                current_ids: self.known_ids,
                report: TouchReport {
                    touches: self.last_touches,
                },
            };
        }

        let num_points = (buffer[2] as usize).min(MAX_TOUCHES);
        let mut touches = [None; MAX_TOUCHES];
        let mut current_ids = 0u16;

        for i in 0..num_points {
            let point = &buffer[3 + i * 6..3 + i * 6 + 6];
            let (xh, xl, yh, yl) = (point[0], point[1], point[2], point[3]);
            let id = (yh >> 4) & 0xf;
            let event_type = (xh >> 6) & 0x3;

            // `id` is a 4-bit field (0..16), but the buffer only has
            // `MAX_TOUCHES` (10) slots. Defense in depth against a torn
            // read (see the module doc comment) producing an
            // out-of-range id -- skip rather than index out of bounds.
            if id as usize >= MAX_TOUCHES {
                continue;
            }

            // Only counted as active alongside actually storing a
            // `Touch` -- unlike the reference driver, which also counts
            // an `EVENT_TOUCH_UP` entry as still "known" for one extra
            // frame before its release fires. Keeping "active" and
            // "has a reported position" the same bitmask by construction
            // rules out an id going active with no `Touch` behind it,
            // which `TouchEventIter` would otherwise have no position to
            // hand out for (this drove a real `unwrap`-on-`None` panic
            // during bring-up: an id's first-ever appearance in the
            // buffer was, on that occasion, an `EVENT_TOUCH_UP` entry).
            // The cost is releasing one poll earlier than the reference
            // driver in the case it specifically handled -- an
            // acceptable trade for a driver that can't panic.
            if event_type == EVENT_TOUCH_DOWN || event_type == EVENT_TOUCH_CONTACT {
                let x = (((xh & 0xf) as u16) << 8) | xl as u16;
                let y = (((yh & 0xf) as u16) << 8) | yl as u16;
                current_ids |= 1 << id;
                touches[id as usize] = Some(Touch { id, x, y });
            }
        }

        let previous_ids = self.known_ids;
        self.known_ids = current_ids;
        self.last_touches = touches;

        TouchUpdate {
            previous_ids,
            current_ids,
            report: TouchReport { touches },
        }
    }

    /// Invalidates the ARM cache over the touch buffer (the firmware
    /// writes it over the bus, outside the ARM core's cache — the same
    /// reasoning as [`crate::mailbox`]'s call buffer) and reads it back
    /// byte by byte.
    fn read_buffer(&self) -> [u8; BUFFER_SIZE] {
        invalidate_range(self.address, BUFFER_SIZE);
        let mut buffer = [0u8; BUFFER_SIZE];
        for (i, byte) in buffer.iter_mut().enumerate() {
            // Safety: `self.address` is the buffer address the firmware
            // itself handed out, sized to exactly `BUFFER_SIZE`.
            *byte = unsafe { core::ptr::read_volatile((self.address as *const u8).add(i)) };
        }
        buffer
    }
}
