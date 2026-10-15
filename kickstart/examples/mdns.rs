#![no_std]
#![no_main]

// The mDNS responder on a real link: read the name from `kickstart.toml`,
// bring up Ethernet, take a DHCP lease, and answer to `<hostname>.local`
// -- `kickstart.local` if the card gives no name.
//
// Ethernet only; `wifi` adds the radio as a fallback. A 3B and a 3B+ have
// different Ethernet chips and neither is named here -- `net::discover`
// finds whichever is on the bus. The only example that answers to a name;
// the others are reached at the address on their `DHCP:` line.
//
// On the card, beside `config.txt`, optionally:
//
//   kickstart.toml -- `hostname`: letters, digits and hyphens, up to 63. A
//                     trailing `.local` is accepted and dropped. See
//                     `kickstart.toml.example`.
//
// No card, no file or a bad value each leave the board on `kickstart`,
// with a line saying why.
//
// Two pieces of plumbing are easy to miss and each produces the same
// silence -- a name that does not resolve, with nothing reported:
//
//   * `EthernetConfig::all_multicast`, which `common::start` sets. The chip
//     drops multicast before the host sees it, and every mDNS query and
//     announcement is multicast. Nothing else a board does notices,
//     because DHCP is broadcast -- which is why it takes a responder to
//     surface it.
//   * `Stack::join_multicast_group`, which `mdns::run` does itself.
//
// Verify it from another machine on the same link:
//
//     ping kickstart.local
//     avahi-resolve -n kickstart.local
//     dig +short @<the board's address> -p 5353 kickstart.local
//
// (or whatever `hostname` the file sets).
//
// **Not `dig @224.0.0.251`.** That looks like the obvious test and it
// cannot work: `dig` checks that a reply comes from the server it asked,
// and a responder answers a legacy query from its own address rather than
// from the group -- so `dig` discards a perfectly good answer and reports
// a timeout. Asking the board's own address is the same legacy path (RFC
// 6762 §6.7) with a source `dig` will accept.
//
// `avahi-resolve` can answer out of its cache, which a responder fills by
// announcing. It therefore proves the announcement arrived, *not* that a
// query was answered; only the `dig` line above proves that.
//
// Build it with `scripts/build-example.sh mdns` (kernel7.img) or
// `scripts/build-example64.sh mdns` (kernel8.img).

use core::cell::Cell;

use common::settings;
use critical_section::Mutex;
use rpi_kickstart::{logln, mdns, net};

mod common;

/// The name the board answers to when the card gives none.
const DEFAULT_NAME: &str = "kickstart";

/// The name the board answers to.
///
/// A static behind a function rather than a value passed down, because
/// `mdns::run` takes `fn() -> &'static str` — so a board that changes its
/// name at run time, from a web form say, answers to the new one on the
/// next query. This example sets it once, at boot, from the settings,
/// which is the simple end of that seam.
static NAME: Mutex<Cell<&'static str>> = Mutex::new(Cell::new(DEFAULT_NAME));

fn name() -> &'static str {
    critical_section::with(|cs| NAME.borrow(cs).get())
}

#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let mut board = common::boot("mdns");

    // Only `hostname` is wanted from the card, and the card is not kept.
    if let Some((settings, text)) = settings::read(&mut board).settings
        && let Some(hostname) = settings::hostname(&settings, &text)
    {
        critical_section::with(|cs| NAME.borrow(cs).set(hostname));
    }
    logln!("mdns: answering to {}.local", name());

    let usb = common::usb(&mut board);
    let mut hardware = net::Hardware::new().usb(usb.dwc2);
    let interface = common::discover(&mut hardware, &board, &usb);

    common::start(&board, &usb, interface, |spawner, stack| {
        spawner.spawn(mdns_task(stack).unwrap());
    })
}

// The crate hands back an `async fn` and the application wraps it, which is
// the whole reason `mdns::run` is not a task itself: an
// `#[embassy_executor::task]` cannot be generic, so the concrete types have
// to be named somewhere the application owns.
#[embassy_executor::task]
async fn mdns_task(stack: embassy_net::Stack<'static>) -> ! {
    mdns::run(stack, name).await
}
