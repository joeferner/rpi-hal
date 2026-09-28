#![no_std]
#![no_main]

// The card: mount it, read `kickstart.toml`, load the web assets under
// `/WWW` into RAM, come up on Ethernet at the address the lease gives, and
// serve them there over HTTP.
//
// What it shows is the storage side of a board -- `storage::mount`,
// `config::load` and `config::save`, `site::load` -- and the web module's
// socket loop with `SiteFiles` as the whole router: `http://<address>/`
// is `/WWW/index.html`, any other path is the file of that name, and a
// path the card does not have is a 404.
//
// On the card, beside `config.txt`:
//
//   kickstart.toml -- optional. Copy `kickstart.toml.example` from this
//                     repository. This example reads `hostname` -- only to
//                     check it and write it back, since nothing here
//                     answers to a name -- and reports any key the schema
//                     does not know.
//   www/           -- optional: the web assets, one console line each. With
//                     none, every path is a 404.
//
// A `hostname` written as `name.local` is accepted, and the file is then
// saved back through `config::save` with the bare `name` -- which is how
// this example exercises writing as well as reading. The saved file is
// regenerated whole, so it loses the template's comments. It is dated
// 1980: nothing here sets the clock (see `ntp_tls`), and a board must not
// have to wait for the network to write a file.
//
// A bad value is reported with its line and column and that setting falls
// back to its default, rather than halting: an example that can still come
// up should.
//
// Build it with `scripts/build-example.sh site`.

extern crate alloc;

use alloc::string::String;

use common::settings::{self, Card, Settings};
use picoserve::Router;
use rpi_kickstart::config::{self, Spanned};
use rpi_kickstart::site::{self, Site};
use rpi_kickstart::{logln, net, web};
use static_cell::StaticCell;

mod common;

static SITE: StaticCell<Site> = StaticCell::new();

#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let mut board = common::boot("site");

    let loaded = settings::read(&mut board);
    let mut card = loaded.card;
    if let Some(card) = card.as_mut() {
        last_written(card);
    }
    if let Some((mut settings, text)) = loaded.settings
        && let Some(hostname) = settings::hostname(&settings, &text)
    {
        logln!("settings: hostname = {hostname:?}");
        if let Some(card) = card.as_mut() {
            normalize_hostname(card, &mut settings, hostname);
        }
    }

    let site = SITE.init(match card.as_mut() {
        Some(card) => site::load(card).unwrap_or_else(|e| {
            logln!(
                "site: reading /{} failed: {e}; serving no pages",
                site::WWW_DIR
            );
            Site::default()
        }),
        None => Site::default(),
    });
    logln!(
        "site: {} {}",
        site.len(),
        if site.len() == 1 { "file" } else { "files" }
    );

    let usb = common::usb(&mut board);
    let mut hardware = net::Hardware::new().usb(usb.dwc2);
    let interface = common::discover(&mut hardware, &board, &usb);

    common::start(&board, &usb, interface, |spawner, stack| {
        for id in 0..common::WEB_POOL {
            spawner.spawn(web_task(id, stack, site).unwrap());
        }
    })
}

// A task of the example's own, because a task cannot be generic -- and the
// router is the example's too, since routing is what differs from board to
// board. This one has no routes at all: `SiteFiles` is the fallback every
// board's router is built over, and here it is the whole of it.
#[embassy_executor::task(pool_size = common::WEB_POOL)]
async fn web_task(id: usize, stack: embassy_net::Stack<'static>, site: &'static Site) -> ! {
    let app = Router::from_service(web::SiteFiles(site));
    web::serve(id, stack, &app, &web::ServeConfig::DEFAULT).await
}

/// When the settings file was last written: dated by the clock of the
/// board that wrote it, or no timestamp if that board had none.
fn last_written(card: &mut Card) {
    let Ok(root) = card.root_dir() else {
        return;
    };
    let Some(entry) = root.get(settings::FILE) else {
        return;
    };
    let at = entry.modified();
    if at == resident_fat::DateTime::EPOCH {
        logln!("settings: {} carries no timestamp", settings::FILE);
    } else {
        logln!(
            "settings: {} last written {:04}-{:02}-{:02} {:02}:{:02}:{:02} UTC",
            settings::FILE,
            at.year,
            at.month,
            at.day,
            at.hour,
            at.minute,
            at.second
        );
    }
}

/// Writes the settings back with `hostname` as the label it resolved to,
/// if the file spelled it some other way — `kickstart.local` for
/// `kickstart`, which `value::label` accepts and drops.
///
/// This example's use of `config::save`, and a deliberate trigger for it:
/// put `.local` on the name, boot, and the file on the card comes back
/// without it.
fn normalize_hostname(card: &mut Card, settings: &mut Settings, name: &str) {
    let Some(written) = &settings.hostname else {
        return;
    };
    if written.as_ref() == name {
        return;
    }
    // The span is carried over for form's sake; nothing reads it on the
    // way out, and the next load gives the new file's own.
    settings.hostname = Some(Spanned::new(written.span(), String::from(name)));
    match config::save(card, settings::FILE, settings) {
        Ok(text) => logln!(
            "settings: wrote {} back with hostname = {name:?} ({} bytes)",
            settings::FILE,
            text.len()
        ),
        Err(e) => logln!("settings: saving {} failed: {e}", settings::FILE),
    }
}
