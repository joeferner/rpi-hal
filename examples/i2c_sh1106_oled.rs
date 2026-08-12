//! Drives a 1.3" SH1106-based 128x64 I2C OLED module (the common
//! "HuiTec"-style module also sold under various names) over I2C1
//! (GPIO2 SDA1/GPIO3 SCL1): draws a static test pattern once, then
//! blinks the whole display inverted/normal once a second as a
//! lightweight "still alive" indicator.
//!
//! Init sequence and command-byte protocol confirmed against a real
//! Arduino U8glib driver for this exact module (`tmp/IIC_OLED_Libraries-FZ1113/1.3Inch`,
//! `U8glib/utility/u8g_dev_sh1106_128x64.c`'s `..._huitec_init_seq`
//! and `u8g_com_arduino_ssd_i2c.c`'s I2C framing), not guessed from a
//! generic SSD1306 tutorial -- this module's init sequence has real,
//! SH1106-specific values (e.g. `0xAD 0x8B` to enable the internal
//! charge pump, `0xDA 0x12` for COM pin config) that a
//! SSD1306-flavored init wouldn't reliably reproduce.
//!
//! Wiring: OLED SDA -> GPIO2, SCL -> GPIO3, plus VCC/GND. Most of
//! these modules already carry their own I2C pull-ups; add external
//! 4.7k ones if the display doesn't respond (see `i2c_scan.rs` to
//! confirm it ACKs at 0x3C before suspecting the init sequence).
#![no_std]
#![no_main]

use core::fmt::Write as _;
use embedded_hal::i2c::I2c as _;
use rpi_hal::halt;
use rpi_hal::{i2c::I2c, pac, uart::Uart};

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// SH1106's fixed 7-bit I2C address on this module (`0x3C`) -- matches
/// `u8g_com_arduino_ssd_i2c.c`'s `I2C_SLA` (`0x3c*2`, an 8-bit
/// address-plus-R/W-bit form; this driver's `I2c` takes the plain
/// 7-bit form instead, per `embedded-hal`'s convention).
const OLED_ADDR: u8 = 0x3c;

/// Control byte prefixing a run of command bytes (`Co=0`, `D/C=0`):
/// every byte until the next `STOP` is a command.
const CONTROL_CMD: u8 = 0x00;

/// Control byte prefixing a run of display-RAM data bytes (`Co=0`,
/// `D/C=1`).
const CONTROL_DATA: u8 = 0x40;

const WIDTH: usize = 128;
const HEIGHT: usize = 64;
const PAGES: usize = HEIGHT / 8;

/// The real command bytes from `u8g_dev_sh1106_128x64_huitec_init_seq`
/// -- `U8G_ESC_CS`/`U8G_ESC_ADR`/`U8G_ESC_RST` entries in that sequence
/// are pseudo-ops for U8glib's generic transport layer (chip-select,
/// register-select, reset-pulse), not real bytes sent over the wire,
/// so they're omitted here; everything below is sent verbatim.
const INIT_SEQUENCE: [u8; 25] = [
    0xae, // display off
    0x02, 0x10, // column start address = 2 (low nibble 2, high nibble 0) --
    // this module's 128 visible columns start at column 2 of the
    // SH1106's 132-column internal RAM
    0x40, // display start line = 0
    0xb0, // page address = 0
    0x81, 0x80, // contrast = 0x80
    0xa1, // segment remap (mirror X)
    0xa6, // normal (non-inverted) display
    0xa8, 0x3f, // multiplex ratio = 63 (64 rows)
    0xad, 0x8b, // enable internal charge pump
    0x30, // charge pump voltage
    0xc8, // COM output scan direction, reversed (mirror Y)
    0xd3, 0x00, // display offset = 0
    0xd5, 0x80, // display clock divider/oscillator frequency
    0xd9, 0x1f, // pre-charge period
    0xda, 0x12, // COM pins hardware configuration
    0xdb, 0x40, // VCOMH deselect level
];

const DISPLAY_ON: u8 = 0xaf;
const DISPLAY_NORMAL: u8 = 0xa6;
const DISPLAY_INVERTED: u8 = 0xa7;

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let mut i2c = I2c::<pac::BSC1>::init(&peripherals.GPIO, peripherals.BSC1, 0x05dc);

    if send_command(&mut i2c, OLED_ADDR, &INIT_SEQUENCE).is_err() {
        let _ = writeln!(
            uart,
            "SH1106 didn't ack at 0x{OLED_ADDR:02X} -- check wiring"
        );
        loop {
            unsafe { core::arch::asm!("wfe") };
        }
    }
    let _ = send_command(&mut i2c, OLED_ADDR, &[DISPLAY_ON]);

    let mut framebuffer = [0u8; WIDTH * PAGES];
    draw_test_pattern(&mut framebuffer);
    write_frame(&mut i2c, &framebuffer);
    let _ = writeln!(uart, "SH1106 initialized, test pattern drawn");

    let mut inverted = false;
    loop {
        delay(150_000_000);
        inverted = !inverted;
        let command = if inverted {
            DISPLAY_INVERTED
        } else {
            DISPLAY_NORMAL
        };
        let _ = send_command(&mut i2c, OLED_ADDR, &[command]);
    }
}

/// Sends `bytes` as one complete command-mode I2C transaction.
fn send_command(i2c: &mut I2c, addr: u8, bytes: &[u8]) -> Result<(), rpi_hal::i2c::Error> {
    let mut buf = [0u8; INIT_SEQUENCE.len() + 1];
    buf[0] = CONTROL_CMD;
    buf[1..=bytes.len()].copy_from_slice(bytes);
    i2c.write(addr, &buf[..=bytes.len()])
}

/// Sets the pixel at (`x`, `y`) in a page-major, MSB-unused-bit-low
/// framebuffer (each byte holds 8 vertically-stacked pixels, LSB =
/// top row of that page) -- SH1106/SSD1306's native RAM layout.
fn set_pixel(framebuffer: &mut [u8; WIDTH * PAGES], x: usize, y: usize) {
    if x >= WIDTH || y >= HEIGHT {
        return;
    }
    let page = y / 8;
    framebuffer[page * WIDTH + x] |= 1 << (y % 8);
}

/// A couple of diagonal lines forming an X corner-to-corner -- simple
/// to compute, and still enough to confirm addressing is correct
/// across the whole display (a column/page offset bug would show up
/// immediately as a bent or discontinuous line).
fn draw_test_pattern(framebuffer: &mut [u8; WIDTH * PAGES]) {
    for x in 0..WIDTH {
        let y = x * (HEIGHT - 1) / (WIDTH - 1);
        set_pixel(framebuffer, x, y);
        set_pixel(framebuffer, x, HEIGHT - 1 - y);
    }
}

/// Writes the whole framebuffer out, one page (8 pixel rows, 128
/// bytes) at a time -- mirrors `u8g_dev_sh1106_128x64_fn`'s
/// `U8G_DEV_MSG_PAGE_NEXT` handler: a column/page-select command
/// transaction, then a data transaction for that page's 128 bytes.
fn write_frame(i2c: &mut I2c, framebuffer: &[u8; WIDTH * PAGES]) {
    for page in 0..PAGES {
        let _ = send_command(i2c, OLED_ADDR, &[0x10, 0x02, 0xb0 | page as u8]);

        let mut buf = [0u8; WIDTH + 1];
        buf[0] = CONTROL_DATA;
        buf[1..].copy_from_slice(&framebuffer[page * WIDTH..(page + 1) * WIDTH]);
        let _ = i2c.write(OLED_ADDR, &buf);
    }
}

fn delay(cycles: u32) {
    for _ in 0..cycles {
        unsafe { core::arch::asm!("nop") };
    }
}
