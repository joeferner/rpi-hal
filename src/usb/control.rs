//! USB control-transfer support built on
//! [`crate::usb::dwc2::Dwc2Host`] — the standard requests that enumerate
//! and configure a device, plus the hub-class requests that drive a
//! hub's downstream ports.
//!
//! The device requests take a
//! [`ControlEndpoint`] locating the target
//! (address, speed, endpoint-0 max packet size, and — for a device
//! behind a hub — its split-transaction target), so the same functions
//! reach a device on the root port or behind the hub. GET_DESCRIPTOR
//! (device, configuration, hub class), SET_ADDRESS, SET_CONFIGURATION,
//! and the HID SET_PROTOCOL/SET_IDLE requests are here. The hub port
//! requests ([`set_port_power`](crate::usb::control::set_port_power),
//! [`get_port_status`](crate::usb::control::get_port_status),
//! [`set_port_reset`](crate::usb::control::set_port_reset),
//! [`clear_port_feature`](crate::usb::control::clear_port_feature))
//! address the hub directly.

use crate::timer::Timer;
use crate::usb::descriptor::DeviceDescriptor;
use crate::usb::dwc2::{Channel, ControlEndpoint, TransferError};

/// How many times to retry a single NAK'd transaction before giving
/// up. NAKs are normal USB flow control (a device saying "not ready
/// yet, ask again"), not an error — but this driver won't retry
/// forever.
const MAX_NAK_RETRIES: u32 = 10_000;

/// `bRequest` code for GET_STATUS (USB 2.0 spec Table 9-4).
const REQUEST_GET_STATUS: u8 = 0x00;
/// `bRequest` code for CLEAR_FEATURE.
const REQUEST_CLEAR_FEATURE: u8 = 0x01;
/// `bRequest` code for SET_FEATURE.
const REQUEST_SET_FEATURE: u8 = 0x03;
/// `bRequest` code for GET_DESCRIPTOR.
const REQUEST_GET_DESCRIPTOR: u8 = 0x06;
/// `bRequest` code for SET_ADDRESS.
const REQUEST_SET_ADDRESS: u8 = 0x05;
/// `bRequest` code for SET_CONFIGURATION.
const REQUEST_SET_CONFIGURATION: u8 = 0x09;

/// Hub port feature selector PORT_RESET (USB 2.0 spec Table 11-17).
const PORT_FEATURE_RESET: u16 = 4;
/// Hub port feature selector PORT_POWER.
const PORT_FEATURE_POWER: u16 = 8;

/// Hub port change-feature selector C_PORT_CONNECTION (USB 2.0 spec
/// Table 11-17) — pass to [`clear_port_feature`] to acknowledge a
/// connection-status change (`wPortChange` bit 0).
pub const PORT_FEATURE_C_CONNECTION: u16 = 16;
/// Hub port change-feature selector C_PORT_RESET — pass to
/// [`clear_port_feature`] to acknowledge reset completion
/// (`wPortChange` bit 4).
pub const PORT_FEATURE_C_RESET: u16 = 20;

/// `bDescriptorType` for a DEVICE descriptor (USB 2.0 spec Table 9-5).
const DESCRIPTOR_TYPE_DEVICE: u8 = 1;
/// `bDescriptorType` for a CONFIGURATION descriptor.
const DESCRIPTOR_TYPE_CONFIGURATION: u8 = 2;
/// `bDescriptorType` for a HUB class descriptor (USB 2.0 spec §11.23.2).
const DESCRIPTOR_TYPE_HUB: u8 = 0x29;
/// `bDescriptorType` for a HID REPORT descriptor (HID spec §7.1.1) — the
/// self-describing report layout, fetched from the *interface*.
const DESCRIPTOR_TYPE_HID_REPORT: u8 = 0x22;

/// The 8-byte SETUP packet of a USB control transfer (USB 2.0 spec
/// §9.3) — the request's type/recipient, code, and parameters. Built
/// here and serialized with [`Setup::to_bytes`] for the SETUP stage.
#[derive(Clone, Copy)]
struct Setup {
    /// `bmRequestType`: transfer direction (bit 7), type
    /// (standard/class/vendor, bits 6:5), and recipient
    /// (device/interface/endpoint/other, bits 4:0).
    request_type: u8,
    /// `bRequest`: the request code.
    request: u8,
    /// `wValue`: request-specific.
    value: u16,
    /// `wIndex`: request-specific (often an interface, endpoint, or
    /// hub port number).
    index: u16,
    /// `wLength`: number of data-stage bytes to transfer.
    length: u16,
}

impl Setup {
    fn to_bytes(self) -> [u8; 8] {
        let value = self.value.to_le_bytes();
        let index = self.index.to_le_bytes();
        let length = self.length.to_le_bytes();
        [
            self.request_type,
            self.request,
            value[0],
            value[1],
            index[0],
            index[1],
            length[0],
            length[1],
        ]
    }
}

/// Runs `attempt` repeatedly while it returns [`TransferError::Nak`],
/// up to [`MAX_NAK_RETRIES`] times, returning its first non-NAK result
/// (or [`TransferError::NakTimeout`] if the budget runs out). A NAK
/// doesn't abort a control transfer — the device is just saying "not
/// ready yet, ask again" — so retrying the same stage in place is
/// valid, unlike a real error which would require restarting the whole
/// transfer.
fn retry_on_nak<T>(
    mut attempt: impl FnMut() -> Result<T, TransferError>,
) -> Result<T, TransferError> {
    for _ in 0..MAX_NAK_RETRIES {
        match attempt() {
            Err(TransferError::Nak) => continue,
            other => return other,
        }
    }
    Err(TransferError::NakTimeout)
}

/// Runs a complete device-to-host control transfer — SETUP, DATA IN,
/// STATUS OUT — reading the data stage into `buf`, each stage retried
/// on NAK. Returns the number of data bytes actually received (a device
/// may answer with fewer than requested via a short packet). `buf` must
/// be no larger than the driver's DMA scratch buffer; see
/// [`Channel::control_data_in`](crate::usb::dwc2::Channel::control_data_in).
fn control_in(
    channel: &mut Channel,
    timer: &Timer,
    endpoint: ControlEndpoint,
    setup: Setup,
    buf: &mut [u8],
) -> Result<usize, TransferError> {
    let setup = setup.to_bytes();
    retry_on_nak(|| channel.control_setup(endpoint, &setup, timer))?;
    let received = retry_on_nak(|| channel.control_data_in(endpoint, buf, timer))?;
    retry_on_nak(|| channel.control_status_out(endpoint, timer))?;
    Ok(received)
}

/// Runs a no-data control transfer — SETUP then a zero-length IN
/// status stage — for host-to-device requests that carry no data
/// (SET_ADDRESS, SET_CONFIGURATION), each stage retried on NAK.
fn control_no_data(
    channel: &mut Channel,
    timer: &Timer,
    endpoint: ControlEndpoint,
    setup: Setup,
) -> Result<(), TransferError> {
    let setup = setup.to_bytes();
    retry_on_nak(|| channel.control_setup(endpoint, &setup, timer))?;
    retry_on_nak(|| channel.control_status_in(endpoint, timer))?;
    Ok(())
}

/// Runs a host-to-device control transfer that carries data — SETUP,
/// DATA OUT, then a zero-length IN status stage — sending `data` in the
/// data stage, each stage retried on NAK. The mirror of [`control_in`]
/// for the OUT direction (e.g. a vendor register write).
fn control_out(
    channel: &mut Channel,
    timer: &Timer,
    endpoint: ControlEndpoint,
    setup: Setup,
    data: &[u8],
) -> Result<(), TransferError> {
    let setup = setup.to_bytes();
    retry_on_nak(|| channel.control_setup(endpoint, &setup, timer))?;
    retry_on_nak(|| channel.control_data_out(endpoint, data, timer))?;
    retry_on_nak(|| channel.control_status_in(endpoint, timer))?;
    Ok(())
}

/// The async twin of [`retry_on_nak`], for the interrupt-driven control
/// stages. Same budget, same reasoning; the only difference is that
/// `attempt` is an async closure, so the future it returns can borrow the
/// channel it drives.
#[cfg(feature = "async")]
async fn retry_on_nak_async<T>(
    mut attempt: impl AsyncFnMut() -> Result<T, TransferError>,
) -> Result<T, TransferError> {
    for _ in 0..MAX_NAK_RETRIES {
        match attempt().await {
            Err(TransferError::Nak) => continue,
            other => return other,
        }
    }
    Err(TransferError::NakTimeout)
}

/// The async twin of [`control_in`], stage for stage.
#[cfg(feature = "async")]
async fn control_in_async(
    channel: &mut Channel<'_>,
    timer: &Timer,
    endpoint: ControlEndpoint,
    setup: Setup,
    buf: &mut [u8],
) -> Result<usize, TransferError> {
    let setup = setup.to_bytes();
    retry_on_nak_async(async || channel.control_setup_async(endpoint, &setup, timer).await).await?;
    let received =
        retry_on_nak_async(async || channel.control_data_in_async(endpoint, buf, timer).await)
            .await?;
    retry_on_nak_async(async || channel.control_status_out_async(endpoint, timer).await).await?;
    Ok(received)
}

/// The async twin of [`control_out`], stage for stage.
#[cfg(feature = "async")]
async fn control_out_async(
    channel: &mut Channel<'_>,
    timer: &Timer,
    endpoint: ControlEndpoint,
    setup: Setup,
    data: &[u8],
) -> Result<(), TransferError> {
    let setup = setup.to_bytes();
    retry_on_nak_async(async || channel.control_setup_async(endpoint, &setup, timer).await).await?;
    retry_on_nak_async(async || channel.control_data_out_async(endpoint, data, timer).await)
        .await?;
    retry_on_nak_async(async || channel.control_status_in_async(endpoint, timer).await).await?;
    Ok(())
}

/// How many times a descriptor read is attempted before its failure is
/// reported — see [`descriptor_in`].
const DESCRIPTOR_TRIES: u32 = 3;

/// How long to wait between descriptor read attempts, in milliseconds.
const DESCRIPTOR_RETRY_MS: u32 = 5;

/// Runs a descriptor read ([`control_in`] with a GET_DESCRIPTOR setup),
/// re-running the whole transfer up to [`DESCRIPTOR_TRIES`] times if it
/// fails, and returning the last attempt's result.
///
/// Descriptor reads are the one place a host has to allow for devices
/// simply being flaky, rather than treating a failure as the answer. A USB
/// gamepad here (0428:4001) STALLs the first configuration-descriptor read
/// issued after it is addressed and then answers the *identical* request
/// milliseconds later — nothing about the request differs between the two,
/// and raising the SetAddress recovery wait from the spec's 2ms to Linux's
/// 10ms didn't change it, so the first-attempt failure is the device's
/// behavior and not a matter of giving it longer. Linux's usbcore has the
/// same allowance in `usb_get_descriptor` (three tries per read, commented
/// "some devices are flakey"), which is what this mirrors.
///
/// Retrying is safe because a control transfer restarts from its SETUP
/// stage, and a SETUP always clears a control endpoint's protocol stall
/// (USB 2.0 spec §8.5.3.4) — there is no leftover state from the failed
/// attempt to clean up first. It is also confined to descriptor reads:
/// they're idempotent, unlike the state-changing requests here (a repeated
/// SET_ADDRESS or SET_CONFIGURATION is not the same as one).
fn descriptor_in(
    channel: &mut Channel,
    timer: &Timer,
    endpoint: ControlEndpoint,
    setup: Setup,
    buf: &mut [u8],
) -> Result<usize, TransferError> {
    let mut result = control_in(channel, timer, endpoint, setup, buf);
    for _ in 1..DESCRIPTOR_TRIES {
        if result.is_ok() {
            break;
        }
        timer.delay_ms(DESCRIPTOR_RETRY_MS);
        result = control_in(channel, timer, endpoint, setup, buf);
    }
    result
}

/// Builds a GET_DESCRIPTOR [`Setup`] (USB 2.0 spec §9.4.3):
/// `request_type` selects standard-vs-class and recipient,
/// `descriptor_type`/`index` go in `wValue`, `length` in `wLength`.
fn get_descriptor_setup(request_type: u8, descriptor_type: u8, index: u8, length: u16) -> Setup {
    Setup {
        request_type,
        request: REQUEST_GET_DESCRIPTOR,
        value: ((descriptor_type as u16) << 8) | index as u16,
        index: 0,
        length,
    }
}

/// Requests the first 8 bytes of `device`'s device descriptor
/// (`bLength`, `bDescriptorType`, `bcdUSB` low/high, `bDeviceClass`,
/// `bDeviceSubClass`, `bDeviceProtocol`, `bMaxPacketSize0`) — the
/// standard "probe" every real USB host issues before it knows the
/// device's actual max packet size. `device.max_packet_size` is
/// ignored: the probe always uses 8, the conservative default every
/// USB device is guaranteed to support at endpoint 0.
pub fn get_device_descriptor_header(
    channel: &mut Channel,
    timer: &Timer,
    device: ControlEndpoint,
) -> Result<[u8; 8], TransferError> {
    let endpoint = ControlEndpoint {
        max_packet_size: 8,
        ..device
    };
    // bmRequestType=0x80: device-to-host, standard, recipient=device.
    let setup = get_descriptor_setup(0x80, DESCRIPTOR_TYPE_DEVICE, 0, 8);
    let mut descriptor = [0u8; 8];
    descriptor_in(channel, timer, endpoint, setup, &mut descriptor)?;
    Ok(descriptor)
}

/// Reads the full 18-byte device descriptor of `device`.
///
/// Done in the two steps real enumeration uses: first
/// [`get_device_descriptor_header`] to learn the endpoint-0 max packet
/// size (`bMaxPacketSize0`), then the full read using that size — so
/// `device.max_packet_size` is ignored (this discovers it). The max
/// packet size has to be right before requesting more than 8 bytes: a
/// device whose real max packet size is larger would answer an 18-byte
/// request in one oversized packet, overrunning the channel (a babble
/// error).
pub fn get_device_descriptor(
    channel: &mut Channel,
    timer: &Timer,
    device: ControlEndpoint,
) -> Result<[u8; 18], TransferError> {
    let header = get_device_descriptor_header(channel, timer, device)?;
    // bMaxPacketSize0 is the 8th byte of the descriptor.
    let endpoint = ControlEndpoint {
        max_packet_size: header[7] as u16,
        ..device
    };
    // bmRequestType=0x80: device-to-host, standard, recipient=device.
    let setup = get_descriptor_setup(0x80, DESCRIPTOR_TYPE_DEVICE, 0, 18);
    let mut descriptor = [0u8; 18];
    descriptor_in(channel, timer, endpoint, setup, &mut descriptor)?;
    Ok(descriptor)
}

/// Reads `probe`'s device descriptor and assigns it `new_address`
/// (SET_ADDRESS) — the two steps that turn a freshly-reset,
/// still-at-address-0 device into an addressed one ready for further
/// configuration. Returns the device's now-addressed endpoint 0 (its
/// address updated to `new_address` and its `max_packet_size` set to the
/// real `bMaxPacketSize0` learned from the descriptor) alongside the
/// parsed [`DeviceDescriptor`].
///
/// `probe` is the unaddressed device: address 0, its speed and (for a
/// device behind a hub) split target set, `max_packet_size` left at the
/// 8-byte default (which [`get_device_descriptor`] uses for its probe
/// regardless). `new_address` must be 1..=127 and not already in use.
pub fn probe_and_address(
    channel: &mut Channel,
    timer: &Timer,
    probe: ControlEndpoint,
    new_address: u8,
) -> Result<(ControlEndpoint, DeviceDescriptor), TransferError> {
    let descriptor = DeviceDescriptor::from_bytes(&get_device_descriptor(channel, timer, probe)?);
    set_address(channel, timer, probe, new_address)?;
    let endpoint = ControlEndpoint {
        address: new_address,
        max_packet_size: descriptor.max_packet_size0 as u16,
        ..probe
    };
    Ok((endpoint, descriptor))
}

/// Fixed length of a configuration descriptor's own header (USB 2.0
/// spec §9.6.3), before the interface and endpoint descriptors that
/// follow it in the same transfer.
const CONFIGURATION_DESCRIPTOR_HEADER_LEN: usize = 9;

/// Reads configuration descriptor `index` of `device` into `buf`,
/// returning the number of bytes read.
///
/// Done in two steps, like the device descriptor read: first the
/// 9-byte header to learn the total size of the whole configuration
/// block from its `wTotalLength` field, then the full block — the
/// configuration descriptor followed by its interface and endpoint
/// descriptors — up to `min(wTotalLength, buf.len())` bytes.
///
/// `device.max_packet_size` must be the real endpoint-0 max packet size
/// (from the device descriptor — requesting more than 8 bytes with the
/// wrong packet size overruns the channel). `buf` must be no larger than
/// the driver's DMA scratch buffer: a configuration whose `wTotalLength`
/// exceeds `buf.len()` is read truncated (the returned length says how
/// much was read), which covers the small single-configuration devices
/// this stack targets so far but not a large composite configuration.
pub fn get_configuration_descriptor(
    channel: &mut Channel,
    timer: &Timer,
    device: ControlEndpoint,
    index: u8,
    buf: &mut [u8],
) -> Result<usize, TransferError> {
    // Header first: its `wTotalLength` (bytes 2..4) is the size of the
    // entire configuration block, which the full read below requests.
    // bmRequestType=0x80: device-to-host, standard, recipient=device.
    let mut header = [0u8; CONFIGURATION_DESCRIPTOR_HEADER_LEN];
    let setup = get_descriptor_setup(
        0x80,
        DESCRIPTOR_TYPE_CONFIGURATION,
        index,
        CONFIGURATION_DESCRIPTOR_HEADER_LEN as u16,
    );
    descriptor_in(channel, timer, device, setup, &mut header)?;

    let total_length = u16::from_le_bytes([header[2], header[3]]) as usize;
    let to_read = total_length.min(buf.len());

    let setup = get_descriptor_setup(0x80, DESCRIPTOR_TYPE_CONFIGURATION, index, to_read as u16);
    descriptor_in(channel, timer, device, setup, &mut buf[..to_read])?;
    Ok(to_read)
}

/// Reads a hub's class descriptor (USB 2.0 spec §11.23.2) from `device`
/// (the hub) into `buf`, returning the number of bytes read (capped at
/// `buf.len()`). The hub descriptor reports `bNbrPorts` (how many
/// downstream ports), the hub's characteristics, and `bPwrOn2PwrGood`
/// (the delay, in 2ms units, to wait after powering a port before its
/// devices are stable) — all needed before driving the downstream
/// ports.
///
/// `device.max_packet_size` must be the hub's `bMaxPacketSize0`. This is
/// a class request (`bmRequestType=0xA0`), unlike the standard device/
/// configuration descriptor reads above.
pub fn get_hub_descriptor(
    channel: &mut Channel,
    timer: &Timer,
    device: ControlEndpoint,
    buf: &mut [u8],
) -> Result<usize, TransferError> {
    // bmRequestType=0xA0: device-to-host, class, recipient=device.
    let setup = get_descriptor_setup(0xA0, DESCRIPTOR_TYPE_HUB, 0, buf.len() as u16);
    descriptor_in(channel, timer, device, setup, buf)
}

/// Reads the HID report descriptor of `interface` on `device` into `buf`,
/// returning the number of bytes read (HID spec §7.1.1).
///
/// The report descriptor is what makes a HID device self-describing: it
/// declares every field of every report — bit position, width, HID usage,
/// logical range — so a host can decode reports from a device it knows
/// nothing about. Parse it with
/// [`ReportDescriptor`](crate::hid_report::ReportDescriptor).
/// Devices outside the boot subclass (a gamepad, say) send *only* this
/// format, so reading it is the first step in talking to one.
///
/// Unlike the device/configuration descriptors this is fetched from the
/// interface (`wIndex` is the interface number, not zero), and the length
/// must be exactly the `wDescriptorLength` the interface's HID class
/// descriptor declared: a device answering a longer request than it has
/// data for STALLs rather than returning a short packet. So `buf` is
/// passed already trimmed to that length by the caller (see
/// [the gamepad driver](crate::usb::hid::gamepad)), and — like every
/// control transfer here — must be no larger than the driver's DMA
/// scratch buffer.
pub fn get_report_descriptor(
    channel: &mut Channel,
    timer: &Timer,
    device: ControlEndpoint,
    interface: u8,
    buf: &mut [u8],
) -> Result<usize, TransferError> {
    // bmRequestType=0x81: device-to-host, standard, recipient=interface.
    let mut setup = get_descriptor_setup(0x81, DESCRIPTOR_TYPE_HID_REPORT, 0, buf.len() as u16);
    setup.index = interface as u16;
    descriptor_in(channel, timer, device, setup, buf)
}

/// USB SetAddress recovery time (USB 2.0 spec §9.2.6.3): after the
/// status stage completes the device has up to 2ms to actually switch
/// to its new address, during which it need not respond at either
/// address.
const SET_ADDRESS_RECOVERY_MS: u32 = 2;

/// Assigns `new_address` to `device` (SET_ADDRESS, USB 2.0 spec
/// §9.4.6), which must still hold its current address. This is a
/// no-data control transfer — a SETUP stage followed directly by a
/// zero-length IN status stage. After it returns the device responds
/// only at `new_address` and its old address is free again; the spec's
/// 2ms recovery time has already elapsed, so a transfer to the device
/// at `new_address` immediately afterwards is safe.
///
/// `new_address` must be 1..=127 (0 is the default address every
/// unaddressed device starts at).
pub fn set_address(
    channel: &mut Channel,
    timer: &Timer,
    device: ControlEndpoint,
    new_address: u8,
) -> Result<(), TransferError> {
    // bmRequestType=0x00: host-to-device, standard, recipient=device.
    let setup = Setup {
        request_type: 0x00,
        request: REQUEST_SET_ADDRESS,
        value: new_address as u16,
        index: 0,
        length: 0,
    };

    control_no_data(channel, timer, device, setup)?;
    timer.delay_ms(SET_ADDRESS_RECOVERY_MS);
    Ok(())
}

/// Activates configuration `configuration_value` on `device`
/// (SET_CONFIGURATION, USB 2.0 spec §9.4.7) — a no-data control
/// transfer (SETUP then a zero-length IN status stage). After it
/// returns the device is in the Configured state and its
/// non-endpoint-0 endpoints are usable.
///
/// `configuration_value` is a configuration descriptor's
/// `bConfigurationValue` (not its index in the descriptor list); `0`
/// puts the device back into the unconfigured Address state.
pub fn set_configuration(
    channel: &mut Channel,
    timer: &Timer,
    device: ControlEndpoint,
    configuration_value: u8,
) -> Result<(), TransferError> {
    // bmRequestType=0x00: host-to-device, standard, recipient=device.
    let setup = Setup {
        request_type: 0x00,
        request: REQUEST_SET_CONFIGURATION,
        value: configuration_value as u16,
        index: 0,
        length: 0,
    };

    control_no_data(channel, timer, device, setup)
}

/// Runs a vendor-specific control-IN transfer to `device` — the request
/// type fixed at device-to-host, vendor, recipient device
/// (`bmRequestType = 0xC0`) — issuing `request` with `value`/`index` and
/// reading its data stage into `buf`. Returns the number of bytes read.
///
/// Vendor requests aren't standardized by USB; each vendor-specific
/// device defines its own. This is the read half used to reach the
/// registers of a device like the on-board LAN9514 USB-Ethernet
/// controller (see [`crate::usb::lan9514`]), whose registers are read via
/// a vendor request carrying the register offset in `index`.
pub fn vendor_in(
    channel: &mut Channel,
    timer: &Timer,
    device: ControlEndpoint,
    request: u8,
    value: u16,
    index: u16,
    buf: &mut [u8],
) -> Result<usize, TransferError> {
    // bmRequestType=0xC0: device-to-host, vendor, recipient=device.
    let setup = Setup {
        request_type: 0xC0,
        request,
        value,
        index,
        length: buf.len() as u16,
    };
    control_in(channel, timer, device, setup, buf)
}

/// Runs a vendor-specific control-OUT transfer to `device` — the request
/// type fixed at host-to-device, vendor, recipient device
/// (`bmRequestType = 0x40`) — issuing `request` with `value`/`index` and
/// sending `data` as its data stage. The write half of vendor register
/// access (see [`vendor_in`]); for the LAN9514 this writes a register
/// whose offset is carried in `index`.
pub fn vendor_out(
    channel: &mut Channel,
    timer: &Timer,
    device: ControlEndpoint,
    request: u8,
    value: u16,
    index: u16,
    data: &[u8],
) -> Result<(), TransferError> {
    // bmRequestType=0x40: host-to-device, vendor, recipient=device.
    let setup = Setup {
        request_type: 0x40,
        request,
        value,
        index,
        length: data.len() as u16,
    };
    control_out(channel, timer, device, setup, data)
}

/// The async twin of [`vendor_in`], for a caller running under an
/// executor — the register-read half of a driver like
/// [`crate::usb::lan9514`] once its frame path has moved onto the
/// interrupt-driven primitives.
///
/// Available only with the `async` feature enabled.
#[cfg(feature = "async")]
#[cfg_attr(docsrs, doc(cfg(feature = "async")))]
pub async fn vendor_in_async(
    channel: &mut Channel<'_>,
    timer: &Timer,
    device: ControlEndpoint,
    request: u8,
    value: u16,
    index: u16,
    buf: &mut [u8],
) -> Result<usize, TransferError> {
    // bmRequestType=0xC0: device-to-host, vendor, recipient=device.
    let setup = Setup {
        request_type: 0xC0,
        request,
        value,
        index,
        length: buf.len() as u16,
    };
    control_in_async(channel, timer, device, setup, buf).await
}

/// The async twin of [`vendor_out`].
///
/// Available only with the `async` feature enabled.
#[cfg(feature = "async")]
#[cfg_attr(docsrs, doc(cfg(feature = "async")))]
pub async fn vendor_out_async(
    channel: &mut Channel<'_>,
    timer: &Timer,
    device: ControlEndpoint,
    request: u8,
    value: u16,
    index: u16,
    data: &[u8],
) -> Result<(), TransferError> {
    // bmRequestType=0x40: host-to-device, vendor, recipient=device.
    let setup = Setup {
        request_type: 0x40,
        request,
        value,
        index,
        length: data.len() as u16,
    };
    control_out_async(channel, timer, device, setup, data).await
}

/// Selects the HID protocol on `interface` of `device` (HID spec
/// §7.2.6): `protocol` = 0 for the boot protocol (fixed 8-byte report
/// format needing no report-descriptor parsing), 1 for the report
/// protocol. A no-data class request to the interface.
pub fn set_protocol(
    channel: &mut Channel,
    timer: &Timer,
    device: ControlEndpoint,
    interface: u8,
    protocol: u8,
) -> Result<(), TransferError> {
    // bmRequestType=0x21: host-to-device, class, recipient=interface;
    // bRequest=0x0B (SET_PROTOCOL).
    let setup = Setup {
        request_type: 0x21,
        request: 0x0b,
        value: protocol as u16,
        index: interface as u16,
        length: 0,
    };
    control_no_data(channel, timer, device, setup)
}

/// Sets the HID idle rate on `interface` of `device` (HID spec §7.2.4).
/// `duration` is in 4ms units; `0` means "only report on change" —
/// ideal for a polled keyboard, so an unchanged poll simply NAKs
/// instead of re-sending the last report. A no-data class request to
/// the interface. Some devices STALL this; the caller may ignore an
/// error.
pub fn set_idle(
    channel: &mut Channel,
    timer: &Timer,
    device: ControlEndpoint,
    interface: u8,
    duration: u8,
) -> Result<(), TransferError> {
    // bmRequestType=0x21: host-to-device, class, recipient=interface;
    // bRequest=0x0A (SET_IDLE); wValue high byte = duration.
    let setup = Setup {
        request_type: 0x21,
        request: 0x0a,
        value: (duration as u16) << 8,
        index: interface as u16,
        length: 0,
    };
    control_no_data(channel, timer, device, setup)
}

/// Powers on downstream `port` (1-based) of the hub at `hub_address`
/// via SET_FEATURE(PORT_POWER) (USB 2.0 spec §11.24.2.13) — a no-data
/// class request addressed to the port ("other" recipient). After
/// powering a port, wait the hub descriptor's `bPwrOn2PwrGood` time
/// before the port's status is meaningful. `low_speed` should reflect
/// `Dwc2Host::port_speed() == 2` for the hub itself.
pub fn set_port_power(
    channel: &mut Channel,
    timer: &Timer,
    hub_address: u8,
    port: u8,
    low_speed: bool,
) -> Result<(), TransferError> {
    let endpoint = ControlEndpoint {
        address: hub_address,
        low_speed,
        // No data stage; 8 is the safe endpoint-0 default.
        max_packet_size: 8,
        split: None,
    };
    // bmRequestType=0x23: host-to-device, class, recipient=other (port).
    let setup = Setup {
        request_type: 0x23,
        request: REQUEST_SET_FEATURE,
        value: PORT_FEATURE_POWER,
        index: port as u16,
        length: 0,
    };
    control_no_data(channel, timer, endpoint, setup)
}

/// Reads downstream `port`'s status via GET_STATUS (USB 2.0 spec
/// §11.24.2.7), returning `(wPortStatus, wPortChange)`.
///
/// In `wPortStatus`: bit 0 = a device is connected, bit 1 = the port is
/// enabled, bit 4 = the port is being reset, bit 8 = the port is
/// powered, bit 9 = a low-speed device is attached, bit 10 = a
/// high-speed device (neither 9 nor 10 set means full speed).
/// `wPortChange`'s bits flag status changes to acknowledge with
/// CLEAR_FEATURE. `low_speed` should reflect `Dwc2Host::port_speed() ==
/// 2` for the hub itself.
pub fn get_port_status(
    channel: &mut Channel,
    timer: &Timer,
    hub_address: u8,
    port: u8,
    low_speed: bool,
) -> Result<(u16, u16), TransferError> {
    let endpoint = ControlEndpoint {
        address: hub_address,
        low_speed,
        // 4-byte reply fits comfortably in the endpoint-0 default of 8.
        max_packet_size: 8,
        split: None,
    };
    // bmRequestType=0xA3: device-to-host, class, recipient=other (port).
    let setup = Setup {
        request_type: 0xA3,
        request: REQUEST_GET_STATUS,
        value: 0,
        index: port as u16,
        length: 4,
    };
    let mut status = [0u8; 4];
    control_in(channel, timer, endpoint, setup, &mut status)?;
    Ok((
        u16::from_le_bytes([status[0], status[1]]),
        u16::from_le_bytes([status[2], status[3]]),
    ))
}

/// Resets downstream `port` of the hub at `hub_address` via
/// SET_FEATURE(PORT_RESET) (USB 2.0 spec §11.24.2.13) — a no-data class
/// request to the port. The hub drives USB reset signaling and, when
/// done, enables the port and sets its C_PORT_RESET change bit; poll
/// [`get_port_status`] for PORT_ENABLE (and the now-valid speed bits),
/// then acknowledge with [`clear_port_feature`] and
/// [`PORT_FEATURE_C_RESET`]. Only meaningful once the port reports a
/// device connected. `low_speed` should reflect `Dwc2Host::port_speed()
/// == 2` for the hub itself.
pub fn set_port_reset(
    channel: &mut Channel,
    timer: &Timer,
    hub_address: u8,
    port: u8,
    low_speed: bool,
) -> Result<(), TransferError> {
    let endpoint = ControlEndpoint {
        address: hub_address,
        low_speed,
        max_packet_size: 8,
        split: None,
    };
    // bmRequestType=0x23: host-to-device, class, recipient=other (port).
    let setup = Setup {
        request_type: 0x23,
        request: REQUEST_SET_FEATURE,
        value: PORT_FEATURE_RESET,
        index: port as u16,
        length: 0,
    };
    control_no_data(channel, timer, endpoint, setup)
}

/// Clears `feature` on downstream `port` of the hub at `hub_address`
/// via CLEAR_FEATURE (USB 2.0 spec §11.24.2.2) — used to acknowledge a
/// port status change (the `PORT_FEATURE_C_*` selectors, e.g.
/// [`PORT_FEATURE_C_CONNECTION`]/[`PORT_FEATURE_C_RESET`]). A no-data
/// class request to the port. `low_speed` should reflect
/// `Dwc2Host::port_speed() == 2` for the hub itself.
pub fn clear_port_feature(
    channel: &mut Channel,
    timer: &Timer,
    hub_address: u8,
    port: u8,
    feature: u16,
    low_speed: bool,
) -> Result<(), TransferError> {
    let endpoint = ControlEndpoint {
        address: hub_address,
        low_speed,
        max_packet_size: 8,
        split: None,
    };
    // bmRequestType=0x23: host-to-device, class, recipient=other (port).
    let setup = Setup {
        request_type: 0x23,
        request: REQUEST_CLEAR_FEATURE,
        value: feature,
        index: port as u16,
        length: 0,
    };
    control_no_data(channel, timer, endpoint, setup)
}
