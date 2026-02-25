//! CDC ECM transport for XReal One IMU over raw USB.
//!
//! Windows lacks a built-in CDC ECM driver (only CDC NCM via UsbNcm.sys).
//! This module bypasses that entirely:
//!
//! 1. Opens the XReal One USB device directly via WinUSB (nusb)
//! 2. Speaks CDC ECM: raw Ethernet frames on bulk endpoints
//! 3. Runs a user-space TCP/IP stack (smoltcp) over those frames
//! 4. Establishes TCP to 169.254.2.1:52998 for IMU data
//!
//! Prerequisite: install WinUSB driver for the CDC ECM interface via Zadig.

use anyhow::{anyhow, Result};
use nusb::MaybeFuture;
use smoltcp::iface::{Config, Interface, SocketSet};
use smoltcp::phy::{Device, DeviceCapabilities, Medium, RxToken, TxToken};
use smoltcp::socket::tcp;
use smoltcp::time::Instant as SmolInstant;
use smoltcp::wire::{EthernetAddress, HardwareAddress, IpAddress, IpCidr};
use std::collections::VecDeque;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc as std_mpsc;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::mpsc;

/// XReal One USB identifiers.
const XREAL_VID: u16 = 0x3318;
const XREAL_PID: u16 = 0x0438;

/// XReal One IMU TCP endpoint.
const XREAL_IMU_IP: [u8; 4] = [169, 254, 2, 1];
const XREAL_IMU_PORT: u16 = 52998;

/// Our IP on the USB link (link-local, same /16 subnet).
const HOST_IP: [u8; 4] = [169, 254, 2, 2];

/// Locally-administered MAC for our end of the link.
const HOST_MAC: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

/// CDC ECM class request: SET_ETHERNET_PACKET_FILTER.
const SET_ETHERNET_PACKET_FILTER: u8 = 0x43;

/// Accept all packet types.
const PACKET_TYPE_ALL: u16 = 0x001F;

/// Max Ethernet frame size.
const MAX_FRAME_SIZE: usize = 1514;

/// TCP connection timeout.
const TCP_CONNECT_TIMEOUT: Duration = Duration::from_secs(10);

/// Shared frame counters for diagnostics.
struct FrameCounters {
    rx: AtomicU64,
    tx: AtomicU64,
}

/// Connect to the XReal One IMU via USB CDC ECM.
///
/// Returns a channel receiver yielding raw TCP data (IMU protocol bytes).
/// Three background threads handle USB I/O and the TCP stack.
pub fn connect_usb() -> Result<mpsc::UnboundedReceiver<Vec<u8>>> {
    // 1. Find the XReal One.
    let device_info = nusb::list_devices()
        .wait()
        .map_err(|e| anyhow!("Failed to enumerate USB devices: {e}"))?
        .find(|d| d.vendor_id() == XREAL_VID && d.product_id() == XREAL_PID)
        .ok_or_else(|| {
            anyhow!(
                "XReal One not found on USB (VID {:04x} PID {:04x})",
                XREAL_VID,
                XREAL_PID
            )
        })?;

    tracing::info!(
        vendor = format!("{:04x}", device_info.vendor_id()),
        product = format!("{:04x}", device_info.product_id()),
        product_string = ?device_info.product_string(),
        "Found XReal One USB device"
    );

    // 2. Open the device.
    let device = device_info
        .open()
        .wait()
        .map_err(|e| anyhow!("Failed to open USB device (is WinUSB installed via Zadig?): {e}"))?;

    // 3. Dump all USB interfaces for diagnostics, then find CDC ECM.
    let config = device
        .active_configuration()
        .map_err(|e| anyhow!("Failed to read USB configuration descriptor: {e}"))?;

    dump_usb_descriptors(&config);
    let ecm = find_ecm_endpoints(&config)?;

    tracing::info!(
        comm = ecm.comm_iface,
        data = ecm.data_iface,
        data_alt = ecm.data_alt_setting,
        bulk_in = format!("0x{:02x}", ecm.bulk_in_ep),
        bulk_out = format!("0x{:02x}", ecm.bulk_out_ep),
        "Found CDC ECM endpoints"
    );

    // 4. Claim interfaces.
    let comm = device
        .claim_interface(ecm.comm_iface)
        .wait()
        .map_err(|e| anyhow!("Failed to claim Communication interface (install WinUSB via Zadig): {e}"))?;

    let data = device
        .claim_interface(ecm.data_iface)
        .wait()
        .map_err(|e| anyhow!("Failed to claim Data interface (install WinUSB via Zadig): {e}"))?;

    // Always set alt setting 1 on the data interface — CDC ECM spec requires
    // it to activate the bulk endpoints. Even if descriptors show endpoints
    // at alt 0, the device may still need this control request.
    tracing::info!("Activating CDC ECM data interface (alt setting 1)...");
    match data.set_alt_setting(1).wait() {
        Ok(()) => tracing::info!("Data interface alt setting 1 activated"),
        Err(e) => tracing::warn!(?e, "set_alt_setting(1) failed, trying alt 0"),
    }

    // 5. Enable packet reception via SET_ETHERNET_PACKET_FILTER.
    match comm.control_out(
        nusb::transfer::ControlOut {
            control_type: nusb::transfer::ControlType::Class,
            recipient: nusb::transfer::Recipient::Interface,
            request: SET_ETHERNET_PACKET_FILTER,
            value: PACKET_TYPE_ALL,
            index: ecm.comm_iface as u16,
            data: &[],
        },
        Duration::from_millis(1000),
    ).wait() {
        Ok(()) => tracing::info!("SET_ETHERNET_PACKET_FILTER succeeded"),
        Err(e) => tracing::warn!(?e, "SET_ETHERNET_PACKET_FILTER failed"),
    }

    // 6. Create bulk endpoint reader/writer.
    let reader = data
        .endpoint::<nusb::transfer::Bulk, nusb::transfer::In>(ecm.bulk_in_ep)
        .map_err(|e| anyhow!("Failed to open bulk IN endpoint: {e}"))?
        .reader(MAX_FRAME_SIZE);

    let writer = data
        .endpoint::<nusb::transfer::Bulk, nusb::transfer::Out>(ecm.bulk_out_ep)
        .map_err(|e| anyhow!("Failed to open bulk OUT endpoint: {e}"))?
        .writer(MAX_FRAME_SIZE);

    // 7. Spawn background threads with shared counters.
    let counters = Arc::new(FrameCounters {
        rx: AtomicU64::new(0),
        tx: AtomicU64::new(0),
    });

    let (imu_tx, imu_rx) = mpsc::unbounded_channel();

    // USB reader → frame channel
    let (frame_tx, frame_rx) = std_mpsc::channel::<Vec<u8>>();
    let rx_counters = counters.clone();
    std::thread::Builder::new()
        .name("xreal-usb-reader".into())
        .spawn(move || usb_reader_thread(reader, frame_tx, rx_counters))?;

    // Frame channel → USB writer
    let (write_tx, write_rx) = std_mpsc::channel::<Vec<u8>>();
    let tx_counters = counters.clone();
    std::thread::Builder::new()
        .name("xreal-usb-writer".into())
        .spawn(move || usb_writer_thread(writer, write_rx, tx_counters))?;

    // smoltcp TCP → IMU data channel
    std::thread::Builder::new()
        .name("xreal-ecm-tcp".into())
        .spawn(move || {
            if let Err(e) = ecm_tcp_thread(frame_rx, write_tx, imu_tx, counters) {
                tracing::error!(?e, "ECM TCP thread exited with error");
            }
        })?;

    Ok(imu_rx)
}

// ---------------------------------------------------------------------------
// Descriptor discovery
// ---------------------------------------------------------------------------

struct EcmEndpoints {
    comm_iface: u8,
    data_iface: u8,
    data_alt_setting: u8,
    bulk_in_ep: u8,
    bulk_out_ep: u8,
}

/// Log all USB interfaces for debugging.
fn dump_usb_descriptors(config: &nusb::descriptors::ConfigurationDescriptor) {
    tracing::info!("=== USB Descriptor Dump ===");
    for iface_desc in config.interface_alt_settings() {
        let iface_num = iface_desc.interface_number();
        let alt = iface_desc.alternate_setting();
        let class = iface_desc.class();
        let subclass = iface_desc.subclass();
        let protocol = iface_desc.protocol();

        let class_name = match (class, subclass) {
            (0x02, 0x02) => "CDC ACM",
            (0x02, 0x06) => "CDC ECM",
            (0x02, 0x0D) => "CDC NCM",
            (0x02, _) => "CDC Other",
            (0x0A, _) => "CDC Data",
            (0x0E, _) => "Video",
            (0x01, _) => "Audio",
            (0xFF, _) => "Vendor-Specific",
            _ => "Other",
        };

        let mut ep_info = Vec::new();
        for ep in iface_desc.endpoints() {
            let dir = match ep.direction() {
                nusb::transfer::Direction::In => "IN",
                nusb::transfer::Direction::Out => "OUT",
            };
            let tt = match ep.transfer_type() {
                nusb::descriptors::TransferType::Control => "ctrl",
                nusb::descriptors::TransferType::Isochronous => "iso",
                nusb::descriptors::TransferType::Bulk => "bulk",
                nusb::descriptors::TransferType::Interrupt => "int",
            };
            ep_info.push(format!("0x{:02x} {} {}", ep.address(), dir, tt));
        }

        tracing::info!(
            iface = iface_num,
            alt,
            class = format!("{:02x}/{:02x}/{:02x}", class, subclass, protocol),
            name = class_name,
            endpoints = ?ep_info,
            "  Interface"
        );
    }
    tracing::info!("=== End Descriptor Dump ===");
}

fn find_ecm_endpoints(config: &nusb::descriptors::ConfigurationDescriptor) -> Result<EcmEndpoints> {
    let mut comm_iface = None;
    let mut data_iface = None;
    let mut bulk_in = None;
    let mut bulk_out = None;
    let mut data_alt = 0u8;

    for iface_desc in config.interface_alt_settings() {
        let class = iface_desc.class();
        let subclass = iface_desc.subclass();
        let iface_num = iface_desc.interface_number();
        let alt = iface_desc.alternate_setting();

        // CDC Communication: class 0x02, subclass 0x06 (ECM) or 0x0D (NCM)
        if class == 0x02 && (subclass == 0x06 || subclass == 0x0D) {
            comm_iface = Some(iface_num);
        }

        // CDC Data: class 0x0A — look for bulk endpoints
        if class == 0x0A {
            for ep in iface_desc.endpoints() {
                if ep.transfer_type() != nusb::descriptors::TransferType::Bulk {
                    continue;
                }
                match ep.direction() {
                    nusb::transfer::Direction::In => {
                        bulk_in = Some(ep.address());
                        data_iface = Some(iface_num);
                        data_alt = alt;
                    }
                    nusb::transfer::Direction::Out => {
                        bulk_out = Some(ep.address());
                    }
                }
            }
        }
    }

    Ok(EcmEndpoints {
        comm_iface: comm_iface.ok_or_else(|| anyhow!("No CDC ECM Communication interface found"))?,
        data_iface: data_iface.ok_or_else(|| anyhow!("No CDC Data interface with bulk endpoints"))?,
        bulk_in_ep: bulk_in.ok_or_else(|| anyhow!("No bulk IN endpoint found"))?,
        bulk_out_ep: bulk_out.ok_or_else(|| anyhow!("No bulk OUT endpoint found"))?,
        data_alt_setting: data_alt,
    })
}

// ---------------------------------------------------------------------------
// Background threads
// ---------------------------------------------------------------------------

fn usb_reader_thread(
    mut reader: nusb::io::EndpointRead<nusb::transfer::Bulk>,
    frame_tx: std_mpsc::Sender<Vec<u8>>,
    counters: Arc<FrameCounters>,
) {
    tracing::debug!("USB reader thread started");
    let mut buf = vec![0u8; MAX_FRAME_SIZE];
    loop {
        match reader.read(&mut buf) {
            Ok(n) if n > 0 => {
                let count = counters.rx.fetch_add(1, Ordering::Relaxed) + 1;
                if count <= 5 || count % 100 == 0 {
                    // Log first few frames and then periodically.
                    let hex_preview: String = buf[..n.min(32)]
                        .iter()
                        .map(|b| format!("{:02x}", b))
                        .collect::<Vec<_>>()
                        .join(" ");
                    tracing::info!(
                        rx_count = count,
                        len = n,
                        preview = hex_preview,
                        "USB RX frame"
                    );
                }
                if frame_tx.send(buf[..n].to_vec()).is_err() {
                    tracing::debug!("Frame receiver dropped, reader exiting");
                    return;
                }
            }
            Ok(_) => continue,
            Err(e) => {
                tracing::error!(?e, "USB bulk IN read error");
                return;
            }
        }
    }
}

fn usb_writer_thread(
    mut writer: nusb::io::EndpointWrite<nusb::transfer::Bulk>,
    write_rx: std_mpsc::Receiver<Vec<u8>>,
    counters: Arc<FrameCounters>,
) {
    tracing::debug!("USB writer thread started");
    while let Ok(frame) = write_rx.recv() {
        let count = counters.tx.fetch_add(1, Ordering::Relaxed) + 1;
        if count <= 5 || count % 100 == 0 {
            let hex_preview: String = frame[..frame.len().min(32)]
                .iter()
                .map(|b| format!("{:02x}", b))
                .collect::<Vec<_>>()
                .join(" ");
            tracing::info!(
                tx_count = count,
                len = frame.len(),
                preview = hex_preview,
                "USB TX frame"
            );
        }
        if let Err(e) = writer.write_all(&frame) {
            tracing::error!(?e, "USB bulk OUT write error");
            return;
        }
    }
    tracing::debug!("Frame sender dropped, writer exiting");
}

// ---------------------------------------------------------------------------
// smoltcp TCP/IP stack
// ---------------------------------------------------------------------------

fn ecm_tcp_thread(
    frame_rx: std_mpsc::Receiver<Vec<u8>>,
    frame_tx: std_mpsc::Sender<Vec<u8>>,
    imu_tx: mpsc::UnboundedSender<Vec<u8>>,
    counters: Arc<FrameCounters>,
) -> Result<()> {
    let mut device = EcmDevice {
        frame_rx,
        frame_tx,
        rx_queue: VecDeque::new(),
    };

    let hw_addr = HardwareAddress::Ethernet(EthernetAddress(HOST_MAC));
    let config = Config::new(hw_addr);
    let mut iface = Interface::new(config, &mut device, smol_now());

    iface.update_ip_addrs(|addrs| {
        addrs
            .push(IpCidr::new(
                IpAddress::Ipv4(smoltcp::wire::Ipv4Address(HOST_IP)),
                16,
            ))
            .expect("failed to add IP address");
    });

    let tcp_rx_buf = tcp::SocketBuffer::new(vec![0u8; 65536]);
    let tcp_tx_buf = tcp::SocketBuffer::new(vec![0u8; 4096]);
    let mut sockets = SocketSet::new(vec![]);
    let tcp_handle = sockets.add(tcp::Socket::new(tcp_rx_buf, tcp_tx_buf));

    {
        let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);
        socket
            .connect(
                iface.context(),
                (
                    IpAddress::Ipv4(smoltcp::wire::Ipv4Address(XREAL_IMU_IP)),
                    XREAL_IMU_PORT,
                ),
                49152u16,
            )
            .map_err(|e| anyhow!("TCP connect failed: {e}"))?;
    }

    tracing::info!("TCP SYN queued to XReal One IMU via USB CDC ECM");

    let start = Instant::now();
    let mut connected = false;
    let mut recv_buf = [0u8; 4096];
    let mut last_status = Instant::now();

    loop {
        let timestamp = smol_now();
        device.drain_rx();

        let _changed = iface.poll(timestamp, &mut device, &mut sockets);

        let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);

        if !connected && socket.may_recv() {
            tracing::info!("TCP connected to XReal One IMU via USB");
            connected = true;
        }

        if socket.can_recv() {
            match socket.recv_slice(&mut recv_buf) {
                Ok(n) if n > 0 => {
                    if imu_tx.send(recv_buf[..n].to_vec()).is_err() {
                        tracing::info!("IMU receiver dropped, ECM thread exiting");
                        return Ok(());
                    }
                }
                Ok(_) => {}
                Err(e) => tracing::warn!(?e, "TCP recv error"),
            }
        }

        if connected && !socket.is_active() {
            tracing::warn!("TCP connection to XReal One IMU closed");
            return Ok(());
        }

        // Periodic status while connecting.
        if !connected && last_status.elapsed() > Duration::from_secs(1) {
            last_status = Instant::now();
            let rx = counters.rx.load(Ordering::Relaxed);
            let tx = counters.tx.load(Ordering::Relaxed);
            let tcp_state = socket.state();
            tracing::info!(
                elapsed_s = start.elapsed().as_secs(),
                usb_rx_frames = rx,
                usb_tx_frames = tx,
                queued_rx = device.rx_queue.len(),
                tcp_state = ?tcp_state,
                "Waiting for TCP connection..."
            );
        }

        if !connected && start.elapsed() > TCP_CONNECT_TIMEOUT {
            let rx = counters.rx.load(Ordering::Relaxed);
            let tx = counters.tx.load(Ordering::Relaxed);
            return Err(anyhow!(
                "TCP connection timed out ({TCP_CONNECT_TIMEOUT:?}). \
                 USB frames: {tx} sent, {rx} received. \
                 If 0 received, the device may not be responding — \
                 check Zadig WinUSB is installed for BOTH MI_03 and MI_04."
            ));
        }

        std::thread::sleep(Duration::from_micros(100));
    }
}

// ---------------------------------------------------------------------------
// smoltcp Device implementation
// ---------------------------------------------------------------------------

struct EcmDevice {
    frame_rx: std_mpsc::Receiver<Vec<u8>>,
    frame_tx: std_mpsc::Sender<Vec<u8>>,
    rx_queue: VecDeque<Vec<u8>>,
}

impl EcmDevice {
    fn drain_rx(&mut self) {
        while let Ok(frame) = self.frame_rx.try_recv() {
            self.rx_queue.push_back(frame);
        }
    }
}

impl Device for EcmDevice {
    type RxToken<'a> = EcmRxToken;
    type TxToken<'a> = EcmTxToken;

    fn receive(&mut self, _timestamp: SmolInstant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        let frame = self.rx_queue.pop_front()?;
        Some((
            EcmRxToken { frame },
            EcmTxToken {
                tx: self.frame_tx.clone(),
            },
        ))
    }

    fn transmit(&mut self, _timestamp: SmolInstant) -> Option<Self::TxToken<'_>> {
        Some(EcmTxToken {
            tx: self.frame_tx.clone(),
        })
    }

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.medium = Medium::Ethernet;
        caps.max_transmission_unit = MAX_FRAME_SIZE;
        caps
    }
}

struct EcmRxToken {
    frame: Vec<u8>,
}

impl RxToken for EcmRxToken {
    fn consume<R, F>(mut self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        f(&mut self.frame)
    }
}

struct EcmTxToken {
    tx: std_mpsc::Sender<Vec<u8>>,
}

impl TxToken for EcmTxToken {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let mut buffer = vec![0u8; len];
        let result = f(&mut buffer);
        let _ = self.tx.send(buffer);
        result
    }
}

fn smol_now() -> SmolInstant {
    use std::sync::OnceLock;
    static EPOCH: OnceLock<Instant> = OnceLock::new();
    let epoch = EPOCH.get_or_init(Instant::now);
    SmolInstant::from_micros(epoch.elapsed().as_micros() as i64)
}
