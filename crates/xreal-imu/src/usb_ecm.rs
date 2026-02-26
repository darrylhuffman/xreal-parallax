//! CDC ECM/NCM transport for XReal One IMU over raw USB.
//!
//! Windows lacks built-in CDC ECM support and the NCM driver (UsbNcm.sys)
//! fails for this device. This module bypasses all OS drivers:
//!
//! 1. Opens the XReal One USB device directly via WinUSB (nusb)
//! 2. Speaks CDC NCM or ECM to exchange Ethernet frames
//! 3. Runs a user-space TCP/IP stack (smoltcp) over those frames
//! 4. Establishes TCP to 169.254.2.1:52998 for IMU data
//!
//! The XReal One exposes TWO network interfaces:
//!   - Interface 1/2: CDC NCM (primary — IMU TCP service lives here)
//!   - Interface 3/4: CDC ECM (secondary — IPv6 only)
//!
//! Prerequisite: install WinUSB driver for the network interfaces via Zadig.

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

/// CDC class request: SET_ETHERNET_PACKET_FILTER.
const SET_ETHERNET_PACKET_FILTER: u8 = 0x43;

/// Accept all packet types.
const PACKET_TYPE_ALL: u16 = 0x001F;

/// Max Ethernet frame size.
const MAX_FRAME_SIZE: usize = 1514;

/// Max NTB (NCM Transfer Block) size for USB transfers.
const MAX_NTB_SIZE: usize = 4096;

/// TCP connection timeout.
const TCP_CONNECT_TIMEOUT: Duration = Duration::from_secs(10);

/// Whether to use NCM framing (NTB headers) or raw ECM frames.
#[derive(Debug, Clone, Copy, PartialEq)]
enum FramingMode {
    Ncm,
    Ecm,
}

struct FrameCounters {
    rx: AtomicU64,
    tx: AtomicU64,
}

/// Connect to the XReal One IMU via USB CDC NCM/ECM.
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
        product_string = ?device_info.product_string(),
        "Found XReal One USB device"
    );

    // 2. Open the device.
    let device = device_info
        .open()
        .wait()
        .map_err(|e| anyhow!("Failed to open USB device (is WinUSB installed via Zadig?): {e}"))?;

    // 3. Dump descriptors and discover interfaces.
    let config = device
        .active_configuration()
        .map_err(|e| anyhow!("Failed to read USB configuration: {e}"))?;

    dump_usb_descriptors(&config);
    let all = find_all_network_interfaces(&config);

    // 4. Try NCM first (primary interface, where IMU service lives), then ECM.
    let mut last_err = anyhow!("No network interfaces found");

    for candidate in &all {
        let mode_name = match candidate.mode {
            FramingMode::Ncm => "NCM",
            FramingMode::Ecm => "ECM",
        };
        tracing::info!(
            mode = mode_name,
            comm = candidate.comm_iface,
            data = candidate.data_iface,
            bulk_in = format!("0x{:02x}", candidate.bulk_in_ep),
            bulk_out = format!("0x{:02x}", candidate.bulk_out_ep),
            "Trying {} interface...", mode_name
        );

        match try_connect(&device, candidate) {
            Ok(rx) => return Ok(rx),
            Err(e) => {
                tracing::warn!("  {} failed: {e:#}", mode_name);
                last_err = e;
            }
        }
    }

    Err(last_err)
}

/// Try to establish IMU connection on a specific network interface.
fn try_connect(
    device: &nusb::Device,
    iface: &NetworkInterface,
) -> Result<mpsc::UnboundedReceiver<Vec<u8>>> {
    let mode = iface.mode;

    // Claim interfaces.
    let comm = device
        .claim_interface(iface.comm_iface)
        .wait()
        .map_err(|e| anyhow!("Failed to claim comm interface {}: {e}", iface.comm_iface))?;

    let data = device
        .claim_interface(iface.data_iface)
        .wait()
        .map_err(|e| anyhow!("Failed to claim data interface {}: {e}", iface.data_iface))?;

    // Activate bulk endpoints (alt setting 1).
    tracing::info!("  Activating data interface alt setting 1...");
    data.set_alt_setting(1)
        .wait()
        .map_err(|e| anyhow!("  set_alt_setting(1) failed: {e}"))?;
    tracing::info!("  Alt setting 1 activated");

    // Enable packet filter.
    match comm.control_out(
        nusb::transfer::ControlOut {
            control_type: nusb::transfer::ControlType::Class,
            recipient: nusb::transfer::Recipient::Interface,
            request: SET_ETHERNET_PACKET_FILTER,
            value: PACKET_TYPE_ALL,
            index: iface.comm_iface as u16,
            data: &[],
        },
        Duration::from_millis(1000),
    ).wait() {
        Ok(()) => tracing::info!("  SET_ETHERNET_PACKET_FILTER succeeded"),
        Err(e) => tracing::warn!("  SET_ETHERNET_PACKET_FILTER: {e}"),
    }

    // Open bulk endpoints.
    let reader = data
        .endpoint::<nusb::transfer::Bulk, nusb::transfer::In>(iface.bulk_in_ep)
        .map_err(|e| anyhow!("Failed to open bulk IN: {e}"))?
        .reader(MAX_NTB_SIZE);

    let writer = data
        .endpoint::<nusb::transfer::Bulk, nusb::transfer::Out>(iface.bulk_out_ep)
        .map_err(|e| anyhow!("Failed to open bulk OUT: {e}"))?
        .writer(MAX_NTB_SIZE);

    // Spawn threads.
    let counters = Arc::new(FrameCounters {
        rx: AtomicU64::new(0),
        tx: AtomicU64::new(0),
    });

    let (imu_tx, imu_rx) = mpsc::unbounded_channel();
    let (frame_tx, frame_rx) = std_mpsc::channel::<Vec<u8>>();
    let (write_tx, write_rx) = std_mpsc::channel::<Vec<u8>>();

    let rx_cnt = counters.clone();
    std::thread::Builder::new()
        .name("xreal-usb-reader".into())
        .spawn(move || usb_reader_thread(reader, frame_tx, rx_cnt, mode))?;

    let tx_cnt = counters.clone();
    std::thread::Builder::new()
        .name("xreal-usb-writer".into())
        .spawn(move || usb_writer_thread(writer, write_rx, tx_cnt, mode))?;

    let tcp_cnt = counters.clone();
    std::thread::Builder::new()
        .name("xreal-ecm-tcp".into())
        .spawn(move || {
            if let Err(e) = ecm_tcp_thread(frame_rx, write_tx, imu_tx, tcp_cnt) {
                tracing::error!(?e, "TCP thread exited with error");
            }
        })?;

    Ok(imu_rx)
}

// ---------------------------------------------------------------------------
// Descriptor discovery
// ---------------------------------------------------------------------------

struct NetworkInterface {
    mode: FramingMode,
    comm_iface: u8,
    data_iface: u8,
    bulk_in_ep: u8,
    bulk_out_ep: u8,
}

fn dump_usb_descriptors(config: &nusb::descriptors::ConfigurationDescriptor) {
    tracing::info!("=== USB Descriptor Dump ===");
    for iface_desc in config.interface_alt_settings() {
        let class_name = match (iface_desc.class(), iface_desc.subclass()) {
            (0x02, 0x02) => "CDC ACM",
            (0x02, 0x06) => "CDC ECM",
            (0x02, 0x0D) => "CDC NCM",
            (0x0A, _) => "CDC Data",
            (0x0E, _) => "Video",
            (0x01, _) => "Audio",
            (0x03, _) => "HID",
            (0xFF, _) => "Vendor",
            _ => "Other",
        };
        let eps: Vec<String> = iface_desc.endpoints().map(|ep| {
            let dir = if ep.direction() == nusb::transfer::Direction::In { "IN" } else { "OUT" };
            let tt = match ep.transfer_type() {
                nusb::descriptors::TransferType::Bulk => "bulk",
                nusb::descriptors::TransferType::Interrupt => "int",
                nusb::descriptors::TransferType::Isochronous => "iso",
                nusb::descriptors::TransferType::Control => "ctrl",
            };
            format!("0x{:02x} {} {}", ep.address(), dir, tt)
        }).collect();
        tracing::info!(
            "  iface={} alt={} class={:02x}/{:02x} {} eps={:?}",
            iface_desc.interface_number(),
            iface_desc.alternate_setting(),
            iface_desc.class(),
            iface_desc.subclass(),
            class_name,
            eps,
        );
    }
    tracing::info!("=== End Descriptor Dump ===");
}

/// Find all CDC network interfaces (NCM first, then ECM).
fn find_all_network_interfaces(
    config: &nusb::descriptors::ConfigurationDescriptor,
) -> Vec<NetworkInterface> {
    let mut ncm_comm = Vec::new();
    let mut ecm_comm = Vec::new();
    let mut data_ifaces: Vec<(u8, u8, u8, u8)> = Vec::new(); // (iface_num, alt, bulk_in, bulk_out)

    for iface_desc in config.interface_alt_settings() {
        let class = iface_desc.class();
        let subclass = iface_desc.subclass();
        let iface_num = iface_desc.interface_number();

        if class == 0x02 && subclass == 0x0D {
            ncm_comm.push(iface_num);
        }
        if class == 0x02 && subclass == 0x06 {
            ecm_comm.push(iface_num);
        }

        if class == 0x0A {
            let mut bin = 0u8;
            let mut bout = 0u8;
            for ep in iface_desc.endpoints() {
                if ep.transfer_type() == nusb::descriptors::TransferType::Bulk {
                    match ep.direction() {
                        nusb::transfer::Direction::In => bin = ep.address(),
                        nusb::transfer::Direction::Out => bout = ep.address(),
                    }
                }
            }
            if bin != 0 && bout != 0 {
                data_ifaces.push((iface_num, iface_desc.alternate_setting(), bin, bout));
            }
        }
    }

    let mut result = Vec::new();

    // NCM interfaces first (primary, where IMU service lives).
    for &comm in &ncm_comm {
        // Data interface is typically comm + 1.
        if let Some(&(di, _, bin, bout)) = data_ifaces.iter().find(|(n, _, _, _)| *n == comm + 1) {
            result.push(NetworkInterface {
                mode: FramingMode::Ncm,
                comm_iface: comm,
                data_iface: di,
                bulk_in_ep: bin,
                bulk_out_ep: bout,
            });
        }
    }

    // ECM interfaces as fallback.
    for &comm in &ecm_comm {
        if let Some(&(di, _, bin, bout)) = data_ifaces.iter().find(|(n, _, _, _)| *n == comm + 1) {
            result.push(NetworkInterface {
                mode: FramingMode::Ecm,
                comm_iface: comm,
                data_iface: di,
                bulk_in_ep: bin,
                bulk_out_ep: bout,
            });
        }
    }

    result
}

// ---------------------------------------------------------------------------
// NCM Transfer Block (NTB) framing
// ---------------------------------------------------------------------------

/// Wrap a single Ethernet frame in an NCM NTB16 for transmission.
fn ncm_wrap_frame(frame: &[u8], sequence: u16) -> Vec<u8> {
    let ndp_offset: u16 = 12; // NDP immediately after NTH16
    // NDP16: signature(4) + length(2) + next_ndp(2) + 1 entry(4) + terminator(4) = 16
    let ndp_len: u16 = 16;
    let datagram_offset = ndp_offset + ndp_len;
    let block_length = datagram_offset + frame.len() as u16;

    let mut ntb = Vec::with_capacity(block_length as usize);

    // NTH16 (12 bytes)
    ntb.extend_from_slice(b"NCMH");                         // dwSignature
    ntb.extend_from_slice(&12u16.to_le_bytes());             // wHeaderLength
    ntb.extend_from_slice(&sequence.to_le_bytes());          // wSequence
    ntb.extend_from_slice(&block_length.to_le_bytes());      // wBlockLength
    ntb.extend_from_slice(&ndp_offset.to_le_bytes());        // wNdpIndex

    // NDP16 (16 bytes)
    ntb.extend_from_slice(b"NCM0");                          // dwSignature
    ntb.extend_from_slice(&ndp_len.to_le_bytes());           // wLength
    ntb.extend_from_slice(&0u16.to_le_bytes());              // wNextNdpIndex
    ntb.extend_from_slice(&datagram_offset.to_le_bytes());   // wDatagramIndex[0]
    ntb.extend_from_slice(&(frame.len() as u16).to_le_bytes()); // wDatagramLength[0]
    ntb.extend_from_slice(&0u16.to_le_bytes());              // terminator index
    ntb.extend_from_slice(&0u16.to_le_bytes());              // terminator length

    // Datagram (raw Ethernet frame)
    ntb.extend_from_slice(frame);

    ntb
}

/// Extract Ethernet frames from an NCM NTB16.
fn ncm_unwrap_frames(ntb: &[u8]) -> Vec<Vec<u8>> {
    let mut frames = Vec::new();

    if ntb.len() < 12 {
        return frames;
    }

    // Validate NTH16 signature.
    if &ntb[0..4] != b"NCMH" {
        return frames;
    }

    let ndp_index = u16::from_le_bytes([ntb[10], ntb[11]]) as usize;
    if ndp_index + 12 > ntb.len() {
        return frames;
    }

    // Validate NDP16 signature.
    if &ntb[ndp_index..ndp_index + 4] != b"NCM0" && &ntb[ndp_index..ndp_index + 4] != b"NCM1" {
        return frames;
    }

    let ndp_length = u16::from_le_bytes([ntb[ndp_index + 4], ntb[ndp_index + 5]]) as usize;

    // Parse datagram pointer entries (start at ndp_index + 8).
    let mut offset = ndp_index + 8;
    while offset + 4 <= ndp_index + ndp_length && offset + 4 <= ntb.len() {
        let dg_index = u16::from_le_bytes([ntb[offset], ntb[offset + 1]]) as usize;
        let dg_length = u16::from_le_bytes([ntb[offset + 2], ntb[offset + 3]]) as usize;

        if dg_index == 0 && dg_length == 0 {
            break; // Terminator
        }

        if dg_index + dg_length <= ntb.len() && dg_length > 0 {
            frames.push(ntb[dg_index..dg_index + dg_length].to_vec());
        }

        offset += 4;
    }

    frames
}

// ---------------------------------------------------------------------------
// Background threads
// ---------------------------------------------------------------------------

fn usb_reader_thread(
    mut reader: nusb::io::EndpointRead<nusb::transfer::Bulk>,
    frame_tx: std_mpsc::Sender<Vec<u8>>,
    counters: Arc<FrameCounters>,
    mode: FramingMode,
) {
    tracing::debug!(mode = ?mode, "USB reader thread started");
    let mut buf = vec![0u8; MAX_NTB_SIZE];
    loop {
        match reader.read(&mut buf) {
            Ok(n) if n > 0 => {
                let raw = &buf[..n];

                let frames = match mode {
                    FramingMode::Ncm => ncm_unwrap_frames(raw),
                    FramingMode::Ecm => vec![raw.to_vec()],
                };

                for frame in frames {
                    let count = counters.rx.fetch_add(1, Ordering::Relaxed) + 1;
                    if count <= 5 || count % 500 == 0 {
                        let preview: String = frame[..frame.len().min(32)]
                            .iter()
                            .map(|b| format!("{:02x}", b))
                            .collect::<Vec<_>>()
                            .join(" ");
                        let ethertype = if frame.len() >= 14 {
                            format!("{:02x}{:02x}", frame[12], frame[13])
                        } else {
                            "??".into()
                        };
                        tracing::info!(
                            rx = count,
                            len = frame.len(),
                            ethertype,
                            preview,
                            "RX"
                        );
                    }
                    if frame_tx.send(frame).is_err() {
                        return;
                    }
                }
            }
            Ok(_) => continue,
            Err(e) => {
                tracing::error!(?e, "USB bulk IN error");
                return;
            }
        }
    }
}

fn usb_writer_thread(
    mut writer: nusb::io::EndpointWrite<nusb::transfer::Bulk>,
    write_rx: std_mpsc::Receiver<Vec<u8>>,
    counters: Arc<FrameCounters>,
    mode: FramingMode,
) {
    tracing::debug!(mode = ?mode, "USB writer thread started");
    let mut sequence: u16 = 0;
    while let Ok(frame) = write_rx.recv() {
        let count = counters.tx.fetch_add(1, Ordering::Relaxed) + 1;
        if count <= 5 || count % 500 == 0 {
            let preview: String = frame[..frame.len().min(32)]
                .iter()
                .map(|b| format!("{:02x}", b))
                .collect::<Vec<_>>()
                .join(" ");
            tracing::info!(tx = count, len = frame.len(), preview, "TX");
        }

        let data = match mode {
            FramingMode::Ncm => {
                let ntb = ncm_wrap_frame(&frame, sequence);
                sequence = sequence.wrapping_add(1);
                ntb
            }
            FramingMode::Ecm => frame,
        };

        if let Err(e) = writer.write_all(&data) {
            tracing::error!(?e, "USB bulk OUT error");
            return;
        }
    }
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

    tracing::info!("TCP SYN queued via smoltcp");

    let start = Instant::now();
    let mut connected = false;
    let mut recv_buf = [0u8; 4096];
    let mut last_status = Instant::now();

    loop {
        let timestamp = smol_now();
        device.drain_rx();
        iface.poll(timestamp, &mut device, &mut sockets);

        let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);

        if !connected && socket.may_recv() {
            tracing::info!("TCP connected to XReal One IMU!");
            connected = true;
        }

        if socket.can_recv() {
            match socket.recv_slice(&mut recv_buf) {
                Ok(n) if n > 0 => {
                    if imu_tx.send(recv_buf[..n].to_vec()).is_err() {
                        return Ok(());
                    }
                }
                Ok(_) => {}
                Err(e) => tracing::warn!(?e, "TCP recv error"),
            }
        }

        if connected && !socket.is_active() {
            tracing::warn!("TCP connection closed");
            return Ok(());
        }

        if !connected && last_status.elapsed() > Duration::from_secs(1) {
            last_status = Instant::now();
            let rx = counters.rx.load(Ordering::Relaxed);
            let tx = counters.tx.load(Ordering::Relaxed);
            tracing::info!(
                elapsed = start.elapsed().as_secs(),
                rx_frames = rx,
                tx_frames = tx,
                tcp = ?socket.state(),
                "Waiting for TCP..."
            );
        }

        if !connected && start.elapsed() > TCP_CONNECT_TIMEOUT {
            let rx = counters.rx.load(Ordering::Relaxed);
            let tx = counters.tx.load(Ordering::Relaxed);
            return Err(anyhow!(
                "TCP timed out ({TCP_CONNECT_TIMEOUT:?}). USB: {tx} TX, {rx} RX."
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

    fn receive(&mut self, _ts: SmolInstant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        let frame = self.rx_queue.pop_front()?;
        Some((
            EcmRxToken { frame },
            EcmTxToken { tx: self.frame_tx.clone() },
        ))
    }

    fn transmit(&mut self, _ts: SmolInstant) -> Option<Self::TxToken<'_>> {
        Some(EcmTxToken { tx: self.frame_tx.clone() })
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
    fn consume<R, F: FnOnce(&mut [u8]) -> R>(mut self, f: F) -> R {
        f(&mut self.frame)
    }
}

struct EcmTxToken {
    tx: std_mpsc::Sender<Vec<u8>>,
}

impl TxToken for EcmTxToken {
    fn consume<R, F: FnOnce(&mut [u8]) -> R>(self, len: usize, f: F) -> R {
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
