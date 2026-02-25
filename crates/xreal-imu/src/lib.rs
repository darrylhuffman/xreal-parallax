pub mod fusion;
pub mod protocol;
pub mod types;
pub mod usb_ecm;

use anyhow::Result;
use fusion::SensorFusion;
use protocol::ProtocolParser;
use std::net::Ipv4Addr;
use tokio::io::AsyncReadExt;
use tokio::net::TcpStream;
use tokio::sync::{mpsc, watch};
use types::Orientation;

/// XReal One IMU endpoint (for direct TCP when the OS network driver works).
const XREAL_ONE_ADDR: (Ipv4Addr, u16) = (Ipv4Addr::new(169, 254, 2, 1), 52998);

/// Commands sent to the IMU processing task.
enum ImuCommand {
    SetZero,
    Recalibrate(u32),
}

/// Client for the XReal One IMU.
///
/// Connects to the glasses over USB CDC ECM (preferred) or direct TCP,
/// parses the binary IMU stream, runs Madgwick sensor fusion, and publishes
/// the latest orientation.
pub struct ImuClient {
    orientation_rx: watch::Receiver<Orientation>,
    command_tx: mpsc::UnboundedSender<ImuCommand>,
    _task: tokio::task::JoinHandle<()>,
}

impl ImuClient {
    /// Connect to the XReal One IMU and start processing.
    ///
    /// Tries USB CDC ECM first (works without a Windows network driver),
    /// then falls back to direct TCP (requires working CDC ECM OS driver).
    pub async fn connect(madgwick_beta: f32, calibration_samples: u32) -> Result<Self> {
        // Try USB transport first (works on Windows without CDC ECM driver).
        tracing::info!("Attempting USB CDC ECM connection to XReal One IMU...");
        match tokio::task::spawn_blocking(usb_ecm::connect_usb).await? {
            Ok(usb_rx) => {
                tracing::info!("Connected to XReal One IMU via USB CDC ECM");
                let (orientation_tx, orientation_rx) = watch::channel(Orientation::default());
                let (command_tx, command_rx) = mpsc::unbounded_channel();

                let task = tokio::spawn(imu_read_loop_channel(
                    usb_rx,
                    orientation_tx,
                    command_rx,
                    madgwick_beta,
                    calibration_samples,
                ));

                return Ok(Self {
                    orientation_rx,
                    command_tx,
                    _task: task,
                });
            }
            Err(e) => {
                tracing::warn!("USB CDC ECM unavailable: {e:#}");
                tracing::info!("Falling back to direct TCP...");
            }
        }

        // Fall back to direct TCP (requires working OS network driver).
        let addr = format!("{}:{}", XREAL_ONE_ADDR.0, XREAL_ONE_ADDR.1);
        tracing::info!(%addr, "Connecting to XReal One IMU via TCP (3s timeout)");

        let stream = tokio::time::timeout(
            std::time::Duration::from_secs(3),
            TcpStream::connect(&addr),
        )
        .await
        .map_err(|_| anyhow::anyhow!("IMU connection timed out after 3s"))?
        ?;
        tracing::info!("Connected to XReal One IMU via TCP");

        let (orientation_tx, orientation_rx) = watch::channel(Orientation::default());
        let (command_tx, command_rx) = mpsc::unbounded_channel();

        let task = tokio::spawn(imu_read_loop_tcp(
            stream,
            orientation_tx,
            command_rx,
            madgwick_beta,
            calibration_samples,
        ));

        Ok(Self {
            orientation_rx,
            command_tx,
            _task: task,
        })
    }

    /// Create a mock client for development without glasses connected.
    pub fn mock() -> Self {
        let (orientation_tx, orientation_rx) = watch::channel(Orientation::default());
        let (command_tx, _) = mpsc::unbounded_channel();
        let task = tokio::spawn(async move {
            let _tx = orientation_tx;
            tokio::signal::ctrl_c().await.ok();
        });
        Self {
            orientation_rx,
            command_tx,
            _task: task,
        }
    }

    /// Get the latest fused orientation (non-blocking).
    pub fn orientation(&self) -> Orientation {
        *self.orientation_rx.borrow()
    }

    /// Set current head position as the zero reference.
    pub fn set_zero(&self) {
        let _ = self.command_tx.send(ImuCommand::SetZero);
    }

    /// Restart gyro bias calibration.
    pub fn recalibrate(&self, samples: u32) {
        let _ = self.command_tx.send(ImuCommand::Recalibrate(samples));
    }
}

/// Process IMU data arriving via a channel (USB CDC ECM path).
async fn imu_read_loop_channel(
    mut data_rx: mpsc::UnboundedReceiver<Vec<u8>>,
    orientation_tx: watch::Sender<Orientation>,
    mut command_rx: mpsc::UnboundedReceiver<ImuCommand>,
    madgwick_beta: f32,
    calibration_samples: u32,
) {
    let mut parser = ProtocolParser::new();
    let mut fusion = SensorFusion::new(madgwick_beta, calibration_samples);
    let mut sample_count: u64 = 0;

    loop {
        tokio::select! {
            Some(data) = data_rx.recv() => {
                parser.push_data(&data);
                process_samples(&mut parser, &mut fusion, &orientation_tx, &mut sample_count);
            }
            Some(cmd) = command_rx.recv() => {
                handle_command(cmd, &mut fusion);
            }
            else => break,
        }
    }
    tracing::warn!("USB IMU data stream ended");
}

/// Process IMU data arriving via TCP stream (direct network path).
async fn imu_read_loop_tcp(
    mut stream: TcpStream,
    orientation_tx: watch::Sender<Orientation>,
    mut command_rx: mpsc::UnboundedReceiver<ImuCommand>,
    madgwick_beta: f32,
    calibration_samples: u32,
) {
    let mut parser = ProtocolParser::new();
    let mut fusion = SensorFusion::new(madgwick_beta, calibration_samples);
    let mut buf = [0u8; 4096];
    let mut sample_count: u64 = 0;

    loop {
        tokio::select! {
            result = stream.read(&mut buf) => {
                match result {
                    Ok(0) => {
                        tracing::warn!("IMU TCP connection closed");
                        break;
                    }
                    Ok(n) => {
                        parser.push_data(&buf[..n]);
                        process_samples(&mut parser, &mut fusion, &orientation_tx, &mut sample_count);
                    }
                    Err(e) => {
                        tracing::error!(?e, "IMU TCP read error");
                        break;
                    }
                }
            }
            Some(cmd) = command_rx.recv() => {
                handle_command(cmd, &mut fusion);
            }
        }
    }
}

/// Drain parsed IMU samples and run fusion.
fn process_samples(
    parser: &mut ProtocolParser,
    fusion: &mut SensorFusion,
    orientation_tx: &watch::Sender<Orientation>,
    sample_count: &mut u64,
) {
    while let Some(result) = parser.next_sample() {
        match result {
            Ok(sample) => {
                if let Some(orientation) = fusion.update(&sample) {
                    let _ = orientation_tx.send(orientation);
                }
                *sample_count += 1;
                if *sample_count % 1000 == 0 {
                    tracing::debug!(samples = *sample_count, "IMU samples processed");
                }
            }
            Err(e) => {
                tracing::trace!(?e, "Skipping non-sensor packet");
            }
        }
    }
}

/// Handle an IMU command.
fn handle_command(cmd: ImuCommand, fusion: &mut SensorFusion) {
    match cmd {
        ImuCommand::SetZero => fusion.set_zero(),
        ImuCommand::Recalibrate(n) => fusion.recalibrate(n),
    }
}
