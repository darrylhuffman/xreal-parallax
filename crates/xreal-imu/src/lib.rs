pub mod fusion;
pub mod protocol;
pub mod types;

use anyhow::Result;
use fusion::SensorFusion;
use protocol::ProtocolParser;
use std::net::Ipv4Addr;
use tokio::io::AsyncReadExt;
use tokio::net::TcpStream;
use tokio::sync::watch;
use types::Orientation;

/// XReal One IMU endpoint.
const XREAL_ONE_ADDR: (Ipv4Addr, u16) = (Ipv4Addr::new(169, 254, 2, 1), 52998);

/// Commands sent to the IMU processing task.
enum ImuCommand {
    SetZero,
    Recalibrate(u32),
}

/// Client for the XReal One IMU.
///
/// Connects to the glasses over TCP, parses the binary IMU stream,
/// runs Madgwick sensor fusion, and publishes the latest orientation.
pub struct ImuClient {
    orientation_rx: watch::Receiver<Orientation>,
    command_tx: tokio::sync::mpsc::UnboundedSender<ImuCommand>,
    _task: tokio::task::JoinHandle<()>,
}

impl ImuClient {
    /// Connect to the XReal One IMU and start processing.
    pub async fn connect(madgwick_beta: f32, calibration_samples: u32) -> Result<Self> {
        let addr = format!("{}:{}", XREAL_ONE_ADDR.0, XREAL_ONE_ADDR.1);
        tracing::info!(%addr, "Connecting to XReal One IMU");

        let stream = TcpStream::connect(&addr).await?;
        tracing::info!("Connected to XReal One IMU");

        let (orientation_tx, orientation_rx) = watch::channel(Orientation::default());
        let (command_tx, command_rx) = tokio::sync::mpsc::unbounded_channel();

        let task = tokio::spawn(imu_read_loop(
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
        let (command_tx, _) = tokio::sync::mpsc::unbounded_channel();
        let task = tokio::spawn(async move {
            // Keep the sender alive.
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

/// Background task: read TCP stream, parse packets, run fusion, publish orientation.
async fn imu_read_loop(
    mut stream: TcpStream,
    orientation_tx: watch::Sender<Orientation>,
    mut command_rx: tokio::sync::mpsc::UnboundedReceiver<ImuCommand>,
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

                        // Drain all available samples.
                        while let Some(result) = parser.next_sample() {
                            match result {
                                Ok(sample) => {
                                    if let Some(orientation) = fusion.update(&sample) {
                                        let _ = orientation_tx.send(orientation);
                                    }
                                    sample_count += 1;
                                    if sample_count % 1000 == 0 {
                                        tracing::debug!(sample_count, "IMU samples processed");
                                    }
                                }
                                Err(e) => {
                                    tracing::trace!(?e, "Skipping non-sensor packet");
                                }
                            }
                        }
                    }
                    Err(e) => {
                        tracing::error!(?e, "IMU TCP read error");
                        break;
                    }
                }
            }
            Some(cmd) = command_rx.recv() => {
                match cmd {
                    ImuCommand::SetZero => fusion.set_zero(),
                    ImuCommand::Recalibrate(n) => fusion.recalibrate(n),
                }
            }
        }
    }
}
