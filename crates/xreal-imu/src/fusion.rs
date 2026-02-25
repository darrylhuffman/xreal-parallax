use crate::types::{Orientation, RawImuSample};
use ahrs::{Ahrs, Madgwick};
use glam::{Quat, Vec3};
use nalgebra::Vector3;

/// Sensor fusion using the Madgwick AHRS filter.
///
/// Converts raw gyro + accelerometer readings into a stable orientation quaternion.
pub struct SensorFusion {
    filter: Madgwick<f64>,
    /// Gyroscope bias (average of calibration samples).
    gyro_bias: Vec3,
    /// Calibration state.
    calibration: CalibrationState,
    /// Reference quaternion for "set zero".
    zero_ref: Quat,
    /// Last known time delta.
    _sample_dt: f64,
}

enum CalibrationState {
    Collecting {
        samples: Vec<Vec3>,
        target: usize,
    },
    Calibrated,
}

impl SensorFusion {
    pub fn new(beta: f32, calibration_samples: u32) -> Self {
        // The Madgwick filter needs a sample period. We estimate ~1000Hz from the sensor.
        let sample_dt = 1.0 / 1000.0;
        Self {
            filter: Madgwick::new(sample_dt, beta as f64),
            gyro_bias: Vec3::ZERO,
            calibration: CalibrationState::Collecting {
                samples: Vec::with_capacity(calibration_samples as usize),
                target: calibration_samples as usize,
            },
            zero_ref: Quat::IDENTITY,
            _sample_dt: sample_dt,
        }
    }

    /// Process a raw IMU sample and return the current orientation (if calibrated).
    pub fn update(&mut self, sample: &RawImuSample) -> Option<Orientation> {
        match &mut self.calibration {
            CalibrationState::Collecting { samples, target } => {
                samples.push(sample.gyro);
                if samples.len() >= *target {
                    // Compute average gyro bias.
                    let sum: Vec3 = samples.iter().copied().sum();
                    self.gyro_bias = sum / samples.len() as f32;
                    self.calibration = CalibrationState::Calibrated;
                    tracing::info!(
                        bias_x = self.gyro_bias.x,
                        bias_y = self.gyro_bias.y,
                        bias_z = self.gyro_bias.z,
                        "IMU calibration complete"
                    );
                }
                None
            }
            CalibrationState::Calibrated => {
                // Subtract gyro bias.
                let corrected_gyro = sample.gyro - self.gyro_bias;

                // Convert to nalgebra vectors for ahrs crate.
                let gyro = Vector3::new(
                    corrected_gyro.x as f64,
                    corrected_gyro.y as f64,
                    corrected_gyro.z as f64,
                );
                let accel = Vector3::new(
                    sample.accel.x as f64,
                    sample.accel.y as f64,
                    sample.accel.z as f64,
                );

                // Update Madgwick filter.
                if self.filter.update_imu(&gyro, &accel).is_err() {
                    return None;
                }

                // Extract quaternion from filter.
                let q = self.filter.quat;
                let absolute = Quat::from_xyzw(
                    q.coords[0] as f32,
                    q.coords[1] as f32,
                    q.coords[2] as f32,
                    q.coords[3] as f32, // w component
                );

                // Apply zero reference: output orientation relative to "home".
                let relative = self.zero_ref.conjugate() * absolute;

                Some(Orientation {
                    quaternion: relative,
                })
            }
        }
    }

    /// Set current orientation as the zero reference.
    pub fn set_zero(&mut self) {
        let q = self.filter.quat;
        self.zero_ref = Quat::from_xyzw(
            q.coords[0] as f32,
            q.coords[1] as f32,
            q.coords[2] as f32,
            q.coords[3] as f32,
        );
        tracing::info!("Zero reference set");
    }

    /// Restart calibration.
    pub fn recalibrate(&mut self, samples: u32) {
        self.calibration = CalibrationState::Collecting {
            samples: Vec::with_capacity(samples as usize),
            target: samples as usize,
        };
        self.gyro_bias = Vec3::ZERO;
        tracing::info!(samples, "Recalibration started");
    }

    /// Whether calibration is complete.
    pub fn is_calibrated(&self) -> bool {
        matches!(self.calibration, CalibrationState::Calibrated)
    }
}
