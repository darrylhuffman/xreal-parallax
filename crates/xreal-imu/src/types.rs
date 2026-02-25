use glam::{Quat, Vec3};

/// Raw sensor reading from the XReal One IMU.
#[derive(Debug, Clone, Copy)]
pub struct RawImuSample {
    /// Gyroscope angular velocity (rad/s).
    pub gyro: Vec3,
    /// Accelerometer linear acceleration (m/s^2).
    pub accel: Vec3,
}

/// Fused orientation output from the sensor fusion filter.
#[derive(Debug, Clone, Copy)]
pub struct Orientation {
    /// Absolute orientation as a unit quaternion.
    pub quaternion: Quat,
}

impl Default for Orientation {
    fn default() -> Self {
        Self {
            quaternion: Quat::IDENTITY,
        }
    }
}
