use glam::{Mat4, Quat, Vec3};

/// Camera for rendering the AR scene.
///
/// Position is fixed at the origin. Only orientation changes (from IMU).
/// For stereo rendering, a horizontal offset is applied for each eye.
pub struct Camera {
    /// Head orientation from IMU.
    pub orientation: Quat,
    /// Vertical field of view in degrees (XReal One: ~46°).
    pub fov_y_degrees: f32,
    /// Aspect ratio (width / height). Per eye: 1920/1080 ≈ 1.778.
    pub aspect_ratio: f32,
    /// Near clipping plane (meters).
    pub near: f32,
    /// Far clipping plane (meters).
    pub far: f32,
}

impl Camera {
    pub fn new() -> Self {
        Self {
            orientation: Quat::IDENTITY,
            fov_y_degrees: 46.0,
            aspect_ratio: 1920.0 / 1080.0,
            near: 0.1,
            far: 100.0,
        }
    }

    /// View matrix (inverse of camera world transform).
    pub fn view_matrix(&self) -> Mat4 {
        // Camera is at origin, only rotated.
        Mat4::from_quat(self.orientation.conjugate())
    }

    /// View matrix with a horizontal eye offset for stereo rendering.
    pub fn stereo_view_matrix(&self, eye_offset: f32) -> Mat4 {
        let right = self.orientation * Vec3::X;
        let eye_pos = right * eye_offset;
        let rotation = Mat4::from_quat(self.orientation.conjugate());
        let translation = Mat4::from_translation(-eye_pos);
        rotation * translation
    }

    /// Perspective projection matrix.
    pub fn projection_matrix(&self) -> Mat4 {
        Mat4::perspective_rh(
            self.fov_y_degrees.to_radians(),
            self.aspect_ratio,
            self.near,
            self.far,
        )
    }
}
