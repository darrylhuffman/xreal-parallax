use glam::{Quat, Vec2, Vec3};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AppConfig {
    /// Interpupillary distance in millimeters (typically 55-75mm).
    pub ipd_mm: f32,
    /// Number of virtual monitors to create (1-4).
    pub virtual_monitor_count: u32,
    /// Target display name or index for the XReal One output.
    /// `None` means auto-detect (look for 3840x1080).
    pub target_display: Option<String>,
    /// IMU configuration.
    pub imu: ImuConfig,
    /// Panel layout.
    pub layout: LayoutConfig,
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            ipd_mm: 63.0,
            virtual_monitor_count: 4,
            target_display: None,
            imu: ImuConfig::default(),
            layout: LayoutConfig::default(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImuConfig {
    /// Madgwick filter beta parameter (convergence speed). Higher = more responsive, less smooth.
    pub madgwick_beta: f32,
    /// Number of stationary samples for gyro bias calibration.
    pub calibration_samples: u32,
}

impl Default for ImuConfig {
    fn default() -> Self {
        Self {
            madgwick_beta: 0.1,
            calibration_samples: 500,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LayoutConfig {
    /// Active layout preset. `Custom` means panels are user-positioned.
    pub preset: LayoutPreset,
    /// Per-panel configuration.
    pub panels: Vec<PanelConfig>,
}

impl Default for LayoutConfig {
    fn default() -> Self {
        Self {
            preset: LayoutPreset::CurvedArc,
            panels: PanelConfig::curved_arc(4),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LayoutPreset {
    /// 4 equal monitors in a 120-degree arc.
    CurvedArc,
    /// Monitors side-by-side in a flat line.
    FlatRow,
    /// 2x2 grid, top row angled down.
    Stacked2x2,
    /// 1 large curved ultrawide + 1 small side panel.
    UltrawideSidecar,
    /// 2 curved ultrawides stacked + small side panel.
    DualUltrawideStack,
    /// User-defined positions.
    Custom,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PanelConfig {
    /// Panel identifier (matches virtual display ID).
    pub id: u32,
    /// Virtual monitor resolution.
    pub resolution: (u32, u32),
    /// World-space center position (meters).
    #[serde(with = "vec3_serde")]
    pub position: Vec3,
    /// Panel orientation quaternion.
    #[serde(with = "quat_serde")]
    pub rotation: Quat,
    /// Physical width and height in meters.
    #[serde(with = "vec2_serde")]
    pub scale: Vec2,
    /// Curvature: 0.0 = flat, higher = more curved. Typical range 0.0-0.5.
    pub curvature: f32,
    /// Number of mesh segments for curved panels (higher = smoother).
    pub curve_segments: u32,
}

impl PanelConfig {
    /// Generate a curved arc layout with `count` panels.
    pub fn curved_arc(count: u32) -> Vec<PanelConfig> {
        let radius = 2.5_f32;
        let total_arc = 120.0_f32.to_radians();
        let panel_width = 1.2_f32;
        let panel_height = panel_width / (16.0 / 9.0);

        (0..count)
            .map(|i| {
                let t = if count > 1 {
                    i as f32 / (count - 1) as f32
                } else {
                    0.5
                };
                let angle = -total_arc / 2.0 + t * total_arc;

                let x = radius * angle.sin();
                let z = -radius * angle.cos();
                let position = Vec3::new(x, 0.0, z);

                // Face toward the user (origin)
                let look_dir = -position.normalize();
                let rotation = Quat::from_rotation_arc(Vec3::NEG_Z, look_dir);

                PanelConfig {
                    id: i,
                    resolution: (1920, 1080),
                    position,
                    rotation,
                    scale: Vec2::new(panel_width, panel_height),
                    curvature: 0.15,
                    curve_segments: 32,
                }
            })
            .collect()
    }

    /// Flat row layout.
    pub fn flat_row(count: u32) -> Vec<PanelConfig> {
        let panel_width = 1.2_f32;
        let panel_height = panel_width / (16.0 / 9.0);
        let gap = 0.05;
        let total_width = count as f32 * panel_width + (count - 1) as f32 * gap;

        (0..count)
            .map(|i| {
                let x = -total_width / 2.0 + panel_width / 2.0 + i as f32 * (panel_width + gap);
                PanelConfig {
                    id: i,
                    resolution: (1920, 1080),
                    position: Vec3::new(x, 0.0, -2.5),
                    rotation: Quat::IDENTITY,
                    scale: Vec2::new(panel_width, panel_height),
                    curvature: 0.0,
                    curve_segments: 1,
                }
            })
            .collect()
    }

    /// 2x2 stacked grid.
    pub fn stacked_2x2() -> Vec<PanelConfig> {
        let w = 1.1_f32;
        let h = w / (16.0 / 9.0);
        let gap = 0.05;

        let positions = [
            Vec3::new(-(w + gap) / 2.0, h / 2.0 + gap / 2.0, -2.5),
            Vec3::new((w + gap) / 2.0, h / 2.0 + gap / 2.0, -2.5),
            Vec3::new(-(w + gap) / 2.0, -(h / 2.0 + gap / 2.0), -2.5),
            Vec3::new((w + gap) / 2.0, -(h / 2.0 + gap / 2.0), -2.5),
        ];

        positions
            .iter()
            .enumerate()
            .map(|(i, &pos)| {
                // Top row tilts down slightly
                let pitch = if pos.y > 0.0 { 0.1_f32 } else { -0.05 };
                PanelConfig {
                    id: i as u32,
                    resolution: (1920, 1080),
                    position: pos,
                    rotation: Quat::from_rotation_x(pitch),
                    scale: Vec2::new(w, h),
                    curvature: 0.0,
                    curve_segments: 1,
                }
            })
            .collect()
    }

    /// Ultrawide center + small sidecar.
    pub fn ultrawide_sidecar() -> Vec<PanelConfig> {
        let uw_width = 2.0;
        let uw_height = uw_width / (21.0 / 9.0);
        let side_width = 0.8;
        let side_height = side_width / (16.0 / 9.0);

        vec![
            PanelConfig {
                id: 0,
                resolution: (3440, 1440),
                position: Vec3::new(0.0, 0.0, -2.5),
                rotation: Quat::IDENTITY,
                scale: Vec2::new(uw_width, uw_height),
                curvature: 0.2,
                curve_segments: 48,
            },
            PanelConfig {
                id: 1,
                resolution: (1280, 720),
                position: Vec3::new(1.4, 0.0, -2.2),
                rotation: Quat::from_rotation_y(-0.3),
                scale: Vec2::new(side_width, side_height),
                curvature: 0.0,
                curve_segments: 1,
            },
        ]
    }
}

// Serde helpers for glam types (which implement Serialize but we want
// a cleaner TOML representation as arrays).

mod vec3_serde {
    use glam::Vec3;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    pub fn serialize<S: Serializer>(v: &Vec3, s: S) -> Result<S::Ok, S::Error> {
        [v.x, v.y, v.z].serialize(s)
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<Vec3, D::Error> {
        let [x, y, z] = <[f32; 3]>::deserialize(d)?;
        Ok(Vec3::new(x, y, z))
    }
}

mod vec2_serde {
    use glam::Vec2;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    pub fn serialize<S: Serializer>(v: &Vec2, s: S) -> Result<S::Ok, S::Error> {
        [v.x, v.y].serialize(s)
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<Vec2, D::Error> {
        let [x, y] = <[f32; 2]>::deserialize(d)?;
        Ok(Vec2::new(x, y))
    }
}

mod quat_serde {
    use glam::Quat;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    pub fn serialize<S: Serializer>(q: &Quat, s: S) -> Result<S::Ok, S::Error> {
        [q.x, q.y, q.z, q.w].serialize(s)
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<Quat, D::Error> {
        let [x, y, z, w] = <[f32; 4]>::deserialize(d)?;
        Ok(Quat::from_xyzw(x, y, z, w))
    }
}
