use glam::{Mat4, Quat, Vec2, Vec3};
use xreal_config::PanelConfig;

/// A renderable panel in the 3D scene.
pub struct ScenePanel {
    pub id: u32,
    pub position: Vec3,
    pub rotation: Quat,
    pub scale: Vec2,
    pub curvature: f32,
    pub curve_segments: u32,
    /// Whether this panel's mesh needs regeneration (curvature/scale changed).
    pub mesh_dirty: bool,
}

impl ScenePanel {
    pub fn from_config(config: &PanelConfig) -> Self {
        Self {
            id: config.id,
            position: config.position,
            rotation: config.rotation,
            scale: config.scale,
            curvature: config.curvature,
            curve_segments: config.curve_segments,
            mesh_dirty: true,
        }
    }

    /// Compute the model matrix for this panel.
    pub fn model_matrix(&self) -> Mat4 {
        Mat4::from_scale_rotation_translation(
            Vec3::ONE, // Scale is baked into mesh geometry.
            self.rotation,
            self.position,
        )
    }

    /// Export back to a config struct.
    pub fn to_config(&self, resolution: (u32, u32)) -> PanelConfig {
        PanelConfig {
            id: self.id,
            resolution,
            position: self.position,
            rotation: self.rotation,
            scale: self.scale,
            curvature: self.curvature,
            curve_segments: self.curve_segments,
        }
    }
}

/// The 3D scene containing all panels.
pub struct Scene {
    pub panels: Vec<ScenePanel>,
}

impl Scene {
    pub fn from_configs(configs: &[PanelConfig]) -> Self {
        Self {
            panels: configs.iter().map(ScenePanel::from_config).collect(),
        }
    }

    /// Reset panels to a given layout.
    pub fn apply_layout(&mut self, configs: &[PanelConfig]) {
        self.panels = configs.iter().map(ScenePanel::from_config).collect();
    }

    /// Get a mutable reference to a panel by ID.
    pub fn panel_mut(&mut self, id: u32) -> Option<&mut ScenePanel> {
        self.panels.iter_mut().find(|p| p.id == id)
    }
}
