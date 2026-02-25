pub mod mouse;

use glam::{Quat, Vec2, Vec3};

/// Describes a modification to a panel's transform.
pub enum PanelAction {
    /// Move panel to a new position.
    Move(Vec3),
    /// Change panel depth (distance from user).
    Depth(f32),
    /// Resize panel.
    Resize(Vec2),
    /// Adjust curvature.
    Curvature(f32),
    /// Rotate panel.
    Rotate(Quat),
}
