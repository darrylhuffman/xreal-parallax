use crate::PanelAction;
use glam::{Mat4, Quat, Vec2, Vec3, Vec4};
use winit::event::{ElementState, MouseButton, MouseScrollDelta};
use winit::keyboard::ModifiersState;

/// Manages ALT+mouse interaction for panel manipulation.
pub struct InteractionManager {
    /// Whether ALT is currently held.
    alt_held: bool,
    /// Shift/Ctrl modifiers.
    modifiers: ModifiersState,
    /// Currently hovered panel (ray-cast result).
    pub hovered_panel: Option<u32>,
    /// Panel being actively dragged.
    dragging: Option<DragState>,
    /// Current cursor position in normalized device coords.
    cursor_ndc: Vec2,
    /// Window size for NDC conversion.
    window_size: Vec2,
}

struct DragState {
    panel_id: u32,
    button: MouseButton,
    start_ndc: Vec2,
    initial_position: Vec3,
    initial_scale: Vec2,
    initial_curvature: f32,
    initial_rotation: Quat,
    /// Depth of the panel when drag started (for view-plane projection).
    panel_depth: f32,
}

impl InteractionManager {
    pub fn new(window_width: f32, window_height: f32) -> Self {
        Self {
            alt_held: false,
            modifiers: ModifiersState::empty(),
            hovered_panel: None,
            dragging: None,
            cursor_ndc: Vec2::ZERO,
            window_size: Vec2::new(window_width, window_height),
        }
    }

    pub fn is_manipulating(&self) -> bool {
        self.alt_held
    }

    pub fn set_window_size(&mut self, width: f32, height: f32) {
        self.window_size = Vec2::new(width, height);
    }

    pub fn on_modifiers_changed(&mut self, modifiers: ModifiersState) {
        self.modifiers = modifiers;
        self.alt_held = modifiers.alt_key();

        // If ALT released while dragging, cancel the drag.
        if !self.alt_held {
            self.dragging = None;
        }
    }

    pub fn on_cursor_moved(&mut self, x: f64, y: f64) -> Option<(u32, PanelAction)> {
        // Convert pixel coords to NDC (-1..1).
        self.cursor_ndc = Vec2::new(
            (x as f32 / self.window_size.x) * 2.0 - 1.0,
            1.0 - (y as f32 / self.window_size.y) * 2.0, // Flip Y.
        );

        // Process active drag.
        if let Some(drag) = &self.dragging {
            let delta_ndc = self.cursor_ndc - drag.start_ndc;
            let panel_id = drag.panel_id;

            return match drag.button {
                MouseButton::Left => {
                    // Move in view plane.
                    let scale_factor = drag.panel_depth * 2.0;
                    let world_delta = Vec3::new(
                        delta_ndc.x * scale_factor,
                        delta_ndc.y * scale_factor,
                        0.0,
                    );
                    Some((panel_id, PanelAction::Move(drag.initial_position + world_delta)))
                }
                MouseButton::Right => {
                    // Resize: horizontal = width, vertical = height.
                    let new_scale = Vec2::new(
                        (drag.initial_scale.x + delta_ndc.x * 2.0).max(0.3),
                        (drag.initial_scale.y - delta_ndc.y * 2.0).max(0.2),
                    );
                    Some((panel_id, PanelAction::Resize(new_scale)))
                }
                MouseButton::Middle => {
                    // Horizontal: curvature, Vertical: pitch rotation.
                    if delta_ndc.x.abs() > delta_ndc.y.abs() {
                        let new_curve = (drag.initial_curvature + delta_ndc.x * 0.5).max(0.0);
                        Some((panel_id, PanelAction::Curvature(new_curve)))
                    } else {
                        let pitch = delta_ndc.y * std::f32::consts::PI * 0.25;
                        let new_rotation =
                            drag.initial_rotation * Quat::from_rotation_x(pitch);
                        Some((panel_id, PanelAction::Rotate(new_rotation)))
                    }
                }
                _ => None,
            };
        }

        None
    }

    pub fn on_mouse_button(
        &mut self,
        button: MouseButton,
        state: ElementState,
        // Current state of the hovered panel.
        panel_state: Option<(u32, Vec3, Vec2, f32, Quat, f32)>, // id, pos, scale, curvature, rotation, depth
    ) {
        if !self.alt_held {
            return;
        }

        match state {
            ElementState::Pressed => {
                if let Some((id, pos, scale, curvature, rotation, depth)) = panel_state {
                    self.dragging = Some(DragState {
                        panel_id: id,
                        button,
                        start_ndc: self.cursor_ndc,
                        initial_position: pos,
                        initial_scale: scale,
                        initial_curvature: curvature,
                        initial_rotation: rotation,
                        panel_depth: depth,
                    });
                }
            }
            ElementState::Released => {
                if self
                    .dragging
                    .as_ref()
                    .map_or(false, |d| d.button == button)
                {
                    self.dragging = None;
                }
            }
        }
    }

    pub fn on_scroll(&mut self, delta: MouseScrollDelta) -> Option<(u32, PanelAction)> {
        if !self.alt_held {
            return None;
        }

        let scroll_y = match delta {
            MouseScrollDelta::LineDelta(_, y) => y,
            MouseScrollDelta::PixelDelta(pos) => pos.y as f32 / 100.0,
        };

        let panel_id = self.hovered_panel?;

        if self.modifiers.shift_key() {
            // Shift + scroll: yaw rotation.
            let yaw = scroll_y * 0.05;
            Some((panel_id, PanelAction::Rotate(Quat::from_rotation_y(yaw))))
        } else if self.modifiers.control_key() {
            // Ctrl + scroll: roll rotation.
            let roll = scroll_y * 0.05;
            Some((panel_id, PanelAction::Rotate(Quat::from_rotation_z(roll))))
        } else {
            // Plain scroll: depth adjustment.
            Some((panel_id, PanelAction::Depth(scroll_y * 0.1)))
        }
    }
}

/// Cast a ray from NDC coordinates through the camera and test against panels.
///
/// Returns (panel_id, world_hit_point, depth) of the closest hit.
pub fn raycast_panels(
    cursor_ndc: Vec2,
    inv_view_proj: Mat4,
    panels: &[(u32, Mat4, Vec2)], // (id, model_matrix, scale)
) -> Option<(u32, Vec3, f32)> {
    // Unproject cursor to world ray.
    let near_ndc = Vec4::new(cursor_ndc.x, cursor_ndc.y, 0.0, 1.0);
    let far_ndc = Vec4::new(cursor_ndc.x, cursor_ndc.y, 1.0, 1.0);

    let near_world = inv_view_proj * near_ndc;
    let far_world = inv_view_proj * far_ndc;

    let near_world = near_world.truncate() / near_world.w;
    let far_world = far_world.truncate() / far_world.w;

    let ray_origin = near_world;
    let ray_dir = (far_world - near_world).normalize();

    let mut closest: Option<(u32, Vec3, f32)> = None;

    for &(id, model, scale) in panels {
        // Panel center and normal in world space.
        let center = model.col(3).truncate();
        let normal = (model * Vec4::new(0.0, 0.0, 1.0, 0.0)).truncate().normalize();

        // Ray-plane intersection.
        let denom = normal.dot(ray_dir);
        if denom.abs() < 1e-6 {
            continue;
        }

        let t = (center - ray_origin).dot(normal) / denom;
        if t < 0.0 {
            continue;
        }

        let hit = ray_origin + ray_dir * t;

        // Check if hit is within panel bounds (approximate for flat panels).
        let local = model.inverse() * Vec4::new(hit.x, hit.y, hit.z, 1.0);
        let hw = scale.x / 2.0;
        let hh = scale.y / 2.0;

        if local.x.abs() <= hw && local.y.abs() <= hh {
            if closest.is_none() || t < closest.unwrap().2 {
                closest = Some((id, hit, t));
            }
        }
    }

    closest
}
