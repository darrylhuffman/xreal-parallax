use bytemuck::{Pod, Zeroable};
use glam::Vec2;

/// Vertex format for panel meshes.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct PanelVertex {
    pub position: [f32; 3],
    pub uv: [f32; 2],
    pub normal: [f32; 3],
}

impl PanelVertex {
    pub fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                // position
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x3,
                },
                // uv
                wgpu::VertexAttribute {
                    offset: 12,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x2,
                },
                // normal
                wgpu::VertexAttribute {
                    offset: 20,
                    shader_location: 2,
                    format: wgpu::VertexFormat::Float32x3,
                },
            ],
        }
    }
}

/// A generated panel mesh (vertices + indices).
pub struct PanelMesh {
    pub vertices: Vec<PanelVertex>,
    pub indices: Vec<u32>,
}

/// Generate a panel mesh — flat or curved.
///
/// - `scale`: width and height in meters.
/// - `curvature`: 0.0 = flat, higher = more curved (cylindrical arc).
/// - `segments`: number of horizontal subdivisions (1 = flat quad, 32+ = smooth curve).
pub fn generate_panel_mesh(scale: Vec2, curvature: f32, segments: u32) -> PanelMesh {
    let segments = segments.max(1);

    if curvature <= 0.001 {
        // Flat quad.
        generate_flat_mesh(scale)
    } else {
        // Curved cylindrical arc.
        generate_curved_mesh(scale, curvature, segments)
    }
}

fn generate_flat_mesh(scale: Vec2) -> PanelMesh {
    let hw = scale.x / 2.0;
    let hh = scale.y / 2.0;

    let vertices = vec![
        PanelVertex {
            position: [-hw, hh, 0.0],
            uv: [0.0, 0.0],
            normal: [0.0, 0.0, 1.0],
        },
        PanelVertex {
            position: [hw, hh, 0.0],
            uv: [1.0, 0.0],
            normal: [0.0, 0.0, 1.0],
        },
        PanelVertex {
            position: [-hw, -hh, 0.0],
            uv: [0.0, 1.0],
            normal: [0.0, 0.0, 1.0],
        },
        PanelVertex {
            position: [hw, -hh, 0.0],
            uv: [1.0, 1.0],
            normal: [0.0, 0.0, 1.0],
        },
    ];

    let indices = vec![0, 2, 1, 1, 2, 3];

    PanelMesh { vertices, indices }
}

fn generate_curved_mesh(scale: Vec2, curvature: f32, segments: u32) -> PanelMesh {
    let half_h = scale.y / 2.0;

    // Arc geometry:
    //   arc_angle = width * curvature
    //   radius = width / arc_angle
    let arc_angle = scale.x * curvature;
    let radius = scale.x / arc_angle;

    let cols = segments + 1;
    let mut vertices = Vec::with_capacity((cols * 2) as usize);

    for i in 0..cols {
        let t = i as f32 / segments as f32;
        let theta = -arc_angle / 2.0 + t * arc_angle;

        let x = radius * theta.sin();
        let z = radius * theta.cos() - radius; // Center the arc at z=0

        // Normal points outward from the cylinder center.
        let nx = theta.sin();
        let nz = theta.cos();

        // Top vertex.
        vertices.push(PanelVertex {
            position: [x, half_h, z],
            uv: [t, 0.0],
            normal: [nx, 0.0, nz],
        });

        // Bottom vertex.
        vertices.push(PanelVertex {
            position: [x, -half_h, z],
            uv: [t, 1.0],
            normal: [nx, 0.0, nz],
        });
    }

    // Triangle strip indices: for each column pair, two triangles.
    let mut indices = Vec::with_capacity((segments * 6) as usize);
    for i in 0..segments {
        let tl = i * 2;
        let bl = tl + 1;
        let tr = tl + 2;
        let br = tl + 3;

        indices.push(tl);
        indices.push(bl);
        indices.push(tr);

        indices.push(tr);
        indices.push(bl);
        indices.push(br);
    }

    PanelMesh { vertices, indices }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flat_mesh_has_correct_geometry() {
        let mesh = generate_panel_mesh(Vec2::new(2.0, 1.0), 0.0, 1);
        assert_eq!(mesh.vertices.len(), 4);
        assert_eq!(mesh.indices.len(), 6);
    }

    #[test]
    fn curved_mesh_has_correct_segments() {
        let mesh = generate_panel_mesh(Vec2::new(2.0, 1.0), 0.3, 16);
        assert_eq!(mesh.vertices.len(), 34); // (16 + 1) * 2
        assert_eq!(mesh.indices.len(), 96); // 16 * 6
    }

    #[test]
    fn zero_curvature_is_flat() {
        let mesh = generate_panel_mesh(Vec2::new(1.0, 1.0), 0.0, 32);
        // Even with 32 segments requested, curvature=0 produces a flat quad.
        assert_eq!(mesh.vertices.len(), 4);
    }
}
