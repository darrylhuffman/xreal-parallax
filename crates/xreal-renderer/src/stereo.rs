use crate::camera::Camera;
use crate::panel::generate_panel_mesh;
use crate::pipeline::{RenderPipeline, Uniforms};
use crate::scene::Scene;
use bytemuck;
use wgpu::util::DeviceExt;

/// Per-eye render target resolution.
const EYE_WIDTH: u32 = 1920;
const EYE_HEIGHT: u32 = 1080;

/// Manages SBS stereoscopic rendering.
///
/// Renders the scene from two eye positions, composites into a
/// 3840x1080 side-by-side framebuffer.
pub struct StereoRenderer {
    /// Interpupillary distance in meters.
    pub ipd: f32,
    /// Left eye offscreen render target.
    left_eye: EyeTarget,
    /// Right eye offscreen render target.
    right_eye: EyeTarget,
    /// Render pipeline for panels.
    pub render_pipeline: RenderPipeline,
}

struct EyeTarget {
    texture: wgpu::Texture,
    view: wgpu::TextureView,
    depth_view: wgpu::TextureView,
}

impl EyeTarget {
    fn new(device: &wgpu::Device, label: &str, color_format: wgpu::TextureFormat) -> Self {
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some(label),
            size: wgpu::Extent3d {
                width: EYE_WIDTH,
                height: EYE_HEIGHT,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: color_format,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let view = texture.create_view(&Default::default());

        let depth = device.create_texture(&wgpu::TextureDescriptor {
            label: Some(&format!("{}_depth", label)),
            size: wgpu::Extent3d {
                width: EYE_WIDTH,
                height: EYE_HEIGHT,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let depth_view = depth.create_view(&Default::default());

        Self {
            texture,
            view,
            depth_view,
        }
    }
}

/// GPU resources for a single panel (mesh + texture).
pub struct PanelGpuResources {
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
    pub index_count: u32,
    pub texture: wgpu::Texture,
    pub texture_view: wgpu::TextureView,
    pub texture_bind_group: wgpu::BindGroup,
    pub resolution: (u32, u32),
}

impl StereoRenderer {
    pub fn new(device: &wgpu::Device, color_format: wgpu::TextureFormat, ipd_mm: f32) -> Self {
        let left_eye = EyeTarget::new(device, "left_eye", color_format);
        let right_eye = EyeTarget::new(device, "right_eye", color_format);
        let render_pipeline =
            RenderPipeline::new(device, color_format, EYE_WIDTH, EYE_HEIGHT);

        Self {
            ipd: ipd_mm / 1000.0, // Convert mm to meters.
            left_eye,
            right_eye,
            render_pipeline,
        }
    }

    /// Create GPU resources for a panel (mesh + texture placeholder).
    pub fn create_panel_resources(
        &self,
        device: &wgpu::Device,
        scale: glam::Vec2,
        curvature: f32,
        curve_segments: u32,
        resolution: (u32, u32),
    ) -> PanelGpuResources {
        let mesh = generate_panel_mesh(scale, curvature, curve_segments);

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("panel_vertex_buffer"),
            contents: bytemuck::cast_slice(&mesh.vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("panel_index_buffer"),
            contents: bytemuck::cast_slice(&mesh.indices),
            usage: wgpu::BufferUsages::INDEX,
        });

        // Create a placeholder texture (checkerboard pattern for dev).
        let (tex, view) =
            create_placeholder_texture(device, resolution.0, resolution.1);

        let bind_group = self.render_pipeline.create_texture_bind_group(device, &view);

        PanelGpuResources {
            vertex_buffer,
            index_buffer,
            index_count: mesh.indices.len() as u32,
            texture: tex,
            texture_view: view,
            texture_bind_group: bind_group,
            resolution,
        }
    }

    /// Regenerate panel mesh (when curvature or scale changes).
    pub fn rebuild_panel_mesh(
        &self,
        device: &wgpu::Device,
        resources: &mut PanelGpuResources,
        scale: glam::Vec2,
        curvature: f32,
        curve_segments: u32,
    ) {
        let mesh = generate_panel_mesh(scale, curvature, curve_segments);

        resources.vertex_buffer =
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("panel_vertex_buffer"),
                contents: bytemuck::cast_slice(&mesh.vertices),
                usage: wgpu::BufferUsages::VERTEX,
            });

        resources.index_buffer =
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("panel_index_buffer"),
                contents: bytemuck::cast_slice(&mesh.indices),
                usage: wgpu::BufferUsages::INDEX,
            });

        resources.index_count = mesh.indices.len() as u32;
    }

    /// Upload captured frame data to a panel's GPU texture.
    pub fn upload_frame(
        &self,
        queue: &wgpu::Queue,
        resources: &PanelGpuResources,
        data: &[u8],
        width: u32,
        height: u32,
    ) {
        queue.write_texture(
            wgpu::ImageCopyTexture {
                texture: &resources.texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            data,
            wgpu::ImageDataLayout {
                offset: 0,
                bytes_per_row: Some(4 * width),
                rows_per_image: Some(height),
            },
            wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
        );
    }

    /// Render the scene from both eyes and compose into the SBS output.
    ///
    /// Returns command buffers to submit. The caller should then copy
    /// the eye textures side-by-side to the surface texture.
    pub fn render_frame(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        scene: &Scene,
        camera: &Camera,
        panel_resources: &[PanelGpuResources],
    ) -> wgpu::CommandBuffer {
        let projection = camera.projection_matrix();
        let half_ipd = self.ipd / 2.0;

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("stereo_render"),
        });

        // Render left eye.
        let left_view = camera.stereo_view_matrix(-half_ipd);
        self.render_eye(
            &mut encoder,
            device,
            queue,
            &self.left_eye,
            scene,
            left_view,
            projection,
            panel_resources,
        );

        // Render right eye.
        let right_view = camera.stereo_view_matrix(half_ipd);
        self.render_eye(
            &mut encoder,
            device,
            queue,
            &self.right_eye,
            scene,
            right_view,
            projection,
            panel_resources,
        );

        encoder.finish()
    }

    fn render_eye(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        device: &wgpu::Device,
        _queue: &wgpu::Queue,
        eye: &EyeTarget,
        scene: &Scene,
        view: glam::Mat4,
        projection: glam::Mat4,
        panel_resources: &[PanelGpuResources],
    ) {
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("eye_render_pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &eye.view,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color {
                        r: 0.0,
                        g: 0.0,
                        b: 0.0,
                        a: 1.0,
                    }),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &eye.depth_view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
        });

        pass.set_pipeline(&self.render_pipeline.pipeline);

        for (panel, resources) in scene.panels.iter().zip(panel_resources.iter()) {
            let model = panel.model_matrix();
            let uniforms = Uniforms::new(model, view, projection);

            let uniform_buffer =
                device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("uniform_buffer"),
                    contents: bytemuck::cast_slice(&[uniforms]),
                    usage: wgpu::BufferUsages::UNIFORM,
                });

            let uniform_bind_group = self
                .render_pipeline
                .create_uniform_bind_group(device, &uniform_buffer);

            pass.set_bind_group(0, &uniform_bind_group, &[]);
            pass.set_bind_group(1, &resources.texture_bind_group, &[]);
            pass.set_vertex_buffer(0, resources.vertex_buffer.slice(..));
            pass.set_index_buffer(resources.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..resources.index_count, 0, 0..1);
        }
    }

    /// Copy both eye textures side-by-side onto the output surface.
    pub fn compose_sbs(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        output: &wgpu::Texture,
    ) {
        // Left eye → left half (0, 0).
        encoder.copy_texture_to_texture(
            wgpu::ImageCopyTexture {
                texture: &self.left_eye.texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::ImageCopyTexture {
                texture: output,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::Extent3d {
                width: EYE_WIDTH,
                height: EYE_HEIGHT,
                depth_or_array_layers: 1,
            },
        );

        // Right eye → right half (1920, 0).
        encoder.copy_texture_to_texture(
            wgpu::ImageCopyTexture {
                texture: &self.right_eye.texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::ImageCopyTexture {
                texture: output,
                mip_level: 0,
                origin: wgpu::Origin3d { x: EYE_WIDTH, y: 0, z: 0 },
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::Extent3d {
                width: EYE_WIDTH,
                height: EYE_HEIGHT,
                depth_or_array_layers: 1,
            },
        );
    }

    /// Get the left eye texture view (for mono/debug rendering).
    pub fn left_eye_view(&self) -> &wgpu::TextureView {
        &self.left_eye.view
    }
}

/// Create a checkerboard placeholder texture for development.
fn create_placeholder_texture(
    device: &wgpu::Device,
    width: u32,
    height: u32,
) -> (wgpu::Texture, wgpu::TextureView) {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("panel_texture"),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8UnormSrgb,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    let view = texture.create_view(&Default::default());
    (texture, view)
}
