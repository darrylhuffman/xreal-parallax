use anyhow::Result;
use std::sync::Arc;
use tracing::{error, info, warn};
use wgpu::util::DeviceExt;
use winit::application::ApplicationHandler;
use winit::dpi::PhysicalSize;
use winit::event::WindowEvent;
use winit::event_loop::{ActiveEventLoop, EventLoop};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Window, WindowId};
use xreal_capture::ScreenCapture;
use xreal_config::AppConfig;
use xreal_imu::ImuClient;
use xreal_input::mouse::InteractionManager;
use xreal_input::PanelAction;
use xreal_renderer::camera::Camera;
use xreal_renderer::scene::Scene;
use xreal_renderer::stereo::{PanelGpuResources, StereoRenderer};

/// Application state.
struct App {
    config: AppConfig,
    imu_client: ImuClient,
    window: Option<Arc<Window>>,
    gpu: Option<GpuState>,
    interaction: InteractionManager,
    /// Whether to render in SBS stereo mode (true) or mono debug mode (false).
    stereo_mode: bool,
    /// Whether the egui settings overlay is visible.
    settings_visible: bool,
}

struct GpuState {
    device: wgpu::Device,
    queue: wgpu::Queue,
    surface: wgpu::Surface<'static>,
    surface_config: wgpu::SurfaceConfiguration,
    stereo: StereoRenderer,
    scene: Scene,
    panel_resources: Vec<PanelGpuResources>,
    camera: Camera,
    captures: Vec<xreal_capture::duplication::DxgiCapture>,
    /// Depth texture for mono rendering (matches window size, not eye size).
    mono_depth: wgpu::TextureView,
    frame_count: u64,
}

impl App {
    fn new(config: AppConfig, imu_client: ImuClient) -> Self {
        Self {
            config,
            imu_client,
            window: None,
            gpu: None,
            interaction: InteractionManager::new(1920.0, 1080.0),
            stereo_mode: false, // Start in mono for dev; true for glasses output.
            settings_visible: false,
        }
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }

        // Create window. In stereo mode this would be fullscreen 3840x1080 on the glasses.
        let size = if self.stereo_mode {
            PhysicalSize::new(3840, 1080)
        } else {
            PhysicalSize::new(1920, 1080)
        };

        let attrs = Window::default_attributes()
            .with_title("XReal Multi-Monitor")
            .with_inner_size(size);

        let window = Arc::new(event_loop.create_window(attrs).expect("Failed to create window"));
        self.window = Some(window.clone());

        // Initialize wgpu.
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::DX12 | wgpu::Backends::VULKAN,
            ..Default::default()
        });

        let surface = instance
            .create_surface(window.clone())
            .expect("Failed to create surface");

        let (device, queue, adapter) = pollster::block_on(async {
            let adapter = instance
                .request_adapter(&wgpu::RequestAdapterOptions {
                    power_preference: wgpu::PowerPreference::HighPerformance,
                    compatible_surface: Some(&surface),
                    force_fallback_adapter: false,
                })
                .await
                .expect("No suitable GPU adapter found");

            info!(name = adapter.get_info().name, "Using GPU");

            let (device, queue) = adapter
                .request_device(
                    &wgpu::DeviceDescriptor {
                        label: Some("xreal_device"),
                        required_features: wgpu::Features::empty(),
                        required_limits: wgpu::Limits::default(),
                        memory_hints: Default::default(),
                    },
                    None,
                )
                .await
                .expect("Failed to create device");

            (device, queue, adapter)
        });

        let win_size = window.inner_size();
        let surface_caps = surface.get_capabilities(&adapter);

        // Pick a format that supports COPY_DST (needed for SBS compose).
        let format = surface_caps
            .formats
            .iter()
            .find(|f| f.is_srgb())
            .copied()
            .unwrap_or(surface_caps.formats[0]);

        let surface_config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_DST,
            format,
            width: win_size.width,
            height: win_size.height,
            present_mode: wgpu::PresentMode::AutoVsync,
            alpha_mode: wgpu::CompositeAlphaMode::Auto,
            view_formats: vec![],
            desired_maximum_frame_latency: 2,
        };
        surface.configure(&device, &surface_config);

        // Create stereo renderer.
        let stereo = StereoRenderer::new(&device, format, self.config.ipd_mm);

        // Create scene from config.
        let scene = Scene::from_configs(&self.config.layout.panels);

        // Create GPU resources for each panel.
        let panel_resources: Vec<PanelGpuResources> = self
            .config
            .layout
            .panels
            .iter()
            .map(|p| {
                stereo.create_panel_resources(
                    &device,
                    p.scale,
                    p.curvature,
                    p.curve_segments,
                    p.resolution,
                )
            })
            .collect();

        // Create stub captures for each panel.
        let captures: Vec<_> = self
            .config
            .layout
            .panels
            .iter()
            .map(|p| {
                xreal_capture::duplication::DxgiCapture::new(
                    p.id,
                    p.resolution.0,
                    p.resolution.1,
                )
                .expect("Failed to create capture")
            })
            .collect();

        let camera = Camera::new();

        let mono_depth = create_depth_texture(&device, win_size.width, win_size.height);

        self.gpu = Some(GpuState {
            device,
            queue,
            surface,
            surface_config,
            stereo,
            scene,
            panel_resources,
            camera,
            captures,
            mono_depth,
            frame_count: 0,
        });

        self.interaction
            .set_window_size(win_size.width as f32, win_size.height as f32);

        info!(
            panels = self.config.layout.panels.len(),
            "Application initialized"
        );
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: WindowId,
        event: WindowEvent,
    ) {
        match event {
            WindowEvent::CloseRequested => {
                // Save config on exit.
                if let Err(e) = xreal_config::save_config(&self.config) {
                    error!(?e, "Failed to save config");
                }
                event_loop.exit();
            }

            WindowEvent::Resized(size) => {
                if size.width > 0 && size.height > 0 {
                    if let Some(gpu) = &mut self.gpu {
                        gpu.surface_config.width = size.width;
                        gpu.surface_config.height = size.height;
                        gpu.surface.configure(&gpu.device, &gpu.surface_config);
                        gpu.mono_depth =
                            create_depth_texture(&gpu.device, size.width, size.height);
                        self.interaction
                            .set_window_size(size.width as f32, size.height as f32);
                    }
                }
            }

            WindowEvent::KeyboardInput { event, .. } => {
                if event.state == winit::event::ElementState::Pressed {
                    match event.physical_key {
                        PhysicalKey::Code(KeyCode::F1) => {
                            self.settings_visible = !self.settings_visible;
                            info!(visible = self.settings_visible, "Settings overlay toggled");
                        }
                        PhysicalKey::Code(KeyCode::F5) => {
                            self.imu_client.set_zero();
                        }
                        PhysicalKey::Code(KeyCode::F9) => {
                            self.stereo_mode = !self.stereo_mode;
                            info!(stereo = self.stereo_mode, "Stereo mode toggled");
                        }
                        PhysicalKey::Code(KeyCode::Escape) => {
                            event_loop.exit();
                        }
                        _ => {}
                    }
                }
            }

            WindowEvent::ModifiersChanged(modifiers) => {
                self.interaction.on_modifiers_changed(modifiers.state());
            }

            WindowEvent::CursorMoved { position, .. } => {
                if let Some(action) = self.interaction.on_cursor_moved(position.x, position.y) {
                    apply_panel_action(&mut self.gpu, action);
                }
            }

            WindowEvent::MouseInput { button, state, .. } => {
                if let Some(gpu) = &self.gpu {
                    // Get hovered panel state for drag initiation.
                    let panel_state = self.interaction.hovered_panel.and_then(|id| {
                        let panel = gpu.scene.panels.iter().find(|p| p.id == id)?;
                        let depth = panel.position.length();
                        Some((
                            id,
                            panel.position,
                            panel.scale,
                            panel.curvature,
                            panel.rotation,
                            depth,
                        ))
                    });
                    self.interaction.on_mouse_button(button, state, panel_state);
                }
            }

            WindowEvent::MouseWheel { delta, .. } => {
                if let Some(action) = self.interaction.on_scroll(delta) {
                    apply_panel_action(&mut self.gpu, action);
                }
            }

            WindowEvent::RedrawRequested => {
                if let Some(gpu) = &mut self.gpu {
                    // Update camera from IMU.
                    let orientation = self.imu_client.orientation();
                    gpu.camera.orientation = orientation.quaternion;

                    // Capture frames (stub: generates test patterns).
                    for (capture, resources) in
                        gpu.captures.iter_mut().zip(gpu.panel_resources.iter())
                    {
                        if let Ok(Some(frame)) = capture.try_capture() {
                            gpu.stereo.upload_frame(
                                &gpu.queue,
                                resources,
                                &frame.data,
                                frame.width,
                                frame.height,
                            );
                        }
                    }

                    // Render.
                    let output = match gpu.surface.get_current_texture() {
                        Ok(output) => output,
                        Err(e) => {
                            warn!(?e, "Failed to get surface texture");
                            return;
                        }
                    };

                    if self.stereo_mode {
                        // SBS stereo rendering.
                        let cmd = gpu.stereo.render_frame(
                            &gpu.device,
                            &gpu.queue,
                            &gpu.scene,
                            &gpu.camera,
                            &gpu.panel_resources,
                        );
                        gpu.queue.submit(std::iter::once(cmd));

                        let mut encoder = gpu.device.create_command_encoder(
                            &wgpu::CommandEncoderDescriptor {
                                label: Some("sbs_compose"),
                            },
                        );
                        gpu.stereo.compose_sbs(&mut encoder, &output.texture);
                        gpu.queue.submit(std::iter::once(encoder.finish()));
                    } else {
                        // Mono debug rendering (left eye only, full window).
                        let view = output
                            .texture
                            .create_view(&wgpu::TextureViewDescriptor::default());

                        // Use actual window aspect ratio for mono mode.
                        gpu.camera.aspect_ratio = gpu.surface_config.width as f32
                            / gpu.surface_config.height as f32;
                        let projection = gpu.camera.projection_matrix();
                        let view_matrix = gpu.camera.view_matrix();

                        let mut encoder = gpu.device.create_command_encoder(
                            &wgpu::CommandEncoderDescriptor {
                                label: Some("mono_render"),
                            },
                        );

                        {
                            let mut pass =
                                encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                                    label: Some("mono_pass"),
                                    color_attachments: &[Some(
                                        wgpu::RenderPassColorAttachment {
                                            view: &view,
                                            resolve_target: None,
                                            ops: wgpu::Operations {
                                                load: wgpu::LoadOp::Clear(wgpu::Color {
                                                    r: 0.02,
                                                    g: 0.02,
                                                    b: 0.05,
                                                    a: 1.0,
                                                }),
                                                store: wgpu::StoreOp::Store,
                                            },
                                        },
                                    )],
                                    depth_stencil_attachment: Some(
                                        wgpu::RenderPassDepthStencilAttachment {
                                            view: &gpu.mono_depth,
                                            depth_ops: Some(wgpu::Operations {
                                                load: wgpu::LoadOp::Clear(1.0),
                                                store: wgpu::StoreOp::Store,
                                            }),
                                            stencil_ops: None,
                                        },
                                    ),
                                    timestamp_writes: None,
                                    occlusion_query_set: None,
                                });

                            pass.set_pipeline(
                                &gpu.stereo.render_pipeline.pipeline,
                            );

                            for (panel, resources) in gpu
                                .scene
                                .panels
                                .iter()
                                .zip(gpu.panel_resources.iter())
                            {
                                let model = panel.model_matrix();
                                let uniforms = xreal_renderer::pipeline::Uniforms::new(
                                    model,
                                    view_matrix,
                                    projection,
                                );

                                let uniform_buffer = gpu.device.create_buffer_init(
                                    &wgpu::util::BufferInitDescriptor {
                                        label: Some("uniform_buffer"),
                                        contents: bytemuck::cast_slice(&[uniforms]),
                                        usage: wgpu::BufferUsages::UNIFORM,
                                    },
                                );

                                let uniform_bind_group = gpu
                                    .stereo
                                    .render_pipeline
                                    .create_uniform_bind_group(
                                        &gpu.device,
                                        &uniform_buffer,
                                    );

                                pass.set_bind_group(0, &uniform_bind_group, &[]);
                                pass.set_bind_group(
                                    1,
                                    &resources.texture_bind_group,
                                    &[],
                                );
                                pass.set_vertex_buffer(
                                    0,
                                    resources.vertex_buffer.slice(..),
                                );
                                pass.set_index_buffer(
                                    resources.index_buffer.slice(..),
                                    wgpu::IndexFormat::Uint32,
                                );
                                pass.draw_indexed(
                                    0..resources.index_count,
                                    0,
                                    0..1,
                                );
                            }
                        }

                        gpu.queue.submit(std::iter::once(encoder.finish()));
                    }

                    output.present();

                    gpu.frame_count += 1;
                    if gpu.frame_count % 300 == 0 {
                        tracing::debug!(frames = gpu.frame_count, "Render heartbeat");
                    }
                }

                // Request next frame.
                if let Some(window) = &self.window {
                    window.request_redraw();
                }
            }

            _ => {}
        }
    }
}

fn apply_panel_action(gpu: &mut Option<GpuState>, (panel_id, action): (u32, PanelAction)) {
    let gpu = match gpu {
        Some(g) => g,
        None => return,
    };

    if let Some(panel) = gpu.scene.panel_mut(panel_id) {
        match action {
            PanelAction::Move(pos) => panel.position = pos,
            PanelAction::Depth(delta) => {
                let forward = panel.position.normalize();
                panel.position += forward * delta;
            }
            PanelAction::Resize(scale) => {
                panel.scale = scale;
                panel.mesh_dirty = true;
            }
            PanelAction::Curvature(c) => {
                panel.curvature = c;
                panel.mesh_dirty = true;
            }
            PanelAction::Rotate(q) => {
                panel.rotation = q;
            }
        }
    }
}

fn create_depth_texture(device: &wgpu::Device, width: u32, height: u32) -> wgpu::TextureView {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("mono_depth_texture"),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Depth32Float,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    texture.create_view(&wgpu::TextureViewDescriptor::default())
}

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize logging.
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "xreal_app=info,xreal_imu=info,xreal_renderer=info".into()),
        )
        .init();

    info!("XReal Multi-Monitor AR Workspace starting");

    // Load config.
    let config = xreal_config::load_config().unwrap_or_else(|e| {
        warn!(?e, "Failed to load config, using defaults");
        AppConfig::default()
    });

    info!(?config.layout.preset, panels = config.layout.panels.len(), "Config loaded");

    // Connect to IMU (fall back to mock if glasses not connected).
    let imu_client = match ImuClient::connect(
        config.imu.madgwick_beta,
        config.imu.calibration_samples,
    )
    .await
    {
        Ok(client) => {
            info!("IMU connected");
            client
        }
        Err(e) => {
            warn!(?e, "IMU not available, using mock (no head tracking)");
            ImuClient::mock()
        }
    };

    // Run the application.
    let event_loop = EventLoop::new()?;
    let mut app = App::new(config, imu_client);
    event_loop.run_app(&mut app)?;

    Ok(())
}
