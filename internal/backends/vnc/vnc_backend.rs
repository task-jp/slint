// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Slint-Royalty-free-2.0 OR LicenseRef-Slint-Software-3.0

//! VNC backend implementation

use std::cell::RefCell;
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use i_slint_core::api::{EventLoopError, LogicalPosition};
use i_slint_core::platform::{Platform, PlatformError, WindowEvent};
use i_slint_core::window::WindowAdapter;
use i_slint_renderer_software::PremultipliedRgbaColor;
use tokio::sync::mpsc;

use crate::input_mapping::{MouseButtonState, map_key_sym};
use crate::vnc_server::{UpdateRegion, VncInputEvent, VncServer};
use crate::vnc_window_adapter::VncWindowAdapter;

/// Configuration for the VNC backend
#[derive(Clone, Debug)]
pub struct VncConfig {
    /// Screen width in pixels
    pub width: u32,
    /// Screen height in pixels
    pub height: u32,
    /// VNC server port (default: 5900)
    pub port: u16,
    /// Server name shown to VNC clients
    pub name: String,
}

impl Default for VncConfig {
    fn default() -> Self {
        // Check environment variables
        let (width, height) = Self::parse_size_env().unwrap_or((800, 600));
        let port = Self::parse_port_env().unwrap_or(5900);

        Self { width, height, port, name: "Slint VNC".to_string() }
    }
}

impl VncConfig {
    fn parse_size_env() -> Option<(u32, u32)> {
        let size = std::env::var("SLINT_VNC_SIZE").ok()?;
        let (w, h) = size.split_once('x')?;
        Some((w.parse().ok()?, h.parse().ok()?))
    }

    fn parse_port_env() -> Option<u16> {
        // SLINT_VNC_DISPLAY=:0 means port 5900
        if let Ok(display) = std::env::var("SLINT_VNC_DISPLAY") {
            let display_num: u16 = display.trim_start_matches(':').parse().ok()?;
            return Some(5900 + display_num);
        }

        // Direct port specification
        std::env::var("SLINT_VNC_PORT").ok()?.parse().ok()
    }
}

/// Builder for creating a VNC backend
#[derive(Default)]
pub struct VncBackendBuilder {
    config: VncConfig,
}

impl VncBackendBuilder {
    /// Create a new VNC backend builder with default settings
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the screen size
    pub fn with_size(mut self, width: u32, height: u32) -> Self {
        self.config.width = width;
        self.config.height = height;
        self
    }

    /// Set the VNC port
    pub fn with_port(mut self, port: u16) -> Self {
        self.config.port = port;
        self
    }

    /// Set the server name
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.config.name = name.into();
        self
    }

    /// Build the VNC backend
    pub fn build(self) -> Result<Box<dyn Platform>, PlatformError> {
        Ok(Box::new(VncBackend::new(self.config)?))
    }
}

/// VNC backend implementing the Slint Platform trait
pub struct VncBackend {
    config: VncConfig,
    window: RefCell<Option<Rc<VncWindowAdapter>>>,
    proxy: VncEventLoopProxy,
    start_time: Instant,
}

impl VncBackend {
    fn new(config: VncConfig) -> Result<Self, PlatformError> {
        let (event_sender, event_receiver) = mpsc::unbounded_channel();

        Ok(Self {
            config,
            window: RefCell::new(None),
            proxy: VncEventLoopProxy {
                sender: event_sender,
                receiver: Arc::new(Mutex::new(Some(event_receiver))),
                quit_flag: Arc::new(AtomicBool::new(false)),
            },
            start_time: Instant::now(),
        })
    }
}

impl Platform for VncBackend {
    fn create_window_adapter(&self) -> Result<Rc<dyn WindowAdapter>, PlatformError> {
        let adapter = VncWindowAdapter::new(self.config.width, self.config.height);

        // Set initial size event
        let logical_size = i_slint_core::api::LogicalSize::new(
            self.config.width as f32,
            self.config.height as f32,
        );
        adapter.window().dispatch_event(WindowEvent::Resized { size: logical_size });

        *self.window.borrow_mut() = Some(adapter.clone());
        Ok(adapter)
    }

    fn run_event_loop(&self) -> Result<(), PlatformError> {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_io()
            .enable_time()
            .build()
            .map_err(|e| PlatformError::Other(format!("Failed to create Tokio runtime: {}", e)))?;

        rt.block_on(self.run_event_loop_async())
    }

    fn new_event_loop_proxy(&self) -> Option<Box<dyn i_slint_core::platform::EventLoopProxy>> {
        Some(Box::new(self.proxy.clone()))
    }

    fn duration_since_start(&self) -> core::time::Duration {
        self.start_time.elapsed()
    }
}

impl VncBackend {
    async fn run_event_loop_async(&self) -> Result<(), PlatformError> {
        let mut vnc_server = VncServer::new(
            self.config.port,
            self.config.width as u16,
            self.config.height as u16,
            &self.config.name,
        )
        .await
        .map_err(|e| PlatformError::Other(format!("Failed to create VNC server: {}", e)))?;

        let quit_flag = self.proxy.quit_flag.clone();
        let mut event_receiver = self.proxy.receiver.lock().unwrap().take().unwrap();

        let mut mouse_state = MouseButtonState::default();
        let mut last_mouse_pos = LogicalPosition::new(0.0, 0.0);

        // Framebuffer for rendering
        let (width, height) = vnc_server.dimensions();
        let pixel_stride = width as usize;
        let mut framebuffer =
            vec![PremultipliedRgbaColor::default(); pixel_stride * height as usize];

        log::info!("VNC event loop started, screen size: {}x{}", width, height);

        while !quit_flag.load(Ordering::Acquire) {
            // 1. Update Slint timers and animations
            i_slint_core::platform::update_timers_and_animations();

            // 2. Process invoke_from_event_loop callbacks
            while let Ok(callback) = event_receiver.try_recv() {
                match callback {
                    EventLoopEvent::Quit => {
                        quit_flag.store(true, Ordering::Release);
                        break;
                    }
                    EventLoopEvent::Callback(f) => f(),
                }
            }

            if quit_flag.load(Ordering::Acquire) {
                break;
            }

            // 3. Try to accept new VNC client connections
            if let Err(e) = vnc_server.accept_client().await {
                log::warn!("Error accepting client: {}", e);
            }

            // 4. Process VNC client messages
            if let Err(e) = vnc_server.process_client_messages().await {
                log::warn!("Error processing client messages: {}", e);
            }

            // 5. Handle VNC input events
            if let Some(window) = self.window.borrow().as_ref() {
                while let Ok(event) = vnc_server.event_receiver().try_recv() {
                    match event {
                        VncInputEvent::Key { keysym, down } => {
                            if let Some(text) = map_key_sym(keysym) {
                                let event = if down {
                                    WindowEvent::KeyPressed { text }
                                } else {
                                    WindowEvent::KeyReleased { text }
                                };
                                window.window().dispatch_event(event);
                            }
                        }
                        VncInputEvent::Pointer { x, y, button_mask } => {
                            let position = LogicalPosition::new(x as f32, y as f32);
                            let new_state = MouseButtonState::from_button_mask(button_mask);

                            // Check for button state changes
                            if let Some((button, pressed)) = new_state.changed_button(&mouse_state)
                            {
                                let event = if pressed {
                                    WindowEvent::PointerPressed { position, button }
                                } else {
                                    WindowEvent::PointerReleased { position, button }
                                };
                                window.window().dispatch_event(event);
                            }

                            // Check for mouse move
                            if position.x != last_mouse_pos.x || position.y != last_mouse_pos.y {
                                window
                                    .window()
                                    .dispatch_event(WindowEvent::PointerMoved { position });
                                last_mouse_pos = position;
                            }

                            // Handle scroll wheel
                            let wheel_delta = new_state.wheel_delta();
                            if wheel_delta != 0 {
                                window.window().dispatch_event(WindowEvent::PointerScrolled {
                                    position,
                                    delta_x: 0.0,
                                    delta_y: wheel_delta as f32 * 15.0,
                                });
                            }

                            mouse_state = new_state;
                        }
                        VncInputEvent::Disconnected => {
                            // Optionally reset mouse state when client disconnects
                            mouse_state = MouseButtonState::default();
                        }
                    }
                }

                // 6. Render and send frame update if needed
                let needs_redraw = window.take_redraw_needed();
                if vnc_server.update_requested() {
                    let is_incremental = vnc_server.is_incremental();

                    // For incremental requests, only render if something changed
                    // For non-incremental (full) requests, always send full screen
                    let regions = if !is_incremental {
                        // Non-incremental: client needs full screen
                        window.renderer().render(&mut framebuffer, pixel_stride);
                        vec![UpdateRegion { x: 0, y: 0, width, height }]
                    } else if needs_redraw {
                        // Incremental with changes: render and send dirty region
                        let dirty_region = window.renderer().render(&mut framebuffer, pixel_stride);

                        if dirty_region.bounding_box_size().width > 0
                            && dirty_region.bounding_box_size().height > 0
                        {
                            vec![UpdateRegion {
                                x: dirty_region.bounding_box_origin().x as u16,
                                y: dirty_region.bounding_box_origin().y as u16,
                                width: dirty_region.bounding_box_size().width as u16,
                                height: dirty_region.bounding_box_size().height as u16,
                            }]
                        } else {
                            // Render returned no dirty region, send empty update
                            vec![]
                        }
                    } else {
                        // Incremental with no changes: send empty update (0 rectangles)
                        vec![]
                    };

                    // Convert pixel buffer to bytes for VNC transmission
                    // PremultipliedRgbaColor is RGBA order, 4 bytes per pixel
                    let byte_stride = pixel_stride * 4;
                    let framebuffer_bytes: &[u8] = bytemuck::cast_slice(&framebuffer);

                    if let Err(e) =
                        vnc_server.send_frame_update(framebuffer_bytes, byte_stride, &regions).await
                    {
                        log::warn!("Error sending frame update: {}", e);
                    }
                }
            }

            // 7. Calculate sleep duration until next timer/animation
            let timeout = i_slint_core::platform::duration_until_next_timer_update()
                .unwrap_or(std::time::Duration::from_millis(16));

            // Sleep for a short time to avoid busy-looping, but wake up for:
            // - Timer events
            // - VNC client connections
            // - Event loop callbacks
            tokio::time::sleep(timeout.min(std::time::Duration::from_millis(16))).await;
        }

        log::info!("VNC event loop ended");
        Ok(())
    }
}

/// Event for the event loop
enum EventLoopEvent {
    Quit,
    Callback(Box<dyn FnOnce() + Send>),
}

/// Event loop proxy for invoke_from_event_loop and quit_event_loop
#[derive(Clone)]
struct VncEventLoopProxy {
    sender: mpsc::UnboundedSender<EventLoopEvent>,
    receiver: Arc<Mutex<Option<mpsc::UnboundedReceiver<EventLoopEvent>>>>,
    quit_flag: Arc<AtomicBool>,
}

impl i_slint_core::platform::EventLoopProxy for VncEventLoopProxy {
    fn quit_event_loop(&self) -> Result<(), EventLoopError> {
        self.quit_flag.store(true, Ordering::Release);
        self.sender.send(EventLoopEvent::Quit).map_err(|_| EventLoopError::EventLoopTerminated)
    }

    fn invoke_from_event_loop(
        &self,
        event: Box<dyn FnOnce() + Send>,
    ) -> Result<(), EventLoopError> {
        self.sender
            .send(EventLoopEvent::Callback(event))
            .map_err(|_| EventLoopError::EventLoopTerminated)
    }
}
