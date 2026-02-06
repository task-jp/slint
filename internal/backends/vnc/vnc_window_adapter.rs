// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Slint-Royalty-free-2.0 OR LicenseRef-Slint-Software-3.0

//! VNC window adapter implementation

use std::cell::Cell;
use std::rc::{Rc, Weak};

use i_slint_core::api::{PhysicalSize, Window, WindowSize};
use i_slint_core::platform::Renderer;
use i_slint_core::window::WindowAdapter;
use i_slint_renderer_software::{RepaintBufferType, SoftwareRenderer};

/// VNC window adapter that renders using SoftwareRenderer
pub struct VncWindowAdapter {
    window: Window,
    renderer: SoftwareRenderer,
    size: Cell<PhysicalSize>,
    needs_redraw: Cell<bool>,
}

impl VncWindowAdapter {
    /// Create a new VNC window adapter with the given size
    pub fn new(width: u32, height: u32) -> Rc<Self> {
        Rc::new_cyclic(|weak: &Weak<Self>| Self {
            window: Window::new(weak.clone()),
            renderer: SoftwareRenderer::new_with_repaint_buffer_type(RepaintBufferType::NewBuffer),
            size: Cell::new(PhysicalSize::new(width, height)),
            needs_redraw: Cell::new(true),
        })
    }

    /// Check if a redraw is needed and reset the flag
    pub fn take_redraw_needed(&self) -> bool {
        self.needs_redraw.replace(false)
    }

    /// Get a reference to the software renderer
    pub fn renderer(&self) -> &SoftwareRenderer {
        &self.renderer
    }

    /// Get the current size
    #[allow(dead_code)]
    pub fn physical_size(&self) -> PhysicalSize {
        self.size.get()
    }
}

impl WindowAdapter for VncWindowAdapter {
    fn window(&self) -> &Window {
        &self.window
    }

    fn size(&self) -> PhysicalSize {
        self.size.get()
    }

    fn set_size(&self, size: WindowSize) {
        let scale_factor = self.window.scale_factor();
        self.size.set(size.to_physical(scale_factor));
        let logical_size = size.to_logical(scale_factor);
        self.window
            .dispatch_event(i_slint_core::platform::WindowEvent::Resized { size: logical_size });
    }

    fn renderer(&self) -> &dyn Renderer {
        &self.renderer
    }

    fn request_redraw(&self) {
        self.needs_redraw.set(true);
    }
}

impl core::ops::Deref for VncWindowAdapter {
    type Target = Window;

    fn deref(&self) -> &Self::Target {
        &self.window
    }
}
