// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Slint-Royalty-free-2.0 OR LicenseRef-Slint-Software-3.0

//! This crate provides a VNC backend for Slint applications.
//!
//! The VNC backend allows Slint applications to be rendered and accessed remotely
//! via the VNC protocol, similar to Qt's VNC QPA plugin.
//!
//! # Usage
//!
//! ```rust,ignore
//! use i_slint_backend_vnc::VncBackendBuilder;
//!
//! let backend = VncBackendBuilder::new()
//!     .with_size(1024, 768)
//!     .with_port(5900)
//!     .build()
//!     .unwrap();
//!
//! slint::platform::set_platform(backend).unwrap();
//! ```
//!
//! # Environment Variables
//!
//! - `SLINT_VNC_DISPLAY=:0` - VNC display number (port = 5900 + display number)
//! - `SLINT_VNC_SIZE=1024x768` - Initial window size

#![doc(html_logo_url = "https://slint.dev/logo/slint-logo-square-light.svg")]

mod input_mapping;
mod vnc_backend;
mod vnc_server;
mod vnc_window_adapter;

pub use vnc_backend::{VncBackend, VncBackendBuilder};
