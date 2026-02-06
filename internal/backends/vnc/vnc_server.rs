// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Slint-Royalty-free-2.0 OR LicenseRef-Slint-Software-3.0

//! VNC server implementation using RFB protocol
//!
//! This module implements a minimal VNC server supporting:
//! - RFB protocol version 3.8
//! - No authentication (security type 1)
//! - Raw and CopyRect encodings
//! - Keyboard and pointer events

use std::io;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::mpsc;

/// RFB protocol version string
const RFB_VERSION: &[u8] = b"RFB 003.008\n";

/// Security types
const SECURITY_TYPE_NONE: u8 = 1;

/// Client-to-server message types
const MSG_SET_PIXEL_FORMAT: u8 = 0;
const MSG_SET_ENCODINGS: u8 = 2;
const MSG_FRAMEBUFFER_UPDATE_REQUEST: u8 = 3;
const MSG_KEY_EVENT: u8 = 4;
const MSG_POINTER_EVENT: u8 = 5;
const MSG_CLIENT_CUT_TEXT: u8 = 6;

/// Server-to-client message types
const MSG_FRAMEBUFFER_UPDATE: u8 = 0;

/// Encoding types
const ENCODING_RAW: i32 = 0;
#[allow(dead_code)]
const ENCODING_COPYRECT: i32 = 1;

/// Pixel format for 32-bit RGBA
#[derive(Clone, Debug)]
pub struct PixelFormat {
    pub bits_per_pixel: u8,
    pub depth: u8,
    pub big_endian: bool,
    pub true_color: bool,
    pub red_max: u16,
    pub green_max: u16,
    pub blue_max: u16,
    pub red_shift: u8,
    pub green_shift: u8,
    pub blue_shift: u8,
}

impl Default for PixelFormat {
    fn default() -> Self {
        Self {
            bits_per_pixel: 32,
            depth: 24,
            big_endian: false,
            true_color: true,
            red_max: 255,
            green_max: 255,
            blue_max: 255,
            red_shift: 16,
            green_shift: 8,
            blue_shift: 0,
        }
    }
}

impl PixelFormat {
    fn to_bytes(&self) -> [u8; 16] {
        let mut buf = [0u8; 16];
        buf[0] = self.bits_per_pixel;
        buf[1] = self.depth;
        buf[2] = if self.big_endian { 1 } else { 0 };
        buf[3] = if self.true_color { 1 } else { 0 };
        buf[4..6].copy_from_slice(&self.red_max.to_be_bytes());
        buf[6..8].copy_from_slice(&self.green_max.to_be_bytes());
        buf[8..10].copy_from_slice(&self.blue_max.to_be_bytes());
        buf[10] = self.red_shift;
        buf[11] = self.green_shift;
        buf[12] = self.blue_shift;
        // buf[13..16] is padding
        buf
    }

    fn from_bytes(buf: &[u8; 16]) -> Self {
        Self {
            bits_per_pixel: buf[0],
            depth: buf[1],
            big_endian: buf[2] != 0,
            true_color: buf[3] != 0,
            red_max: u16::from_be_bytes([buf[4], buf[5]]),
            green_max: u16::from_be_bytes([buf[6], buf[7]]),
            blue_max: u16::from_be_bytes([buf[8], buf[9]]),
            red_shift: buf[10],
            green_shift: buf[11],
            blue_shift: buf[12],
        }
    }
}

/// VNC input event from client
#[derive(Debug, Clone)]
pub enum VncInputEvent {
    /// Key press/release event (keysym, down)
    Key { keysym: u32, down: bool },
    /// Pointer/mouse event (x, y, button_mask)
    Pointer { x: u16, y: u16, button_mask: u8 },
    /// Client disconnected
    Disconnected,
}

/// Frame update region
#[derive(Debug, Clone)]
pub struct UpdateRegion {
    pub x: u16,
    pub y: u16,
    pub width: u16,
    pub height: u16,
}

/// VNC server state
pub struct VncServer {
    listener: TcpListener,
    client: Option<VncClient>,
    width: u16,
    height: u16,
    name: String,
    event_sender: mpsc::Sender<VncInputEvent>,
    event_receiver: mpsc::Receiver<VncInputEvent>,
}

struct VncClient {
    stream: TcpStream,
    pixel_format: PixelFormat,
    update_requested: bool,
    incremental: bool,
}

impl VncServer {
    /// Create a new VNC server listening on the given port
    pub async fn new(port: u16, width: u16, height: u16, name: &str) -> io::Result<Self> {
        let listener = TcpListener::bind(format!("0.0.0.0:{}", port)).await?;
        let (event_sender, event_receiver) = mpsc::channel(256);

        log::info!("VNC server listening on port {}", port);

        Ok(Self {
            listener,
            client: None,
            width,
            height,
            name: name.to_string(),
            event_sender,
            event_receiver,
        })
    }

    /// Get the event receiver for polling input events
    pub fn event_receiver(&mut self) -> &mut mpsc::Receiver<VncInputEvent> {
        &mut self.event_receiver
    }

    /// Check for new client connections (non-blocking)
    pub async fn accept_client(&mut self) -> io::Result<bool> {
        // Use tokio's select with instant timeout to poll for connections
        let accept_result = tokio::select! {
            biased;
            result = self.listener.accept() => Some(result),
            _ = tokio::time::sleep(std::time::Duration::ZERO) => None,
        };

        match accept_result {
            Some(Ok((stream, addr))) => {
                log::info!("VNC client connected from {}", addr);

                // Only allow one client at a time
                if self.client.is_some() {
                    log::warn!("Rejecting client: another client is already connected");
                    return Ok(false);
                }

                match self.handshake(stream).await {
                    Ok(client) => {
                        self.client = Some(client);
                        Ok(true)
                    }
                    Err(e) => {
                        log::error!("Handshake failed: {}", e);
                        Ok(false)
                    }
                }
            }
            Some(Err(e)) => Err(e),
            None => Ok(false),
        }
    }

    async fn handshake(&self, mut stream: TcpStream) -> io::Result<VncClient> {
        // Send protocol version
        stream.write_all(RFB_VERSION).await?;

        // Receive client protocol version
        let mut version = [0u8; 12];
        stream.read_exact(&mut version).await?;
        log::debug!("Client version: {:?}", std::str::from_utf8(&version));

        // Send security types (only None)
        stream.write_all(&[1, SECURITY_TYPE_NONE]).await?;

        // Receive selected security type
        let mut security_type = [0u8; 1];
        stream.read_exact(&mut security_type).await?;
        if security_type[0] != SECURITY_TYPE_NONE {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "Unsupported security type"));
        }

        // Send security result (OK)
        stream.write_all(&[0, 0, 0, 0]).await?;

        // Receive ClientInit (shared flag)
        let mut shared = [0u8; 1];
        stream.read_exact(&mut shared).await?;

        // Send ServerInit
        let pixel_format = PixelFormat::default();
        let name_bytes = self.name.as_bytes();

        let mut server_init = Vec::with_capacity(24 + name_bytes.len());
        server_init.extend_from_slice(&self.width.to_be_bytes());
        server_init.extend_from_slice(&self.height.to_be_bytes());
        server_init.extend_from_slice(&pixel_format.to_bytes());
        server_init.extend_from_slice(&(name_bytes.len() as u32).to_be_bytes());
        server_init.extend_from_slice(name_bytes);
        stream.write_all(&server_init).await?;

        Ok(VncClient { stream, pixel_format, update_requested: false, incremental: false })
    }

    /// Process messages from the client
    pub async fn process_client_messages(&mut self) -> io::Result<()> {
        if self.client.is_none() {
            return Ok(());
        }

        // Disable Nagle algorithm for lower latency on small messages
        if let Some(client) = &self.client {
            client.stream.set_nodelay(true)?;
        }

        loop {
            // Try to read the message type without holding a long borrow
            let read_result = {
                let Some(client) = &self.client else {
                    return Ok(());
                };
                let mut msg_type = [0u8; 1];
                client.stream.try_read(&mut msg_type).map(|n| (n, msg_type[0]))
            };

            match read_result {
                Ok((0, _)) => {
                    // Client disconnected
                    log::info!("VNC client disconnected");
                    let _ = self.event_sender.send(VncInputEvent::Disconnected).await;
                    self.client = None;
                    return Ok(());
                }
                Ok((_, msg_type)) => {
                    self.handle_client_message(msg_type).await?;
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {
                    // No more messages
                    return Ok(());
                }
                Err(e) => {
                    log::error!("Error reading from client: {}", e);
                    self.client = None;
                    let _ = self.event_sender.send(VncInputEvent::Disconnected).await;
                    return Err(e);
                }
            }
        }
    }

    async fn handle_client_message(&mut self, msg_type: u8) -> io::Result<()> {
        let client = self.client.as_mut().unwrap();

        match msg_type {
            MSG_SET_PIXEL_FORMAT => {
                let mut buf = [0u8; 19]; // padding (3) + pixel format (16)
                client.stream.read_exact(&mut buf).await?;
                let pf_bytes: [u8; 16] = buf[3..19].try_into().unwrap();
                client.pixel_format = PixelFormat::from_bytes(&pf_bytes);
                log::debug!("Client set pixel format: {:?}", client.pixel_format);
            }
            MSG_SET_ENCODINGS => {
                let mut buf = [0u8; 3]; // padding (1) + num encodings (2)
                client.stream.read_exact(&mut buf).await?;
                let num_encodings = u16::from_be_bytes([buf[1], buf[2]]) as usize;
                let mut encodings = vec![0u8; num_encodings * 4];
                client.stream.read_exact(&mut encodings).await?;
                log::debug!("Client set {} encodings", num_encodings);
            }
            MSG_FRAMEBUFFER_UPDATE_REQUEST => {
                let mut buf = [0u8; 9];
                client.stream.read_exact(&mut buf).await?;
                client.incremental = buf[0] != 0;
                client.update_requested = true;
                log::trace!("Framebuffer update requested (incremental: {})", client.incremental);
            }
            MSG_KEY_EVENT => {
                let mut buf = [0u8; 7];
                client.stream.read_exact(&mut buf).await?;
                let down = buf[0] != 0;
                let keysym = u32::from_be_bytes([buf[3], buf[4], buf[5], buf[6]]);
                log::trace!("Key event: keysym={:x}, down={}", keysym, down);
                let _ = self.event_sender.send(VncInputEvent::Key { keysym, down }).await;
            }
            MSG_POINTER_EVENT => {
                let mut buf = [0u8; 5];
                client.stream.read_exact(&mut buf).await?;
                let button_mask = buf[0];
                let x = u16::from_be_bytes([buf[1], buf[2]]);
                let y = u16::from_be_bytes([buf[3], buf[4]]);
                log::trace!("Pointer event: x={}, y={}, buttons={:02x}", x, y, button_mask);
                let _ = self.event_sender.send(VncInputEvent::Pointer { x, y, button_mask }).await;
            }
            MSG_CLIENT_CUT_TEXT => {
                let mut buf = [0u8; 7]; // padding (3) + length (4)
                client.stream.read_exact(&mut buf).await?;
                let length = u32::from_be_bytes([buf[3], buf[4], buf[5], buf[6]]) as usize;
                let mut text = vec![0u8; length];
                client.stream.read_exact(&mut text).await?;
                log::debug!("Client cut text: {} bytes", length);
            }
            _ => {
                log::warn!("Unknown message type: {}", msg_type);
            }
        }

        Ok(())
    }

    /// Check if an update was requested by the client
    pub fn update_requested(&self) -> bool {
        self.client.as_ref().is_some_and(|c| c.update_requested)
    }

    /// Check if the client requested an incremental update
    pub fn is_incremental(&self) -> bool {
        self.client.as_ref().is_some_and(|c| c.incremental)
    }

    /// Send a framebuffer update to the client
    ///
    /// The buffer should be in RGBA format (32 bits per pixel, as produced by SoftwareRenderer).
    /// The regions specify which parts of the screen to update.
    /// If regions is empty and the request was incremental, sends an empty update (0 rectangles).
    pub async fn send_frame_update(
        &mut self,
        buffer: &[u8],
        stride: usize,
        regions: &[UpdateRegion],
    ) -> io::Result<()> {
        let Some(client) = &mut self.client else {
            return Ok(());
        };

        if !client.update_requested {
            return Ok(());
        }

        client.update_requested = false;

        // For incremental requests with no dirty regions, send empty update
        if regions.is_empty() {
            let msg = [MSG_FRAMEBUFFER_UPDATE, 0, 0, 0]; // 0 rectangles
            client.stream.write_all(&msg).await?;
            client.stream.flush().await?;
            return Ok(());
        }

        // Build the framebuffer update message
        let mut msg = Vec::new();
        msg.push(MSG_FRAMEBUFFER_UPDATE);
        msg.push(0); // padding
        msg.extend_from_slice(&(regions.len() as u16).to_be_bytes());

        for region in regions {
            // Rectangle header
            msg.extend_from_slice(&region.x.to_be_bytes());
            msg.extend_from_slice(&region.y.to_be_bytes());
            msg.extend_from_slice(&region.width.to_be_bytes());
            msg.extend_from_slice(&region.height.to_be_bytes());
            msg.extend_from_slice(&ENCODING_RAW.to_be_bytes());

            // Convert pixel data based on client's pixel format
            // Source buffer is RGBA (PremultipliedRgbaColor from SoftwareRenderer)
            let src_bpp = 4;
            let dst_bpp = (client.pixel_format.bits_per_pixel / 8) as usize;

            for y in region.y..(region.y + region.height) {
                let row_start = (y as usize) * stride + (region.x as usize) * src_bpp;
                for x in 0..region.width {
                    let pixel_offset = row_start + (x as usize) * src_bpp;
                    if pixel_offset + src_bpp <= buffer.len() {
                        // Source is RGBA
                        let r = buffer[pixel_offset];
                        let g = buffer[pixel_offset + 1];
                        let b = buffer[pixel_offset + 2];

                        // Convert to client's pixel format
                        if dst_bpp == 4 {
                            let pixel = Self::convert_pixel(r, g, b, &client.pixel_format);
                            if client.pixel_format.big_endian {
                                msg.extend_from_slice(&pixel.to_be_bytes());
                            } else {
                                msg.extend_from_slice(&pixel.to_le_bytes());
                            }
                        } else if dst_bpp == 2 {
                            // 16-bit color
                            let pixel = Self::convert_pixel_16(r, g, b, &client.pixel_format);
                            if client.pixel_format.big_endian {
                                msg.extend_from_slice(&pixel.to_be_bytes());
                            } else {
                                msg.extend_from_slice(&pixel.to_le_bytes());
                            }
                        } else {
                            // Fallback: output as RGB with padding
                            msg.extend_from_slice(&[r, g, b, 0]);
                        }
                    }
                }
            }
        }

        client.stream.write_all(&msg).await?;
        client.stream.flush().await?;

        Ok(())
    }

    fn convert_pixel(r: u8, g: u8, b: u8, pf: &PixelFormat) -> u32 {
        let r_scaled = (r as u32 * pf.red_max as u32) / 255;
        let g_scaled = (g as u32 * pf.green_max as u32) / 255;
        let b_scaled = (b as u32 * pf.blue_max as u32) / 255;

        (r_scaled << pf.red_shift) | (g_scaled << pf.green_shift) | (b_scaled << pf.blue_shift)
    }

    fn convert_pixel_16(r: u8, g: u8, b: u8, pf: &PixelFormat) -> u16 {
        let r_scaled = (r as u16 * pf.red_max) / 255;
        let g_scaled = (g as u16 * pf.green_max) / 255;
        let b_scaled = (b as u16 * pf.blue_max) / 255;

        ((r_scaled << pf.red_shift) | (g_scaled << pf.green_shift) | (b_scaled << pf.blue_shift))
            as u16
    }

    /// Check if a client is connected
    #[allow(dead_code)]
    pub fn has_client(&self) -> bool {
        self.client.is_some()
    }

    /// Get the screen dimensions
    pub fn dimensions(&self) -> (u16, u16) {
        (self.width, self.height)
    }
}
