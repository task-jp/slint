// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Slint-Royalty-free-2.0 OR LicenseRef-Slint-Software-3.0

//! Input mapping from VNC keysyms and mouse events to Slint events

use i_slint_core::SharedString;
use i_slint_core::platform::PointerEventButton;

// X11 keysym constants (from X11/keysymdef.h)
mod keysym {
    pub const XK_BACKSPACE: u32 = 0xFF08;
    pub const XK_TAB: u32 = 0xFF09;
    pub const XK_RETURN: u32 = 0xFF0D;
    pub const XK_ESCAPE: u32 = 0xFF1B;
    pub const XK_DELETE: u32 = 0xFFFF;
    pub const XK_BACKTAB: u32 = 0xFE20;

    pub const XK_SHIFT_L: u32 = 0xFFE1;
    pub const XK_SHIFT_R: u32 = 0xFFE2;
    pub const XK_CONTROL_L: u32 = 0xFFE3;
    pub const XK_CONTROL_R: u32 = 0xFFE4;
    pub const XK_CAPS_LOCK: u32 = 0xFFE5;
    pub const XK_META_L: u32 = 0xFFE7;
    pub const XK_META_R: u32 = 0xFFE8;
    pub const XK_ALT_L: u32 = 0xFFE9;
    pub const XK_MODE_SWITCH: u32 = 0xFF7E;

    pub const XK_SPACE: u32 = 0x0020;

    pub const XK_HOME: u32 = 0xFF50;
    pub const XK_LEFT: u32 = 0xFF51;
    pub const XK_UP: u32 = 0xFF52;
    pub const XK_RIGHT: u32 = 0xFF53;
    pub const XK_DOWN: u32 = 0xFF54;
    pub const XK_PAGE_UP: u32 = 0xFF55;
    pub const XK_PAGE_DOWN: u32 = 0xFF56;
    pub const XK_END: u32 = 0xFF57;

    pub const XK_INSERT: u32 = 0xFF63;
    pub const XK_MENU: u32 = 0xFF67;

    pub const XK_PAUSE: u32 = 0xFF13;
    pub const XK_SCROLL_LOCK: u32 = 0xFF14;
    pub const XK_SYS_REQ: u32 = 0xFF15;

    pub const XK_F1: u32 = 0xFFBE;
    pub const XK_F2: u32 = 0xFFBF;
    pub const XK_F3: u32 = 0xFFC0;
    pub const XK_F4: u32 = 0xFFC1;
    pub const XK_F5: u32 = 0xFFC2;
    pub const XK_F6: u32 = 0xFFC3;
    pub const XK_F7: u32 = 0xFFC4;
    pub const XK_F8: u32 = 0xFFC5;
    pub const XK_F9: u32 = 0xFFC6;
    pub const XK_F10: u32 = 0xFFC7;
    pub const XK_F11: u32 = 0xFFC8;
    pub const XK_F12: u32 = 0xFFC9;
    pub const XK_F13: u32 = 0xFFCA;
    pub const XK_F14: u32 = 0xFFCB;
    pub const XK_F15: u32 = 0xFFCC;
    pub const XK_F16: u32 = 0xFFCD;
    pub const XK_F17: u32 = 0xFFCE;
    pub const XK_F18: u32 = 0xFFCF;
    pub const XK_F19: u32 = 0xFFD0;
    pub const XK_F20: u32 = 0xFFD1;
    pub const XK_F21: u32 = 0xFFD2;
    pub const XK_F22: u32 = 0xFFD3;
    pub const XK_F23: u32 = 0xFFD4;
    pub const XK_F24: u32 = 0xFFD5;

    pub const XK_XF86_STOP: u32 = 0x1008FF28;
    pub const XK_XF86_BACK: u32 = 0x1008FF26;
}

/// Convert an X11 keysym to a Unicode codepoint
fn keysym_to_unicode(keysym: u32) -> Option<char> {
    // Unicode keysyms (0x01000000 + unicode_codepoint)
    if keysym >= 0x01000000 && keysym <= 0x0110FFFF {
        return char::from_u32(keysym - 0x01000000);
    }

    // Latin-1 range maps directly
    if keysym >= 0x0020 && keysym <= 0x007E {
        return char::from_u32(keysym);
    }
    if keysym >= 0x00A0 && keysym <= 0x00FF {
        return char::from_u32(keysym);
    }

    // Some special cases for other ranges
    // (there are more, but these cover common cases)
    None
}

/// Map a VNC/X11 keysym to a Slint key text
pub fn map_key_sym(keysym: u32) -> Option<SharedString> {
    use keysym::*;

    let c = match keysym {
        XK_BACKSPACE => '\u{0008}',
        XK_TAB => '\u{0009}',
        XK_RETURN => '\u{000a}',
        XK_ESCAPE => '\u{001b}',
        XK_BACKTAB => '\u{0019}',
        XK_DELETE => '\u{007f}',

        XK_SHIFT_L => '\u{0010}',
        XK_CONTROL_L => '\u{0011}',
        XK_ALT_L => '\u{0012}',
        XK_MODE_SWITCH => '\u{0013}',
        XK_CAPS_LOCK => '\u{0014}',
        XK_SHIFT_R => '\u{0015}',
        XK_CONTROL_R => '\u{0016}',
        XK_META_L => '\u{0017}',
        XK_META_R => '\u{0018}',

        XK_SPACE => '\u{0020}',

        XK_UP => '\u{F700}',
        XK_DOWN => '\u{F701}',
        XK_LEFT => '\u{F702}',
        XK_RIGHT => '\u{F703}',

        XK_F1 => '\u{F704}',
        XK_F2 => '\u{F705}',
        XK_F3 => '\u{F706}',
        XK_F4 => '\u{F707}',
        XK_F5 => '\u{F708}',
        XK_F6 => '\u{F709}',
        XK_F7 => '\u{F70A}',
        XK_F8 => '\u{F70B}',
        XK_F9 => '\u{F70C}',
        XK_F10 => '\u{F70D}',
        XK_F11 => '\u{F70E}',
        XK_F12 => '\u{F70F}',
        XK_F13 => '\u{F710}',
        XK_F14 => '\u{F711}',
        XK_F15 => '\u{F712}',
        XK_F16 => '\u{F713}',
        XK_F17 => '\u{F714}',
        XK_F18 => '\u{F715}',
        XK_F19 => '\u{F716}',
        XK_F20 => '\u{F717}',
        XK_F21 => '\u{F718}',
        XK_F22 => '\u{F719}',
        XK_F23 => '\u{F71A}',
        XK_F24 => '\u{F71B}',

        XK_INSERT => '\u{F727}',
        XK_HOME => '\u{F729}',
        XK_END => '\u{F72B}',
        XK_PAGE_UP => '\u{F72C}',
        XK_PAGE_DOWN => '\u{F72D}',
        XK_SCROLL_LOCK => '\u{F72F}',
        XK_PAUSE => '\u{F730}',
        XK_SYS_REQ => '\u{F731}',
        XK_XF86_STOP => '\u{F734}',
        XK_MENU => '\u{F735}',
        XK_XF86_BACK => '\u{F748}',

        _ => keysym_to_unicode(keysym)?,
    };

    Some(c.into())
}

/// VNC mouse button state
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
pub struct MouseButtonState {
    pub left: bool,
    pub middle: bool,
    pub right: bool,
    pub wheel_up: bool,
    pub wheel_down: bool,
}

impl MouseButtonState {
    /// Create a new button state from VNC button mask
    /// VNC button mask: bit 0=left, 1=middle, 2=right, 3=wheel up, 4=wheel down
    pub fn from_button_mask(mask: u8) -> Self {
        Self {
            left: (mask & 0x01) != 0,
            middle: (mask & 0x02) != 0,
            right: (mask & 0x04) != 0,
            wheel_up: (mask & 0x08) != 0,
            wheel_down: (mask & 0x10) != 0,
        }
    }

    /// Get the button that changed between two states
    pub fn changed_button(&self, prev: &Self) -> Option<(PointerEventButton, bool)> {
        if self.left != prev.left {
            return Some((PointerEventButton::Left, self.left));
        }
        if self.middle != prev.middle {
            return Some((PointerEventButton::Middle, self.middle));
        }
        if self.right != prev.right {
            return Some((PointerEventButton::Right, self.right));
        }
        None
    }

    /// Check if wheel was scrolled
    pub fn wheel_delta(&self) -> i32 {
        let mut delta = 0;
        if self.wheel_up {
            delta += 1;
        }
        if self.wheel_down {
            delta -= 1;
        }
        delta
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_button_mask_parsing() {
        let state = MouseButtonState::from_button_mask(0x01);
        assert!(state.left);
        assert!(!state.middle);
        assert!(!state.right);

        let state = MouseButtonState::from_button_mask(0x04);
        assert!(!state.left);
        assert!(state.right);

        let state = MouseButtonState::from_button_mask(0x08);
        assert!(state.wheel_up);
        assert!(!state.wheel_down);
    }

    #[test]
    fn test_changed_button() {
        let prev = MouseButtonState::default();
        let curr = MouseButtonState::from_button_mask(0x01);
        assert_eq!(curr.changed_button(&prev), Some((PointerEventButton::Left, true)));

        let prev = MouseButtonState::from_button_mask(0x01);
        let curr = MouseButtonState::default();
        assert_eq!(curr.changed_button(&prev), Some((PointerEventButton::Left, false)));
    }

    #[test]
    fn test_keysym_mapping() {
        // Test basic ASCII
        assert_eq!(map_key_sym(0x61), Some("a".into())); // 'a'
        assert_eq!(map_key_sym(0x41), Some("A".into())); // 'A'

        // Test special keys
        assert_eq!(map_key_sym(keysym::XK_RETURN), Some("\u{000a}".into()));
        assert_eq!(map_key_sym(keysym::XK_ESCAPE), Some("\u{001b}".into()));
        assert_eq!(map_key_sym(keysym::XK_F1), Some("\u{F704}".into()));

        // Test Unicode keysym
        assert_eq!(map_key_sym(0x01000041), Some("A".into())); // Unicode 'A'
    }
}
