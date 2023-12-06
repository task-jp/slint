// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

fn main() -> std::io::Result<()> {
    #[allow(unused)]
    let mut board_config_path: Option<std::path::PathBuf> = None;

    cfg_if::cfg_if! {
        if #[cfg(feature = "pico-st7789")] {
            board_config_path = Some([env!("CARGO_MANIFEST_DIR"), "pico_st7789", "board_config.toml"].iter().collect());
        } else if #[cfg(feature = "rp2040-lcd-1-28")] {
            board_config_path = Some([env!("CARGO_MANIFEST_DIR"), "rp2040_lcd_1_28", "board_config.toml"].iter().collect());
        } else if #[cfg(feature = "stm32h735g")] {
            board_config_path = Some([env!("CARGO_MANIFEST_DIR"), "stm32h735g", "board_config.toml"].iter().collect());
        }
    }

    if let Some(path) = board_config_path {
        println!("cargo:BOARD_CONFIG_PATH={}", path.display())
    }

    println!("cargo:EMBED_TEXTURES=1");

    Ok(())
}
