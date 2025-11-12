// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() -> std::io::Result<()> {
    #[allow(unused)]
    let mut board_config_path: Option<PathBuf> = None;
    #[allow(unused)]
    let mut memory_x_source: Option<PathBuf> = None;

    cfg_if::cfg_if! {
        if #[cfg(feature = "pico-st7789")] {
            board_config_path = Some([env!("CARGO_MANIFEST_DIR"), "pico_st7789", "board_config.toml"].iter().collect());
        } else if #[cfg(feature = "pico2-st7789")] {
            board_config_path = Some([env!("CARGO_MANIFEST_DIR"), "pico2_st7789", "board_config.toml"].iter().collect());
        } else if #[cfg(feature = "stm32h735g")] {
            board_config_path = Some([env!("CARGO_MANIFEST_DIR"), "stm32h735g", "board_config.toml"].iter().collect());
            memory_x_source = Some([env!("CARGO_MANIFEST_DIR"), "stm32h735g", "memory.x"].iter().collect());
        } else if #[cfg(feature = "stm32u5g9j-dk2")] {
            board_config_path = Some([env!("CARGO_MANIFEST_DIR"), "stm32u5g9j_dk2", "board_config.toml"].iter().collect());
        }
    }

    if let Some(path) = board_config_path {
        println!("cargo:BOARD_CONFIG_PATH={}", path.display())
    }

    // Copy memory.x to OUT_DIR and add it to the linker search path.
    // This ensures the board-specific memory.x takes precedence over
    // any generic memory.x from dependencies (e.g., ft5336).
    // Since Cargo 1.88 (PR #15221), OUT_DIR paths are prioritized over external paths,
    // so we need to place our memory.x in OUT_DIR to ensure it's found first.
    if let Some(source) = memory_x_source {
        let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
        let dest = out_dir.join("memory.x");
        fs::copy(&source, &dest)?;
        println!("cargo:rustc-link-search={}", out_dir.display());
        println!("cargo:rerun-if-changed={}", source.display());
    }

    println!("cargo:EMBED_TEXTURES=1");

    Ok(())
}
