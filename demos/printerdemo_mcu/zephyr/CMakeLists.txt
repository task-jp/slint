# Copyright © SixtyFPS GmbH <info@slint.dev>
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.20.0)

if(NOT BOARD)
    message(FATAL_ERROR "No BOARD option specified")
endif()

if(BOARD STREQUAL "native_sim/native/64")
    set(Rust_CARGO_TARGET "x86_64-unknown-linux-gnu")
    set(SLINT_LIBRARY_CARGO_FLAGS "-Zbuild-std=core,alloc")
    set(BOARD_CONF_NAME "native_sim_64")
elseif(BOARD STREQUAL "mimxrt1170_evk@B/mimxrt1176/cm7")
    set(Rust_CARGO_TARGET "thumbv7em-none-eabihf")
    set(BOARD_CONF_NAME "mimxrt1170_evk_mimxrt1176_cm7")
elseif(BOARD STREQUAL "frdm_mcxn947/mcxn947/cpu0")
    set(Rust_CARGO_TARGET "thumbv8m.main-none-eabihf")
    set(SLINT_LIBRARY_CARGO_FLAGS "-Zbuild-std=core,alloc")
    set(BOARD_CONF_NAME "frdm_mcxn947_mcxn947_cpu0")
elseif(BOARD STREQUAL "m5stack_core2/esp32/procpu")
    # The plain ESP32's Xtensa CPU needs the esp-rs toolchain (espup install).
    set(Rust_TOOLCHAIN "esp")
    set(Rust_CARGO_TARGET "xtensa-esp32-none-elf")
    set(SLINT_LIBRARY_CARGO_FLAGS "-Zbuild-std=core,alloc")
    set(BOARD_CONF_NAME "m5stack_core2_esp32_procpu")
    # A full 320x240 RGB565 frame is 150 KB, which does not fit in the
    # ESP32's remaining internal heap; render line by line instead.
    set(SLINT_ZEPHYR_RENDER_BY_LINE 1)
else()
    # See rustc --print target-list for available targets. Match that up with you supported board
    # https://docs.zephyrproject.org/latest/boards
    message(FATAL_ERROR "Unsupported BOARD option specified: ${BOARD}")
endif()

# This panel takes native endian data, and its 480x320 frame does not fit in RAM.
if(SHIELD MATCHES "lcd_par_s035")
    set(SLINT_ZEPHYR_RGB565_NATIVE_ENDIAN 1)
    set(SLINT_ZEPHYR_RENDER_BY_LINE 1)
endif()

# Use shared Zephyr configuration from zephyr-common
set(ZEPHYR_COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../zephyr-common)
set(CONF_FILE ${ZEPHYR_COMMON_DIR}/prj.conf)
if(EXISTS ${ZEPHYR_COMMON_DIR}/boards/${BOARD_CONF_NAME}.overlay)
    set(DTC_OVERLAY_FILE ${ZEPHYR_COMMON_DIR}/boards/${BOARD_CONF_NAME}.overlay)
endif()
if(EXISTS ${ZEPHYR_COMMON_DIR}/boards/${BOARD_CONF_NAME}.conf)
    set(EXTRA_CONF_FILE ${ZEPHYR_COMMON_DIR}/boards/${BOARD_CONF_NAME}.conf)
endif()

find_package(Zephyr)
project(slint_zephyr_printer_demo_mcu LANGUAGES CXX)

set(SLINT_FEATURE_FREESTANDING ON)
set(SLINT_FEATURE_RENDERER_SOFTWARE ON)
set(DEFAULT_SLINT_EMBED_RESOURCES "embed-for-software-renderer" CACHE STRING "")
set(CMAKE_BUILD_TYPE Release)
set(BUILD_SHARED_LIBS OFF)

# Corrosion asks rustc which native libraries the target links against and passes them
# on. For the bare-metal boards it cannot: the probe crate does not build for a no-std
# target, so the list is empty and clearing it changes nothing. native_sim is a hosted
# target, where the probe succeeds and comes back with gcc_s among others. Zephyr links
# native_sim partially (ld -r), which will not take a shared library, so the link then
# needs a static libgcc_s.a that many distributions do not ship.
if(CONFIG_ARCH_POSIX)
    set(Rust_CARGO_TARGET_LINK_NATIVE_LIBS "" CACHE INTERNAL "" FORCE)
endif()

# TODO: FetchContent?
add_subdirectory(../../.. slint_build)

# The app target is defined by the call to find_package(Zephyr)
target_sources(app PRIVATE ../../zephyr-common/slint-zephyr.cpp main.cpp)
target_include_directories(app PRIVATE ../../zephyr-common)
target_link_libraries(app PRIVATE Slint::Slint)

# How the panel is mounted and whether its driver takes native endian pixel data are board and
# shield properties; slint-zephyr.cpp defaults both to the Zephyr documented behavior.
if(DEFINED SLINT_ZEPHYR_PANEL_ROTATION)
    target_compile_definitions(app PRIVATE
        SLINT_ZEPHYR_PANEL_ROTATION=${SLINT_ZEPHYR_PANEL_ROTATION})
endif()
if(DEFINED SLINT_ZEPHYR_RGB565_NATIVE_ENDIAN)
    target_compile_definitions(app PRIVATE
        SLINT_ZEPHYR_RGB565_NATIVE_ENDIAN=${SLINT_ZEPHYR_RGB565_NATIVE_ENDIAN})
endif()
if(DEFINED SLINT_ZEPHYR_RENDER_BY_LINE)
    target_compile_definitions(app PRIVATE SLINT_ZEPHYR_RENDER_BY_LINE)
endif()

slint_target_sources(app ../ui/printerdemo.slint)
