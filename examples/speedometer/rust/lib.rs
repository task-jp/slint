// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

#[cfg(feature = "sw-renderer")]
slint::slint! {
    export { MainWindow } from "../demo.slint";
}

#[cfg(all(not(feature = "sw-renderer"), not(target_os = "android")))]
slint::slint! {
    export { MainWindow } from "../demo.slint";
}

#[cfg(all(not(feature = "sw-renderer"), target_os = "android"))]
slint::slint! {
    export { MainWindow } from "../demo-centered.slint";
}

#[cfg_attr(target_arch = "wasm32", wasm_bindgen(start))]
pub fn main() {
    // This provides better error messages in debug mode.
    // It's disabled in release mode so it doesn't bloat up the file size.
    #[cfg(all(debug_assertions, target_arch = "wasm32"))]
    console_error_panic_hook::set_once();

    let app = MainWindow::new().expect("MainWindow::new() failed");

    app.run().expect("MainWindow::run() failed");
}

#[cfg(target_os = "android")]
#[unsafe(no_mangle)]
fn android_main(android_app: slint::android::AndroidApp) {
    slint::android::init(android_app).unwrap();
    main();
}
