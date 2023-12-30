// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

#![no_std]
#![cfg_attr(not(feature = "simulator"), no_main)]

slint::include_modules!();

#[mcu_board_support::entry]
fn main() -> ! {
    mcu_board_support::init();

    let hello_slint = HelloSlint::new().unwrap();
    hello_slint.run().unwrap();

    panic!("The MCU demo should not quit")
}
