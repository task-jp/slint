#![no_std]
#![no_main]

extern crate alloc;

use alloc::rc::Rc;
use core::cell::RefCell;

slint::include_modules!();

#[derive(Default)]
struct CalcState {
    prev_value: i32,
    current_value: i32,
    operator: slint::SharedString,
}

#[mcu_board_support::entry]
fn main() -> ! {
    mcu_board_support::init();
    let app = App::new().unwrap();
    let weak = app.as_weak();
    let state: Rc<RefCell<CalcState>> = Rc::new(RefCell::new(CalcState::default()));
    app.global::<CalcLogic>().on_button_pressed(move |value| {
        let app = weak.upgrade().unwrap();
        let mut state = state.borrow_mut();
        if let Ok(val) = value.parse::<i32>() {
            state.current_value *= 10;
            state.current_value += val;
            app.set_value(state.current_value);
            return;
        }
        match value.as_str() {
            "C" => {
                state.prev_value = 0;
                state.current_value = 0;
                state.operator = "".into();
                app.set_value(state.current_value);
            }
            "+" => {
                state.prev_value = state.current_value;
                state.current_value = 0;
                state.operator = "+".into();
            }
            "-" => {
                state.prev_value = state.current_value;
                state.current_value = 0;
                state.operator = "-".into();
            }
            "*" => {
                state.prev_value = state.current_value;
                state.current_value = 0;
                state.operator = "*".into();
            }
            "/" => {
                state.prev_value = state.current_value;
                state.current_value = 0;
                state.operator = "/".into();
            }
            "=" => {
                match state.operator.as_str() {
                    "+" => state.current_value = state.prev_value + state.current_value,
                    "-" => state.current_value = state.prev_value - state.current_value,
                    "*" => state.current_value = state.prev_value * state.current_value,
                    "/" => state.current_value = state.prev_value / state.current_value,
                    _ => {}
                }
                app.set_value(state.current_value);
            }
            _ => {}
        }
    });

    app.run().unwrap();

    panic!("The MCU demo should not quit")
}
