// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

use alloc::boxed::Box;
use alloc::rc::Rc;
use core::cell::RefCell;
use display_interface::WriteOnlyDataCommand;
use display_interface_spi::SPIInterface;
use embedded_hal::digital::v2::OutputPin;
use embedded_graphics_core::prelude::RawData;
use esp32s3_hal::{
    clock::{ClockControl, CpuClock},
    i2c::I2C,
    peripherals::Peripherals,
    prelude::*,
    spi::{Spi, SpiMode},
    systimer::SystemTimer,
    Delay, IO,
};
use esp_alloc::EspHeap;
use esp_backtrace as _;
use gc9a01::GC9A01;
use slint::platform::WindowEvent;
pub use xtensa_lx_rt::entry;

#[global_allocator]
static ALLOCATOR: EspHeap = EspHeap::empty();

pub fn init() {
    const HEAP_SIZE: usize = 250 * 1024;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    unsafe { ALLOCATOR.init(&mut HEAP as *mut u8, core::mem::size_of_val(&HEAP)) }
    slint::platform::set_platform(Box::new(EspBackend::default()))
        .expect("backend already initialized");
}

#[derive(Default)]
struct EspBackend {
    window: RefCell<Option<Rc<slint::platform::software_renderer::MinimalSoftwareWindow>>>,
}

impl slint::platform::Platform for EspBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
            slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
        );
        self.window.replace(Some(window.clone()));
        Ok(window)
    }

    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_millis(
            SystemTimer::now() / (SystemTimer::TICKS_PER_SECOND / 1000),
        )
    }

    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        let peripherals = Peripherals::take();
        let mut system = peripherals.SYSTEM.split();
        let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock240MHz).freeze();

        let mut delay = Delay::new(&clocks);
        let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

        let i2c = I2C::new(
            peripherals.I2C0,
            io.pins.gpio11,
            io.pins.gpio12,
            400u32.kHz(),
            &mut system.peripheral_clock_control,
            &clocks,
        );

        let mut touch = ft3267::FT3267::new(i2c);

        let sclk = io.pins.gpio6;
        let mosi = io.pins.gpio5;

        let spi = Spi::new_no_cs_no_miso(
            peripherals.SPI2,
            sclk,
            mosi,
            60u32.MHz(),
            SpiMode::Mode0,
            &mut system.peripheral_clock_control,
            &clocks,
        );

        let dc = io.pins.gpio4.into_push_pull_output();
        let cs = io.pins.gpio7.into_push_pull_output();
        let rst = io.pins.gpio8.into_push_pull_output();

        let di = SPIInterface::new(spi, dc, cs);
        let display = GC9A01::new(
            di,
            rst,
            &mut delay,
            gc9a01::Orientation::Portrait,
            gc9a01::DisplaySize240x240,
        )
        .unwrap();

        let mut backlight = io.pins.gpio9.into_push_pull_output();
        backlight.set_high().unwrap();

        let size = slint::PhysicalSize::new(240, 240);

        self.window.borrow().as_ref().unwrap().set_size(size);

        let mut buffer_provider = DrawBuffer {
            display,
            buffer: &mut [slint::platform::software_renderer::Rgb565Pixel(0); 320],
        };

        let mut last_touch = None;
        let button = slint::platform::PointerEventButton::Left;

        loop {
            slint::platform::update_timers_and_animations();

            if let Some(window) = self.window.borrow().clone() {
                let touch0 = touch.touch()[0];
                let event = match touch0 {
                    Some((x, y)) => {
                        let position = slint::PhysicalPosition::new(
                            ((239. - y as f32) * size.width as f32 / 239.) as _,
                            (x as f32 * size.height as f32 / 239.) as _,
                        )
                        .to_logical(window.scale_factor());
                        match last_touch.replace(position) {
                            Some(_) => Some(WindowEvent::PointerMoved { position }),
                            None => Some(WindowEvent::PointerPressed { position, button }),
                        }
                    }
                    None => {
                        if let Some(position) = last_touch.take() {
                            Some(WindowEvent::PointerReleased { position, button })
                        } else {
                            None
                        }
                    }
                };
                if let Some(event) = event {
                    window.dispatch_event(event);
                }

                window.draw_if_needed(|renderer| {
                    renderer.render_by_line(&mut buffer_provider);
                });
                if window.has_active_animations() {
                    continue;
                }
            }
            // TODO
        }
    }

    fn debug_log(&self, arguments: core::fmt::Arguments) {
        esp_println::println!("{}", arguments);
    }
}

struct DrawBuffer<'a, Display> {
    display: Display,
    buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}

impl<
        IFACE: WriteOnlyDataCommand,
        RST: OutputPin<Error = core::convert::Infallible>,
    > slint::platform::software_renderer::LineBufferProvider
    for &mut DrawBuffer<'_, gc9a01::GC9A01<IFACE, RST>>
{
    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;

    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [slint::platform::software_renderer::Rgb565Pixel]),
    ) {
        let buffer = &mut self.buffer[range.clone()];

        render_fn(buffer);

        // We send empty data just to get the device in the right window
        self.display
            .draw_raw_iter(
                range.start as u16,
                line as _,
                range.end as u16,
                line as u16,
                buffer
                    .iter()
                    .map(|x| embedded_graphics_core::pixelcolor::raw::RawU16::new(x.0).into_inner()),
            )
            .unwrap();
    }
}

mod ft3267 {
    use embedded_hal::blocking::i2c::WriteRead;

    pub struct FT3267<I2C> {
        i2c: I2C,
        address: u8,
    }

    impl<I2C, E> FT3267<I2C>
    where
        I2C: WriteRead<Error = E>,
    {
        pub fn new(i2c: I2C) -> Self {
            Self { i2c, address: 0x38 }
        }

        pub fn touch(&mut self) -> [Option<(u16, u16)>; 2] {
            const FT_TP_STATUS: usize = 0x02;
            const FT_TP1_XH: usize = 0x03;
            const FT_TP1_XL: usize = 0x04;
            const FT_TP1_YH: usize = 0x05;
            const FT_TP1_YL: usize = 0x06;
            const FT_TP2_XH: usize = 0x09;
            const FT_TP2_XL: usize = 0x0a;
            const FT_TP2_YH: usize = 0x0b;
            const FT_TP2_YL: usize = 0x0c;

            let mut data: [u8; 13] = [0; 13];
            for i in 0..13 {
                if let Ok(d) = self.read(i) {
                    data[i as usize] = d;
                }
            }
            let count = data[FT_TP_STATUS];
            let mut points: [Option<(u16, u16)>; 2] = [None, None];
            if count > 0 {
                let x1 = ((data[FT_TP1_XH] as u16 & 0x0F) << 8) | (data[FT_TP1_XL] as u16);
                let y1 = ((data[FT_TP1_YH] as u16 & 0x0F) << 8) | (data[FT_TP1_YL] as u16);
                points[0] = Some((y1, 240 - x1));
            }
            if count > 1 {
                let x2 = ((data[FT_TP2_XH] as u16 & 0x0F) << 8) | (data[FT_TP2_XL] as u16);
                let y2 = ((data[FT_TP2_YH] as u16 & 0x0F) << 8) | (data[FT_TP2_YL] as u16);
                points[1] = Some((y2, 240 - x2));
            }
            points
        }

        fn read(&mut self, register: u8) -> Result<u8, E> {
            let mut data = [0];
            self.i2c.write_read(self.address, &[register], &mut data).map(|_| data[0])
        }
    }
}
