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
    timer::TimerGroup,
    Delay, Rtc, IO,
};
use esp_alloc::EspHeap;
use esp_backtrace as _;
use gc9a01::GC9A01;
use slint::platform::WindowEvent;
pub use xtensa_lx_rt::entry;

#[cfg(feature = "dial")]
use critical_section::Mutex;
#[cfg(feature = "dial")]
use esp32s3_hal::{
    interrupt,
    peripherals,
    pcnt::{
        PCNT,
        channel::{self, PcntSource},
        unit::{self, Unit},
    },
};
#[cfg(feature = "dial")]
use core::{
    cmp::min,
    sync::atomic::{AtomicI32, Ordering},
};


#[global_allocator]
static ALLOCATOR: EspHeap = EspHeap::empty();

#[cfg(feature = "dial")]
static UNIT0: Mutex<RefCell<Option<Unit>>> = Mutex::new(RefCell::new(None));
#[cfg(feature = "dial")]
static VALUE: AtomicI32 = AtomicI32::new(0);

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

        let mut rtc = Rtc::new(peripherals.RTC_CNTL);
        let timer_group0 =
            TimerGroup::new(peripherals.TIMG0, &clocks, &mut system.peripheral_clock_control);
        let mut wdt0 = timer_group0.wdt;
        let timer_group1 =
            TimerGroup::new(peripherals.TIMG1, &clocks, &mut system.peripheral_clock_control);
        let mut wdt1 = timer_group1.wdt;

        rtc.rwdt.disable();
        wdt0.disable();
        wdt1.disable();

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

        #[cfg(feature = "dial")]
        {
            let mut mtdo = io.pins.gpio40.into_pull_up_input();
            let mut mtdi = io.pins.gpio41.into_pull_up_input();
            let pcnt = PCNT::new(peripherals.PCNT, &mut system.peripheral_clock_control);
            let mut u0: unit::Unit = pcnt.get_unit(unit::Number::Unit1);
            u0.configure(unit::Config {
                low_limit: -100,
                high_limit: 100,
                filter: Some(min(10u16 * 80, 1023u16)),
                ..Default::default()
            })
            .unwrap();
            let mut ch0 = u0.get_channel(channel::Number::Channel0);
            ch0.configure(
                PcntSource::from_pin(&mut mtdi),
                PcntSource::from_pin(&mut mtdo),
                channel::Config {
                    lctrl_mode: channel::CtrlMode::Reverse,
                    hctrl_mode: channel::CtrlMode::Keep,
                    pos_edge: channel::EdgeMode::Decrement,
                    neg_edge: channel::EdgeMode::Increment,
                    invert_ctrl: false,
                    invert_sig: false,
                },
            );
            let mut ch1 = u0.get_channel(channel::Number::Channel1);
            ch1.configure(
                PcntSource::from_pin(&mut mtdo),
                PcntSource::from_pin(&mut mtdi),
                channel::Config {
                    lctrl_mode: channel::CtrlMode::Reverse,
                    hctrl_mode: channel::CtrlMode::Keep,
                    pos_edge: channel::EdgeMode::Increment,
                    neg_edge: channel::EdgeMode::Decrement,
                    invert_ctrl: false,
                    invert_sig: false,
                },
            );
            u0.events(unit::Events {
                low_limit: true,
                high_limit: true,
                thresh0: false,
                thresh1: false,
                zero: false,
            });
            u0.listen();
            u0.resume();
    
            critical_section::with(|cs| UNIT0.borrow_ref_mut(cs).replace(u0));
    
            interrupt::enable(peripherals::Interrupt::PCNT, interrupt::Priority::Priority2).unwrap();
        }
        
        let size = slint::PhysicalSize::new(240, 240);

        self.window.borrow().as_ref().unwrap().set_size(size);

        let mut buffer_provider = DrawBuffer {
            display,
            buffer: &mut [slint::platform::software_renderer::Rgb565Pixel(0); 320],
        };

        let mut last_touch = None;
        let button = slint::platform::PointerEventButton::Left;
        #[cfg(feature = "dial")]
        let mut last_dial_value = 0;

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

                #[cfg(feature = "dial")]
                critical_section::with(|cs| {
                    let mut u0 = UNIT0.borrow_ref_mut(cs);
                    let u0 = u0.as_mut().unwrap();
                    let value: i32 = u0.get_value() as i32 + VALUE.load(Ordering::SeqCst);
                    if (last_dial_value - value).abs() > 3  {
                        window.dispatch_event(WindowEvent::PointerScrolled {
                            position: last_touch.unwrap_or(slint::LogicalPosition::new(0f32, 0f32)),
                            delta_x: 0f32,
                            delta_y: (last_dial_value - value) as f32,
                        });
                        last_dial_value = value;
                    }
                });
        
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

#[cfg(feature = "dial")]
#[interrupt]
fn PCNT() {
    critical_section::with(|cs| {
        let mut u0 = UNIT0.borrow_ref_mut(cs);
        let u0 = u0.as_mut().unwrap();
        if u0.interrupt_set() {
            let events = u0.get_events();
            if events.high_limit {
                VALUE.fetch_add(100, Ordering::SeqCst);
            } else if events.low_limit {
                VALUE.fetch_add(-100, Ordering::SeqCst);
            }
            u0.reset_interrupt();
        }
    });
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
