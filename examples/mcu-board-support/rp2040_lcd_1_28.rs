// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

extern crate alloc;

use alloc::boxed::Box;
use alloc::rc::Rc;

use core::cell::RefCell;
use core::convert::Infallible;
use cortex_m::interrupt::Mutex;

pub use cortex_m_rt::entry;
use defmt_rtt as _;
use embedded_alloc::Heap;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;
use fugit::RateExtU32;

use renderer::Rgb565Pixel;
use waveshare_rp2040_lcd_1_28::hal::gpio;
use waveshare_rp2040_lcd_1_28::hal::pac::interrupt;
use waveshare_rp2040_lcd_1_28::hal::timer::{Alarm, Alarm0};
use waveshare_rp2040_lcd_1_28::hal::{self, pac, prelude::*, Timer};

use slint::platform::software_renderer as renderer;

use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::pixelcolor::raw::RawU16;

#[cfg(feature = "panic-probe")]
use panic_probe as _;

// mod display_interface_spi;

const HEAP_SIZE: usize = 200 * 1024;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

static ALARM0: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));
static TIMER: Mutex<RefCell<Option<Timer>>> = Mutex::new(RefCell::new(None));

const DISPLAY_SIZE: slint::PhysicalSize = slint::PhysicalSize::new(240, 240);

// /// The Pixel type of the backing store
pub type TargetPixel = Rgb565Pixel;

pub fn init() {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        waveshare_rp2040_lcd_1_28::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = hal::sio::Sio::new(pac.SIO);
    let pins = waveshare_rp2040_lcd_1_28::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let sys_freq = clocks.system_clock.freq().to_Hz();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_freq);

    let dc = pins.gp8.into_push_pull_output();
    let cs = pins.gp9.into_push_pull_output();
    let spi_sclk = pins.gp10.into_function::<gpio::FunctionSpi>();
    let spi_mosi = pins.gp11.into_function::<gpio::FunctionSpi>();
    let rst = pins.gp12.into_push_pull_output_in_state(hal::gpio::PinState::High);
    let mut bl = pins.gp25.into_push_pull_output_in_state(hal::gpio::PinState::Low);

    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI1, (spi_mosi, spi_sclk));

    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        8_000_000u32.MHz(),
        &embedded_hal::spi::MODE_3,
    );

    let di = display_interface_spi::SPIInterface::new(spi, dc, cs);

    // initialize PWM for backlight
    let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM6
    let mut pwm = pwm_slices.pwm6;
    pwm.set_ph_correct();
    pwm.enable();

    // Output channel B on PWM6 to GPIO 13
    let mut channel = pwm.channel_b;
    channel.output_to(pins.gp13);

    let mut display = gc9a01a::GC9A01A::new(di, rst, channel);

    display.reset(&mut delay).unwrap();
    // Initialize registers
    display.initialize(&mut delay).unwrap();
    // Fill screen with single color
    display.clear(Rgb565::BLACK).unwrap();
    let _ = bl.set_state(hal::gpio::PinState::High);
    // Turn on backlight
    display.set_backlight(55000);

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut alarm0 = timer.alarm_0().unwrap();
    alarm0.enable_interrupt();

    cortex_m::interrupt::free(|cs| {
        ALARM0.borrow(cs).replace(Some(alarm0));
        TIMER.borrow(cs).replace(Some(timer));
    });

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    unsafe { ALLOCATOR.init(&mut HEAP as *const u8 as usize, core::mem::size_of_val(&HEAP)) }

    let buffer_provider = DrawBuffer::new(display);

    slint::platform::set_platform(Box::new(WaveshareRp2040LCD1_28Backend {
        window: Default::default(),
        buffer_provider: buffer_provider.into(),
    }))
    .expect("backend already initialized");
}

struct DrawBuffer<Display> {
    display: Display,
    line_buffer: [slint::platform::software_renderer::Rgb565Pixel; DISPLAY_SIZE.width as usize],
}

impl<Display> DrawBuffer<Display> {
    fn new(display: Display) -> Self {
        let line_buffer =
            [slint::platform::software_renderer::Rgb565Pixel(0); DISPLAY_SIZE.width as usize];
        Self {
            display,
            line_buffer: line_buffer,
        }
    }
}

impl<
        DI: display_interface::WriteOnlyDataCommand,
        RST: OutputPin<Error = Infallible>,
        PWM: PwmPin,
    > renderer::LineBufferProvider for &mut DrawBuffer<gc9a01a::GC9A01A<DI, RST, PWM>>
{
    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;

    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [Self::TargetPixel]),
    ) {
        render_fn(&mut self.line_buffer[range.clone()]);

        self.display
            .fill_contiguous(
                &Rectangle::new(
                    Point::new(range.start as _, line as _),
                    Size::new(range.len() as _, 1),
                ),
                self.line_buffer[range.clone()].iter().map(|p| RawU16::new(p.0).into()),
            )
            .map_err(drop)
            .unwrap();
    }
}

#[derive(Default)]
struct WaveshareRp2040LCD1_28Backend<DrawBuffer> {
    window: RefCell<Option<Rc<renderer::MinimalSoftwareWindow>>>,
    buffer_provider: RefCell<DrawBuffer>,
}

impl<DrawBuffer> slint::platform::Platform for WaveshareRp2040LCD1_28Backend<DrawBuffer> 
    where for<'a> &'a mut DrawBuffer: renderer::LineBufferProvider
{
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        let window =
            renderer::MinimalSoftwareWindow::new(renderer::RepaintBufferType::ReusedBuffer);
        self.window.replace(Some(window.clone()));
        Ok(window)
    }

    fn duration_since_start(&self) -> core::time::Duration {
        let counter = cortex_m::interrupt::free(|cs| {
            TIMER.borrow(cs).borrow().as_ref().map(|t| t.get_counter().ticks()).unwrap_or_default()
        });
        core::time::Duration::from_micros(counter)
    }

    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        self.window.borrow().as_ref().unwrap().set_size(DISPLAY_SIZE);

        loop {
            slint::platform::update_timers_and_animations();

            if let Some(window) = self.window.borrow().clone() {
                window.draw_if_needed(|renderer| {
                    let mut buffer_provider = self.buffer_provider.borrow_mut();
                    renderer.render_by_line(&mut *buffer_provider);
                });

                if window.has_active_animations() {
                    continue;
                }
            }

            let sleep_duration = match slint::platform::duration_until_next_timer_update() {
                None => None,
                Some(d) => {
                    let micros = d.as_micros() as u32;
                    if micros < 10 {
                        // Cannot wait for less than 10µs, or `schedule()` panics
                        continue;
                    } else {
                        Some(fugit::MicrosDurationU32::micros(micros))
                    }
                }
            };

            cortex_m::interrupt::free(|cs| {
                if let Some(duration) = sleep_duration {
                    ALARM0.borrow(cs).borrow_mut().as_mut().unwrap().schedule(duration).unwrap();
                }
            });
            cortex_m::asm::wfe();
        }
    }

    fn debug_log(&self, arguments: core::fmt::Arguments) {
        use alloc::string::ToString;
        defmt::println!("{=str}", arguments.to_string());
    }
}

#[interrupt]
fn TIMER_IRQ_0() {
    cortex_m::interrupt::free(|cs| {
        ALARM0.borrow(cs).borrow_mut().as_mut().unwrap().clear_interrupt();
    });
}

#[cfg(not(feature = "panic-probe"))]
#[inline(never)]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // Safety: it's ok to steal here since we are in the panic handler, and the rest of the code will not be run anymore
    let (mut pac, core) = unsafe { (pac::Peripherals::steal(), pac::CorePeripherals::steal()) };

    let sio = hal::sio::Sio::new(pac.SIO);
    let pins = waveshare_rp2040_lcd_1_28::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Re-init the display
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        waveshare_rp2040_lcd_1_28::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let spi_sclk = pins.gp10.into_function::<gpio::FunctionSpi>();
    let spi_mosi = pins.gp11.into_function::<gpio::FunctionSpi>();

    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI1, (spi_mosi, spi_sclk));
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        8_000_000u32.MHz(),
        &embedded_hal::spi::MODE_0,
    );

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let rst = pins.gp12.into_push_pull_output_in_state(hal::gpio::PinState::High);
    let _bl = pins.gp25.into_push_pull_output_in_state(hal::gpio::PinState::High);
    let dc = pins.gp8.into_push_pull_output();
    let cs = pins.gp9.into_push_pull_output();
    let di = display_interface_spi::SPIInterface::new(spi, dc, cs);

    // initialize PWM for backlight
    let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM6
    let mut pwm = pwm_slices.pwm6;
    pwm.set_ph_correct();
    pwm.enable();

    // Output channel B on PWM6 to GPIO 13
    let mut channel = pwm.channel_b;
    channel.output_to(pins.gp13);

    let mut display = gc9a01a::GC9A01A::new(di, rst, channel);

    use core::fmt::Write;
    use embedded_graphics::{
        mono_font::{ascii::FONT_6X10, MonoTextStyle},
        prelude::*,
        text::Text,
    };

    display.reset(&mut delay).unwrap();
    display.initialize(&mut delay).unwrap();
    display.clear(Rgb565::new(0x00, 0x25, 0xff)).unwrap();

    struct WriteToScreen<'a, D> {
        x: i32,
        y: i32,
        width: i32,
        style: MonoTextStyle<'a, Rgb565>,
        display: &'a mut D,
    }
    let mut writer = WriteToScreen {
        x: 0,
        y: 10,
        width: display.bounding_box().size.width as i32 / 6 - 1,
        style: MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE),
        display: &mut display,
    };
    impl<'a, D: DrawTarget<Color = Rgb565>> Write for WriteToScreen<'a, D> {
        fn write_str(&mut self, mut s: &str) -> Result<(), core::fmt::Error> {
            while !s.is_empty() {
                let (x, y) = (self.x, self.y);
                let end_of_line = s
                    .find(|c| {
                        if c == '\n' || self.x > self.width {
                            self.x = 0;
                            self.y += 1;
                            true
                        } else {
                            self.x += 1;
                            false
                        }
                    })
                    .unwrap_or(s.len());
                let (line, rest) = s.split_at(end_of_line);
                let sz = self.style.font.character_size;
                Text::new(line, Point::new(x * sz.width as i32, y * sz.height as i32), self.style)
                    .draw(self.display)
                    .map_err(|_| core::fmt::Error)?;
                s = rest.strip_prefix('\n').unwrap_or(rest);
            }
            Ok(())
        }
    }
    write!(writer, "{}", info).unwrap();
    loop {}
}
