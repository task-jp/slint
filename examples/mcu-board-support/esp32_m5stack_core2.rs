use alloc::boxed::Box;
use alloc::rc::Rc;
use core::cell::RefCell;
use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics_core::geometry::OriginDimensions;
use embedded_hal::digital::v2::OutputPin;
use esp32_hal::{
    clock::{ClockControl, CpuClock},
    i2c::I2C,
    peripherals::{self, Peripherals, TIMG0},
    prelude::*,
    spi::{master::Spi, SpiMode},
    timer::{Timer, Timer0, TimerGroup},
    interrupt::{self, Priority},
    Delay, Rtc, IO,
};
use esp_alloc::EspHeap;
use esp_backtrace as _;
use mipidsi::Display;
use slint::platform::WindowEvent;
pub use xtensa_lx_rt::entry;
use shared_bus;
use dummy_pin::DummyPin;

use critical_section::Mutex;
static TIMER00: Mutex<RefCell<Option<Timer<Timer0<TIMG0>>>>> = Mutex::new(RefCell::new(None));
static mut SYSTEM_COUNTER: u64 = 0;

#[global_allocator]
static ALLOCATOR: EspHeap = EspHeap::empty();

pub fn init() {
    const HEAP_SIZE: usize = 150 * 1024; // 150KB, original for CoreS3 is 200KB
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    unsafe { ALLOCATOR.init(&mut HEAP as *mut u8, core::mem::size_of_val(&HEAP)) }
    slint::platform::set_platform(Box::new(EspBackend::default()))
        .expect("backend already initialized");
}

#[interrupt]
fn TG0_T0_LEVEL() {
    critical_section::with(|cs| {
        let mut timer = TIMER00.borrow_ref_mut(cs);
        let timer = timer.as_mut().unwrap();

        if timer.is_interrupt_set() {
            timer.clear_interrupt();
            timer.start(1u64.millis());
            unsafe {
                core::ptr::write_volatile(&mut SYSTEM_COUNTER, SYSTEM_COUNTER.wrapping_add(1));
            }
        }
    });
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
            unsafe {
                core::ptr::read_volatile(&mut SYSTEM_COUNTER)
            }
        )
    }

    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        let peripherals = Peripherals::take();
        let system = peripherals.SYSTEM.split();
        let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock240MHz).freeze();

        let mut rtc = Rtc::new(peripherals.RTC_CNTL);
        let timer_group0 =
            TimerGroup::new(peripherals.TIMG0, &clocks);
        let mut wdt0 = timer_group0.wdt;
        let timer_group1 =
            TimerGroup::new(peripherals.TIMG1, &clocks);
        let mut wdt1 = timer_group1.wdt;

        rtc.rwdt.disable();
        wdt0.disable();
        wdt1.disable();

        let mut timer00 = timer_group0.timer0;
        interrupt::enable(peripherals::Interrupt::TG0_T0_LEVEL, Priority::Priority2).unwrap();
        timer00.start(1u64.millis());
        timer00.listen();
        critical_section::with(|cs| {
            TIMER00.borrow_ref_mut(cs).replace(timer00);
        });

        let mut delay = Delay::new(&clocks);
        let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

        let i2c = I2C::new(
            peripherals.I2C0,
            io.pins.gpio21, //sda
            io.pins.gpio22, //scl
            400u32.kHz(),
            &clocks,
        );

        let i2c_bus = shared_bus::BusManagerSimple::new(i2c);
        let mut axp = axp192::Axp192::new(i2c_bus.acquire_i2c());
        // core power
        axp.set_dcdc1_voltage(3350).unwrap();
        // LCD power
        axp.set_ldo2_voltage(3300).unwrap();
        axp.set_ldo2_on(true).unwrap();
        // LCD backlight
        axp.set_dcdc3_voltage(3300).unwrap();
        axp.set_dcdc3_on(true).unwrap();
        // LCD reset
        axp.set_gpio4_mode(axp192::GpioMode34::NmosOpenDrainOutput).unwrap();
        axp.set_gpio4_output(false).unwrap();
        delay.delay_us(100 * 1000u32);
        axp.set_gpio4_output(true).unwrap();
        // power LED
        axp.set_gpio1_mode(axp192::GpioMode12::NmosOpenDrainOutput).unwrap();
        axp.set_gpio1_output(false).unwrap();

        let mut touch = ft6x36::Ft6x36::new(i2c_bus.acquire_i2c(), ft6x36::Dimension(320, 240));
        touch.init().unwrap();

        let spi = Spi::new(
            peripherals.SPI3,
            io.pins.gpio18, //sclk
            io.pins.gpio23, //sdo
            io.pins.gpio38, //sdi
            io.pins.gpio5,  //cs
            60u32.MHz(),
            SpiMode::Mode0,
            &clocks,
        );

        let dc = io.pins.gpio15.into_push_pull_output();
        let rst = DummyPin::new_low(); // connected with AXP192

        let di = SPIInterfaceNoCS::new(spi, dc);
        let display = mipidsi::Builder::ili9342c_rgb565(di)
            .with_display_size(320, 240)
            .with_invert_colors(mipidsi::ColorInversion::Inverted)
            .with_color_order(mipidsi::options::ColorOrder::Bgr)
            .init(&mut delay, Some(rst))
            .unwrap();

        let size = display.size();
        let size = slint::PhysicalSize::new(size.width, size.height);

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
                touch.get_touch_event()
                    .ok()
                    .and_then(|touch_event| {
                        let time = self.duration_since_start();
                        let event = touch_event.p1
                            .map(|p1| {
                                let position = slint::PhysicalPosition::new(
                                    ((p1.x as f32) * size.width as f32 / 319.) as _,
                                    (p1.y as f32 * size.height as f32 / 239.) as _,
                                ).to_logical(window.scale_factor());
                                let _ = last_touch.replace(position);
                                match p1.touch_type {
                                    ft6x36::TouchType::Press => WindowEvent::PointerPressed { position, button },
                                    ft6x36::TouchType::Contact => WindowEvent::PointerMoved { position },
                                    ft6x36::TouchType::Release => todo!(),
                                    ft6x36::TouchType::Invalid => todo!(),
                                }
                            }).or_else(|| {
                                match last_touch {
                                    Some(_) => last_touch.take().map(|position| WindowEvent::PointerReleased {
                                        position,
                                        button,
                                    }),
                                    None => None,
                                }
                            });
                        if !event.is_none() {
                            let e = event.unwrap();
                            let is_pointer_release_event = matches!(e, WindowEvent::PointerReleased { .. });
                            window.dispatch_event(e);
                            // removes hover state on widgets
                            if is_pointer_release_event {
                                window.dispatch_event(WindowEvent::PointerExited);
                            }
                        }
                        touch.process_event(time, touch_event)
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

struct DrawBuffer<'a, Display> {
    display: Display,
    buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}

impl<
        DI: display_interface::WriteOnlyDataCommand,
        RST: OutputPin<Error = core::convert::Infallible>,
    > slint::platform::software_renderer::LineBufferProvider
    for &mut DrawBuffer<'_, Display<DI, mipidsi::models::ILI9342CRgb565, RST>>
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
            .set_pixels(
                range.start as u16,
                line as _,
                range.end as u16,
                line as u16,
                buffer
                    .iter()
                    .map(|x| embedded_graphics_core::pixelcolor::raw::RawU16::new(x.0).into()),
            )
            .unwrap();
    }
}
