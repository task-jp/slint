// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

extern crate alloc;

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::vec;
use core::cell::RefCell;
use core::convert::Infallible;
use cortex_m::interrupt::Mutex;
use cortex_m::singleton;
pub use cortex_m_rt::entry;
use defmt_rtt as _;
use embedded_alloc::LlffHeap as Heap;
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;
use embedded_hal::spi::{ErrorType, Operation, SpiBus, SpiDevice};
use fugit::{Hertz, RateExtU32};
use hal::dma::{DMAExt, SingleChannel, WriteTarget};
use hal::gpio::{self, Interrupt as GpioInterrupt};
use hal::timer::{Alarm, Alarm0};
use pac::interrupt;
#[cfg(feature = "panic-probe")]
use panic_probe as _;
use renderer::Rgb565Pixel;
use waveshare_rp2040_lcd_1_69::{hal::{self, pac, prelude::*, Timer}, XOSC_CRYSTAL_FREQ};
use slint::platform::{software_renderer as renderer, PointerEventButton, WindowEvent};

const HEAP_SIZE: usize = 200 * 1024;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

type IrqPin = gpio::Pin<gpio::bank0::Gpio21, gpio::FunctionSio<gpio::SioInput>, gpio::PullUp>;
static IRQ_PIN: Mutex<RefCell<Option<IrqPin>>> = Mutex::new(RefCell::new(None));

static ALARM0: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));
static TIMER: Mutex<RefCell<Option<Timer>>> = Mutex::new(RefCell::new(None));

// 16ns for serial clock cycle (write), page 43 of https://www.waveshare.com/w/upload/a/ae/ST7789_Datasheet.pdf
const SPI_ST7789VW_MAX_FREQ: Hertz<u32> = Hertz::<u32>::Hz(62_500_000);

const DISPLAY_SIZE: slint::PhysicalSize = slint::PhysicalSize::new(320, 240);

/// The Pixel type of the backing store
pub type TargetPixel = Rgb565Pixel;

type SpiPins = (
    gpio::Pin<gpio::bank0::Gpio11, gpio::FunctionSpi, gpio::PullDown>,
    gpio::Pin<gpio::bank0::Gpio12, gpio::FunctionSpi, gpio::PullDown>,
    gpio::Pin<gpio::bank0::Gpio10, gpio::FunctionSpi, gpio::PullDown>,
);

type EnabledSpi = hal::Spi<hal::spi::Enabled, pac::SPI1, SpiPins, 8>;
type SpiRefCell = RefCell<(EnabledSpi, Hertz<u32>)>;
type Display<DI, RST> = mipidsi::Display<DI, mipidsi::models::ST7789, RST>;

#[derive(Clone)]
struct SharedSpiWithFreq<CS> {
    refcell: &'static SpiRefCell,
    cs: CS,
    freq: Hertz<u32>,
}

impl<CS> ErrorType for SharedSpiWithFreq<CS> {
    type Error = <EnabledSpi as ErrorType>::Error;
}

impl<CS: OutputPin<Error = Infallible>> SpiDevice for SharedSpiWithFreq<CS> {
    #[inline]
    fn transaction(&mut self, operations: &mut [Operation<u8>]) -> Result<(), Self::Error> {
        let mut borrowed = self.refcell.borrow_mut();
        if borrowed.1 != self.freq {
            borrowed.0.flush()?;
            // the touchscreen and the LCD have different frequencies
            borrowed.0.set_baudrate(125_000_000u32.Hz(), self.freq);
            borrowed.1 = self.freq;
        }
        self.cs.set_low()?;
        for op in operations {
            match op {
                Operation::Read(words) => borrowed.0.read(words),
                Operation::Write(words) => borrowed.0.write(words),
                Operation::Transfer(read, write) => borrowed.0.transfer(read, write),
                Operation::TransferInPlace(words) => borrowed.0.transfer_in_place(words),
                Operation::DelayNs(_) => unimplemented!(),
            }?;
        }
        borrowed.0.flush()?;
        drop(borrowed);
        self.cs.set_high()?;
        Ok(())
    }
}

struct SharedIrqPin(&'static Mutex<RefCell<Option<IrqPin>>>);

impl embedded_hal::digital::ErrorType for SharedIrqPin {
    type Error = Infallible;
}

impl embedded_hal::digital::InputPin for SharedIrqPin {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        cortex_m::interrupt::free(|cs| {
            self.0.borrow(cs).borrow_mut().as_mut().map_or(Ok(false), |p| p.is_high())
        })
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        cortex_m::interrupt::free(|cs| {
            self.0.borrow(cs).borrow_mut().as_mut().map_or(Ok(true), |p| p.is_low())
        })
    }
}
pub fn init() {
    // Get all peripherals
    let pac = pac::Peripherals::take().unwrap();
    
    // Extract all peripherals we need first to avoid ownership issues
    let mut resets = pac.RESETS;
    let watchdog_peripheral = pac.WATCHDOG;
    let xosc_peripheral = pac.XOSC;
    let clocks_peripheral = pac.CLOCKS;
    let pll_sys_peripheral = pac.PLL_SYS;
    let pll_usb_peripheral = pac.PLL_USB;
    let timer_peripheral = pac.TIMER;
    let i2c1_peripheral = pac.I2C1; 
    let io_bank0 = pac.IO_BANK0;
    let pads_bank0 = pac.PADS_BANK0;
    let sio_peripheral = pac.SIO;
    let spi1_peripheral = pac.SPI1;
    let dma_peripheral = pac.DMA;
    
    // Create watchdog 
    let mut watchdog = hal::watchdog::Watchdog::new(watchdog_peripheral);
    
    // Initialize clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        xosc_peripheral,
        clocks_peripheral,
        pll_sys_peripheral,
        pll_usb_peripheral,
        &mut resets,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Initialize heap allocator
    unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as usize, HEAP_SIZE) }

    // Save peripheral clock frequency for later use
    let peripheral_clock_freq = clocks.peripheral_clock.freq();
    
    // Initialize Timer
    let mut timer = Timer::new(timer_peripheral, &mut resets, &clocks);
    
    // Now we can safely extract SIO and setup pins
    let sio = hal::sio::Sio::new(sio_peripheral);
    let pins = waveshare_rp2040_lcd_1_69::Pins::new(
        io_bank0, 
        pads_bank0, 
        sio.gpio_bank0, 
        &mut resets
    );
    
    // Initialize DMA after USB setup
    let dma_channels = dma_peripheral.split(&mut resets);

    // --- Initialize SPI ---
    let spi_sclk = pins.gp10.into_function::<gpio::FunctionSpi>();
    let spi_mosi = pins.gp11.into_function::<gpio::FunctionSpi>();
    let spi_miso = pins.gp12.into_function::<gpio::FunctionSpi>();
    let spi = hal::Spi::new(spi1_peripheral, (spi_mosi, spi_miso, spi_sclk));
    let spi = spi.init(
        &mut resets,
        peripheral_clock_freq,
        SPI_ST7789VW_MAX_FREQ,
        &embedded_hal::spi::MODE_3,
    );
    // SAFETY: This is not safe :-( But we need to access the SPI and its control pins for the PIO
    let stolen_spi = unsafe { core::ptr::read(&spi as *const _) };
    let spi_refcell = singleton!(:SpiRefCell = SpiRefCell::new((spi, 0.Hz()))).unwrap();

    // --- Initialize I2C ---
    let sda_pin = pins.gp6.into_pull_up_input().into_function::<gpio::FunctionI2C>();
    let scl_pin = pins.gp7.into_pull_up_input().into_function::<gpio::FunctionI2C>();
    let i2c = hal::I2C::i2c1(
        i2c1_peripheral,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut resets,
        peripheral_clock_freq,
    );

    // --- Initialize Display ---
    let rst_disp = pins.gp13.into_push_pull_output(); // Renamed to avoid conflict with touch rst
    let backlight = pins.gp25.into_push_pull_output();

    let dc_disp = pins.gp8.into_push_pull_output(); // Renamed
    let cs_disp = pins.gp9.into_push_pull_output(); // Renamed

    // SAFETY: This is not safe :-( But we need to access the SPI control pins for the PIO DMA
    let (dc_copy, cs_copy) =
        unsafe { (core::ptr::read(&dc_disp as *const _), core::ptr::read(&cs_disp as *const _)) };

    let mipidsi_buffer = singleton!(:[u8; 512] = [0; 512]).unwrap();
    let display_spi = SharedSpiWithFreq { refcell: spi_refcell, cs: cs_disp, freq: SPI_ST7789VW_MAX_FREQ };
    let di = mipidsi::interface::SpiInterface::new(display_spi, dc_disp, mipidsi_buffer);
    let display = mipidsi::Builder::new(mipidsi::models::ST7789, di)
        .reset_pin(rst_disp)
        .display_size(DISPLAY_SIZE.height as _, DISPLAY_SIZE.width as _)
        .orientation(mipidsi::options::Orientation::new().rotate(mipidsi::options::Rotation::Deg90))
        .invert_colors(mipidsi::options::ColorInversion::Inverted)
        .init(&mut timer)
        .unwrap();

    // --- Initialize Touch Controller ---
    let int_pin = pins.gpio21.into_pull_up_input();
    let rst_touch = pins.gpio22.into_push_pull_output();
    int_pin.set_interrupt_enabled(GpioInterrupt::LevelLow, true);

    cortex_m::interrupt::free(|cs| {
        IRQ_PIN.borrow(cs).replace(Some(int_pin));
    });

    // Create the touch controller, passing a shared reference to the IRQ_PIN.
    let mut touch = cst816s::CST816S::new(i2c, SharedIrqPin(&IRQ_PIN), rst_touch);
    
    // Setup the touch controller (required for proper functionality)
    touch.setup(&mut timer).unwrap();
    
    // --- Setup Timer Alarm & Interrupts ---
    let mut alarm0 = timer.alarm_0().unwrap(); // Timer already initialized
    alarm0.enable_interrupt();

    cortex_m::interrupt::free(|cs| {
        ALARM0.borrow(cs).replace(Some(alarm0));
        TIMER.borrow(cs).replace(Some(timer));
    });

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    let pio = PioTransfer::Idle(
        dma_channels.ch0,
        vec![Rgb565Pixel::default(); DISPLAY_SIZE.width as _].leak(),
        stolen_spi,
    );
    let buffer_provider = DrawBuffer {
        display,
        buffer: vec![Rgb565Pixel::default(); DISPLAY_SIZE.width as _].leak(),
        pio: Some(pio),
        stolen_pin: (dc_copy, cs_copy),
    };

    slint::platform::set_platform(Box::new(PicoBackend {
        window: Default::default(),
        buffer_provider: buffer_provider.into(),
        touch: touch.into(),
        backlight: Some(backlight).into(),
    }))
    .expect("backend already initialized");
}

struct PicoBackend<DrawBuffer, Touch, Backlight> {
    window: RefCell<Option<Rc<renderer::MinimalSoftwareWindow>>>,
    buffer_provider: RefCell<DrawBuffer>,
    touch: RefCell<Touch>,
    backlight: RefCell<Option<Backlight>>,
}

impl<
        DI: mipidsi::interface::Interface<Word = u8>,
        RST: embedded_hal::digital::OutputPin<Error = Infallible>,
        TO: WriteTarget<TransmittedWord = u8> + embedded_hal_nb::spi::FullDuplex,
        CH: SingleChannel,
        DC_: embedded_hal::digital::OutputPin<Error = Infallible>,
        CS_: embedded_hal::digital::OutputPin<Error = Infallible>,
        I2C_: I2c,
        IPinT: embedded_hal::digital::InputPin,
        OPinT: embedded_hal::digital::StatefulOutputPin,
        BL: embedded_hal::digital::OutputPin<Error = Infallible>,
    > slint::platform::Platform
    for PicoBackend<
        DrawBuffer<Display<DI, RST>, PioTransfer<TO, CH>, (DC_, CS_)>,
        cst816s::CST816S<I2C_, IPinT, OPinT>,
        BL,
    >
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
        let mut last_touch = None;

        self.window.borrow().as_ref().unwrap().set_size(DISPLAY_SIZE);

        loop {
            slint::platform::update_timers_and_animations();

            if let Some(window) = self.window.borrow().clone() {
                window.draw_if_needed(|renderer| {
                    let mut buffer_provider = self.buffer_provider.borrow_mut();
                    renderer.render_by_line(&mut *buffer_provider);
                    buffer_provider.flush_frame();
                    if let Some(mut backlight) = self.backlight.take() {
                        backlight.set_high().unwrap();
                    }
                });

                let mut touch = self.touch.borrow_mut();

                // Handle touch events from the CST816S controller
                let button = PointerEventButton::Left;
                let mut event_opt = None;
                
                if let Some(touch_event) = touch.read_one_touch_event(true) {
                    let raw_x = touch_event.x as i32;
                    let raw_y = touch_event.y as i32;

                    let touch_x = raw_y; // Invert Y axis for display
                    let touch_y = DISPLAY_SIZE.height as i32 - raw_x; // Map X to Y
                    
                    let touch_x = touch_x.clamp(0, DISPLAY_SIZE.width as i32 - 1);
                    let touch_y = touch_y.clamp(0, DISPLAY_SIZE.height as i32 - 1);
                    
                    // Create position for the touch event
                    let position = slint::PhysicalPosition::new(touch_x, touch_y)
                        .to_logical(window.scale_factor());
                    
                    if last_touch.is_none() {
                        event_opt = Some(WindowEvent::PointerPressed { position, button });
                    } else {
                        event_opt = Some(WindowEvent::PointerMoved { position });
                    }
                    last_touch.replace(position);
                } else if let Some(position) = last_touch.take() {
                    event_opt = Some(WindowEvent::PointerReleased { position, button });
                }
                
                if let Some(event) = event_opt {
                    let is_pointer_release_event =
                        matches!(event, WindowEvent::PointerReleased { .. });
                    
                    window.try_dispatch_event(event)?;

                    // removes hover state on widgets
                    if is_pointer_release_event {
                        window.try_dispatch_event(WindowEvent::PointerExited)?;
                    }
                    // Don't go to sleep after a touch event that forces a redraw
                    continue;
                }

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
                    defmt::debug!("Going to sleep for {} µs", duration.to_micros());
                    ALARM0.borrow(cs).borrow_mut().as_mut().unwrap().schedule(duration).unwrap();
                }

                IRQ_PIN
                    .borrow(cs)
                    .borrow()
                    .as_ref()
                    .unwrap()
                    .set_interrupt_enabled(GpioInterrupt::LevelLow, true);
            });
            cortex_m::asm::wfe();
        }
    }

    fn debug_log(&self, arguments: core::fmt::Arguments) {
        use alloc::string::ToString;
        defmt::info!("[SLINT] {=str}", arguments.to_string());
    }
}

enum PioTransfer<TO: WriteTarget, CH: SingleChannel> {
    Idle(CH, &'static mut [TargetPixel], TO),
    Running(hal::dma::single_buffer::Transfer<CH, PartialReadBuffer, TO>),
}

impl<TO: WriteTarget<TransmittedWord = u8>, CH: SingleChannel> PioTransfer<TO, CH> {
    fn wait(self) -> (CH, &'static mut [TargetPixel], TO) {
        match self {
            PioTransfer::Idle(a, b, c) => (a, b, c),
            PioTransfer::Running(dma) => {
                let (a, b, to) = dma.wait();
                (a, b.0, to)
            }
        }
    }
}

struct DrawBuffer<Display, PioTransfer, Stolen> {
    display: Display,
    buffer: &'static mut [TargetPixel],
    pio: Option<PioTransfer>,
    stolen_pin: Stolen,
}

impl<
        DI: mipidsi::interface::Interface<Word = u8>,
        RST: OutputPin<Error = Infallible>,
        TO: WriteTarget<TransmittedWord = u8>,
        CH: SingleChannel,
        DC_: OutputPin<Error = Infallible>,
        CS_: OutputPin<Error = Infallible>,
    > renderer::LineBufferProvider
    for &mut DrawBuffer<Display<DI, RST>, PioTransfer<TO, CH>, (DC_, CS_)>
{
    type TargetPixel = TargetPixel;

    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [TargetPixel]),
    ) {
        render_fn(&mut self.buffer[range.clone()]);

        /* -- Send the pixel without DMA
        self.display.set_pixels(
            range.start as _,
            line as _,
            range.end as _,
            line as _,
            self.buffer[range.clone()]
                .iter()
                .map(|x| embedded_graphics::pixelcolor::raw::RawU16::new(x.0).into()),
        );
        return;*/

        // convert from little to big endian before sending to the DMA channel
        for x in &mut self.buffer[range.clone()] {
            *x = Rgb565Pixel(x.0.to_be())
        }
        let (ch, mut b, spi) = self.pio.take().unwrap().wait();
        core::mem::swap(&mut self.buffer, &mut b);

        // We send empty data just to get the device in the right window
        self.display
            .set_pixels(
                range.start as u16,
                line as _,
                range.end as u16,
                line as u16,
                core::iter::empty(),
            )
            .unwrap();

        self.stolen_pin.1.set_low().unwrap();
        self.stolen_pin.0.set_high().unwrap();
        let mut dma = hal::dma::single_buffer::Config::new(ch, PartialReadBuffer(b, range), spi);
        dma.pace(hal::dma::Pace::PreferSink);
        self.pio = Some(PioTransfer::Running(dma.start()));
        /*let (a, b, c) = dma.start().wait();
        self.pio = Some(PioTransfer::Idle(a, b.0, c));*/
    }
}

impl<
        DI: mipidsi::interface::Interface<Word = u8>,
        RST: OutputPin<Error = Infallible>,
        TO: WriteTarget<TransmittedWord = u8> + embedded_hal_nb::spi::FullDuplex,
        CH: SingleChannel,
        DC_: OutputPin<Error = Infallible>,
        CS_: OutputPin<Error = Infallible>,
    > DrawBuffer<Display<DI, RST>, PioTransfer<TO, CH>, (DC_, CS_)>
{
    fn flush_frame(&mut self) {
        let (ch, b, mut spi) = self.pio.take().unwrap().wait();
        self.stolen_pin.1.set_high().unwrap();

        // After the DMA operated, we need to empty the receive FIFO, otherwise the touch screen
        // driver will pick wrong values.
        // Continue to read as long as we don't get a Err(WouldBlock)
        defmt::debug!("Emptying receive FIFO after DMA completion");
        let mut fifo_count = 0;
        while !spi.read().is_err() {
            fifo_count += 1;
        }
        defmt::debug!("Cleared {} bytes from FIFO", fifo_count);

        self.pio = Some(PioTransfer::Idle(ch, b, spi));
    }
}

struct PartialReadBuffer(&'static mut [Rgb565Pixel], core::ops::Range<usize>);
unsafe impl embedded_dma::ReadBuffer for PartialReadBuffer {
    type Word = u8;

    unsafe fn read_buffer(&self) -> (*const <Self as embedded_dma::ReadBuffer>::Word, usize) {
        let act_slice = &self.0[self.1.clone()];
        (act_slice.as_ptr() as *const u8, act_slice.len() * core::mem::size_of::<Rgb565Pixel>())
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    cortex_m::interrupt::free(|cs| {
        let mut pin = IRQ_PIN.borrow(cs).borrow_mut();
        let pin = pin.as_mut().unwrap();
        pin.set_interrupt_enabled(GpioInterrupt::LevelLow, false);
        pin.clear_interrupt(GpioInterrupt::LevelLow);
    });
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
    let mut pac = unsafe { pac::Peripherals::steal() };

    let sio = hal::sio::Sio::new(pac.SIO);
    let pins = waveshare_rp2040_lcd_1_69::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    let mut led = pins.gp25.into_push_pull_output(); // Using GP25 as LED pin
    led.set_high().unwrap();
    
    // パニックを単純なメッセージとして記録
    defmt::error!("PANIC occurred!");

    // Re-init the display
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
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
    let spi_miso = pins.gp12.into_function::<gpio::FunctionSpi>();

    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI1, (spi_mosi, spi_miso, spi_sclk));
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        4_000_000u32.Hz(),
        &embedded_hal::spi::MODE_3,
    );

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let rst = pins.gp15.into_push_pull_output();
    let mut bl = pins.gp13.into_push_pull_output();
    let dc = pins.gp8.into_push_pull_output();
    let cs = pins.gp9.into_push_pull_output();
    bl.set_high().unwrap();
    let spi = singleton!(:SpiRefCell = SpiRefCell::new((spi, 0.Hz()))).unwrap();
    let display_spi = SharedSpiWithFreq { refcell: spi, cs, freq: SPI_ST7789VW_MAX_FREQ };
    let mut buffer = [0_u8; 512];
    let di = mipidsi::interface::SpiInterface::new(display_spi, dc, &mut buffer);
    let mut display = mipidsi::Builder::new(mipidsi::models::ST7789, di)
        .reset_pin(rst)
        .display_size(DISPLAY_SIZE.height as _, DISPLAY_SIZE.width as _)
        .orientation(mipidsi::options::Orientation::new().rotate(mipidsi::options::Rotation::Deg90))
        .invert_colors(mipidsi::options::ColorInversion::Inverted)
        .init(&mut timer)
        .unwrap();

    use core::fmt::Write;
    use embedded_graphics::{
        mono_font::{ascii::FONT_6X10, MonoTextStyle},
        pixelcolor::Rgb565,
        prelude::*,
        text::Text,
    };

    display.fill_solid(&display.bounding_box(), Rgb565::new(0x00, 0x25, 0xff)).unwrap();

    struct WriteToScreen<'a, D> {
        x: i32,
        y: i32,
        width: i32,
        style: MonoTextStyle<'a, Rgb565>,
        display: &'a mut D,
    }
    let mut writer = WriteToScreen {
        x: 0,
        y: 1,
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

    loop {
        use embedded_hal::delay::DelayNs as _;
        timer.delay_ms(100);
        led.set_low().unwrap();
        timer.delay_ms(100);
        led.set_high().unwrap();
    }
}
