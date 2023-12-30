extern crate alloc;

use alloc::boxed::Box;
use alloc::rc::Rc;
use core::cell::RefCell;
#[cfg(feature = "defmt-rtt")]
use defmt_rtt as _;

use embedded_alloc::Heap;
use embedded_graphics_core::prelude::RawData;
use embedded_hal::digital::v2::OutputPin;

pub use wio::entry;
use wio::hal::clock::GenericClockController;
use wio::hal::delay::Delay;
use wio::pac::{interrupt, CorePeripherals, Peripherals};
use wio::prelude::*;
use wio::Pins;
use wio::{button_interrupt, Button, ButtonController, ButtonEvent};

use cortex_m::interrupt::{free as disable_interrupts, CriticalSection};

use wio_terminal as wio;

use display_interface::WriteOnlyDataCommand;
use ili9341::Ili9341;

use slint::platform::software_renderer as renderer;
use slint::platform::WindowEvent::KeyPressed;
use slint::platform::WindowEvent::KeyReleased;

const DISPLAY_SIZE: slint::PhysicalSize = slint::PhysicalSize::new(320, 240);

const HEAP_SIZE: usize = 100 * 1024;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

#[cfg(feature = "panic-halt")]
use panic_halt as _;
#[cfg(feature = "panic-probe")]
use panic_probe as _;

use heapless::{consts::U8, spsc::Queue};

pub fn init() {
    unsafe { ALLOCATOR.init(&mut HEAP as *const u8 as usize, core::mem::size_of_val(&HEAP)) }
    slint::platform::set_platform(Box::new(WioTermialBackend { window: Default::default() }))
        .expect("backend already initialized");
}

struct WioTermialBackend {
    window: RefCell<Option<Rc<renderer::MinimalSoftwareWindow>>>,
}

impl slint::platform::Platform for WioTermialBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        let window =
            renderer::MinimalSoftwareWindow::new(renderer::RepaintBufferType::ReusedBuffer);
        self.window.replace(Some(window.clone()));
        Ok(window)
    }

    fn duration_since_start(&self) -> core::time::Duration {
        // let counter = cortex_m::interrupt::free(|cs| {
        //     TIMER.borrow(cs).borrow().as_ref().map(|t| t.get_counter().ticks()).unwrap_or_default()
        // });
        core::time::Duration::from_micros(0) // TODO
    }

    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        let mut peripherals = Peripherals::take().unwrap();
        let mut core = CorePeripherals::take().unwrap();
        let mut clocks = GenericClockController::with_internal_32kosc(
            peripherals.GCLK,
            &mut peripherals.MCLK,
            &mut peripherals.OSC32KCTRL,
            &mut peripherals.OSCCTRL,
            &mut peripherals.NVMCTRL,
        );
        let mut delay = Delay::new(core.SYST, &mut clocks);
        let sets = Pins::new(peripherals.PORT).split();

        let (display, _backlight) = sets
            .display
            .init(&mut clocks, peripherals.SERCOM7, &mut peripherals.MCLK, 58.MHz(), &mut delay)
            .unwrap();

        self.window.borrow().as_ref().unwrap().set_size(DISPLAY_SIZE);

        let button_ctrlr = sets.buttons.init(peripherals.EIC, &mut clocks, &mut peripherals.MCLK);
        let nvic = &mut core.NVIC;
        disable_interrupts(|_| unsafe {
            button_ctrlr.enable(nvic);
            BUTTON_CTRLR = Some(button_ctrlr);
        });

        let mut consumer = unsafe { Q.split().1 };

        let mut buffer_provider = DrawBuffer {
            display,
            buffer: &mut [slint::platform::software_renderer::Rgb565Pixel(0); 320],
        };

        loop {
            slint::platform::update_timers_and_animations();

            if let Some(window) = self.window.borrow().clone() {
                window.draw_if_needed(|renderer| {
                    renderer.render_by_line(&mut buffer_provider);
                });

                if let Some(button_event) = consumer.dequeue() {
                    let text = match button_event.button {
                        Button::TopLeft => slint::platform::Key::F1,
                        Button::TopMiddle => slint::platform::Key::F2,
                        // Button::TopRight => slint::platform::Key::F3,
                        Button::Up => slint::platform::Key::UpArrow,
                        Button::Left => slint::platform::Key::LeftArrow,
                        Button::Right => slint::platform::Key::RightArrow,
                        Button::Down => slint::platform::Key::DownArrow,
                        Button::Click => slint::platform::Key::Return,
                    };
                    let event = if button_event.down { KeyPressed { text: text.into() } } else { KeyReleased { text: text.into() } };
                    window.dispatch_event(event);
                }

                if window.has_active_animations() {
                    continue;
                }
            }
        }
    }

    #[cfg(feature = "defmt-rtt")]
    fn debug_log(&self, arguments: core::fmt::Arguments) {
        use alloc::string::ToString;
        defmt::println!("{=str}", arguments.to_string());
    }
}

static mut BUTTON_CTRLR: Option<ButtonController> = None;
static mut Q: Queue<ButtonEvent, U8> = Queue(heapless::i::Queue::new());

button_interrupt!(
    BUTTON_CTRLR,
    unsafe fn on_button_event(_cs: &CriticalSection, event: ButtonEvent) {
        let mut q = Q.split().0;
        q.enqueue(event).ok();
    }
);

struct DrawBuffer<'a, LCD> {
    display: LCD,
    buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}

impl<IFACE: WriteOnlyDataCommand, RST: OutputPin>
    slint::platform::software_renderer::LineBufferProvider
    for &mut DrawBuffer<'_, Ili9341<IFACE, RST>>
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

        let _ = self.display.draw_raw_iter(
            range.start as u16,
            line as _,
            range.end as u16,
            line as u16,
            buffer
                .iter()
                .map(|x| embedded_graphics_core::pixelcolor::raw::RawU16::new(x.0).into_inner()),
        );
    }
}
