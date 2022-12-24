#![no_main]
#![no_std]

extern crate panic_halt;
extern crate stm32f4xx_hal as hal;

/// Peripheral Access Crate for our device
pub use hal::pac;

#[rtic::app(device = crate::pac, peripherals = true, dispatchers = [SPI5])]
mod app {
    use core::cell::RefCell;
    use core::fmt::Write;
    use cortex_m::{asm::wfi, interrupt::Mutex};
    use embedded_graphics::text::Text;
    use hal::{gpio::*, i2c::I2c1, prelude::*};
    use heapless::spsc::Queue;
    use lm75::Lm75;
    use shared_bus::{BusManager, I2cProxy};
    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, Ssd1306};

    pub type MainI2cSCL = PB8<AF4<OpenDrain>>;
    pub type MainI2cSDA = PB9<AF4<OpenDrain>>;
    pub type MainI2C = I2c1<(MainI2cSCL, MainI2cSDA)>;
    pub type SharedI2C = BusManager<Mutex<RefCell<MainI2C>>>;
    pub type I2CProxy = I2cProxy<'static, Mutex<RefCell<MainI2C>>>;

    const TEMP_PROBE_ADDRESS: u8 = 0x48;
    const TEMP_POINT_COUNT: usize = 32;
    const TEMP_QUEUE_SIZE: usize = TEMP_POINT_COUNT + 1;
    const SCREEN_WIDTH: usize = 128;
    const SCREEN_HEIGHT: usize = 64;
    const SCREEN_GRAPH_HEIGHT: usize = 64 - 16;
    const TEMP_ACCURACY: f32 = 0.125;

    #[shared]
    struct Shared {
        i2c: &'static SharedI2C,

        temp_data: Queue<f32, TEMP_QUEUE_SIZE>,
    }

    #[local]
    struct Local {
        display: Ssd1306<
            I2CInterface<I2CProxy>,
            DisplaySize128x64,
            BufferedGraphicsMode<DisplaySize128x64>,
        >,
        temp_probe: Lm75<I2CProxy, lm75::ic::Lm75>,
    }

    #[monotonic(binds = TIM5, default = true)]
    type MicrosecMono = hal::timer::MonoTimerUs<crate::pac::TIM5>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Init clocks
        let dp = ctx.device;

        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(100.MHz()).freeze();

        // Timers
        let mono = dp.TIM5.monotonic_us(&clocks);

        // I2C bus

        let gpiob = dp.GPIOB.split();
        let i2c = dp.I2C1.i2c(
            (
                gpiob.pb8.into_alternate_open_drain(),
                gpiob.pb9.into_alternate_open_drain(),
            ),
            400.kHz(),
            &clocks,
        );

        let managed_i2c: &'static _ = shared_bus::new_cortexm!(MainI2C = i2c).unwrap();

        let temp_probe = Lm75::new(managed_i2c.acquire_i2c(), TEMP_PROBE_ADDRESS);

        // Display configure
        // Set Reset pin to High
        let gpioa = dp.GPIOA.split();
        gpioa.pa8.into_push_pull_output().set_high();

        // Configure interface
        let interface = ssd1306::I2CDisplayInterface::new(managed_i2c.acquire_i2c());
        let mut display =
            ssd1306::Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
                .into_buffered_graphics_mode();
        display.init().unwrap();

        temperature_take::spawn().ok();
        draw::spawn().ok();

        (
            Shared {
                i2c: managed_i2c,
                temp_data: Queue::new(),
            },
            Local {
                display,
                temp_probe,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            wfi();
        }
    }

    #[task(local = [temp_probe], shared = [temp_data])]
    fn temperature_take(mut ctx: temperature_take::Context) {
        temperature_take::spawn_after(1000.millis()).ok();

        let Ok(temp) = ctx.local.temp_probe.read_temperature() else {
            return;
        };

        ctx.shared.temp_data.lock(|q| {
            if q.is_full() {
                q.dequeue();
            }
            q.enqueue(temp).ok();
        });
    }

    #[task(local = [display], shared = [temp_data])]
    fn draw(mut ctx: draw::Context) {
        use embedded_graphics::mono_font::iso_8859_5::FONT_6X10;
        use embedded_graphics::mono_font::MonoTextStyleBuilder;
        use embedded_graphics::pixelcolor::BinaryColor;
        use embedded_graphics::prelude::*;
        use embedded_graphics::primitives::{Polyline, PrimitiveStyleBuilder};

        draw::spawn_after(1000.millis()).ok();

        let mut temperatures = [0.0; TEMP_POINT_COUNT];
        ctx.shared.temp_data.lock(|q| {
            for (i, v) in q.iter().cloned().enumerate() {
                temperatures[i] = v;
            }
        });

        let primitive_style = PrimitiveStyleBuilder::new()
            .stroke_width(1)
            .stroke_color(BinaryColor::On)
            .fill_color(BinaryColor::Off)
            .build();

        let step = SCREEN_WIDTH / TEMP_POINT_COUNT;
        let point_range_x = (0..SCREEN_WIDTH).rev().step_by(step);
        let mut points = [Point::default(); TEMP_POINT_COUNT];

        let max_temp = temperatures.iter().cloned().reduce(f32::max).unwrap();
        let min_temp = temperatures.iter().cloned().reduce(f32::min).unwrap();

        for (pos, x) in point_range_x.enumerate() {
            let temp = temperatures[TEMP_POINT_COUNT - pos - 1];
            let mut normalized_temp = temp - min_temp;
            // If this happens -> temperture have changed and we divide
            if (max_temp - min_temp) > TEMP_ACCURACY {
                normalized_temp /= max_temp - min_temp;
            };

            let display_temp = (normalized_temp) * ((SCREEN_GRAPH_HEIGHT as f32) - 1.0);

            points[pos] = Point {
                x: x as i32,
                y: SCREEN_HEIGHT as i32 - display_temp as i32 - 1,
            };
        }

        let polyline = Polyline::new(&points).into_styled(primitive_style);

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();

        let mut min_temp_string: heapless::String<16> = Default::default();
        let mut max_temp_string: heapless::String<16> = Default::default();
        let mut cur_temp_string: heapless::String<16> = Default::default();

        write!(&mut min_temp_string, "{:.2}", min_temp).ok();
        write!(&mut max_temp_string, "{:.2}", max_temp).ok();
        write!(&mut cur_temp_string, "{:.2}", temperatures[0]).ok();

        let display = ctx.local.display;
        display.clear();

        polyline.draw(display).ok();

        Text::new(&min_temp_string, Point { x: 0, y: 60 }, text_style)
            .draw(display)
            .ok();
        Text::new(&max_temp_string, Point { x: 0, y: 13 }, text_style)
            .draw(display)
            .ok();

        Text::new(&cur_temp_string, Point { x: 90, y: 13 }, text_style)
            .draw(display)
            .ok();

        display.flush().unwrap_or_else(|_| {
            // Redraw now if error occurs
            draw::spawn().ok();
        });
    }
}
