#![no_main]
#![no_std]

extern crate panic_halt;
extern crate stm32f4xx_hal as hal;

mod drawables;

/// Peripheral Access Crate for our device
pub use hal::pac;

#[rtic::app(device = crate::pac, peripherals = true, dispatchers = [SPI5])]
mod app {
    use core::cell::RefCell;
    use cortex_m::{asm::wfi, interrupt::Mutex};
    use embedded_storage::nor_flash::NorFlash;
    use hal::{
        flash::{FlashExt, LockedFlash},
        gpio::*,
        i2c::I2c1,
        prelude::*,
    };
    use heapless::spsc::Queue;
    use lm75::Lm75;
    use shared_bus::{BusManager, I2cProxy};
    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, Ssd1306};

    use crate::drawables::prelude::*;

    pub type MainI2cSCL = PB8<AF4<OpenDrain>>;
    pub type MainI2cSDA = PB9<AF4<OpenDrain>>;
    pub type MainI2C = I2c1<(MainI2cSCL, MainI2cSDA)>;
    pub type SharedI2C = BusManager<Mutex<RefCell<MainI2C>>>;
    pub type I2CProxy = I2cProxy<'static, Mutex<RefCell<MainI2C>>>;

    const TEMP_PROBE_ADDRESS: u8 = 0x48;
    const TEMP_POINT_COUNT: usize = 60 * 30; // So 30 minutes avg
    const TEMP_QUEUE_SIZE: usize = TEMP_POINT_COUNT + 1;
    const TEMP_UPDATE_TIME_MS: u32 = 1000;

    const PLOT_POINT_COUNT: usize = 30;
    const PLOT_QUEUE_SIZE: usize = PLOT_POINT_COUNT + 1;

    #[shared]
    struct Shared {
        temp_data: Queue<f32, TEMP_QUEUE_SIZE>,
        plot_data: Queue<f32, PLOT_QUEUE_SIZE>,

        locked_flash: LockedFlash,
    }

    #[local]
    struct Local {
        display: Ssd1306<
            I2CInterface<I2CProxy>,
            DisplaySize128x64,
            BufferedGraphicsMode<DisplaySize128x64>,
        >,
        temp_probe: Lm75<I2CProxy, lm75::ic::Pct2075>,
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

        // Flash
        let locked_flash = LockedFlash::new(dp.FLASH);

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

        let temp_probe = Lm75::new_pct2075(managed_i2c.acquire_i2c(), TEMP_PROBE_ADDRESS);

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

        load_temperature::spawn().ok();
        temperature_take::spawn().ok();
        draw::spawn().ok();

        (
            Shared {
                temp_data: Queue::new(),
                plot_data: Queue::new(),
                locked_flash,
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

    #[task(shared=[locked_flash, plot_data])]
    fn load_temperature(ctx: load_temperature::Context) {
        let temp_size = core::mem::size_of::<f32>();

        (ctx.shared.locked_flash, ctx.shared.plot_data).lock(|f, d| {
            let size = f.len();
            let start_sector = size - temp_size * PLOT_POINT_COUNT;

            let flash_content = &f.read()[start_sector..];

            // Clear previous data
            while !d.is_empty() {
                d.dequeue();
            }

            for i in 0..PLOT_POINT_COUNT {
                let offset = i * temp_size;
                let mut raw_data = [0u8; 4];
                raw_data.copy_from_slice(&flash_content[offset..(offset + temp_size)]);

                let val = f32::from_be_bytes(raw_data);
                d.enqueue(val).ok();
            }
        });
    }

    #[task(shared=[locked_flash, plot_data])]
    fn save_temperature(ctx: save_temperature::Context) {
        let temp_size = core::mem::size_of::<f32>();

        (ctx.shared.locked_flash, ctx.shared.plot_data).lock(|f, d| {
            let size = f.len();
            let start_sector = size - temp_size * PLOT_POINT_COUNT;

            let mut unlocked_flash = f.unlocked();
            NorFlash::erase(&mut unlocked_flash, start_sector as u32, size as u32).unwrap();

            for (i, temp) in d.iter().enumerate() {
                let offset = (i * temp_size) as u32;
                let raw_data = temp.to_be_bytes();

                NorFlash::write(&mut unlocked_flash, start_sector as u32 + offset, &raw_data)
                    .unwrap();
            }
        });
    }

    #[task(local = [temp_probe], shared = [temp_data])]
    fn temperature_take(mut ctx: temperature_take::Context) {
        temperature_take::spawn_after(TEMP_UPDATE_TIME_MS.millis()).ok();

        let Ok(temp) = ctx.local.temp_probe.read_temperature() else {
            return;
        };

        ctx.shared.temp_data.lock(|q| {
            // If queue is full send new average value to plot
            if q.is_full() {
                // Get avg temperature
                let avg: f32 = q
                    .iter()
                    .cloned()
                    .reduce(|accum, item| accum + item)
                    .unwrap()
                    / TEMP_POINT_COUNT as f32;
                temperature_plot::spawn(avg).ok();

                // Clear queue
                while !q.is_empty() {
                    q.dequeue();
                }
            }
            q.enqueue(temp).ok();
        });
    }

    #[task(shared = [plot_data])]
    fn temperature_plot(mut ctx: temperature_plot::Context, new_temp: f32) {
        ctx.shared.plot_data.lock(|q| {
            if q.is_full() {
                q.dequeue();
            }
            q.enqueue(new_temp).ok();
        });

        save_temperature::spawn().ok();
    }

    #[task(local = [display], shared = [plot_data, temp_data])]
    fn draw(mut ctx: draw::Context) {
        use embedded_graphics::mono_font::iso_8859_5::FONT_6X10;
        use embedded_graphics::mono_font::MonoTextStyleBuilder;
        use embedded_graphics::pixelcolor::BinaryColor;
        use embedded_graphics::prelude::*;
        use embedded_graphics::primitives::PrimitiveStyleBuilder;

        draw::spawn_after(TEMP_UPDATE_TIME_MS.millis()).ok();

        let mut temperatures = [0.0; PLOT_POINT_COUNT];
        ctx.shared.plot_data.lock(|q| {
            for (i, v) in q.iter().cloned().enumerate() {
                temperatures[i] = v;
            }
        });

        let current_temp = ctx
            .shared
            .temp_data
            .lock(|q| q.iter().cloned().last().unwrap());

        let primitive_style = PrimitiveStyleBuilder::new()
            .stroke_width(1)
            .stroke_color(BinaryColor::On)
            .fill_color(BinaryColor::Off)
            .build();

        let max_temp = temperatures.iter().cloned().reduce(f32::max).unwrap();
        let min_temp = temperatures.iter().cloned().reduce(f32::min).unwrap();

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();

        let display = ctx.local.display;
        display.clear();

        let plot = Plot::new(primitive_style)
            .with_position(Point::new(0, 16))
            .with_size(Size::new(128, 64 - 16));

        plot.generate(&temperatures).draw(display).ok();

        TemperatureText::new(Point { x: 0, y: 60 }, text_style)
            .with_temperature(min_temp)
            .draw(display)
            .ok();

        TemperatureText::new(Point { x: 0, y: 13 }, text_style)
            .with_temperature(max_temp)
            .draw(display)
            .ok();

        TemperatureText::new(Point { x: 90, y: 13 }, text_style)
            .with_temperature(current_temp)
            .draw(display)
            .ok();

        display.flush().unwrap_or_else(|_| {
            // Redraw now if error occurs
            draw::spawn().ok();
        });
    }
}
