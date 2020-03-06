#![no_std]
#![no_main]

// pick a panicking behavior
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support

#[cfg(not(debug_assertions))]
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
#[cfg(debug_assertions)]
extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

#[cfg(debug_assertions)]
use cortex_m_log::printer::semihosting;

#[cfg(debug_assertions)]
use cortex_m_log::{println};

use cortex_m_log::{d_println};
use cortex_m_rt::{entry, ExceptionFrame};

#[cfg(feature = "stm32h7x")]
use stm32h7xx_hal as p_hal;

#[cfg(feature = "stm32f4x")]
use stm32f4xx_hal as p_hal;

#[cfg(feature = "stm32f3x")]
use stm32f3xx_hal as p_hal;

use p_hal::prelude::*;
use p_hal::stm32;
use rand_core::RngCore;

use stm32::I2C1;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_hal::blocking::delay::DelayMs;


use ssd1306::prelude::*;
use embedded_graphics::fonts::Font6x8;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Rect};
// use cortex_m::asm::bkpt;

use core::fmt;
use arrayvec::ArrayString;

use bno080::wrapper::BNO080;
use bno080::interface::{I2cInterface};
// use bno080::interface::{SpiInterface};

#[macro_use]
extern crate cortex_m_rt;


#[cfg(debug_assertions)]
// type DebugLog = cortex_m_log::printer::dummy::Dummy;
type DebugLog = cortex_m_log::printer::semihosting::Semihosting<cortex_m_log::modes::InterruptFree, cortex_m_semihosting::hio::HStdout>;
//type DebugLog = cortex_m_log::printer::itm::Itm<cortex_m_log::modes::InterruptFree>


// #[cfg(feature = "stm32f3x")]
// type ImuI2cPortType = p_hal::i2c::I2c<I2C1,
//     (p_hal::gpio::gpiob::PB8<p_hal::gpio::AF4>,
//      p_hal::gpio::gpiob::PB9<p_hal::gpio::AF4>)
// >;

#[cfg(feature = "stm32f4x")]
pub type ImuI2cPortType = p_hal::i2c::I2c<I2C1,
    (p_hal::gpio::gpiob::PB8<p_hal::gpio::AlternateOD<p_hal::gpio::AF4>>,
     p_hal::gpio::gpiob::PB9<p_hal::gpio::AlternateOD<p_hal::gpio::AF4>>)
>;

#[cfg(feature = "stm32h7x")]
pub type ImuI2cPortType = p_hal::i2c::I2c<I2C1,
    (p_hal::gpio::gpiob::PB8<p_hal::gpio::Alternate<p_hal::gpio::AF4>>,
     p_hal::gpio::gpiob::PB9<p_hal::gpio::Alternate<p_hal::gpio::AF4>>)
>;

// cortex-m-rt is setup to call DefaultHandler for a number of fault conditions
// // we can override this in debug mode for handy debugging
// #[exception]
// fn DefaultHandler(_irqn: i16) {
//     bkpt();
//     d_println!(get_debug_log(), "IRQn = {}", _irqn);
// }

type ImuDriverType = bno080::wrapper::BNO080<I2cInterface<ImuI2cPortType>>;


// // cortex-m-rt calls this for serious faults.  can set a breakpoint to debug
#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}






/// Used in debug builds to provide a logging outlet
#[cfg(debug_assertions)]
fn get_debug_log() -> DebugLog {
    // cortex_m_log::printer::Dummy::new()
    semihosting::InterruptFree::<_>::stdout().unwrap()
}


// #[cfg(feature = "stm32f3x")]
// fn setup_peripherals() -> (
//     ImuI2cPortType,
//     impl OutputPin + ToggleableOutputPin,
//     impl  DelayMs<u8>,
// ) {
//     let dp = stm32::Peripherals::take().unwrap();
//     let cp = cortex_m::Peripherals::take().unwrap();
//
//     // Set up the system clock
//     let mut rcc = dp.RCC.constrain();
//     let mut flash = dp.FLASH.constrain();
//
//     // HSI: use default internal oscillator
//     //let clocks = rcc.cfgr.freeze(&mut flash.acr);
//     // HSE: external crystal oscillator must be connected
//     let clocks = rcc
//         .cfgr
//         .use_hse(8_000_000.hz())
//         .sysclk(72_000_000.hz())
//         .pclk1(36.mhz())
//         .pclk2(36.mhz())
//         .freeze(&mut flash.acr);
//
//     let  delay_source =  p_hal::delay::Delay::new(cp.SYST, clocks);
//
//     let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
//     let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);
//
//     //stm32f334discovery
//     // let mut user_led1 = gpiob.pb6.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
//     // stm32f303 robodyn
//     let mut user_led1 = gpioc.pc13.into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
//     user_led1.set_high().unwrap();
//
//     // setup i2c1 and imu driver
//     let scl = gpiob.pb8
//         .into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper)
//         .into_af4(&mut gpiob.moder, &mut gpiob.afrh);
//
//     let sda = gpiob.pb9
//         .into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper)
//         .into_af4(&mut gpiob.moder, &mut gpiob.afrh);
//
//     let i2c_port = p_hal::i2c::I2c::i2c1(
//         dp.I2C1, (scl, sda), 1000.khz(), clocks, &mut rcc.apb1);
//
//
//     (i2c_port, user_led1, delay_source)
//
// }


#[cfg(feature = "stm32f4x")]
fn setup_peripherals() ->  (
    ImuI2cPortType,
    impl OutputPin + ToggleableOutputPin,
    impl  DelayMs<u8>,
    p_hal::rng::Rng
    ) {

    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Set up the system clock
    let rcc = dp.RCC.constrain();
    // HSI: use default internal oscillator
    //let clocks = rcc.cfgr.freeze();
    // HSE: external crystal oscillator must be connected
    let clocks = rcc
        .cfgr
        .use_hse(8_000_000.hz())
        .sysclk(120_000_000.hz())
        .pclk1(36.mhz())
        .pclk2(36.mhz())
        .freeze();

    // The following will invoke the RNG_CLK assert
    // let clocks = rcc
    //     .cfgr
    //     .use_hse(8_000_000.hz())
    //     .sysclk(8.mhz())
    //     .freeze();

    let delay_source =  p_hal::delay::Delay::new(cp.SYST, clocks);


    let hclk = clocks.hclk();
    let rng_clk = clocks.pll48clk().unwrap_or(0u32.hz());
    let pclk1 = clocks.pclk1();
    d_println!(get_debug_log(), "hclk: {} /16: {} pclk1: {} rng_clk: {}", hclk.0, hclk.0 / 16, pclk1.0, rng_clk.0);

    let rand_source = dp.RNG.constrain(clocks);

    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let user_led1 = gpioc.pc13.into_push_pull_output(); //f401CxUx

    // setup i2c1
    // NOTE: stm32f401CxUx board lacks external pull-ups on i2c pins
    // NOTE: eg f407 discovery board already has external pull-ups
    // NOTE: sensor breakout boards may have their own pull-ups: check carefully
    let scl = gpiob.pb8
        .into_alternate_af4()
        //.internal_pull_up(true)
        .set_open_drain();

    let sda = gpiob.pb9
        .into_alternate_af4()
        //.internal_pull_up(true)
        .set_open_drain();
    let i2c_port = p_hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), 1000.khz(), clocks);

    (i2c_port, user_led1, delay_source, rand_source)
}

// #[cfg(feature = "stm32h7x")]
// fn setup_peripherals() ->  (
//     ImuI2cPortType,
//     impl OutputPin + ToggleableOutputPin,
//     impl  DelayMs<u8>,
// ) {
//     let dp = stm32::Peripherals::take().unwrap();
//     let cp = cortex_m::Peripherals::take().unwrap();
//
//
//     // Set up the system clock
//     let rcc = dp.RCC.constrain();
//
//     let pwr = dp.PWR.constrain();
//     let vos = pwr.freeze();
//
//     //use the existing sysclk
//     let mut ccdr = rcc.freeze(vos, &dp.SYSCFG);
//     let clocks = ccdr.clocks;
//
//     let delay_source =  p_hal::delay::Delay::new(cp.SYST, clocks);
//
//     let gpiob = dp.GPIOB.split(&mut ccdr.ahb4);
//
//     let user_led1 = gpiob.pb0.into_push_pull_output(); //h743 discovery
//
//
//     // setup i2c1
//     // NOTE:  f407 discovery board already has external pull-ups
//     let scl = gpiob.pb8
//         .into_alternate_af4()
//         .set_open_drain();
//
//     let sda = gpiob.pb9
//         .into_alternate_af4()
//         .set_open_drain();
//     let i2c_port = p_hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), &ccdr);
//
//     (i2c_port, user_led1, delay_source)
// }

// const LABEL_TEXT_HEIGHT: i32 = 5;
// const BAR_VERT_INSET: i32 = 5;
const SCREEN_WIDTH: i32 = 128;
const SCREEN_HEIGHT: i32 = 32;
// const MAX_BAR_HEIGHT: i32 = (SCREEN_HEIGHT - BAR_VERT_INSET);
//const MAX_BAR_HEIGHT_F64: f64 = MAX_BAR_HEIGHT as f64;
// const BAR_WIDTH: i32 = 2;
//const AVG_BAR_HEIGHT: i32 = BAR_VERT_INSET + (MAX_BAR_HEIGHT / 2);

#[entry]
fn main() -> ! {

    let (i2c_port, mut user_led1, mut delay_source, mut rand_source) = setup_peripherals();
    #[cfg(debug_assertions)]
    let mut log = get_debug_log();
    let i2c_bus = shared_bus::CortexMBusManager::new(i2c_port);

    let mut disp: GraphicsMode<_> = ssd1306::Builder::new().connect_i2c(i2c_bus.acquire()).into();
    disp.init().unwrap();
    disp.set_rotation(DisplayRotation::Rotate0).unwrap();
    disp.flush().unwrap();


    // let spi_iface = bno080::interface::SpiInterface::new(
    //     spi_port, csn, hintn, waken, rst);
    // let imu_driver = BNO080::new_with_interface(spi_iface);

    let iface = I2cInterface::new(i2c_bus.acquire(), bno080::interface::i2c::DEFAULT_ADDRESS);
    let mut imu_driver = BNO080::new_with_interface(iface);
    // imu_driver.enable_debugging(debug_func);
    let res = imu_driver.init(delay_source);
    if res.is_ok() {
        res = imu_driver.enable_rotation_vector(IMU_REPORTING_INTERVAL_MS);
    }

    let _ = user_led1.set_low();
    d_println!(log, "ready!");
    delay_source.delay_ms(1u8);

    let mut tracker = SensorValueTracker::new(0.1);
    let mut xpos: i32  = 0;
    let mut format_buf = ArrayString::<[u8; 20]>::new();
    let mut rbuf: [u8; 8] = [0; 8];

    //let mut read_count: i32 = 0;
    loop {
        let rand_val = rand_source.next_u32();
        imu_driver.handle_one_message();

        //d_println!(log, "{} {:.2}",read_count, _pres);

        //clear bar area
        disp.draw( Rect::new(Coord::new(0, 0),
                             Coord::new(SCREEN_WIDTH, SCREEN_HEIGHT))
            .with_fill(Some(0u8.into())).into_iter());
        //
        // //d_println!(log, "{:.2} / {:.2} ", abs_press, avg_press);
        // let ypix = (MAX_BAR_HEIGHT * rand_val) / core::i32::MAX;
        // let ypos = MAX_BAR_HEIGHT - ypix;
        //
        // // draw new bar
        // disp.draw( Rect::new(Coord::new(xpos, ypos), Coord::new(xpos + BAR_WIDTH, SCREEN_HEIGHT)).with_fill(Some(1u8.into())).into_iter());

        //overdraw the label
        format_buf.clear();
        if fmt::write(&mut format_buf, format_args!("{}", rand_val)).is_ok() {
            disp.draw(
                Font6x8::render_str(format_buf.as_str())
                    .with_stroke(Some(1u8.into()))
                    .translate(Coord::new(20, SCREEN_HEIGHT  / 2))
                    .into_iter(),
            );
        }
        disp.flush().unwrap();

        // xpos = xpos + BAR_WIDTH;
        // if xpos > SCREEN_WIDTH { xpos = 0; }


        let _ = user_led1.toggle();
        delay_source.delay_ms(250u8);
    }

}


/// Implements exponential weighted moving average of sensor readings,
/// including exponentially fading minimum and maximum
pub struct SensorValueTracker {
    /// recent minimum value (not global minimum)
    local_min: f64,
    /// recent maximum value (not global maximum)
    local_max: f64,
    /// exponentially weighted moving average
    average: f64,
    /// weighting factor-- bigger alpha causes faster fade of old values
    alpha: f64,
}

impl SensorValueTracker {
    pub fn new(alpha: f64) -> Self {
        Self {
            local_min: core::f64::NAN,
            local_max: core::f64::NAN,
            average: core::f64::NAN,
            alpha: alpha
        }
    }

    pub fn average(&self) -> f64 {
        self.average
    }

    pub fn range(&self) -> f64 {
        let mut range = self.local_max - self.local_min;
        if range == 0.0 {
            range = 1.0;
        }

        if range < 0.0 { -range }
        else { range }
    }

    pub fn update(&mut self, new_value: f64) -> f64 {
        //seed the EMWA with the initial value
        if self.local_min.is_nan() { self.local_min = new_value; }
        if self.local_max.is_nan() { self.local_max = new_value; }
        if self.average.is_nan() { self.average = new_value; }

        self.average = (self.alpha * new_value) + (1.0 - self.alpha) * self.average;

        // extrema fade toward average
        if new_value > self.local_max { self.local_max = new_value; }
        else if new_value > self.average {
            self.local_max = (self.alpha * new_value) + (1.0 - self.alpha) * self.local_max;
        }
        if new_value < self.local_min { self.local_min = new_value; }
        else if new_value < self.average {
            self.local_min = (self.alpha * new_value) + (1.0 - self.alpha) * self.local_min;
        }

        self.average
    }

}


