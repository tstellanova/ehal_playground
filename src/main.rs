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

#[cfg(debug_assertions)]
use cortex_m_semihosting::{hprintln};

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

use stm32::I2C1;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_hal::blocking::delay::DelayMs;

// use ssd1306::prelude::*;
// use embedded_graphics::fonts::Font6x8;
// use embedded_graphics::prelude::*;
// use embedded_graphics::primitives::{Rect};
// use core::fmt;
// use arrayvec::ArrayString;

// use cortex_m::asm::bkpt;
use p_hal::time::{U32Ext, Hertz};

/// Sensors
use bno080::wrapper::BNO080;
use bno080::interface::{I2cInterface};
// use bno080::interface::{SpiInterface};
use em7180::USFS;
use bmp280_ehal::{BMP280};
use cortex_m::asm::bkpt;

#[macro_use]
extern crate cortex_m_rt;


#[cfg(debug_assertions)]
// type DebugLog = cortex_m_log::printer::dummy::Dummy;
type DebugLog = cortex_m_log::printer::semihosting::Semihosting<cortex_m_log::modes::InterruptFree, cortex_m_semihosting::hio::HStdout>;
//type DebugLog = cortex_m_log::printer::itm::Itm<cortex_m_log::modes::InterruptFree>


#[cfg(feature = "stm32f3x")]
type ImuI2cPortType = p_hal::i2c::I2c<I2C1,
    (p_hal::gpio::gpiob::PB8<p_hal::gpio::AF4>,
     p_hal::gpio::gpiob::PB9<p_hal::gpio::AF4>)
>;

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

// type ImuDriverType = bno080::wrapper::BNO080<I2cInterface<ImuI2cPortType>>;


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


#[cfg(feature = "stm32f3x")]
fn setup_peripherals() -> (
    ImuI2cPortType,
    impl OutputPin + ToggleableOutputPin,
    impl  DelayMs<u8>,
) {
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let i2c_freq: Hertz = 400.khz().into();
    // Set up the system clock
    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();

    // HSI: use default internal oscillator
    //let clocks = rcc.cfgr.freeze(&mut flash.acr);
    // HSE: external crystal oscillator must be connected
    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(72.mhz()) //72 works
        .pclk1(24.mhz()) // 24 works
        .freeze(&mut flash.acr);

    let  delay_source =  p_hal::delay::Delay::new(cp.SYST, clocks);

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);

    //stm32f334discovery:
    // let mut user_led1 = gpiob.pb6.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    // stm32f303 robodyn:
    let mut user_led1 = gpioc.pc13.into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
    user_led1.set_high().unwrap();

    // setup i2c1 and imu driver
    let scl = gpiob.pb8
        .into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper)
        .into_af4(&mut gpiob.moder, &mut gpiob.afrh);

    let sda = gpiob.pb9
        .into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper)
        .into_af4(&mut gpiob.moder, &mut gpiob.afrh);

    let i2c_port = p_hal::i2c::I2c::i2c1(
        dp.I2C1, (scl, sda), i2c_freq, clocks, &mut rcc.apb1);

    (i2c_port, user_led1, delay_source)
}


#[cfg(feature = "stm32f4x")]
fn setup_peripherals() ->  (
    ImuI2cPortType,
    impl OutputPin + ToggleableOutputPin,
    impl  DelayMs<u8>
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
        .use_hse(8.mhz()) //f4 discovery board has 8 MHz crystal for HSE
        .sysclk(128.mhz())
        .pclk1(48.mhz())
        // .pclk2(48.mhz())
        .freeze();

    let delay_source =  p_hal::delay::Delay::new(cp.SYST, clocks);

    // let hclk = clocks.hclk();
    // let rng_clk = clocks.pll48clk().unwrap_or(0u32.hz());
    // let pclk1 = clocks.pclk1();
    // d_println!(get_debug_log(), "hclk: {} /16: {} pclk1: {} rng_clk: {}", hclk.0, hclk.0 / 16, pclk1.0, rng_clk.0);

    let gpiob = dp.GPIOB.split();
    //let gpioc = dp.GPIOC.split();
    let gpiod = dp.GPIOD.split();

    //let user_led1 = gpioc.pc13.into_push_pull_output(); //f401CxUx
    let user_led1 = gpiod.pd12.into_push_pull_output(); //f4discovery

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

    (i2c_port, user_led1, delay_source)
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
// const SCREEN_WIDTH: i32 = 128;
// const SCREEN_HEIGHT: i32 = 32;
// const MAX_BAR_HEIGHT: i32 = (SCREEN_HEIGHT - BAR_VERT_INSET);
// const MAX_BAR_HEIGHT_F64: f64 = MAX_BAR_HEIGHT as f64;
// const BAR_WIDTH: i32 = 2;
// const AVG_BAR_HEIGHT: i32 = BAR_VERT_INSET + (MAX_BAR_HEIGHT / 2);




#[entry]
fn main() -> ! {

    let (i2c_port, mut user_led1, mut delay_source) =
        setup_peripherals();
    #[cfg(debug_assertions)]
    let mut log = get_debug_log();
    let i2c_bus = shared_bus::CortexMBusManager::new(i2c_port);

    let mut barometer = BMP280::new(i2c_bus.acquire()).unwrap();
    barometer.reset();

    let mut ahrs = USFS::new_inv_usfs_03(i2c_bus.acquire(),
                                         em7180::EM7180_DEFAULT_ADDRESS,
                                         0, //unused for now
                                         false).unwrap();

    // let mut format_buf = ArrayString::<[u8; 20]>::new();
    // let mut disp: GraphicsMode<_> = ssd1306::Builder::new().connect_i2c(i2c_bus.acquire()).into();
    // disp.init().unwrap();
    // disp.set_rotation(DisplayRotation::Rotate0).unwrap();
    // disp.flush().unwrap();

    // let spi_iface = bno080::interface::SpiInterface::new(
    //     spi_port, csn, hintn, waken, rst);
    // let imu_driver = BNO080::new_with_interface(spi_iface);

    let i2c_iface = I2cInterface::new(i2c_bus.acquire(), bno080::interface::i2c::DEFAULT_ADDRESS);
    let mut imu_driver = BNO080::new_with_interface(i2c_iface);

    let res = imu_driver.init(&mut delay_source);
    if res.is_ok() {
        let res2 = imu_driver.enable_rotation_vector(50);
        hprintln!("rotv: {:?}", res2).unwrap();
    }
    else {
        hprintln!("init failed: {:?}", res).unwrap();
        bkpt();
    }

    let _ = user_led1.set_low();
    d_println!(log, "ready!");
    delay_source.delay_ms(1u8);

    //let mut tracker = SensorValueTracker::new(0.1);
    //let mut xpos: i32  = 0;

    loop {
        imu_driver.handle_all_messages(&mut delay_source);
        let abs_press = 10.0 * barometer.pressure_one_shot();
        hprintln!("press: {:.2}", abs_press).unwrap();

        if ahrs.quat_available() {
            let quat = ahrs.read_sentral_quat_qata().unwrap();
            let qx = quat[0];
            hprintln!("qx: {:.2}", qx).unwrap();
        }

        //let quat = imu_driver.read_quaternion().unwrap();
        //let qx = quat[0];
        //d_println!(log, "{:.6} ",qx);
        //clear bar area

        // qx += 0.001;
        //
        // disp.draw( Rect::new(Coord::new(0, 0),
        //                      Coord::new(SCREEN_WIDTH, SCREEN_HEIGHT))
        //     .with_fill(Some(0u8.into())).into_iter());
        //
        // //overdraw the label
        // format_buf.clear();
        // if fmt::write(&mut format_buf, format_args!("{:.6}", qx)).is_ok() {
        //     disp.draw(
        //         Font6x8::render_str(format_buf.as_str())
        //             .with_stroke(Some(1u8.into()))
        //             .translate(Coord::new(20, SCREEN_HEIGHT  / 2))
        //             .into_iter(),
        //     );
        // }
        // disp.flush().unwrap();

        // xpos = xpos + BAR_WIDTH;
        // if xpos > SCREEN_WIDTH { xpos = 0; }

        let _ = user_led1.toggle();
        delay_source.delay_ms(100u8);
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


