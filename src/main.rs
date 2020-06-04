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
use cortex_m_log::println;

#[cfg(debug_assertions)]
use cortex_m_semihosting::hprintln;

use cortex_m_log::d_println;
use cortex_m_rt::{entry, ExceptionFrame};

use stm32f3xx_hal as p_hal;

use p_hal::prelude::*;
use p_hal::stm32 as pac;

use pac::{I2C1, USART1};

use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;

use arrayvec::ArrayString;
use core::fmt;

use p_hal::time::{Hertz, U32Ext};

use cortex_m_rt as rt;

#[cfg(debug_assertions)]
// type DebugLog = cortex_m_log::printer::dummy::Dummy;
type DebugLog = cortex_m_log::printer::semihosting::Semihosting<
    cortex_m_log::modes::InterruptFree,
    cortex_m_semihosting::hio::HStdout,
>;
//type DebugLog = cortex_m_log::printer::itm::Itm<cortex_m_log::modes::InterruptFree>

type ImuI2cPortType = p_hal::i2c::I2c<
    I2C1,
    (
        p_hal::gpio::gpiob::PB8<p_hal::gpio::AF4>,
        p_hal::gpio::gpiob::PB9<p_hal::gpio::AF4>,
    ),
>;

type Usart1PortType = p_hal::serial::Serial<
    USART1,
    (
        p_hal::gpio::gpiob::PB6<p_hal::gpio::AF7>, //tx
        p_hal::gpio::gpiob::PB7<p_hal::gpio::AF7>, //rx
    ),
>;




// // cortex-m-rt calls this for serious faults.  can set a breakpoint to debug
// #[exception]
// fn HardFault(ef: &ExceptionFrame) -> ! {
//     panic!("{:#?}", ef);
// }

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
    Usart1PortType,
    impl DelayMs<u8> + DelayUs<u32>,
) {
    let dp = pac::Peripherals::take().unwrap();
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
        .sysclk(64.mhz()) //72 used to work?
        .pclk1(24.mhz()) // 24 works
        .freeze(&mut flash.acr);

    let delay_source = p_hal::delay::Delay::new(cp.SYST, clocks);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);

    // stm32f303 robotdyn:
    let mut user_led1 = gpioc
        .pc13
        .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
    user_led1.set_high().unwrap();

    let i2c_port = {
        let scl = gpiob
            .pb8
            .into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper)
            .into_af4(&mut gpiob.moder, &mut gpiob.afrh);

        let sda = gpiob
            .pb9
            .into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper)
            .into_af4(&mut gpiob.moder, &mut gpiob.afrh);

        p_hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), i2c_freq, clocks, &mut rcc.apb1)
    };

    let gps1_port = {
        let rx = gpiob.pb7.into_af7(&mut gpiob.moder, &mut gpiob.afrl);
        let tx = gpiob.pb6.into_af7(&mut gpiob.moder, &mut gpiob.afrl);
        p_hal::serial::Serial::usart1(dp.USART1, (tx,rx), 115200.bps(), clocks, &mut rcc.apb2)
    };

    (i2c_port, user_led1, gps1_port, delay_source)
}




#[entry]
fn main() -> ! {

    let (_i2c_port, mut user_led1, gps1_port, mut delay_source) = setup_peripherals();
    #[cfg(debug_assertions)]
    let mut log = get_debug_log();
    // let i2c_bus = shared_bus::CortexMBusManager::new(i2c_port);


    let _ = user_led1.set_low();
    d_println!(log, "ready!");
    delay_source.delay_ms(1u8);

    let (_gps_tx, gps_rx)  = gps1_port.split();
    let mut ublox = ublox_core::new_serial_driver(gps_rx);
    ublox.setup(&mut delay_source).unwrap();

    d_println!(log, "loopstart!");
    loop {
        // check GNSS
        if let Ok(msg_count) = ublox.handle_one_message() {
            //console_print(&mut po_tx, format_args!(">>> msg_count: {} \r\n", msg_count));
            if msg_count > 0 {
                if let Some(nav_pvt) = ublox.take_last_nav_pvt() {
                    d_println!(log, ">>> nav_pvt lat, lon: {}, {} \r\n",
                            nav_pvt.lat,
                            nav_pvt.lon);
                }
                if let Some(nav_dop) = ublox.take_last_nav_dop() {
                    d_println!(log, ">>> nav_dop {} \r\n", nav_dop.itow);
                }
                if let Some(mon_hw) = ublox.take_last_mon_hw() {
                    d_println!(log,">>> mon_hw jam: {} \r\n", mon_hw.jam_ind);
                }
            }
        }

        let _ = user_led1.toggle();
        delay_source.delay_ms(250u8);
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
            alpha: alpha,
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

        if range < 0.0 {
            -range
        } else {
            range
        }
    }

    pub fn update(&mut self, new_value: f64) -> f64 {
        //seed the EMWA with the initial value
        if self.local_min.is_nan() {
            self.local_min = new_value;
        }
        if self.local_max.is_nan() {
            self.local_max = new_value;
        }
        if self.average.is_nan() {
            self.average = new_value;
        }

        self.average = (self.alpha * new_value) + (1.0 - self.alpha) * self.average;

        // extrema fade toward average
        if new_value > self.local_max {
            self.local_max = new_value;
        } else if new_value > self.average {
            self.local_max = (self.alpha * new_value) + (1.0 - self.alpha) * self.local_max;
        }
        if new_value < self.local_min {
            self.local_min = new_value;
        } else if new_value < self.average {
            self.local_min = (self.alpha * new_value) + (1.0 - self.alpha) * self.local_min;
        }

        self.average
    }
}
