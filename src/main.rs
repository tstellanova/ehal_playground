#![no_std]
#![no_main]

// pick a panicking behavior
use panic_rtt_core::{self, rprintln, rtt_init_print};

use cortex_m_rt as rt;
use rt::{entry};

use stm32f3xx_hal as p_hal;

use p_hal::prelude::*;
use p_hal::stm32 as pac;

use pac::{I2C1, USART1};

use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;


use p_hal::time::{Hertz, U32Ext};


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

    // let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
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
        p_hal::serial::Serial::usart1(dp.USART1, (tx,rx), 9600.bps(), clocks, &mut rcc.apb2)
    };

    (i2c_port, user_led1, gps1_port, delay_source)
}




#[entry]
fn main() -> ! {

    rtt_init_print!(NoBlockTrim);
    rprintln!("-- > MAIN --");

    let (_i2c_port, mut user_led1, gps1_port, mut delay_source) = setup_peripherals();

    let _ = user_led1.set_low();
    rprintln!("ready!");
    delay_source.delay_ms(1u8);

    let ( _gps_tx,  gps_rx)  = gps1_port.split();
    let mut ublox = ublox_core::new_serial_driver(gps_rx);
    ublox.setup(&mut delay_source).unwrap();

    rprintln!("loopstart!");
    loop {
        // check GNSS
        if let Ok(msg_count) = ublox.handle_one_message() {
            //console_print(&mut po_tx, format_args!(">>> msg_count: {} \r\n", msg_count));
            if msg_count > 0 {
                if let Some(nav_pvt) = ublox.take_last_nav_pvt() {
                    rprintln!(">>> nav_pvt lat, lon: {}, {} \r\n",
                            nav_pvt.lat,
                            nav_pvt.lon);
                }
                if let Some(nav_dop) = ublox.take_last_nav_dop() {
                    rprintln!(">>> nav_dop {} \r\n", nav_dop.itow);
                }
                if let Some(mon_hw) = ublox.take_last_mon_hw() {
                    rprintln!(">>> mon_hw jam: {} \r\n", mon_hw.jam_ind);
                }
            }
        }

        let _ = user_led1.toggle();
    }
}

