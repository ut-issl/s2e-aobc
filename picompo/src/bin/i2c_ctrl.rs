#![no_main]
#![no_std]

use bsp::{hal::Sio, XOSC_CRYSTAL_FREQ};
use defmt_rtt as _;
use embedded_hal::i2c::I2c as _;
use hal::{fugit::RateExtU32 as _, I2C};
use panic_probe as _;
use rp_pico::{
    self as bsp, hal,
    hal::{pac, prelude::*},
};

#[bsp::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
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

    let sio = Sio::new(pac.SIO);
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut i2c = I2C::new_controller(
        pac.I2C1,
        pins.gpio10.into_function(),
        pins.gpio11.into_function(),
        400u32.kHz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );

    cortex_m::asm::delay(60_000_000);
    let init = [0x11, 0x01, 0x00, 0x10, 0x00, 0x00, 0x8B, 0x80, 0xC0];
    let get_temp = [0x11, 0x87, 0x03, 0xFE, 0xEB, 0xC0];
    let get_speed = [0x11, 0x87, 0x15, 0x49, 0x9E, 0xC0];

    loop {
        defmt::println!("Controller is Writing: init");
        let mut tlm = [0u8; 15];
        if let Err(e) = i2c.write_read(0x26u8, &init, &mut tlm) {
            defmt::println!("I2C Controller Write Error: {:?}", e);
            cortex_m::asm::delay(60_000_000);
        }
        defmt::println!("init: {=[u8]:02x}", tlm);
        cortex_m::asm::delay(60_000_000);

        defmt::println!("Controller is Writing: get_temp");
        let mut tlm = [0u8; 15];
        if let Err(e) = i2c.write_read(0x26u8, &get_temp, &mut tlm) {
            defmt::println!("I2C Controller Write Error: {:?}", e);
            cortex_m::asm::delay(60_000_000);
        }
        defmt::println!("get_temp: {=[u8]:02x}", tlm);
        cortex_m::asm::delay(60_000_000);

        defmt::println!("Controller is Writing: get_speed");
        let mut tlm = [0u8; 15];
        if let Err(e) = i2c.write_read(0x26u8, &get_speed, &mut tlm) {
            defmt::println!("I2C Controller Write Error: {:?}", e);
            cortex_m::asm::delay(60_000_000);
        }
        defmt::println!("get_speed: {=[u8]:02x}", tlm);
        cortex_m::asm::delay(60_000_000);
    }
}
