#![no_main]
#![no_std]

use bsp::{hal::usb::UsbBus, XOSC_CRYSTAL_FREQ};
use defmt_rtt as _;
use hal::Sio;
use panic_probe as _;
use rp_pico::{self as bsp, hal, hal::pac};
use usb_device::{class_prelude::UsbBusAllocator, prelude::*};
use zerocopy::{AsBytes, FromBytes, FromZeroes};

#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

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
    let _pins = hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    let bus = UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    );
    let bus_allocator = UsbBusAllocator::new(bus);
    let mut bulk = picompo::usb::BulkClass::<_, 1, 1>::new(&bus_allocator);
    let mut winusb = picompo::WinUsbClass;
    let mut dev = UsbDeviceBuilder::new(&bus_allocator, UsbVidPid(0x6666, 0xE233))
        .max_packet_size_0(64)
        .unwrap()
        .strings(&[StringDescriptors::new(LangID::EN_US)
            .product("UsbHello")
            .serial_number("UsbHello")
            .manufacturer("ArkEdge Space Inc.")])
        .unwrap()
        .build();

    let mut hello = UsbHello::new();

    loop {
        dev.poll(&mut [&mut winusb, &mut bulk]);
        bulk.poll_out(&mut hello);
        bulk.poll_in(&mut hello);
    }
}

#[derive(FromZeroes, FromBytes)]
#[repr(C)]
struct OutReport {
    data: [u8; 4],
}

#[derive(FromZeroes, AsBytes)]
#[repr(C)]
struct InReport {
    data: [u8; 4],
}

struct UsbHello {
    data: [u8; 4],
}

impl UsbHello {
    pub fn new() -> Self {
        Self { data: [0; 4] }
    }
}

impl picompo::usb::Service for UsbHello {
    fn handle_out(&mut self, ep: usize, data: &[u8]) {
        if ep == 0 {
            let Some(report) = OutReport::ref_from(data) else {
                defmt::println!("Invalid OUT: {=[u8]:02x}", data);
                return;
            };
            defmt::println!("OUT: {=[u8]:02x}", report.data);
            self.data.copy_from_slice(&report.data);
        }
    }

    fn handle_in(&mut self, ep: usize, write: impl FnOnce(&[u8])) {
        if ep == 0 {
            let report = InReport { data: self.data };
            //defmt::println!("IN: {=[u8]:02x}", report.data);
            write(report.as_bytes());
        }
    }
}
