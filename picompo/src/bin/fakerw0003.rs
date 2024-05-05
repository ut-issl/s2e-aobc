#![no_main]
#![no_std]

use bsp::{
    hal::{usb::UsbBus, Sio},
    XOSC_CRYSTAL_FREQ,
};
use defmt_rtt as _;
use embedded_hal::digital::InputPin;
use hal::{
    gpio::{
        self,
        bank0::{Gpio1, Gpio5, Gpio9},
        FunctionSioInput, PullUp,
    },
    I2C,
};
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

struct AddressSelector {
    x: gpio::Pin<Gpio1, FunctionSioInput, PullUp>,
    y: gpio::Pin<Gpio5, FunctionSioInput, PullUp>,
    z: gpio::Pin<Gpio9, FunctionSioInput, PullUp>,
}

impl AddressSelector {
    pub fn select(&mut self) -> Option<u8> {
        let x = self.x.is_low().unwrap();
        let y = self.y.is_low().unwrap();
        let z = self.z.is_low().unwrap();
        defmt::println!("x: {}, y: {}, z: {}", x, y, z);
        match (x, y, z) {
            (true, false, false) => Some(0x24),
            (false, true, false) => Some(0x25),
            (false, false, true) => Some(0x26),
            _ => None,
        }
    }
}

fn hex(byte: u8) -> [u8; 2] {
    let hex = b"0123456789ABCDEF";
    [hex[(byte >> 4) as usize], hex[(byte & 0x0F) as usize]]
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
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut address_selector = AddressSelector {
        x: pins.gpio1.into_pull_up_input(),
        y: pins.gpio5.into_pull_up_input(),
        z: pins.gpio9.into_pull_up_input(),
    };
    let self_address = address_selector.select().unwrap_or(0x00);

    let product_name = cortex_m::singleton!(: [u8; 18] = [0; 18]).unwrap();
    product_name.copy_from_slice(b"Fake RW-0.003 0x??");
    let addr_hex_offset = product_name.len() - 2;
    product_name[addr_hex_offset..].copy_from_slice(&hex(self_address));

    let product_name = core::str::from_utf8(product_name).unwrap();

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
    let mut dev = UsbDeviceBuilder::new(&bus_allocator, UsbVidPid(0x6666, 0x0222))
        .max_packet_size_0(64)
        .unwrap()
        .strings(&[StringDescriptors::new(LangID::EN_US)
            .product(product_name)
            .serial_number(product_name)
            .manufacturer("ArkEdge Space Inc.")])
        .unwrap()
        .build();

    let i2c_sda = pins.gpio16.into_function();
    let i2c_scl = pins.gpio17.into_function();
    let i2c0 = I2C::new_peripheral_event_iterator(
        pac.I2C0,
        i2c_sda,
        i2c_scl,
        &mut pac.RESETS,
        self_address,
    );
    let mut rw0003 = Rw0003::new();
    let mut sinclair = picompo::sinclair::NspSlipI2c::new(i2c0, self_address, 0x11);

    loop {
        dev.poll(&mut [&mut winusb, &mut bulk]);
        bulk.poll_out(&mut rw0003);
        sinclair.poll(&mut rw0003);
        bulk.poll_in(&mut rw0003);
    }
}

#[derive(FromZeroes, FromBytes)]
#[repr(C)]
struct OutReport {
    temperature: [u8; 4],
    speed: [u8; 4],
}

#[derive(FromZeroes, AsBytes)]
#[repr(C)]
struct InReport {
    is_initialized: u8,
    mode: u8,
    _padding: [u8; 2],
    torque: [u8; 4],
}

impl InReport {
    fn new(is_initialized: bool, mode: u8, torque: [u8; 4]) -> Self {
        Self {
            is_initialized: is_initialized as u8,
            mode,
            _padding: [0; 2],
            torque,
        }
    }
}

struct Rw0003 {
    is_initialized: bool,
    mode: u8,
    torque: [u8; 4],
    temperature: [u8; 4],
    speed: [u8; 4],
}

impl Rw0003 {
    fn new() -> Self {
        Self {
            is_initialized: false,
            mode: 0,
            torque: [0xBE; 4],
            temperature: [0xDE; 4],
            speed: [0xAD; 4],
        }
    }

    fn init(&mut self, args: &[u8]) {
        if args == [0x00, 0x10, 0x00, 0x00] {
            self.is_initialized = true;
            defmt::println!("Initialized");
        }
    }

    fn read_file(&mut self, args: &[u8], reply: impl FnOnce(&[u8])) {
        // READ FILE
        if !self.is_initialized {
            defmt::println!("READ FILE: Not initialized");
            return;
        }
        if args.len() != 1 {
            defmt::println!("Invalid body length for READ FILE: {}", args.len());
            return;
        }
        let file_addr = args[0];
        match file_addr {
            0x03 => {
                // Temperature
                defmt::println!("Get temperature");
                let mut reply_body = [0u8; 5];
                reply_body[0] = 0x03; // Temperature
                reply_body[1..].copy_from_slice(&self.temperature);
                reply(&reply_body);
            }
            0x15 => {
                // Speed
                defmt::println!("Get speed");
                let mut reply_body = [0u8; 5];
                reply_body[0] = 0x15; // Speed
                reply_body[1..].copy_from_slice(&self.speed);
                reply(&reply_body);
            }
            _ => defmt::println!("Unsupported read file address: {:02x}", file_addr),
        }
    }

    fn write_file(&mut self, args: &[u8], _reply: impl FnOnce(&[u8])) {
        // 本来は reply を返すべきだが、AOBC では未使用なので省略している
        if !self.is_initialized {
            defmt::println!("WRITE FILE: Not initialized");
            return;
        }
        if args.is_empty() {
            defmt::println!("Body is required for WRITE FILE: {}", args.len());
            return;
        }
        let file_addr = args[0];
        match file_addr {
            0x00 => {
                // 0x00 is special
                // Mode Store
                if args.len() <= 2 {
                    defmt::println!(
                        "Invalid command value length for mode store: {}",
                        args.len()
                    );
                    return;
                }
                let mode = args[1];
                let value = &args[2..];
                self.store_mode(mode, value);
            }
            _ => defmt::println!("Unsupported write file address: {:02x}", file_addr),
        }
    }

    fn store_mode(&mut self, mode: u8, value: &[u8]) {
        match mode {
            0x00 => {
                // Idle
                defmt::println!("Mode: Idle");
                self.mode = 0x00;
            }
            0x12 => {
                // Torque
                if value.len() != 4 {
                    defmt::println!("Invalid command value length for torque: {}", value.len());
                    return;
                }
                defmt::println!("Mode: Torque ({=[u8]:02x})", value);
                self.mode = 0x12;
                self.torque.copy_from_slice(value);
            }
            _ => defmt::println!("Unsupported mode: {:02x}", mode),
        }
    }
}

impl picompo::sinclair::Service for Rw0003 {
    fn on_command(&mut self, poll: bool, code: u8, body: &[u8], reply: impl FnOnce(&[u8])) {
        defmt::println!(
            "Handle command: poll: {}, code: {}, body: {=[u8]:02x}",
            poll,
            code,
            body
        );
        match code {
            0x01 => self.init(body),
            0x07 => self.read_file(body, reply),
            0x08 => self.write_file(body, reply),
            _ => defmt::println!("Unsupported command: {:02x}", code),
        }
    }
}

impl picompo::usb::Service for Rw0003 {
    fn handle_out(&mut self, ep: usize, data: &[u8]) {
        if ep == 0 {
            let Some(report) = OutReport::ref_from(data) else {
                return;
            };
            self.temperature.copy_from_slice(&report.temperature);
            self.speed.copy_from_slice(&report.speed);
        }
    }

    fn handle_in(&mut self, ep: usize, write: impl FnOnce(&[u8])) {
        if ep == 0 {
            let report = InReport::new(self.is_initialized, self.mode, self.torque);
            write(report.as_bytes());
        }
    }
}
