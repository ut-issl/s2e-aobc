#![no_main]
#![no_std]

use bsp::{
    hal::{usb::UsbBus, Sio},
    XOSC_CRYSTAL_FREQ,
};
use defmt_rtt as _;
use embedded_hal::digital::InputPin;
use hal::{
    fugit::RateExtU32 as _,
    i2c::{peripheral::Event, Peripheral},
    multicore::{Multicore, Stack},
    gpio::{
        self,
        bank0::{Gpio1, Gpio5, Gpio9, Gpio13},
        FunctionSioInput, PullUp,
    },
    I2C,
};
use panic_probe as _;
use picompo::{usb::BulkClass, WinUsbClass};
use rp_pico::{
    self as bsp, hal,
    hal::{pac, prelude::*},
};
use usb_device::{class_prelude::UsbBusAllocator, prelude::*};
use zerocopy::{AsBytes, FromBytes, FromZeroes, LE, U16};

#[derive(FromZeroes, FromBytes)]
#[repr(C)]
struct OutputReport {
    unfiltered_cell_u_ssa1: [u8; 4],
    unfiltered_cell_u_ssa2: [u8; 4],
    unfiltered_cell_u_ssa3: [u8; 4],
    unfiltered_cell_u_ssa4: [u8; 4],
    filtered_cell_u_ssa1f: [u8; 4],
    filtered_cell_u_ssa2f: [u8; 4],
    filtered_cell_u_ssa3f: [u8; 4],
    filtered_cell_u_ssa4f: [u8; 4],
    angular_position_angle_x: [u8; 4],
    angular_position_angle_y: [u8; 4],
    angular_position_sun_detection: [u8; 4],
    angular_position_error_code: [u8; 1],
    _padding: [u8; 3],
}

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
    py: gpio::Pin<Gpio1, FunctionSioInput, PullUp>,
    my: gpio::Pin<Gpio5, FunctionSioInput, PullUp>,
    pz: gpio::Pin<Gpio9, FunctionSioInput, PullUp>,
    mz: gpio::Pin<Gpio13, FunctionSioInput, PullUp>,
}

impl AddressSelector {
    pub fn select(&mut self) -> Option<u8> {
        let py = self.py.is_low().unwrap();
        let my = self.my.is_low().unwrap();
        let pz = self.pz.is_low().unwrap();
        let mz = self.mz.is_low().unwrap();
        defmt::println!("py: {}, my: {}, pz: {}, mz: {}", py, my, pz, mz);
        match (py, my, pz, mz) {
            (true, false, false, false) => Some(0x11),
            (false, true, false, false) => Some(0x13),
            (false, false, true, false) => Some(0x50),
            (false, false, false, true) => Some(0x12),
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

    defmt::println!("hello");

    let mut sio = Sio::new(pac.SIO);
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut address_selector = AddressSelector {
        py: pins.gpio1.into_pull_up_input(),
        my: pins.gpio5.into_pull_up_input(),
        pz: pins.gpio9.into_pull_up_input(),
        mz: pins.gpio13.into_pull_up_input(),
    };
    let self_address = address_selector.select().unwrap_or(0x00);

    let product_name = cortex_m::singleton!(: [u8; 18] = [0; 18]).unwrap();
    // nusb returns uppercased serial number in Windows
    // so that we need to use uppercase letters for the product name
    product_name.copy_from_slice(b"FAKE_NANOSSOC_0X??");
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
    let vid_pid = UsbVidPid(0x6666, 0x0101);
    let mut bulk = BulkClass::<_, 2, 0>::new(&bus_allocator);
    let mut winusb = WinUsbClass;
    let mut dev = UsbDeviceBuilder::new(&bus_allocator, vid_pid)
        .max_packet_size_0(64)
        .unwrap()
        .strings(&[StringDescriptors::new(LangID::EN_US)
            .manufacturer("ArkEdge Space Inc.")
            .product(product_name)
            .serial_number(product_name)])
        .unwrap()
        .build();

    let i2c0_sda = pins.gpio16.into_function();
    let i2c0_scl = pins.gpio17.into_function();
    let i2c0 =
        I2C::new_peripheral_event_iterator(pac.I2C0, i2c0_sda, i2c0_scl, &mut pac.RESETS, self_address);
    let mut nanossoc0 = NanoSsocD60::new(i2c0);

    let mut usbbuf = [0x00; 64];

    loop {
        // ここでusbとのやりとりと，nanossocの模擬をくりかえす
        dev.poll(&mut [&mut winusb, &mut bulk]);
        nanossoc0.poll();

        if let Ok(len) = bulk.out_eps[0].read(&mut usbbuf) {
            if let Some(output_report) = OutputReport::ref_from(&usbbuf[..len]) {
                nanossoc0.unfiltered_cell_u_ssa1 = output_report.unfiltered_cell_u_ssa1;
                nanossoc0.unfiltered_cell_u_ssa1 = output_report.unfiltered_cell_u_ssa1;
                nanossoc0.unfiltered_cell_u_ssa3 = output_report.unfiltered_cell_u_ssa3;
                nanossoc0.unfiltered_cell_u_ssa4 = output_report.unfiltered_cell_u_ssa4;
                nanossoc0.filtered_cell_u_ssa1f = output_report.filtered_cell_u_ssa1f;
                nanossoc0.filtered_cell_u_ssa2f = output_report.filtered_cell_u_ssa2f;
                nanossoc0.filtered_cell_u_ssa3f = output_report.filtered_cell_u_ssa3f;
                nanossoc0.filtered_cell_u_ssa4f = output_report.filtered_cell_u_ssa4f;
                nanossoc0.angular_position_angle_x = output_report.angular_position_angle_x;
                nanossoc0.angular_position_angle_y = output_report.angular_position_angle_y;
                nanossoc0.angular_position_sun_detection =
                    output_report.angular_position_sun_detection;
                nanossoc0.angular_position_error_code = output_report.angular_position_error_code;
            }
        }

    }
}

#[allow(non_camel_case_types)]
type I2C_PERIPHERAL<I2C, SDA, SCL> = hal::I2C<
    I2C,
    (
        hal::gpio::Pin<SDA, hal::gpio::FunctionI2c, hal::gpio::PullDown>,
        hal::gpio::Pin<SCL, hal::gpio::FunctionI2c, hal::gpio::PullDown>,
    ),
    Peripheral,
>;

struct NanoSsocD60<I2C, SDA, SCL>
where
    SDA: hal::gpio::PinId,
    SCL: hal::gpio::PinId,
{
    i2c: I2C_PERIPHERAL<I2C, SDA, SCL>,
    recv_end_cursor: usize,
    recv_buffer: [u8; 64],
    send_start_cursor: usize,
    send_buffer: heapless::Vec<u8, 64>,

    unfiltered_cell_u_ssa1: [u8; 4],
    unfiltered_cell_u_ssa2: [u8; 4],
    unfiltered_cell_u_ssa3: [u8; 4],
    unfiltered_cell_u_ssa4: [u8; 4],
    filtered_cell_u_ssa1f: [u8; 4],
    filtered_cell_u_ssa2f: [u8; 4],
    filtered_cell_u_ssa3f: [u8; 4],
    filtered_cell_u_ssa4f: [u8; 4],
    angular_position_angle_x: [u8; 4],
    angular_position_angle_y: [u8; 4],
    angular_position_sun_detection: [u8; 4],
    angular_position_error_code: [u8; 1],
}

impl<I2C, SDA, SCL> NanoSsocD60<I2C, SDA, SCL>
where
    SDA: hal::gpio::PinId,
    SCL: hal::gpio::PinId,
    I2C: core::ops::Deref<Target = pac::i2c0::RegisterBlock>,
{
    pub fn new(i2c: I2C_PERIPHERAL<I2C, SDA, SCL>) -> Self {
        Self {
            i2c,
            recv_end_cursor: 0,
            recv_buffer: [0; 64],
            send_start_cursor: 0,
            send_buffer: heapless::Vec::new(),

            unfiltered_cell_u_ssa1: 12.34f32.to_le_bytes(),
            unfiltered_cell_u_ssa2: 56.78f32.to_le_bytes(),
            unfiltered_cell_u_ssa3: 91.23f32.to_le_bytes(),
            unfiltered_cell_u_ssa4: 45.67f32.to_le_bytes(),
            filtered_cell_u_ssa1f: 12.34f32.to_le_bytes(),
            filtered_cell_u_ssa2f: 56.78f32.to_le_bytes(),
            filtered_cell_u_ssa3f: 91.23f32.to_le_bytes(),
            filtered_cell_u_ssa4f: 45.67f32.to_le_bytes(),
            angular_position_angle_x: 98.76f32.to_le_bytes(),
            angular_position_angle_y: 54.32f32.to_le_bytes(),
            angular_position_sun_detection: 11.11f32.to_le_bytes(),
            angular_position_error_code: [0x00],
        }
    }

    pub fn poll(&mut self) {
        while let Some(event) = self.i2c.next_event() {
            match event {
                Event::TransferWrite => {
                    self.send_start_cursor = 0;
                    self.send_buffer.clear();

                    if self.recv_end_cursor >= self.recv_buffer.len() {
                        // wrap around
                        self.recv_end_cursor = 0;
                    }
                    let read = self.i2c.read(&mut self.recv_buffer[self.recv_end_cursor..]);
                    self.recv_end_cursor += read;
                    self.handle_message().ok();
                }
                Event::TransferRead => {
                    defmt::println!("TransferRead: {}", self.send_start_cursor);
                    if self.send_start_cursor >= self.send_buffer.len() {
                        // buffer underflow
                        defmt::println!("Buffer underflow !!!!!");
                        self.i2c.write(&[0]);
                        continue;
                    }
                    let buf = &self.send_buffer[self.send_start_cursor..];
                    defmt::println!("Sending {} bytes: {=[u8]:02x}", buf.len(), buf);
                    let sent = self.i2c.write(buf);
                    defmt::println!("Sent {} bytes", sent);
                    self.send_start_cursor += sent;
                }
                Event::Start => {
                    defmt::println!("Start");
                    // Clear send buffers
                }
                Event::Restart => {
                    defmt::println!("Restart");
                }
                Event::Stop => {
                    defmt::println!("Stop");
                }
            }
        }
    }

    pub fn handle_message(&mut self) -> Result<(), ()> {
        let src = &self.recv_buffer[..self.recv_end_cursor];
        defmt::println!("Received: {=[u8]:02x}", src);
        let command_code = src[0];
        self.handle_command(command_code);
        Ok(())
    }

    pub fn handle_command(&mut self, src: u8) {
        defmt::println!("src: {}", src);
        match src {
            0x01 => {
                // read unfiltered cells
                defmt::println!("Get unfiltered cells");
                let mut reply_body = [0u8; 18];
                reply_body[0] = 0x11; // length
                reply_body[1..][..4].copy_from_slice(&self.unfiltered_cell_u_ssa1);
                reply_body[5..][..4].copy_from_slice(&self.unfiltered_cell_u_ssa2);
                reply_body[9..][..4].copy_from_slice(&self.unfiltered_cell_u_ssa3);
                reply_body[13..][..4].copy_from_slice(&self.unfiltered_cell_u_ssa4);
                reply_body[17] = self.checksum(&reply_body[..=16]);
                self.reply_message(&reply_body).unwrap();
            }
            0x03 => {
                // read filtered cells
                defmt::println!("Get filtered cells");
                let mut reply_body = [0u8; 18];
                reply_body[0] = 0x11; // length
                reply_body[1..][..4].copy_from_slice(&self.filtered_cell_u_ssa1f);
                reply_body[5..][..4].copy_from_slice(&self.filtered_cell_u_ssa2f);
                reply_body[9..][..4].copy_from_slice(&self.filtered_cell_u_ssa3f);
                reply_body[13..][..4].copy_from_slice(&self.filtered_cell_u_ssa4f);
                reply_body[17] = self.checksum(&reply_body[..=16]);
                self.reply_message(&reply_body).unwrap();
            }
            0x04 => {
                // read angular position
                defmt::println!("Get angular position");
                let mut reply_body = [0u8; 15];
                reply_body[0] = 0x0E; // length
                reply_body[1..][..4].copy_from_slice(&self.angular_position_angle_x);
                reply_body[5..][..4].copy_from_slice(&self.angular_position_angle_y);
                reply_body[9..][..4].copy_from_slice(&self.angular_position_sun_detection);
                reply_body[13..][..1].copy_from_slice(&self.angular_position_error_code);
                reply_body[14] = self.checksum(&reply_body[..=13]);
                self.reply_message(&reply_body).unwrap();
            }
            _ => {
                // Unknown command
            }
        }
    }

    pub fn checksum(&mut self, data: &[u8]) -> u8 {
        data.iter().fold(0, |acc, x| acc.wrapping_add(*x))
    }
    pub fn reply_message(&mut self, body: &[u8]) -> Result<(), ()> {
        self.send_buffer.clear();
        self.send_start_cursor = 0;
        self.send_buffer.extend_from_slice(body)?;

        Ok(())
    }
}

#[derive(AsBytes, FromZeroes, FromBytes)]
#[repr(C)]
pub struct NspHeader {
    destination_address: u8,
    source_address: u8,
    message_control_field: u8,
}

impl NspHeader {
    pub fn poll_final(&self) -> bool {
        self.message_control_field & (1 << 7) != 0
    }

    pub fn command_code(&self) -> u8 {
        self.message_control_field & 0x0F
    }
}

#[derive(AsBytes, FromZeroes, FromBytes)]
#[repr(C)]
pub struct NspFooter {
    crc: U16<LE>,
}
