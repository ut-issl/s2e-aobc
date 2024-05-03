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
    gpio::{
        self,
        bank0::{Gpio1, Gpio5, Gpio9},
        FunctionSioInput, PullUp,
    },
    i2c::{peripheral::Event, Peripheral},
    multicore::{Multicore, Stack},
    I2C,
};
use panic_probe as _;
use rp_pico::{
    self as bsp, hal,
    hal::{pac, prelude::*},
};
use slippers::{SlipDecoder, SlipEncoder};
use usb_device::{class_prelude::UsbBusAllocator, prelude::*};
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::hid_class::{
    HIDClass, HidClassSettings, HidCountryCode, HidProtocol, HidSubClass, ProtocolModeConfig,
};
use zerocopy::{AsBytes, FromBytes, FromZeroes, Ref, LE, U16};

const RW0003_CRC: crc::Crc<u16> = crc::Crc::<u16>::new(&crc::CRC_16_MCRF4XX);

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = VENDOR_DEFINED_START, usage = 0x01) = {
        output=output;
        input=input;
    }
)]
struct CustomReport {
    output: [u8; 8],
    input: [u8; 8],
}

#[derive(FromZeroes, FromBytes)]
#[repr(C)]
struct OutputReport {
    temperature: [u8; 4],
    speed: [u8; 4],
}

#[derive(FromZeroes, AsBytes)]
#[repr(C)]
struct InputReport {
    is_initialized: u8,
    mode: u8,
    _padding: [u8; 2],
    torque: [u8; 4],
}

impl InputReport {
    fn new(is_initialized: bool, mode: u8, torque: [u8; 4]) -> Self {
        Self {
            is_initialized: is_initialized as u8,
            mode,
            _padding: [0; 2],
            torque,
        }
    }
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

    let mut sio = Sio::new(pac.SIO);
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
    let vid_pid = UsbVidPid(0x6666, 0x6910);
    let mut hid = HIDClass::new_with_settings(
        &bus_allocator,
        CustomReport::desc(),
        1,
        HidClassSettings {
            subclass: HidSubClass::NoSubClass,
            protocol: HidProtocol::Generic,
            config: ProtocolModeConfig::ForceReport,
            locale: HidCountryCode::NotSupported,
        },
    );
    let mut dev = UsbDeviceBuilder::new(&bus_allocator, vid_pid)
        .max_packet_size_0(64)
        .unwrap()
        .strings(&[StringDescriptors::new(LangID::EN_US)
            .product(product_name)
            .manufacturer("ArkEdge Space Inc.")])
        .unwrap()
        .build();

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let core1_stack = cortex_m::singleton!(: Stack<4096> = Stack::new()).unwrap();
    let i2c1 = I2C::new_controller(
        pac.I2C1,
        pins.gpio10.into_function(),
        pins.gpio11.into_function(),
        10u32.kHz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );
    core1
        .spawn(&mut core1_stack.mem, move || {
            core1_task(i2c1);
        })
        .unwrap();

    let i2c_sda = pins.gpio16.into_function();
    let i2c_scl = pins.gpio17.into_function();
    let i2c0 =
        I2C::new_peripheral_event_iterator(pac.I2C0, i2c_sda, i2c_scl, &mut pac.RESETS, self_address);
    let mut rw0003 = Rw0003::new(i2c0, 0x11, self_address);
    let mut usbbuf = [0x00; 64];

    loop {
        dev.poll(&mut [&mut hid]);
        rw0003.poll();

        if let Ok(len) = hid.pull_raw_output(&mut usbbuf) {
            if let Some(output_report) = OutputReport::ref_from(&usbbuf[..len]) {
                rw0003.handle_output_report(output_report);
                defmt::println!(
                    "Received output report: temperature: {=[u8]:02x}, speed: {=[u8]:02x}",
                    rw0003.temperature,
                    rw0003.speed
                );
            }
        }
        let input_report = rw0003.build_input_report();
        hid.push_raw_input(input_report.as_bytes()).ok();
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

struct Rw0003<I2C, SDA, SCL>
where
    SDA: hal::gpio::PinId,
    SCL: hal::gpio::PinId,
{
    i2c: I2C_PERIPHERAL<I2C, SDA, SCL>,
    ctrl_address: u8,
    self_address: u8,
    recv_end_cursor: usize,
    recv_buffer: [u8; 64],
    send_start_cursor: usize,
    send_buffer: heapless::Vec<u8, 64>,

    is_initialized: bool,
    mode: u8,
    torque: [u8; 4],
    temperature: [u8; 4],
    speed: [u8; 4],
}

impl<I2C, SDA, SCL> Rw0003<I2C, SDA, SCL>
where
    SDA: hal::gpio::PinId,
    SCL: hal::gpio::PinId,
    I2C: core::ops::Deref<Target = pac::i2c0::RegisterBlock>,
{
    pub fn new(i2c: I2C_PERIPHERAL<I2C, SDA, SCL>, ctrl_address: u8, self_address: u8) -> Self {
        Self {
            i2c,
            ctrl_address,
            self_address,
            recv_end_cursor: 0,
            recv_buffer: [0; 64],
            send_start_cursor: 0,
            send_buffer: heapless::Vec::new(),

            is_initialized: false,
            mode: 0x00, // IDLE
            torque: [0; 4],
            temperature: 12.34f32.to_le_bytes(),
            speed: 345.67f32.to_le_bytes(),
        }
    }

    pub fn poll(&mut self) {
        while let Some(event) = self.i2c.next_event() {
            match event {
                Event::TransferWrite => {
                    if self.recv_end_cursor >= self.recv_buffer.len() {
                        // wrap around
                        self.recv_end_cursor = 0;
                    }
                    let read = self.i2c.read(&mut self.recv_buffer[self.recv_end_cursor..]);
                    self.recv_end_cursor += read;
                }
                Event::TransferRead => {
                    defmt::println!("TransferRead: {}", self.send_start_cursor);
                    if self.send_start_cursor >= self.send_buffer.len() {
                        // buffer underflow
                        //defmt::println!("Buffer underflow !!!!!");
                        self.i2c.write(&[0]);
                        continue;
                    }
                    let buf = &self.send_buffer[self.send_start_cursor..];
                    defmt::println!("Sending {} bytes: {=[u8]:02x}", buf.len(), buf);
                    let sent = self.i2c.write(buf);
                    //defmt::println!("Sent {} bytes", sent);
                    self.send_start_cursor += sent;
                }
                Event::Start => {
                    defmt::println!("Start");
                    // Clear send buffers
                    self.send_start_cursor = 0;
                    self.send_buffer.clear();
                }
                Event::Restart => {
                    defmt::println!("Restart");
                    self.handle_message().ok();
                }
                Event::Stop => {
                    defmt::println!("Stop");
                }
            }
        }
    }

    pub fn handle_message(&mut self) -> Result<(), ()> {
        let mut message = heapless::Vec::<u8, 64>::new();
        // HACK: put the destination address at the front of message
        message.push(self.self_address).unwrap();
        let src = &self.recv_buffer[..self.recv_end_cursor];
        defmt::println!("Received: {=[u8]:02x}", src);
        slip_decode(src, &mut message)?;
        defmt::println!("Decoded: {=[u8]:02x}", message);
        self.recv_end_cursor = 0;
        let (header_and_body, footer) =
            Ref::<_, NspFooter>::new_from_suffix(message.as_slice()).ok_or(())?;
        let computed = RW0003_CRC.checksum(header_and_body);
        if computed != u16::from(footer.crc) {
            defmt::println!(
                "CRC mismatch: computed: {:04x} vs footer: {:04x}",
                computed,
                u16::from(footer.crc)
            );
            return Err(());
        }
        let (header, body) = Ref::<_, NspHeader>::new_from_prefix(header_and_body).ok_or(())?;
        let poll = header.poll_final();
        let command_code = header.command_code();
        self.handle_command(poll, command_code, body);
        Ok(())
    }

    pub fn handle_command(&mut self, poll: bool, command_code: u8, body: &[u8]) {
        defmt::println!(
            "Handle command: {}, command_code: {}, body: {=[u8]:02x}",
            poll,
            command_code,
            body
        );
        match command_code {
            0x01 => {
                // INIT
                if body == [0x00, 0x10, 0x00, 0x00] {
                    self.is_initialized = true;
                    defmt::println!("Initialized");
                }
            }
            0x07 => {
                // READ FILE
                if !self.is_initialized {
                    defmt::println!("READ FILE: Not initialized");
                    return;
                }
                if body.len() != 1 {
                    defmt::println!("Invalid body length for READ FILE: {}", body.len());
                    return;
                }
                let file_addr = body[0];
                match file_addr {
                    0x00 => {
                        // 0x00 is special
                        // Unsupported
                        defmt::println!("Unsupported file address: {}", file_addr);
                    }
                    0x03 => {
                        // Temperature
                        defmt::println!("Get temperature");
                        let mut reply_body = [0u8; 5];
                        reply_body[0] = 0x03; // Temperature
                        reply_body[1..].copy_from_slice(&self.temperature);
                        self.reply_message(command_code, &reply_body).unwrap();
                    }
                    0x15 => {
                        // Speed
                        defmt::println!("Get speed");
                        let mut reply_body = [0u8; 5];
                        reply_body[0] = 0x15; // Speed
                        reply_body[1..].copy_from_slice(&self.speed);
                        self.reply_message(command_code, &reply_body).unwrap();
                    }
                    _ => {
                        // Unsupported
                        defmt::println!("Unsupported file address: {}", file_addr);
                    }
                }
            }
            0x08 => {
                // WRITE FILE
                if !self.is_initialized {
                    defmt::println!("WRITE FILE: Not initialized");
                    return;
                }
                if body.is_empty() {
                    defmt::println!("Body is required for WRITE FILE: {}", body.len());
                    return;
                }
                let file_addr = body[0];
                match file_addr {
                    0x00 => {
                        // 0x00 is special
                        // Mode Store
                        if body.len() >= 2 {
                            let mode_command_type = body[1];
                            let mode_command_value = &body[2..];
                            match mode_command_type {
                                0x00 => {
                                    // Idle
                                    defmt::println!("Idle");
                                    self.mode = 0x00;
                                },
                                0x12 => {
                                    // Torque
                                    if mode_command_value.len() != 4 {
                                        defmt::println!(
                                            "Invalid command value length for torque: {}",
                                            mode_command_value.len()
                                        );
                                        return;
                                    }
                                    defmt::println!("Torque: {=[u8]:02x}", mode_command_value);
                                    self.mode = 0x12;
                                    self.torque.copy_from_slice(mode_command_value);
                                }
                                _ => {
                                    // Unsupported
                                }
                            }
                        }
                    }
                    _ => {
                        // Unsupported
                    }
                }
            }
            _ => {
                // Unknown command
            }
        }
    }

    pub fn reply_message(&mut self, command_code: u8, body: &[u8]) -> Result<(), ()> {
        let mut message = heapless::Vec::<u8, 64>::new();

        let header = NspHeader {
            destination_address: self.ctrl_address,
            source_address: self.self_address,
            message_control_field: 0xa0 | command_code,
        };

        message.extend_from_slice(header.as_bytes())?;
        message.extend_from_slice(body)?;

        let footer = NspFooter {
            crc: RW0003_CRC.checksum(message.as_slice()).into(),
        };
        message.extend_from_slice(footer.as_bytes())?;

        self.send_buffer.clear();
        self.send_start_cursor = 0;
        slip_encode(&message[2..], &mut self.send_buffer)?;

        Ok(())
    }

    pub fn handle_output_report(&mut self, report: &OutputReport) {
        self.temperature.copy_from_slice(&report.temperature);
        self.speed.copy_from_slice(&report.speed);
    }

    pub fn build_input_report(&mut self) -> InputReport {
        InputReport::new(self.is_initialized, self.mode, self.torque)
    }
}

fn slip_decode<const N: usize>(src: &[u8], dst: &mut heapless::Vec<u8, N>) -> Result<(), ()> {
    let dec = SlipDecoder::new(src);
    for byte in dec.iter() {
        dst.push(byte).map_err(|_| ())?;
    }
    Ok(())
}

fn slip_encode<const N: usize>(src: &[u8], dst: &mut heapless::Vec<u8, N>) -> Result<(), ()> {
    let enc = SlipEncoder::new(src);
    for byte in enc.iter() {
        dst.push(byte).map_err(|_| ())?;
    }
    Ok(())
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

// AOBC side emulation
fn core1_task(mut i2c: impl embedded_hal::i2c::I2c<Error = hal::i2c::Error>) -> ! {
    cortex_m::asm::delay(60_000_000);
    //let init = [0x11, 0x01, 0x00, 0x10, 0x00, 0x00, 0x8B, 0x80, 0xC0];
    //let get_temp = [0x11, 0x87, 0x03, 0xFE, 0xEB, 0xC0];
    let get_speed = [0x11, 0x87, 0x15, 0x49, 0x9E, 0xC0];
    loop {
        defmt::println!("Controller is Writing");
        let mut tlm = [0u8; 15];
        if let Err(e) = i2c.write_read(0x26, &get_speed, &mut tlm) {
            defmt::println!("I2C Controller Write Error: {:?}", e);
            cortex_m::asm::delay(60_000_000);
            continue;
        }
        defmt::println!("Controller Read: {=[u8]:02x}", tlm);
        cortex_m::asm::delay(60_000_000);
    }
}

fn hex(byte: u8) -> [u8; 2] {
    let hex = b"0123456789ABCDEF";
    [hex[(byte >> 4) as usize], hex[(byte & 0x0F) as usize]]
}
