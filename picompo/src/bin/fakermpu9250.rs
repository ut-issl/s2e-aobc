#![no_main]
#![no_std]

use bsp::{
    hal::{usb::UsbBus, Sio},
    XOSC_CRYSTAL_FREQ,
};
use defmt_rtt as _;
use hal::{
    fugit::RateExtU32 as _, i2c::{peripheral::Event, Peripheral}, multicore::{Multicore, Stack}, pac::clocks::fc0_status::FAST_R, I2C
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

const Rm3100_CRC: crc::Crc<u16> = crc::Crc::<u16>::new(&crc::CRC_16_MCRF4XX);

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = VENDOR_DEFINED_START, usage = 0x01) = {
        mx=output;
        my=output;
        mz=output;
    }
)]
struct CustomReport {
    mx: [u8; 4],
    my: [u8; 4],
    mz: [u8; 4],
}

#[derive(FromZeroes, FromBytes)]
#[repr(C)]
struct OutputReport {
    mx: [u8; 4],
    my: [u8; 4],
    mz: [u8; 4],
}

#[derive(FromZeroes, AsBytes)]
#[repr(C)]
struct InputReport {
}

impl InputReport {
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
        16,
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
        .strings(&[StringDescriptors::new(LangID::EN)
            .manufacturer("ArkEdge Space Inc.")
            .product("FAKE-Rm3100")])
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
        I2C::new_peripheral_event_iterator(pac.I2C0, i2c_sda, i2c_scl, &mut pac.RESETS, 0x23u8);
    let mut rm3100 = Rm3100::new(i2c0, 0x11, 0x23);
    let mut usbbuf = [0x00; 64];

    loop {
        // ここでusbとのやりとりと，rm3100の模擬をくりかえす
        dev.poll(&mut [&mut hid]);
        rm3100.poll();

        // S2Eからのデータを受信して格納
        if let Ok(len) = hid.pull_raw_output(&mut usbbuf) {
            if let Some(output_report) = OutputReport::ref_from(&usbbuf[..len]) {
                rm3100.mx = output_report.mx;
                rm3100.my = output_report.my;
                rm3100.mz = output_report.mz;
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

struct Rm3100<I2C, SDA, SCL>
where
    SDA: hal::gpio::PinId,
    SCL: hal::gpio::PinId,
{
    i2c: I2C_PERIPHERAL<I2C, SDA, SCL>,
    ctrl_address: u8,
    self_address: u8,
    is_initialized: bool,
    recv_end_cursor: usize,
    recv_buffer: [u8; 64],
    send_start_cursor: usize,
    send_buffer: heapless::Vec<u8, 64>,
    is_TransferWrite: bool,

    mx: [u8; 4],
    my: [u8; 4],
    mz: [u8; 4],
}

impl<I2C, SDA, SCL> Rm3100<I2C, SDA, SCL>
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
            is_initialized: false,
            recv_end_cursor: 0,
            recv_buffer: [0; 64],
            send_start_cursor: 0,
            send_buffer: heapless::Vec::new(),
            is_TransferWrite: false,

            mx: 1234u32.to_be_bytes(),
            my: 5678u32.to_be_bytes(),
            mz: 9012u32.to_be_bytes(),
        }
    }

    pub fn poll(&mut self) {
        while let Some(event) = self.i2c.next_event() {
            match event {
                Event::TransferWrite => {
                    defmt::println!("TransferWrite: {}", self.recv_end_cursor);
                    self.is_TransferWrite = true;
                    if self.recv_end_cursor >= self.recv_buffer.len() {
                        // wrap around
                        self.recv_end_cursor = 0;
                    }
                    let read = self.i2c.read(&mut self.recv_buffer[self.recv_end_cursor..]);
                    self.recv_end_cursor += read;
                    
                }
                Event::TransferRead => {
                    defmt::println!("TransferRead: {}", self.send_start_cursor);
                    self.is_TransferWrite = false;
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
                }
                Event::Restart => {
                    defmt::println!("Restart");
                }
                Event::Stop => {
                    defmt::println!("Stop");
                    if self.is_TransferWrite {
                        self.handle_message().ok();
                        self.recv_end_cursor = 0;
                    }else{
                        self.send_start_cursor = 0;
                        self.send_buffer.clear();
                        
                    }
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
        defmt::println!("buffer: {=[u8]:02x}", self.recv_buffer);
        match src[0] {
            0x01 => {
                if src[1] == 0x7d {
                    defmt::println!("INIT");
                    self.is_initialized = true;
                    // 初期化の処理をここでする
                }else{
                    defmt::println!("INIT failed");
                
                }
            }
            0x24 => {
                if self.is_initialized {
                    
                        defmt::println!("get magnetometer data");
                        let mut reply_body = [0u8; 9];
                        reply_body[0..][..3].copy_from_slice(&self.mx[1..]);
                        reply_body[3..][..3].copy_from_slice(&self.my[1..]);
                        reply_body[6..][..3].copy_from_slice(&self.mz[1..]);
                        self.reply_message(&reply_body).unwrap();    
 
                }else{
                    defmt::println!("Not initialized");
                }
                
            }

            _ => {
                defmt::println!("Invalid message");
            }
        }
        Ok(())
    }


    pub fn reply_message(&mut self, body: &[u8]) -> Result<(), ()> {
        let mut message = heapless::Vec::<u8, 64>::new();
        message.extend_from_slice(body)?;

        self.send_buffer.clear();
        self.send_start_cursor = 0;
        encode(&message, &mut self.send_buffer)?;

        Ok(())
    }
}

fn slip_decode<const N: usize>(src: &[u8], dst: &mut heapless::Vec<u8, N>) -> Result<(), ()> {
    let dec = SlipDecoder::new(src);
    for byte in dec.iter() {
        dst.push(byte).map_err(|_| ())?;
    }
    Ok(())
}

fn encode<const N: usize>(src: &[u8], dst: &mut heapless::Vec<u8, N>) -> Result<(), ()> {
    // let enc: SlipEncoder = SlipEncoder::new(src);
    for byte in src.iter() {
        dst.push(*byte).map_err(|_| ())?;
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
