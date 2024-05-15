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
        bank0::{Gpio1, Gpio5},
        FunctionSioInput, PullUp,
    },
    I2C
};
use panic_probe as _;
use picompo::{usb::BulkClass, WinUsbClass};
use rp_pico::{
    self as bsp, hal,
    hal::{pac, prelude::*},
};
use usb_device::{class_prelude::UsbBusAllocator, prelude::*};
use zerocopy::{AsBytes, FromBytes, FromZeroes, Ref, LE, U16};

#[derive(FromZeroes, FromBytes)]
#[repr(C)]
struct OutputReport {
    // mx: [u8; 4],
    // my: [u8; 4],
    // mz: [u8; 4],
    tlm:[u8; 9],
    // buffer:[u8; 3],
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
    aobc: gpio::Pin<Gpio1, FunctionSioInput, PullUp>,
    ext: gpio::Pin<Gpio5, FunctionSioInput, PullUp>,
}

impl AddressSelector {
    pub fn select(&mut self) -> Option<u8> {
        let aobc = self.aobc.is_low().unwrap();
        let ext = self.ext.is_low().unwrap();
        defmt::println!("aobc: {}, ext: {}", aobc, ext);
        match (aobc, ext) {
            (true, false) => Some(0x22),
            (false, true) => Some(0x23),
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
        aobc: pins.gpio1.into_pull_up_input(),
        ext: pins.gpio5.into_pull_up_input(),
    };
    let self_address = address_selector.select().unwrap_or(0x00);

    let product_name = cortex_m::singleton!(: [u8; 16] = [0; 16]).unwrap();
    // nusb returns uppercased serial number in Windows
    // so that we need to use uppercase letters for the product name
    product_name.copy_from_slice(b"FAKE_RM3100_0X??");
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
    let mut bulk = BulkClass::<_, 2, 0>::new(&bus_allocator);
    let mut winusb = WinUsbClass;
    let mut dev = UsbDeviceBuilder::new(&bus_allocator, vid_pid)
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
    let i2c0 =
        I2C::new_peripheral_event_iterator(pac.I2C0, i2c_sda, i2c_scl, &mut pac.RESETS, self_address);
    let mut rm3100 = Rm3100::new(i2c0);
    let mut usbbuf = [0x00; 64];

    loop {
        // ここでusbとのやりとりと，rm3100の模擬をくりかえす
        dev.poll(&mut [&mut winusb, &mut bulk]);
        rm3100.poll();

        if let Ok(len) = bulk.out_eps[0].read(&mut usbbuf) {
            if let Some(output_report) = OutputReport::ref_from(&usbbuf[..len]) {
                // rm3100.mx = output_report.mx;
                // rm3100.my = output_report.my;
                // rm3100.mz = output_report.mz;
                rm3100.tlm = output_report.tlm;
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

    is_initialized: bool,
    recv_end_cursor: usize,
    recv_buffer: [u8; 64],
    send_start_cursor: usize,
    send_buffer: heapless::Vec<u8, 64>,
    is_transfer_write: bool,

    // mx: [u8; 4],
    // my: [u8; 4],
    // mz: [u8; 4],
    tlm: [u8; 9],
}

impl<I2C, SDA, SCL> Rm3100<I2C, SDA, SCL>
where
    SDA: hal::gpio::PinId,
    SCL: hal::gpio::PinId,
    I2C: core::ops::Deref<Target = pac::i2c0::RegisterBlock>,
{
    pub fn new(i2c: I2C_PERIPHERAL<I2C, SDA, SCL>) -> Self {
        Self {
            i2c,
            
            is_initialized: false,
            recv_end_cursor: 0,
            recv_buffer: [0; 64],
            send_start_cursor: 0,
            send_buffer: heapless::Vec::new(),
            is_transfer_write: false,

            // mx: 1234u32.to_le_bytes(),
            // my: 5678u32.to_le_bytes(),
            // mz: 9012u32.to_le_bytes(),
            tlm: [0; 9],
        }
    }

    pub fn poll(&mut self) {
        while let Some(event) = self.i2c.next_event() {
            match event {
                Event::TransferWrite => {
                    defmt::println!("TransferWrite: {}", self.recv_end_cursor);
                    self.is_transfer_write = true;
                    if self.recv_end_cursor >= self.recv_buffer.len() {
                        // wrap around
                        self.recv_end_cursor = 0;
                    }
                    let read = self.i2c.read(&mut self.recv_buffer[self.recv_end_cursor..]);
                    self.recv_end_cursor += read;
                    
                }
                Event::TransferRead => {
                    defmt::println!("TransferRead: {}", self.send_start_cursor);
                    self.is_transfer_write = false;
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
                    if self.is_transfer_write {
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
        // let mut message = heapless::Vec::<u8, 64>::new();
        // // HACK: put the destination address at the front of message
        // message.push(self.self_address).unwrap();
        if self.recv_end_cursor == 0 {
            return Ok(());
        }
        let src = &self.recv_buffer[..self.recv_end_cursor];
        
        defmt::println!("Received: {=[u8]:02x}", src);
        defmt::println!("buffer: {=[u8]:02x}", self.recv_buffer);
        match src[0] {
            // 0x01 => {
            //     if src[1] == 0x7d {
            //         defmt::println!("INIT");
            //         self.is_initialized = true;
            //         // 初期化の処理をここでする
            //     }else{
            //         defmt::println!("INIT failed");
                
            //     }
            // }
            0x24 => {
                // if self.is_initialized {
                    
                        defmt::println!("get magnetometer data");
                        let mut reply_body = [0u8; 9];
                        // reply_body[0] = self.mx[2];
                        // reply_body[1] = self.mx[1];
                        // reply_body[2] = self.mx[0];
                        // reply_body[3] = self.my[2];
                        // reply_body[4] = self.my[1];
                        // reply_body[5] = self.my[0];
                        // reply_body[6] = self.mz[2];
                        // reply_body[7] = self.mz[1];
                        // reply_body[8] = self.mz[0];
                        reply_body.copy_from_slice(&self.tlm);
                        self.reply_message(&reply_body).unwrap();    
 
                // }else{
                //     defmt::println!("Not initialized");
                // }
                
            }

            _ => {
                defmt::println!("Invalid message");
            }
        }
        Ok(())
    }


    pub fn reply_message(&mut self, body: &[u8]) -> Result<(), ()> {
        // let mut message = heapless::Vec::<u8, 64>::new();
        // message.extend_from_slice(body)?;

        self.send_buffer.clear();
        self.send_start_cursor = 0;
        self.send_buffer.extend_from_slice(body)?;

        Ok(())
    }
}


#[derive(AsBytes, FromZeroes, FromBytes)]
#[repr(C)]
pub struct NspFooter {
    crc: U16<LE>,
}


