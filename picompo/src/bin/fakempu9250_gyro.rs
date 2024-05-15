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
    accel_x_h: u8,
    accel_x_l: u8,
    accel_y_h: u8,
    accel_y_l: u8,
    accel_z_h: u8,
    accel_z_l: u8,
    temp_h: u8,
    temp_l: u8,
    gyro_x_h: u8,
    gyro_x_l: u8,
    gyro_y_h: u8,
    gyro_y_l: u8,
    gyro_z_h: u8,
    gyro_z_l: u8,
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
    let mut bulk = BulkClass::<_, 2, 0>::new(&bus_allocator);
    let mut winusb = WinUsbClass;
    let mut dev = UsbDeviceBuilder::new(&bus_allocator, vid_pid)
        .max_packet_size_0(64)
        .unwrap()
        .strings(&[StringDescriptors::new(LangID::EN_US)
            .product("FAKE_MPU9250_GYRO")
            .serial_number("FAKE_MPU9250_GYRO")
            .manufacturer("ArkEdge Space Inc.")])
        .unwrap()
        .build();

    let i2c_sda = pins.gpio16.into_function();
    let i2c_scl = pins.gpio17.into_function();
    let i2c0 =
    I2C::new_peripheral_event_iterator(pac.I2C0, i2c_sda, i2c_scl, &mut pac.RESETS, 0x51u8);
    let mut mpu9250_gyro = Mpu9250Gyro::new(i2c0);
    let mut usbbuf = [0x00; 64];

    loop {
        // ここでusbとのやりとりと，mpu9250_gyroの模擬をくりかえす
        dev.poll(&mut [&mut winusb, &mut bulk]);
        mpu9250_gyro.poll();

        if let Ok(len) = bulk.out_eps[0].read(&mut usbbuf) {
            if let Some(output_report) = OutputReport::ref_from(&usbbuf[..len]) {
                mpu9250_gyro.accel_x_h = output_report.accel_x_h;
                mpu9250_gyro.accel_x_l = output_report.accel_x_l;
                mpu9250_gyro.accel_y_h = output_report.accel_y_h;
                mpu9250_gyro.accel_y_l = output_report.accel_y_l;
                mpu9250_gyro.accel_z_h = output_report.accel_z_h;
                mpu9250_gyro.accel_z_l = output_report.accel_z_l;
                mpu9250_gyro.temp_h = output_report.temp_h;
                mpu9250_gyro.temp_l = output_report.temp_l;
                mpu9250_gyro.gyro_x_h = output_report.gyro_x_h;
                mpu9250_gyro.gyro_x_l = output_report.gyro_x_l;
                mpu9250_gyro.gyro_y_h = output_report.gyro_y_h;
                mpu9250_gyro.gyro_y_l = output_report.gyro_y_l;
                mpu9250_gyro.gyro_z_h = output_report.gyro_z_h;
                mpu9250_gyro.gyro_z_l = output_report.gyro_z_l;

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

struct Mpu9250Gyro<I2C, SDA, SCL>
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

    accel_x_h: u8,
    accel_x_l: u8,
    accel_y_h: u8,
    accel_y_l: u8,
    accel_z_h: u8,
    accel_z_l: u8,
    temp_h: u8,
    temp_l: u8,
    gyro_x_h: u8,
    gyro_x_l: u8,
    gyro_y_h: u8,
    gyro_y_l: u8,
    gyro_z_h: u8,
    gyro_z_l: u8,
}

impl<I2C, SDA, SCL> Mpu9250Gyro<I2C, SDA, SCL>
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
            
            accel_x_h: 0,
            accel_x_l: 0x12,
            accel_y_h: 0,
            accel_y_l: 0x34,
            accel_z_h: 0,
            accel_z_l: 0x56,
            temp_h: 0,
            temp_l: 0x78,
            gyro_x_h: 0,
            gyro_x_l: 0x90,
            gyro_y_h: 0,
            gyro_y_l: 0x12,
            gyro_z_h: 0,
            gyro_z_l: 0x34,

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
            0x3b => {

                    
                defmt::println!("get gyro data");
                let mut reply_body = [0u8; 14];
                reply_body[0] = self.accel_x_h;
                reply_body[1] = self.accel_x_l;
                reply_body[2] = self.accel_y_h;
                reply_body[3] = self.accel_y_l;
                reply_body[4] = self.accel_z_h;
                reply_body[5] = self.accel_z_l;
                reply_body[6] = self.temp_h;
                reply_body[7] = self.temp_l;
                reply_body[8] = self.gyro_x_h;
                reply_body[9] = self.gyro_x_l;
                reply_body[10] = self.gyro_y_h;
                reply_body[11] = self.gyro_y_l;
                reply_body[12] = self.gyro_z_h;
                reply_body[13] = self.gyro_z_l;
                self.reply_message(&reply_body).unwrap();    
            }
                
            

            _ => {
                defmt::println!("not get data");
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


