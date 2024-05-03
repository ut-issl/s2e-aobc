use std::mem::size_of;

use zerocopy::{AsBytes, FromBytes, FromZeroes};

pub struct Rw0003 {
    device: hidapi::HidDevice,
}

impl Rw0003 {
    pub fn new(device: hidapi::HidDevice) -> Self {
        Self { device }
    }

    pub fn send_output(&self, output: OutputReport) {
        let mut buf = [0; size_of::<OutputReport>() + 1];
        buf[0] = 0; // Report ID
        buf[1..].copy_from_slice(&output.as_bytes());
        self.device.write(&buf).unwrap();
    }

    pub fn recv_input(&self) -> InputReport {
        let mut buf = [0; size_of::<InputReport>()];
        self.device.read(&mut buf).unwrap();
        println!("{:02x?}", buf);
        InputReport::ref_from(&buf).unwrap().clone()
    }
}

#[derive(FromZeroes, AsBytes, FromBytes, Clone)]
#[repr(C)]
pub struct OutputReport {
    pub temperature: [u8; 4],
    pub speed: [u8; 4],
}

#[derive(FromZeroes, AsBytes, FromBytes, Clone)]
#[repr(C)]
pub struct InputReport {
    pub torque: [u8; 4],
}
