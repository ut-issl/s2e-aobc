use futures_lite::future::block_on;
use nusb::transfer::RequestBuffer;
use zerocopy::{AsBytes, FromBytes, FromZeroes};

pub struct Rw0003 {
    iface: nusb::Interface,
}

impl Rw0003 {
    pub fn new(iface: nusb::Interface) -> Self {
        Self { iface }
    }

    pub fn send_output(&self, output: OutputReport) {
        let completion = block_on(self.iface.bulk_out(1, output.as_bytes().to_vec()));
        completion.status.unwrap();
    }

    pub fn recv_input(&self) -> InputReport {
        let buf = RequestBuffer::new(64);
        let completion = block_on(self.iface.bulk_in(0x81, buf));
        completion.status.unwrap();
        InputReport::ref_from(&completion.data).unwrap().clone()
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
    pub is_initialized: u8,
    pub mode: u8,
    _padding: [u8; 2],
    pub torque: [u8; 4],
}
