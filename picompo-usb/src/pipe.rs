use futures_lite::future::block_on;
use nusb::transfer::RequestBuffer;

pub struct Pipe {
    iface: nusb::Interface,
    handle: u32,
}

impl Pipe {
    pub fn new(iface: nusb::Interface, handle: u32,) -> Self {
        Self { iface, handle }
    }

    pub fn send(&self, ep: u8, bytes: &[u8]) {
        let completion = block_on(self.iface.bulk_out(ep, bytes.to_vec()));
        completion.status.unwrap();
    }

    pub fn recv(&self, ep: u8) -> Vec<u8> {
        let buf = RequestBuffer::new(64);
        let completion = block_on(self.iface.bulk_in(0x80 | ep, buf));
        completion.status.unwrap();
        completion.data
    }

    pub fn handle(&self) -> u32 {
        self.handle
    }
}
