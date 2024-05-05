use usb_device::{
    bus::{InterfaceNumber, UsbBus, UsbBusAllocator},
    class::UsbClass,
    endpoint::{EndpointIn, EndpointOut},
};

pub struct BulkClass<'a, B, const O: usize, const I: usize>
where
    B: UsbBus,
{
    ifn: InterfaceNumber,
    out_eps: [EndpointOut<'a, B>; O],
    in_eps: [EndpointIn<'a, B>; I],
}

impl<'a, B, const O: usize, const I: usize> BulkClass<'a, B, O, I>
where
    B: UsbBus,
{
    pub fn new(alloc: &'a UsbBusAllocator<B>) -> Self {
        let ifn = alloc.interface();
        let out_eps = [(); O].map(|()| alloc.bulk(64));
        let in_eps = [(); I].map(|()| alloc.bulk(64));
        Self {
            ifn,
            out_eps,
            in_eps,
        }
    }

    pub fn poll_out(&mut self, handler: &mut impl Service) {
        for (ep, out_ep) in self.out_eps.iter().enumerate() {
            let mut buf = [0x00; 64];
            if let Ok(len) = out_ep.read(&mut buf) {
                handler.handle_out(ep, &buf[..len]);
            }
        }
    }

    pub fn poll_in(&mut self, handler: &mut impl Service) {
        for (ep, in_ep) in self.in_eps.iter().enumerate() {
            handler.handle_in(ep, |buf| {
                in_ep.write(buf).ok();
            });
        }
    }
}

impl<'a, B, const O: usize, const I: usize> UsbClass<B> for BulkClass<'a, B, O, I>
where
    B: UsbBus,
{
    fn get_configuration_descriptors(
        &self,
        writer: &mut usb_device::descriptor::DescriptorWriter,
    ) -> usb_device::Result<()> {
        writer.interface(self.ifn, 0xFF, 0x00, 0x00)?;
        for out_ep in self.out_eps.iter() {
            writer.endpoint(out_ep)?;
        }
        for in_ep in self.in_eps.iter() {
            writer.endpoint(in_ep)?;
        }
        Ok(())
    }
}

pub trait Service {
    fn handle_out(&mut self, ep: usize, data: &[u8]);
    fn handle_in(&mut self, ep: usize, write: impl FnOnce(&[u8]));
}
