use usb_device::{
    bus::UsbBus,
    class::{ControlIn, UsbClass},
    control,
    descriptor::BosWriter,
};

pub struct WinUsbClass;

const CAPABILITIES: &[u8] = &[
    0x00, // wReserved
    0xDF, 0x60, 0xDD, 0xD8, 0x89, 0x45, 0xC7, 0x4C, // PlatformCapabilityUUID
    0x9C, 0xD2, 0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F, // PlatformCapabilityUUID
    0x00, 0x00, 0x03, 0x06, // dwWindowsVersion
    0xA2, 0x00, // wMSOSDescriptorSetTotalLength
    0x41, // bMS_VendorCode
    0x00, // bAltEnumCode
];

const DESCRIPTOR_SET: &[u8] = &[
    // Microsoft OS 2.0 descriptor set header
    0x0A, 0x00, // wLength
    0x00, 0x00, // wDescriptorType
    0x00, 0x00, 0x03, 0x06, // dwWindowsVersion
    0xA2, 0x00, // wTotalLength
    // Microsoft OS 2.0 compatible ID descriptor
    0x14, 0x00, // wLength
    0x03, 0x00, // wDescriptorType
    b'W', b'I', b'N', b'U', b'S', b'B', 0x00, 0x00, // compatible ID
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // sub-compatible ID
    // Microsoft OS 2.0 registry property descriptor
    0x84, 0x00, // wLength
    0x04, 0x00, // wDescriptorType
    0x01, 0x00, // wPropertyDataType
    0x2A, 0x00, // wPropertyNameLength
    b'D', 0x00, b'e', 0x00, b'v', 0x00, b'i', 0x00, // bPropertyName
    b'c', 0x00, b'e', 0x00, b'I', 0x00, b'n', 0x00, // bPropertyName
    b't', 0x00, b'e', 0x00, b'r', 0x00, b'f', 0x00, // bPropertyName
    b'a', 0x00, b'c', 0x00, b'e', 0x00, b'G', 0x00, // bPropertyName
    b'U', 0x00, b'I', 0x00, b'D', 0x00, b's', 0x00, // bPropertyName
    0x00, 0x00, // bPropertyName
    0x50, 0x00, // wPropertyDataLength
    b'{', 0x00, b'D', 0x00, b'4', 0x00, b'B', 0x00, // bPropertyData
    b'5', 0x00, b'9', 0x00, b'8', 0x00, b'8', 0x00, // bPropertyData
    b'4', 0x00, b'-', 0x00, b'A', 0x00, b'D', 0x00, // bPropertyData
    b'1', 0x00, b'0', 0x00, b'-', 0x00, b'4', 0x00, // bPropertyData
    b'5', 0x00, b'C', 0x00, b'5', 0x00, b'-', 0x00, // bPropertyData
    b'8', 0x00, b'F', 0x00, b'9', 0x00, b'3', 0x00, // bPropertyData
    b'-', 0x00, b'B', 0x00, b'B', 0x00, b'7', 0x00, // bPropertyData
    b'A', 0x00, b'0', 0x00, b'9', 0x00, b'7', 0x00, // bPropertyData
    b'7', 0x00, b'8', 0x00, b'8', 0x00, b'B', 0x00, // bPropertyData
    b'8', 0x00, b'}', 0x00, 0x00, 0x00, 0x00, 0x00, // bPropertyData
];

impl<B: UsbBus> UsbClass<B> for WinUsbClass {
    fn get_bos_descriptors(&self, writer: &mut BosWriter) -> usb_device::Result<()> {
        writer.capability(0x05, CAPABILITIES)
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();
        // MS OS 2.0 get descriptors request
        if req.request_type == control::RequestType::Vendor
            && req.request == 0x41 // bMS_VendorCode
            && req.index == 0x0007
        {
            xfer.accept_with_static(DESCRIPTOR_SET).ok();
        }
    }
}
