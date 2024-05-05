use zerocopy::{AsBytes, FromBytes, FromZeroes};

#[derive(Default, Clone, PartialEq, Eq, AsBytes, FromZeroes, FromBytes)]
#[repr(C)]
pub struct Header {
    pub destination_address: u8,
    pub source_address: u8,
    pub message_control_field: MessageControlField,
}

#[derive(Default, Clone, Copy, PartialEq, Eq, AsBytes, FromZeroes, FromBytes)]
#[repr(C)]
pub struct MessageControlField(u8);

impl MessageControlField {
    pub fn poll_final(&self) -> bool {
        self.0 & (1 << 7) != 0
    }

    pub fn set_poll_final(&mut self, value: bool) {
        if value {
            self.0 |= 1 << 7;
        } else {
            self.0 &= !(1 << 7);
        }
    }

    pub fn b_bit(&self) -> bool {
        self.0 & (1 << 6) != 0
    }

    pub fn set_b_bit(&mut self, value: bool) {
        if value {
            self.0 |= 1 << 6;
        } else {
            self.0 &= !(1 << 6);
        }
    }

    pub fn ack(&self) -> bool {
        self.0 & (1 << 5) != 0
    }

    pub fn set_ack(&mut self, value: bool) {
        if value {
            self.0 |= 1 << 5;
        } else {
            self.0 &= !(1 << 5);
        }
    }

    pub fn command_code(&self) -> u8 {
        self.0 & 0x1F
    }

    pub fn set_command_code(&mut self, value: u8) {
        self.0 = (self.0 & !0x1F) | (value & 0x1F);
    }
}

impl From<u8> for MessageControlField {
    fn from(value: u8) -> Self {
        Self(value)
    }
}

impl From<MessageControlField> for u8 {
    fn from(value: MessageControlField) -> u8 {
        value.0
    }
}
