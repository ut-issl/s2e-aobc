mod rw0003;

use std::{collections::HashMap, ffi::CStr, sync::Mutex};

use once_cell::sync::Lazy;
use rw0003::{InputReport, Rw0003};

#[derive(Default)]
struct Picocompo {
    next_handle: u32,
    rw0003_pool: HashMap<u32, Rw0003>,
}

impl Picocompo {
    fn open_device(name: &str) -> Option<nusb::Interface> {
        let device_info = nusb::list_devices()
            .unwrap()
            .find(|device_info| device_info.serial_number() == Some(name));

        let device_info = device_info?;
        let device = device_info.open().unwrap();
        let iface = device.claim_interface(0).unwrap();
        Some(iface)
    }

    fn generate_handle(&mut self) -> u32 {
        self.next_handle += 1;
        self.next_handle
    }

    pub fn rw0003_open(&mut self, name: &str) -> Option<u32> {
        let device = Self::open_device(name)?;
        let handle = self.generate_handle();
        self.rw0003_pool.insert(handle, Rw0003::new(device));
        Some(handle)
    }

    pub fn rw0003_send(&mut self, handle: u32, temperature: f32, speed: f32) -> Result<(), ()> {
        let rw0003 = self.rw0003_pool.get_mut(&handle).ok_or(())?;
        rw0003.send_output(rw0003::OutputReport {
            temperature: temperature.to_le_bytes(),
            speed: speed.to_le_bytes(),
        });
        Ok(())
    }

    pub fn rw0003_recv(&mut self, handle: u32, input: &mut InputReport) -> Result<(), ()> {
        let rw0003 = self.rw0003_pool.get_mut(&handle).ok_or(())?;
        *input = rw0003.recv_input();
        Ok(())
    }
}

static PICOCOMPO: Lazy<Mutex<Picocompo>> = Lazy::new(|| Mutex::new(Picocompo::default()));

/// # Safety
/// `name` は NULL 終端の文字列を指す有効なポインタでなければならない。
#[no_mangle]
pub unsafe extern "C" fn PICOCOMPO_rw0003_open(name: *const i8) -> u32 {
    let mut picocompo = PICOCOMPO.lock().unwrap();

    let name = CStr::from_ptr(name).to_str().unwrap();

    picocompo.rw0003_open(name).unwrap_or(0)
}

#[no_mangle]
pub extern "C" fn PICOCOMPO_rw0003_send(handle: u32, temperature: f32, speed: f32) -> i32 {
    let mut picocompo = PICOCOMPO.lock().unwrap();

    if picocompo.rw0003_send(handle, temperature, speed).is_ok() {
        0
    } else {
        -1
    }
}

/// # Safety
/// `input` は書き込み可能な InputReport を指すポインタでなければならない。
#[no_mangle]
pub unsafe extern "C" fn PICOCOMPO_rw0003_recv(handle: u32, input: *mut InputReport) -> i32 {
    let mut picocompo = PICOCOMPO.lock().unwrap();

    if picocompo.rw0003_recv(handle, &mut *input).is_ok() {
        0
    } else {
        -1
    }
}
