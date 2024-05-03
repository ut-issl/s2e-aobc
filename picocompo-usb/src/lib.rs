mod rw0003;

use std::{collections::HashMap, ffi::CStr, sync::Mutex};

use hidapi::HidDevice;
use once_cell::sync::Lazy;
use rw0003::Rw0003;

#[derive(Default)]
struct Picocompo {
    next_handle: u32,
    rw0003_pool: HashMap<u32, Rw0003>,
}

impl Picocompo {
    fn open_device(name: &str) -> Option<HidDevice> {
        let hidapi = hidapi::HidApi::new().unwrap();
        let device_info = hidapi
            .device_list()
            .find(|device_info| device_info.product_string() == Some(name));

        let Some(device_info) = device_info else {
            return None;
        };

        Some(device_info.open_device(&hidapi).unwrap())
    }

    fn generate_handle(&mut self) -> u32 {
        self.next_handle += 1;
        self.next_handle
    }

    pub fn open_rw0003(&mut self, name: &str) -> Option<u32> {
        let device = Self::open_device(name)?;
        let handle = self.generate_handle();
        self.rw0003_pool.insert(handle, Rw0003::new(device));
        Some(handle)
    }
}

static PICOCOMPO: Lazy<Mutex<Picocompo>> = Lazy::new(|| Mutex::new(Picocompo::default()));

#[no_mangle]
pub unsafe extern "C" fn PICOCOMPO_rw0003_open(name: *const i8) -> u32 {
    let mut picocompo = PICOCOMPO.lock().unwrap();

    let name = CStr::from_ptr(name).to_str().unwrap();

    picocompo.open_rw0003(name).unwrap_or(0)
}
