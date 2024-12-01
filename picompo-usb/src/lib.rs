mod pipe;

use std::{collections::HashMap, ffi::CStr, sync::Mutex};

use nusb::DeviceId;
use once_cell::sync::Lazy;
use pipe::Pipe;

#[derive(Default)]
struct Picompo {
    next_handle: u32,
    id_map: HashMap<u32, DeviceId>,
    pool: HashMap<DeviceId, Pipe>,
}

impl Picompo {
    fn find_device(name: &str) -> Option<nusb::DeviceInfo> {
        let device_info = nusb::list_devices()
            .unwrap()
            .find(|device_info| device_info.serial_number() == Some(name));
        Some(device_info?)
    }

    fn generate_handle(&mut self) -> u32 {
        self.next_handle += 1;
        self.next_handle
    }

    pub fn open(&mut self, name: &str) -> Option<u32> {
        let device_info = Self::find_device(name)?;
        let device_id = device_info.id();
        if self.pool.contains_key(&device_id) {
            return self.pool.get(&device_id).map(|pipe| pipe.handle());
        }
        let device = device_info.open().unwrap();
        let iface = device.claim_interface(0).unwrap();
        let handle = self.generate_handle();
        self.pool.insert(device_id, Pipe::new(iface, handle));
        self.id_map.insert(handle, device_id);
        Some(handle)
    }

    pub fn send(&mut self, handle: u32, ep: u8, data: &[u8]) -> Result<(), ()> {
        println!("SEND handle: {}, len: {}, data: {:02x?}", handle, data.len(), data);
        let device_id = *self.id_map.get(&handle).ok_or(())?;
        let pipe = self.pool.get_mut(&device_id).ok_or(())?;
        pipe.send(ep, data);
        Ok(())
    }

    pub fn recv(&mut self, handle: u32, ep: u8, data: &mut [u8]) -> Result<(), ()> {
        let device_id = *self.id_map.get(&handle).ok_or(())?;
        let pipe = self.pool.get_mut(&device_id).ok_or(())?;
        let input = pipe.recv(ep);
        let len = data.len().min(input.len());
        data[..len].copy_from_slice(&input[..len]);
        Ok(())
    }
}

static PICOMPO: Lazy<Mutex<Picompo>> = Lazy::new(|| Mutex::new(Picompo::default()));

/// # Safety
/// `name` は NULL 終端の文字列を指す有効なポインタでなければならない。
#[no_mangle]
pub unsafe extern "C" fn PICOMPO_open(name: *const i8) -> u32 {
    let mut picompo = PICOMPO.lock().unwrap();

    let name = CStr::from_ptr(name).to_str().unwrap();

    picompo.open(name).unwrap_or(0)
}

/// # Safety
/// `data` は読み取り可能な `data_len` バイトのバッファを指すポインタでなければならない。
#[no_mangle]
pub unsafe extern "C" fn PICOMPO_send(handle: u32, ep: u8, data: *const u8, data_len: u32) -> i32 {
    let mut picompo = PICOMPO.lock().unwrap();

    let data = std::slice::from_raw_parts(data, data_len as usize);

    if picompo.send(handle, ep, data).is_ok() {
        0
    } else {
        -1
    }
}

/// # Safety
/// `buf` は書き込み可能な `buf_len` バイトのバッファを指すポインタでなければならない。
#[no_mangle]
pub unsafe extern "C" fn PICOMPO_recv(handle: u32, ep: u8, buf: *mut u8, buf_len: u32) -> i32 {
    let mut picompo = PICOMPO.lock().unwrap();

    let buf = std::slice::from_raw_parts_mut(buf, buf_len as usize);

    if picompo.recv(handle, ep, &mut *buf).is_ok() {
        0
    } else {
        -1
    }
}
