mod pipe;

use std::{collections::HashMap, ffi::CStr, sync::Mutex};

use once_cell::sync::Lazy;
use pipe::Pipe;

#[derive(Default)]
struct Picompo {
    next_handle: u32,
    pool: HashMap<u32, Pipe>,
}

impl Picompo {
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

    pub fn open(&mut self, name: &str) -> Option<u32> {
        let device = Self::open_device(name)?;
        let handle = self.generate_handle();
        self.pool.insert(handle, Pipe::new(device));
        Some(handle)
    }

    pub fn send(&mut self, handle: u32, data: &[u8]) -> Result<(), ()> {
        let rw0003 = self.pool.get_mut(&handle).ok_or(())?;
        rw0003.send(data);
        Ok(())
    }

    pub fn recv(&mut self, handle: u32, data: &mut [u8]) -> Result<(), ()> {
        let rw0003 = self.pool.get_mut(&handle).ok_or(())?;
        let input = rw0003.recv();
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
pub unsafe extern "C" fn PICOMPO_send(handle: u32, data: *const u8, data_len: u32) -> i32 {
    let mut picompo = PICOMPO.lock().unwrap();

    let data = std::slice::from_raw_parts(data, data_len as usize);

    if picompo.send(handle, data).is_ok() {
        0
    } else {
        -1
    }
}

/// # Safety
/// `buf` は書き込み可能な `buf_len` バイトのバッファを指すポインタでなければならない。
#[no_mangle]
pub unsafe extern "C" fn PICOMPO_recv(handle: u32, buf: *mut u8, buf_len: u32) -> i32 {
    let mut picompo = PICOMPO.lock().unwrap();

    let buf = std::slice::from_raw_parts_mut(buf, buf_len as usize);

    if picompo.recv(handle, &mut *buf).is_ok() {
        0
    } else {
        -1
    }
}
