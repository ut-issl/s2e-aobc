use core::ops::{Deref, DerefMut};

#[derive(Default)]
pub struct Buffer<const N: usize> {
    inner: heapless::Vec<u8, N>,
    start_cursor: usize,
}

impl<const N: usize> Buffer<N> {
    pub fn as_slice(&self) -> &[u8] {
        &self.inner[self.start_cursor..]
    }

    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.inner[self.start_cursor..]
    }

    pub fn clear(&mut self) {
        self.start_cursor = 0;
        self.inner.clear();
    }

    #[allow(clippy::result_unit_err)]
    pub fn extend_from_slice(&mut self, other: &[u8]) -> Result<(), ()> {
        self.inner.extend_from_slice(other)
    }

    pub fn consume(&mut self, n: usize) {
        self.start_cursor += n;
    }

    pub fn write<E>(&mut self, f: impl FnOnce(&mut [u8]) -> Result<usize, E>) -> Result<usize, E> {
        let capacity = self.inner.capacity();
        let orig_len = self.inner.len();
        unsafe {
            self.inner.set_len(capacity);
        }
        match f(&mut self.inner[orig_len..]) {
            Ok(written) => {
                self.inner.truncate(orig_len + written);
                Ok(written)
            },
            Err(e) => {
                self.inner.truncate(orig_len);
                Err(e)
            },
        }
    }
}

impl<const N: usize> Deref for Buffer<N> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

impl<const N: usize> DerefMut for Buffer<N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.as_mut_slice()
    }
}
