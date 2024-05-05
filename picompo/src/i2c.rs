use core::ops::{Deref, DerefMut};

use rp2040_hal::{self as hal, pac};

type HalI2c<I2C, SDA, SCL> = hal::I2C<
    I2C,
    (
        hal::gpio::Pin<SDA, hal::gpio::FunctionI2c, hal::gpio::PullDown>,
        hal::gpio::Pin<SCL, hal::gpio::FunctionI2c, hal::gpio::PullDown>,
    ),
    hal::i2c::Peripheral,
>;

pub use hal::i2c::peripheral::Event as LowLevelEvent;

/// I2C のペリフェラル（スレーブ）を表すトレイト
///
/// 型パラメータを消去して書きやすくするためだけにトレイトを用意している
pub trait Peripheral {
    fn next_low_level_event(&mut self) -> Option<LowLevelEvent>;
    fn read(&mut self, buffer: &mut [u8]) -> usize;
    fn write(&mut self, data: &[u8]) -> usize;
    fn rx_fifo_used(&self) -> u8;
    fn rx_fifo_available(&self) -> u8;
    fn rx_fifo_empty(&self) -> bool;
    fn tx_fifo_used(&self) -> u8;
    fn tx_fifo_available(&self) -> u8;
    fn tx_fifo_full(&self) -> bool;
}

impl<I2C, SDA, SCL> Peripheral for HalI2c<I2C, SDA, SCL>
where
    SDA: hal::gpio::PinId,
    SCL: hal::gpio::PinId,
    I2C: core::ops::Deref<Target = pac::i2c0::RegisterBlock>,
{
    fn next_low_level_event(&mut self) -> Option<LowLevelEvent> {
        HalI2c::next_event(self)
    }

    fn read(&mut self, buf: &mut [u8]) -> usize {
        HalI2c::read(self, buf)
    }

    fn write(&mut self, buf: &[u8]) -> usize {
        HalI2c::write(self, buf)
    }

    fn rx_fifo_used(&self) -> u8 {
        HalI2c::rx_fifo_used(self)
    }

    fn rx_fifo_available(&self) -> u8 {
        HalI2c::rx_fifo_available(self)
    }

    fn rx_fifo_empty(&self) -> bool {
        HalI2c::rx_fifo_empty(self)
    }

    fn tx_fifo_used(&self) -> u8 {
        HalI2c::tx_fifo_used(self)
    }

    fn tx_fifo_available(&self) -> u8 {
        HalI2c::tx_fifo_available(self)
    }

    fn tx_fifo_full(&self) -> bool {
        HalI2c::tx_fifo_full(self)
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    Writing,
    Reading,
}

pub enum Event {
    StartWrite,
    TransferWrite,
    StopWrite,
    StartRead,
    TransferRead,
    StopRead,
}

pub struct StateMachine<I2C> {
    i2c: I2C,
    state: State,
    pending_event: Option<Event>,
}

impl<I2C> StateMachine<I2C>
where
    I2C: Peripheral,
{
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            state: State::Idle,
            pending_event: None,
        }
    }

    pub fn next_event(&mut self) -> Option<Event> {
        // Start, Stop, Restart は他のアドレス宛のトランザクションが Start, Stop, Restart したときにも発生する
        match self.i2c.next_low_level_event()? {
            LowLevelEvent::TransferWrite => {
                if self.state == State::Writing {
                    Some(Event::TransferWrite)
                } else {
                    // トランザクション内で初回の TransferWrite のとき
                    self.state = State::Writing;
                    self.pending_event = Some(Event::TransferWrite);
                    Some(Event::StartWrite)
                }
            }
            LowLevelEvent::TransferRead => {
                if self.state == State::Reading {
                    Some(Event::TransferRead)
                } else {
                    // トランザクション内で初回の TransferRead のとき
                    self.state = State::Reading;
                    self.pending_event = Some(Event::TransferRead);
                    Some(Event::StartRead)
                }
            }
            LowLevelEvent::Restart | LowLevelEvent::Stop => match self.state {
                State::Writing => {
                    self.state = State::Idle;
                    Some(Event::StopWrite)
                }
                State::Reading => {
                    self.state = State::Idle;
                    Some(Event::StopRead)
                }
                State::Idle => None,
            },
            LowLevelEvent::Start => None,
        }
    }
}

impl<I2C> Deref for StateMachine<I2C> {
    type Target = I2C;

    fn deref(&self) -> &Self::Target {
        &self.i2c
    }
}

impl<I2C> DerefMut for StateMachine<I2C> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.i2c
    }
}
