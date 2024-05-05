use core::convert::Infallible;

use slippers::{SlipDecoder, SlipEncoder};
use zerocopy::{AsBytes, FromBytes, FromZeroes, LE, U16};

use crate::buffer::Buffer;
use crate::i2c::{Event, Peripheral, StateMachine};
use crate::nsp;

const CRC: crc::Crc<u16> = crc::Crc::<u16>::new(&crc::CRC_16_MCRF4XX);

pub trait Service {
    fn on_command(&mut self, poll: bool, code: u8, body: &[u8], reply: impl FnOnce(&[u8]));
}

pub struct NspSlipI2c<I2C> {
    inner: SlipI2c<I2C>,
    self_addr: u8,
    ctrl_addr: u8,
}

impl<I2C> NspSlipI2c<I2C>
where
    I2C: Peripheral,
{
    pub fn new(i2c: I2C, self_addr: u8, ctrl_addr: u8) -> Self {
        Self {
            inner: SlipI2c::new(i2c),
            self_addr,
            ctrl_addr,
        }
    }

    pub fn poll(&mut self, service: &mut impl Service) {
        self.inner.poll();

        if let Some(dec) = self.inner.read() {
            let mut cmd_buf = heapless::Vec::<u8, 64>::new();
            // I2C のデータには destination address が含まれていないが、
            // NSP のヘッダとしては必要なのでここで追加する。
            // CRC の計算にも destination address が含まれるので、絶対に必要。
            // Sinclair の仕様がほんとうにダメなのでこのハックが必要。
            cmd_buf.push(self.self_addr).expect("never fail");
            for byte in dec.iter() {
                cmd_buf.push(byte).expect("buffer overflow");
            }
            let Some((cmd_header, cmd_body)) = parse_nsp_message(&cmd_buf) else {
                // パースに失敗した場合は無視する
                return;
            };
            let poll = cmd_header.message_control_field.poll_final();
            let code = cmd_header.message_control_field.command_code();
            service.on_command(poll, code, cmd_body, |reply_body| {
                let mut reply_buf = heapless::Vec::<u8, 64>::new();
                let mut mcf = nsp::MessageControlField::default();
                mcf.set_poll_final(true);
                mcf.set_command_code(code);
                mcf.set_ack(true);
                let reply_header = nsp::Header {
                    destination_address: self.ctrl_addr,
                    source_address: self.self_addr,
                    message_control_field: mcf,
                };
                build_nsp_message(&mut reply_buf, &reply_header, reply_body).expect("buffer overflow");
                // reply のときには destination address と source address は不要なので削る。
                // これも Sinclair の仕様の意味不明なところ。
                self.inner.write(&reply_buf[2..]).expect("data is too large");
            });
            // コマンドハンドラが reply をしなかった場合は Clock Stretching し続けてしまう。
            // それを避けるために空データの送信を開始する。
            // 既に送信済みの場合は無視されるので害はない。
            self.inner.write(&[]).ok();
        }
    }
}

struct SlipI2c<I2C> {
    i2c_sm: StateMachine<I2C>,
    is_from_i2c_buf_ready: bool,
    from_i2c_buf: Buffer<64>,
    to_i2c_buf: Option<Buffer<64>>,
}

impl<I2C> SlipI2c<I2C>
where
    I2C: Peripheral,
{
    fn new(i2c: I2C) -> Self {
        Self {
            i2c_sm: StateMachine::new(i2c),
            is_from_i2c_buf_ready: true,
            from_i2c_buf: Buffer::default(),
            to_i2c_buf: None,
        }
    }

    fn poll(&mut self) {
        while let Some(event) = self.i2c_sm.next_event() {
            match event {
                Event::StartWrite => {
                    self.from_i2c_buf = Buffer::default();
                    self.is_from_i2c_buf_ready = false;
                    // 送り残しをクリアする
                    self.to_i2c_buf = None;
                }
                Event::TransferWrite => {
                    debug_assert!(!self.is_from_i2c_buf_ready);
                    self.from_i2c_buf
                        .write::<Infallible>(|buf| Ok(self.i2c_sm.read(buf)))
                        .ok();
                }
                Event::StopWrite => {
                    self.is_from_i2c_buf_ready = true;
                }
                Event::StartRead => {}
                Event::TransferRead => {
                    if let Some(to_i2c_buf) = self.to_i2c_buf.as_mut() {
                        if to_i2c_buf.is_empty() {
                            // バッファが枯渇した場合は0で埋める
                            self.i2c_sm.write(&[0]);
                        } else {
                            let written = self.i2c_sm.write(to_i2c_buf);
                            to_i2c_buf.consume(written);
                        }
                    } else {
                        // まだコマンドのハンドリングが完了していない可能性があるのでなにもせず待つ。
                        // いわゆる Clock Stretching に相当する。
                    }
                }
                Event::StopRead => {
                    // 送り残しをクリアする
                    self.to_i2c_buf = None;
                }
            }
        }
    }

    fn read(&mut self) -> Option<SlipDecoder> {
        if self.is_from_i2c_buf_ready {
            self.is_from_i2c_buf_ready = false;
            Some(SlipDecoder::new(self.from_i2c_buf.as_slice()))
        } else {
            // Write トランザクション中はデータが中途半端なので返さない
            None
        }
    }

    fn write(&mut self, data: &[u8]) -> Result<(), ()> {
        if self.to_i2c_buf.is_some() {
            // 既に送信中のフレームがある場合は書き込めない
            return Err(());
        }

        let enc = SlipEncoder::new(data);
        let mut to_i2c_buf = Buffer::default();
        to_i2c_buf.write(|to_i2c_buf| {
            let mut len = 0;
            for byte in enc.iter() {
                if len < to_i2c_buf.len() {
                    to_i2c_buf[len] = byte;
                    len += 1;
                } else {
                    return Err(());
                }
            }
            Ok(len)
        })?;
        self.to_i2c_buf = Some(to_i2c_buf);
        Ok(())
    }
}

// このフッタは NSP 標準ではなく Sinclair の独自仕様と思われる
// （NSP 自体、ハッキリした仕様書がなくて正しいことがわからない）
#[derive(FromZeroes, FromBytes, AsBytes)]
#[repr(C)]
struct Footer {
    crc: U16<LE>,
}

fn parse_nsp_message(bytes: &[u8]) -> Option<(&nsp::Header, &[u8])> {
    let (header_and_body, footer) = zerocopy::Ref::<_, Footer>::new_from_suffix(bytes)?;
    let computed_crc = CRC.checksum(header_and_body);
    if computed_crc != u16::from(footer.crc) {
        defmt::warn!(
            "CRC mismatch: computed={}, received={}",
            computed_crc,
            footer.crc
        );
        return None;
    }
    let (header, body) = zerocopy::Ref::<_, nsp::Header>::new_from_prefix(header_and_body)?;
    Some((header.into_ref(), body))
}

fn build_nsp_message<const N: usize>(
    dst: &mut heapless::Vec<u8, N>,
    header: &nsp::Header,
    body: &[u8],
) -> Result<(), ()> {
    dst.extend_from_slice(header.as_bytes())?;
    dst.extend_from_slice(body)?;
    let crc = CRC.checksum(dst);
    let footer = Footer { crc: crc.into() };
    dst.extend_from_slice(footer.as_bytes())?;
    Ok(())
}
