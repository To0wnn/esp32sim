//! A Nordic nRF24L01+ 2.4 GHz transceiver on a board's SPI bus, byte by byte as the chip sees it
//! between a falling and a rising CSN (nRF24L01+ product specification v1.0, section 8.3: the
//! STATUS register shifts out with the command byte). Commands: R_REGISTER / W_REGISTER
//! (0x00 / 0x20 + register, the 5-byte address registers 0x0A, 0x0B and 0x10 LSB first),
//! R_RX_PAYLOAD (0x61), R_RX_PL_WID (0x60), W_TX_PAYLOAD (0xA0), W_TX_PAYLOAD_NOACK (0xB0),
//! FLUSH_TX / FLUSH_RX (0xE1 / 0xE2), ACTIVATE (0x50, a no-op on the + part) and NOP (0xFF).
//! Register reset values from section 9.1.
//!
//! The air is one virtual transmitter (`Air`): while the receiver listens (CE high, PWR_UP and
//! PRIM_RX) on its channel, RPD (register 0x09, section 6.4) reads 1, and a pipe whose address
//! matches receives its packet every `period_ns`.
//! shortcut: data rate and CRC settings are not compared with the transmitter's, and a static
//! payload length different from the packet's still receives (padded or cut). A transmitted
//! packet reaches nobody: with auto-ack it ends in MAX_RT, without it in TX_DS, both at once.
use std::collections::VecDeque;

const CONFIG: usize = 0x00;
const EN_AA: usize = 0x01;
const EN_RXADDR: usize = 0x02;
const SETUP_AW: usize = 0x03;
const RF_CH: usize = 0x05;
const STATUS: usize = 0x07;
const RPD: usize = 0x09;
const RX_PW_P0: usize = 0x11;
const FIFO_STATUS: usize = 0x17;
const DYNPD: usize = 0x1c;
const FEATURE: usize = 0x1d;
/// STATUS bits (section 9.1)
const RX_DR: u8 = 0x40;
const TX_DS: u8 = 0x20;
const MAX_RT: u8 = 0x10;

/// A transmitter on the air: its channel, address (LSB first, as written to TX_ADDR) and
/// packet text; `#n` counts the packets.
#[derive(Clone, Debug)]
pub struct Air { pub channel: u8, pub address: [u8; 5], pub text: String, pub period_ns: u64 }

#[derive(Clone, Copy, Default, Debug)]
pub struct Nrf24Stats { pub frames: u64, pub received: u64, pub sent: u64 }

pub struct Nrf24 {
    regs: [u8; 0x20],
    /// RX_ADDR_P0, RX_ADDR_P1 and TX_ADDR, LSB first
    addr: [[u8; 5]; 3],
    selected: bool,
    ce: bool,
    cmd: u8,
    count: usize,
    payload: Vec<u8>,
    rx: VecDeque<(u8, Vec<u8>)>,
    /// TX FIFO: (sent with W_TX_PAYLOAD_NOACK, payload)
    tx: VecDeque<(bool, Vec<u8>)>,
    pub air: Option<Air>,
    now_ns: u64,
    next_packet_ns: u64,
    pub stats: Nrf24Stats,
}

impl Nrf24 {
    pub fn new(air: Option<Air>) -> Self {
        let mut regs = [0u8; 0x20];
        for (r, v) in [(CONFIG, 0x08), (EN_AA, 0x3f), (EN_RXADDR, 0x03), (SETUP_AW, 0x03), (0x04, 0x03), (RF_CH, 0x02), (0x06, 0x0e),
                       (STATUS, 0x0e), (0x0c, 0xc3), (0x0d, 0xc4), (0x0e, 0xc5), (0x0f, 0xc6), (FIFO_STATUS, 0x11)] { regs[r] = v; }
        Nrf24 { regs, addr: [[0xe7; 5], [0xc2; 5], [0xe7; 5]], selected: false, ce: false, cmd: 0, count: 0, payload: Vec::new(),
                rx: VecDeque::new(), tx: VecDeque::new(), air, now_ns: 0, next_packet_ns: 0, stats: Nrf24Stats::default() }
    }

    /// Chip select: `true` while CSN is driven low.
    pub fn select(&mut self, low: bool) {
        if low == self.selected { return; }
        if !low { self.finish(); }
        self.selected = low;
        self.count = 0;
        self.payload.clear();
    }
    pub fn selected(&self) -> bool { self.selected }

    /// The CE line: rising while listening starts a receive (RPD is sampled for the channel);
    /// while sending it transmits the TX FIFO.
    pub fn set_ce(&mut self, high: bool) {
        if high == self.ce { return; }
        self.ce = high;
        if high && self.listening() {
            self.regs[RPD] = u8::from(self.air.as_ref().is_some_and(|a| a.channel == self.regs[RF_CH]));
            self.next_packet_ns = self.now_ns;
            self.advance_to(self.now_ns);
        }
        self.try_send();
    }

    fn listening(&self) -> bool { self.ce && self.regs[CONFIG] & 3 == 3 }

    /// Emulated time: a listening receiver on the transmitter's channel and address gets its packet.
    pub fn advance_to(&mut self, now_ns: u64) {
        self.now_ns = now_ns;
        if !self.listening() || now_ns < self.next_packet_ns || self.rx.len() >= 3 { return; }
        let Some(air) = &self.air else { return };
        if air.channel != self.regs[RF_CH] { return; }
        let Some(pipe) = self.pipe_for(&air.address) else { return };
        let mut packet = format!("{} #{}", air.text, self.stats.received + 1).into_bytes();
        packet.truncate(32);
        let dynamic = self.regs[FEATURE] & 4 != 0 && self.regs[DYNPD] >> pipe & 1 != 0;
        if !dynamic {
            let width = usize::from(self.regs[RX_PW_P0 + usize::from(pipe)] & 0x3f).min(32);
            if width == 0 { return; }
            packet.resize(width, 0);
        }
        self.next_packet_ns = now_ns + air.period_ns;
        self.rx.push_back((pipe, packet));
        self.regs[STATUS] |= RX_DR;
        self.stats.received += 1;
    }

    /// The enabled pipe whose address, `SETUP_AW` bytes wide, is `address` (pipes 2-5 share the
    /// upper bytes of pipe 1, section 7.6).
    fn pipe_for(&self, address: &[u8; 5]) -> Option<u8> {
        let width = usize::from(self.regs[SETUP_AW] & 3) + 2;
        (0..6u8).find(|&p| {
            if self.regs[EN_RXADDR] >> p & 1 == 0 { return false; }
            let dynamic = self.regs[FEATURE] & 4 != 0 && self.regs[DYNPD] >> p & 1 != 0;
            if !dynamic && (self.regs[RX_PW_P0 + usize::from(p)] & 0x3f) == 0 { return false; }
            let mut a = self.addr[usize::from(p.min(1))];
            if p >= 2 { a[0] = self.regs[0x0a + usize::from(p)]; }
            a[..width] == address[..width]
        })
    }

    fn try_send(&mut self) {
        if !self.ce || self.regs[CONFIG] & 3 != 2 { return; }
        let Some(&(no_ack, _)) = self.tx.front() else { return };
        // pipe 0 auto-ack waits for an ACK nobody sends: the packet stays and MAX_RT rises
        if self.regs[EN_AA] & 1 != 0 && !no_ack {
            self.regs[STATUS] |= MAX_RT;
        } else {
            self.tx.pop_front();
            self.regs[STATUS] |= TX_DS;
        }
        self.stats.sent += 1;
    }

    fn status(&self) -> u8 {
        let pipe = self.rx.front().map_or(7, |&(p, _)| p);
        self.regs[STATUS] & 0x70 | pipe << 1 | u8::from(self.tx.len() >= 3)
    }

    fn read_reg(&self, r: usize, i: usize) -> u8 {
        match r {
            0x0a | 0x0b => self.addr[r - 0x0a][i % 5],
            0x10 => self.addr[2][i % 5],
            STATUS => self.status(),
            FIFO_STATUS => u8::from(self.rx.is_empty()) | u8::from(self.rx.len() >= 3) << 1
                | u8::from(self.tx.is_empty()) << 4 | u8::from(self.tx.len() >= 3) << 5,
            _ => self.regs[r],
        }
    }

    fn write_reg(&mut self, r: usize, i: usize, v: u8) {
        match r {
            0x0a | 0x0b => self.addr[r - 0x0a][i % 5] = v,
            0x10 => self.addr[2][i % 5] = v,
            _ if i > 0 => {}
            // writing 1 clears an interrupt flag
            STATUS => self.regs[STATUS] &= !(v & 0x70),
            RPD | FIFO_STATUS | 0x08 => {}
            _ => self.regs[r] = v,
        }
    }

    /// One byte in on MOSI, the byte the chip shifts out on MISO at the same time.
    pub fn exchange(&mut self, mosi: u8) -> u8 {
        if !self.selected { return 0xff; }
        let i = self.count;
        self.count += 1;
        if i == 0 { self.cmd = mosi; return self.status(); }
        let k = i - 1;
        match self.cmd {
            c @ 0x00..=0x1f => self.read_reg(usize::from(c), k),
            c @ 0x20..=0x3f => { self.write_reg(usize::from(c & 0x1f), k, mosi); 0 }
            0x60 => self.rx.front().map_or(0, |(_, p)| p.len() as u8),
            0x61 => self.rx.front().and_then(|(_, p)| p.get(k).copied()).unwrap_or(0),
            0xa0 | 0xb0 => { if self.payload.len() < 32 { self.payload.push(mosi); } 0 }
            _ => 0,
        }
    }

    /// CSN rose: commands that act on the FIFOs take effect.
    fn finish(&mut self) {
        if self.count == 0 { return; }
        self.stats.frames += 1;
        match self.cmd {
            0x61 if self.count > 1 => { self.rx.pop_front(); }
            0xa0 | 0xb0 if !self.payload.is_empty() && self.tx.len() < 3 => {
                let p = std::mem::take(&mut self.payload);
                self.tx.push_back((self.cmd == 0xb0, p));
                self.try_send();
            }
            0xe1 => self.tx.clear(),
            0xe2 => self.rx.clear(),
            _ => {}
        }
    }

    pub fn transfer(&mut self, tx: &[u8], rx_len: usize) -> Vec<u8> { super::spi_exchange(tx, rx_len, |b| self.exchange(b)) }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(n: &mut Nrf24, bytes: &[u8]) -> Vec<u8> {
        n.select(true);
        let out = n.transfer(bytes, bytes.len());
        n.select(false);
        out
    }

    #[test]
    fn begin_scan_and_receive_as_the_rf24_library_does() {
        let air = Air { channel: 76, address: [0xe7; 5], text: "bench".into(), period_ns: 1000 };
        let mut n = Nrf24::new(Some(air));
        // RF24::begin: CONFIG written, then read back with PWR_UP set
        frame(&mut n, &[0x20, 0x7c]);
        assert_eq!(frame(&mut n, &[0x00, 0xff])[1], 0x7c);
        assert_eq!(frame(&mut n, &[0x03, 0xff])[1], 3, "SETUP_AW: 5-byte addresses (isChipConnected)");
        assert_eq!(frame(&mut n, &[0xff])[0], 0x0e, "STATUS: RX FIFO empty");
        // scan: listen on channel 75 then 76; RPD latches per receive
        frame(&mut n, &[0x25, 75]);
        frame(&mut n, &[0x20, 0x0f]);
        n.set_ce(true); n.set_ce(false);
        assert_eq!(frame(&mut n, &[0x09, 0xff])[1], 0);
        frame(&mut n, &[0x25, 76]);
        n.set_ce(true); n.set_ce(false);
        assert_eq!(frame(&mut n, &[0x09, 0xff])[1], 1, "the transmitter's channel");
        // receive on pipe 1 with the transmitter's address and dynamic payloads
        frame(&mut n, &[0x2b, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7]);
        frame(&mut n, &[0x3d, 0x04]);
        frame(&mut n, &[0x3c, 0x02]);
        n.set_ce(true);
        assert_eq!(frame(&mut n, &[0x17, 0xff])[1] & 1, 0, "RX FIFO not empty");
        assert_eq!(frame(&mut n, &[0xff])[0] >> 1 & 7, 1, "pipe 1");
        let len = frame(&mut n, &[0x60, 0xff])[1];
        assert_eq!(len, 8);
        let p = frame(&mut n, &[0x61, 0, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(&p[1..], b"bench #1");
        frame(&mut n, &[0x27, 0x40]);
        assert_eq!(frame(&mut n, &[0xff])[0], 0x0e, "read and cleared");
        n.advance_to(500);
        assert_eq!(frame(&mut n, &[0x17, 0xff])[1] & 1, 1, "the next packet comes a period later");
        n.advance_to(1000);
        assert_eq!(frame(&mut n, &[0x17, 0xff])[1] & 1, 0);
        // send with auto-ack: nobody answers
        n.set_ce(false);
        frame(&mut n, &[0x20, 0x0e]);
        frame(&mut n, &[0xa0, 1, 2, 3]);
        n.set_ce(true);
        assert_eq!(frame(&mut n, &[0xff])[0] & MAX_RT, MAX_RT);
    }
}
