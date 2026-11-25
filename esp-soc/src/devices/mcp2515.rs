//! A Microchip MCP2515 CAN controller on a board's SPI bus, byte by byte as the chip sees it
//! between a falling and a rising CS (MCP2515 datasheet DS20001801J, section 12): RESET (0xC0),
//! READ (0x03, auto-increment), WRITE (0x02, auto-increment), BIT MODIFY (0x05) and READ STATUS
//! (0xA0, repeated while clocked). After a reset the chip is in configuration mode (CANSTAT 0x80,
//! CANCTRL 0x87, section 10). A REQOP request in CANCTRL takes effect at once in CANSTAT.OPMOD.
//!
//! Setting TXREQ in TXBnCTRL sends the buffer. In loopback mode (section 10.4) the frame lands in
//! RXB0, or RXB1 when RXB0 is full and BUKT is set, with RXnIF and TXnIF raised. In normal mode
//! the bench has no other node, so nobody acknowledges: as on the part, TXREQ stays set, TXERR and
//! MERRF/ERRIF rise and the transmit error counter reaches error passive (TEC 128, EFLG TXEP).
//! shortcut: acceptance filters and masks are not applied, and LOAD TX, RTS, READ RX BUFFER and
//! RX STATUS (unused by the autowp driver) read 0xFF and do nothing.

const CANSTAT: usize = 0x0e;
const CANCTRL: usize = 0x0f;
const TEC: usize = 0x1c;
const CANINTF: usize = 0x2c;
const EFLG: usize = 0x2d;
/// TXBnCTRL bits (section 3.5)
const TXREQ: u8 = 0x08;
const TXERR: u8 = 0x10;

#[derive(Clone, Copy, Default, Debug)]
pub struct Mcp2515Stats { pub frames: u64, pub sent: u64, pub looped_back: u64, pub unacknowledged: u64 }

pub struct Mcp2515 {
    regs: [u8; 0x80],
    selected: bool,
    head: [u8; 4],
    count: usize,
    pub stats: Mcp2515Stats,
}

impl Default for Mcp2515 { fn default() -> Self { Self::new() } }

impl Mcp2515 {
    pub fn new() -> Self {
        let mut m = Mcp2515 { regs: [0; 0x80], selected: false, head: [0; 4], count: 0, stats: Mcp2515Stats::default() };
        m.reset();
        m
    }

    fn reset(&mut self) {
        self.regs = [0; 0x80];
        self.regs[CANSTAT] = 0x80;
        self.regs[CANCTRL] = 0x87;
    }

    /// Chip select: `true` while CS is driven low.
    pub fn select(&mut self, low: bool) {
        if low == self.selected { return; }
        if !low && self.count > 0 { self.stats.frames += 1; }
        self.selected = low;
        self.count = 0;
    }
    pub fn selected(&self) -> bool { self.selected }

    /// The mode CANSTAT.OPMOD reports: 0 normal, 2 loopback, 3 listen-only, 4 configuration.
    pub fn mode(&self) -> u8 { self.regs[CANSTAT] >> 5 }

    /// CANSTAT and CANCTRL appear at the end of every register row (table 11-1).
    fn read(&self, a: usize) -> u8 { self.regs[match a & 0x0f { 0x0e => CANSTAT, 0x0f => CANCTRL, _ => a }] }

    fn write(&mut self, a: usize, v: u8) {
        match a & 0x0f {
            0x0e => return,
            0x0f => { self.regs[CANCTRL] = v; self.regs[CANSTAT] = self.regs[CANSTAT] & 0x1f | v & 0xe0; return; }
            _ => {}
        }
        if matches!(a, 0x30 | 0x40 | 0x50) {
            let was = self.regs[a];
            // only TXREQ and the priority bits are writable (register 3-1)
            self.regs[a] = was & 0x70 | v & 0x0b;
            if v & TXREQ != 0 && was & TXREQ == 0 { self.send((a - 0x30) / 0x10); }
            return;
        }
        self.regs[a] = v;
    }

    fn send(&mut self, n: usize) {
        let base = 0x30 + n * 0x10;
        let dlc = usize::from(self.regs[base + 5] & 0x0f).min(8);
        let frame: Vec<u8> = self.regs[base + 1..base + 6 + dlc].to_vec();
        self.stats.sent += 1;
        match self.mode() {
            2 => {
                let rx = if self.regs[CANINTF] & 1 == 0 { Some(0) } else if self.regs[0x60] & 4 != 0 && self.regs[CANINTF] & 2 == 0 { Some(1) } else { None };
                match rx {
                    Some(r) => {
                        let rbase = 0x61 + r * 0x10;
                        self.regs[rbase..rbase + frame.len()].copy_from_slice(&frame);
                        self.regs[CANINTF] |= 1 << r;
                    }
                    None => self.regs[EFLG] |= 0x40,
                }
                self.regs[base] &= !TXREQ;
                self.regs[CANINTF] |= 4 << n;
                self.stats.looped_back += 1;
            }
            0 => {
                // no acknowledge: the part retransmits until error passive (section 6.6) and keeps TXREQ
                self.regs[base] |= TXERR;
                self.regs[CANINTF] |= 0xa0;
                self.regs[TEC] = 128;
                self.regs[EFLG] |= 0x15;
                self.stats.unacknowledged += 1;
            }
            _ => {}
        }
    }

    /// READ STATUS (figure 12-8): RX0IF, RX1IF, then TXREQ and TXnIF of each transmit buffer.
    fn read_status(&self) -> u8 {
        let f = self.regs[CANINTF];
        let req = |n: usize| u8::from(self.regs[0x30 + n * 0x10] & TXREQ != 0);
        f & 3 | req(0) << 2 | (f >> 2 & 1) << 3 | req(1) << 4 | (f >> 3 & 1) << 5 | req(2) << 6 | (f >> 4 & 1) << 7
    }

    /// One byte in on MOSI, the byte the chip shifts out on MISO at the same time.
    pub fn exchange(&mut self, mosi: u8) -> u8 {
        if !self.selected { return 0xff; }
        let i = self.count;
        self.count += 1;
        if i < self.head.len() { self.head[i] = mosi; }
        let a = usize::from(self.head[1]);
        match self.head[0] {
            0xc0 if i == 0 => { self.reset(); 0xff }
            0x03 if i >= 2 => self.read((a + i - 2) & 0x7f),
            0x02 if i >= 2 => { self.write((a + i - 2) & 0x7f, mosi); 0xff }
            0x05 if i == 3 => { let m = self.head[2]; let v = self.read(a) & !m | mosi & m; self.write(a, v); 0xff }
            0xa0 if i >= 1 => self.read_status(),
            _ => 0xff,
        }
    }

    pub fn transfer(&mut self, tx: &[u8], rx_len: usize) -> Vec<u8> { super::spi_exchange(tx, rx_len, |b| self.exchange(b)) }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(m: &mut Mcp2515, bytes: &[u8]) -> Vec<u8> {
        m.select(true);
        let out = m.transfer(bytes, bytes.len());
        m.select(false);
        out
    }

    #[test]
    fn reset_modes_loopback_and_unacknowledged_send_as_autowp_does() {
        let mut m = Mcp2515::new();
        frame(&mut m, &[0xc0]);
        assert_eq!(frame(&mut m, &[0x03, 0x0e, 0])[2] & 0xe0, 0x80, "configuration mode after reset");
        // setLoopbackMode: BIT MODIFY CANCTRL mask 0xE8
        frame(&mut m, &[0x05, 0x0f, 0xe8, 0x40]);
        assert_eq!(frame(&mut m, &[0x03, 0x0e, 0])[2] & 0xe0, 0x40);
        // sendMessage(TXB0): id 0x123, DLC 2, AA AA; then TXREQ
        frame(&mut m, &[0x02, 0x31, 0x24, 0x60, 0, 0, 2, 0xaa, 0xaa]);
        frame(&mut m, &[0x05, 0x30, 0x08, 0x08]);
        assert_eq!(frame(&mut m, &[0x03, 0x30, 0])[2] & 0x78, 0, "sent: TXREQ clear, no error");
        assert_eq!(frame(&mut m, &[0xa0, 0])[1], 0x09, "READ STATUS: RX0IF and TX0IF");
        assert_eq!(&frame(&mut m, &[0x03, 0x61, 0, 0, 0, 0, 0, 0, 0])[2..], [0x24, 0x60, 0, 0, 2, 0xaa, 0xaa]);
        frame(&mut m, &[0x05, 0x2c, 0x01, 0x00]);
        assert_eq!(frame(&mut m, &[0xa0, 0])[1] & 1, 0, "RX0IF cleared");
        // normal mode: nobody acknowledges
        frame(&mut m, &[0x05, 0x0f, 0xe8, 0x00]);
        frame(&mut m, &[0x02, 0x41, 0x24, 0x60, 0, 0, 1, 0x55]);
        frame(&mut m, &[0x05, 0x40, 0x08, 0x08]);
        assert_eq!(frame(&mut m, &[0x03, 0x40, 0])[2] & 0x18, 0x18, "TXREQ stays, TXERR");
        assert_eq!(&frame(&mut m, &[0x03, 0x1c, 0, 0])[2..], [128, 0], "TEC error passive");
        assert_eq!(frame(&mut m, &[0x03, 0x2d, 0])[2], 0x15, "EFLG TXEP, TXWAR, EWARN");
    }
}
