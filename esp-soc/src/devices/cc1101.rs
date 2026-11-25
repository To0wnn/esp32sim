//! A TI CC1101 sub-GHz transceiver on a board's SPI bus, byte by byte as the chip sees it between
//! a falling and a rising CSn (CC1101 datasheet SWRS061I, section 10). The header byte carries
//! R/W (0x80), burst (0x40) and a 6-bit address; MISO returns the chip status byte with it
//! (section 10.1: CHIP_RDYn 0, STATE in bits 6:4). Addresses 0x00-0x2E are the configuration
//! registers (reset values from table 43); 0x30-0x3D are command strobes, or with the burst bit
//! set on a read, the status registers (section 29.3: PARTNUM 0x00, VERSION 0x14, RSSI,
//! MARCSTATE); 0x3E is the PATABLE.
//!
//! The air is one virtual transmitter (`Air`): in RX, RSSI reads its level when the synthesiser
//! (FREQ2..0, f = FREQ x 26 MHz / 2^16, section 21) is within 10 kHz of it, else the noise floor.
//! shortcut: no packet engine or FIFOs (RXBYTES reads 0), and GDO pins are not driven; a
//! firmware that receives packets or raw OOK on GDO0 sees nothing.

const VERSION: u8 = 0x14;
const NOISE_DBM: f64 = -100.0;
const RESET: [u8; 0x2f] = [
    0x29, 0x2e, 0x3f, 0x07, 0xd3, 0x91, 0xff, 0x04, 0x45, 0x00, 0x00, 0x0f, 0x00, 0x1e, 0xc4, 0xec,
    0x8c, 0x22, 0x02, 0x22, 0xf8, 0x47, 0x07, 0x30, 0x04, 0x36, 0x6c, 0x03, 0x40, 0x91, 0x87, 0x6b,
    0xf8, 0x56, 0x10, 0xa9, 0x0a, 0x20, 0x0d, 0x41, 0x00, 0x59, 0x7f, 0x3f, 0x88, 0x31, 0x0b,
];

/// A carrier on the air: frequency in MHz and level at the antenna in dBm.
#[derive(Clone, Copy, Debug)]
pub struct Air { pub mhz: f64, pub dbm: f64 }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State { Idle, Rx, Tx }

#[derive(Clone, Copy, Default, Debug)]
pub struct Cc1101Stats { pub frames: u64, pub strobes: u64, pub rssi_reads: u64 }

pub struct Cc1101 {
    regs: [u8; 0x2f],
    patable: [u8; 8],
    state: State,
    selected: bool,
    header: u8,
    count: usize,
    pub air: Option<Air>,
    pub stats: Cc1101Stats,
}

impl Cc1101 {
    pub fn new(air: Option<Air>) -> Self {
        Cc1101 { regs: RESET, patable: [0xc6, 0, 0, 0, 0, 0, 0, 0], state: State::Idle, selected: false, header: 0, count: 0, air, stats: Cc1101Stats::default() }
    }

    /// Chip select: `true` while CSn is driven low.
    pub fn select(&mut self, low: bool) {
        if low == self.selected { return; }
        if !low && self.count > 0 { self.stats.frames += 1; }
        self.selected = low;
        self.count = 0;
    }
    pub fn selected(&self) -> bool { self.selected }

    /// The synthesiser frequency in MHz (section 21).
    pub fn mhz(&self) -> f64 {
        let f = u32::from(self.regs[0x0d]) << 16 | u32::from(self.regs[0x0e]) << 8 | u32::from(self.regs[0x0f]);
        f64::from(f) * 26.0 / 65536.0
    }

    /// RSSI in dBm, and the register's two's-complement half-dB encoding with its 74 dB offset
    /// (section 17.3).
    fn rssi(&mut self) -> u8 {
        self.stats.rssi_reads += 1;
        let dbm = match self.air { Some(a) if self.state == State::Rx && (a.mhz - self.mhz()).abs() < 0.01 => a.dbm, _ => NOISE_DBM };
        ((dbm + 74.0) * 2.0).round() as i32 as u8
    }

    fn status_byte(&self) -> u8 { (match self.state { State::Idle => 0, State::Rx => 1, State::Tx => 2 }) << 4 }

    fn strobe(&mut self, s: u8) {
        self.stats.strobes += 1;
        match s {
            0x30 => { self.regs = RESET; self.state = State::Idle; }
            0x34 => self.state = State::Rx,
            0x35 => self.state = State::Tx,
            0x36 => self.state = State::Idle,
            _ => {}
        }
    }

    /// One byte in on MOSI, the byte the chip shifts out on MISO at the same time.
    pub fn exchange(&mut self, mosi: u8) -> u8 {
        if !self.selected { return 0xff; }
        let i = self.count;
        self.count += 1;
        if i == 0 {
            self.header = mosi;
            let status = self.status_byte();
            let a = mosi & 0x3f;
            // a strobe is a header to 0x30-0x3D without the burst bit (section 10.4)
            if (0x30..=0x3d).contains(&a) && mosi & 0x40 == 0 { self.strobe(a); }
            return status;
        }
        let (read, burst, a) = (self.header & 0x80 != 0, self.header & 0x40 != 0, usize::from(self.header & 0x3f));
        let at = if burst { a + i - 1 } else { a };
        match a {
            0x30..=0x3d if read && burst => match a {
                0x30 => 0x00,
                0x31 => VERSION,
                0x34 => self.rssi(),
                0x35 => match self.state { State::Idle => 0x01, State::Rx => 0x0d, State::Tx => 0x13 },
                _ => 0,
            },
            0x3e => {
                let k = if burst { (i - 1) % 8 } else { 0 };
                if read { self.patable[k] } else { self.patable[k] = mosi; self.status_byte() }
            }
            _ if at < self.regs.len() => if read { self.regs[at] } else { self.regs[at] = mosi; self.status_byte() },
            _ => 0,
        }
    }

    pub fn transfer(&mut self, tx: &[u8], rx_len: usize) -> Vec<u8> { super::spi_exchange(tx, rx_len, |b| self.exchange(b)) }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(c: &mut Cc1101, bytes: &[u8]) -> Vec<u8> {
        c.select(true);
        let out = c.transfer(bytes, bytes.len());
        c.select(false);
        out
    }

    #[test]
    fn detect_tune_and_read_rssi_as_smartrc_does() {
        let mut c = Cc1101::new(Some(Air { mhz: 433.92, dbm: -35.0 }));
        frame(&mut c, &[0x30]);
        // getCC1101: VERSION twice through a burst status read
        assert_eq!(frame(&mut c, &[0xf1, 0])[1], 0x14);
        assert_eq!(frame(&mut c, &[0xf0, 0])[1], 0x00, "PARTNUM");
        assert_eq!(frame(&mut c, &[0x88, 0])[1], 0x45, "PKTCTRL0 reset value");
        // setMHZ(433.92): FREQ = 433.92 / 26 * 65536 = 0x10B071
        frame(&mut c, &[0x0d, 0x10]); frame(&mut c, &[0x0e, 0xb0]); frame(&mut c, &[0x0f, 0x71]);
        assert!((c.mhz() - 433.92).abs() < 0.001);
        assert_eq!(frame(&mut c, &[0xf4, 0])[1], 204, "idle: the noise floor, -100 dBm");
        assert_eq!(frame(&mut c, &[0x34])[0] >> 4, 0, "SRX: the status byte shows the state before");
        assert_eq!(frame(&mut c, &[0xf5, 0]), [0x10, 0x0d], "MARCSTATE RX");
        assert_eq!(frame(&mut c, &[0xf4, 0])[1], 78, "-35 dBm: (raw / 2) - 74");
        frame(&mut c, &[0x4d, 0x10, 0xb1, 0x00]);
        assert_eq!(frame(&mut c, &[0xcd, 0, 0, 0])[1..], [0x10, 0xb1, 0x00], "burst write and read");
        assert_eq!(frame(&mut c, &[0xf4, 0])[1], 204, "434.03 MHz is off the carrier");
        frame(&mut c, &[0x7e, 1, 2, 3]);
        assert_eq!(frame(&mut c, &[0xfe, 0, 0])[1..], [1, 2]);
    }
}
