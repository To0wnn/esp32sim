//! A Silicon Labs Si4713 FM transmitter on I2C (address 0x63, SEN high), as programming guide
//! AN332 Rev 1.0 describes it: the host writes a command byte and its arguments; every read
//! returns the status byte (CTS 0x80, ERR 0x40, STCINT 0x01) followed by the last command's
//! response bytes. Commands finish at once, so CTS is always set. The tune commands
//! (TX_TUNE_FREQ, TX_TUNE_POWER, TX_TUNE_MEASURE) set STCINT; TX_TUNE_STATUS with INTACK clears it.
//! TX_TUNE_MEASURE reports a received noise level: the bench has one strong station on
//! `STATION` (10 kHz units), so a sweep finds a peak there.
use esp_periph::i2c::I2cDevice;

pub const SI4713_ADDRESS: u8 = 0x63;
/// The bench's virtual broadcast station: 101.10 MHz.
pub const STATION: u16 = 10110;
const CTS: u8 = 0x80;
const ERR: u8 = 0x40;
const STCINT: u8 = 0x01;

pub struct Si4713 {
    cmd: Vec<u8>, writing: bool,
    status: u8,
    response: Vec<u8>,
    read_at: usize,
    /// Transmit frequency (10 kHz), level (dBuV), antenna capacitor, last measured noise level.
    pub frequency: u16, pub power: u8, pub antcap: u8, noise: u8,
    pub properties: std::collections::BTreeMap<u16, u16>,
    /// The RDS program service name (TX_RDS_PS slots, four characters each).
    pub rds_ps: [u8; 32],
    pub commands: u64,
}

impl Default for Si4713 { fn default() -> Self { Self::new() } }
impl Si4713 {
    pub fn new() -> Self {
        Si4713 { cmd: Vec::new(), writing: false, status: CTS, response: Vec::new(), read_at: 0, frequency: 0, power: 0, antcap: 0,
                 noise: 0, properties: Default::default(), rds_ps: [b' '; 32], commands: 0 }
    }
    /// Received noise level at `freq` (dBuV): the station, its neighbours, a quiet band elsewhere.
    pub fn noise_at(freq: u16) -> u8 {
        match freq.abs_diff(STATION) { 0 => 62, 1..=10 => 50, d => 20 + (d % 7) as u8 }
    }
    fn execute(&mut self) {
        let c = std::mem::take(&mut self.cmd);
        let arg = |i: usize| c.get(i).copied().unwrap_or(0);
        let word = |i: usize| u16::from(arg(i)) << 8 | u16::from(arg(i + 1));
        self.commands += 1;
        self.status &= STCINT;
        self.response = match arg(0) {
            0x01 | 0x11 | 0x80 | 0x81 | 0x14 => vec![],          // POWER_UP, POWER_DOWN, GPO_CTL, GPO_SET, GET_INT_STATUS
            // GET_REV: part 13 (Si4713), firmware 3.0, no patch, component 3.0, chip revision A
            0x10 => vec![13, b'3', b'0', 0, 0, b'3', b'0', b'A'],
            0x12 => { self.properties.insert(word(2), word(4)); vec![] }
            0x13 => { let v = self.properties.get(&word(2)).copied().unwrap_or(0); vec![0, (v >> 8) as u8, v as u8] }
            0x30 => { self.frequency = word(2); self.status |= STCINT; vec![] }
            0x31 => { self.power = arg(3); self.antcap = arg(4); self.status |= STCINT; vec![] }
            0x32 => { self.noise = Self::noise_at(word(2)); self.status |= STCINT; vec![] }
            0x33 => {
                if arg(1) & 1 != 0 { self.status &= !STCINT; }
                vec![0, (self.frequency >> 8) as u8, self.frequency as u8, 0, self.power, self.antcap, self.noise]
            }
            0x34 => vec![0, 0, 0, (-20i8) as u8],                // TX_ASQ_STATUS: no overmodulation, input -20 dBfs
            0x35 => vec![0, 0, 0, 0],                             // TX_RDS_BUFF: FIFO state
            0x36 => {
                let slot = usize::from(arg(1) & 7) * 4;
                for i in 0..4 { self.rds_ps[slot + i] = arg(2 + i); }
                vec![]
            }
            _ => { self.status |= ERR; vec![] }
        };
        self.status |= CTS;
    }
}

impl I2cDevice for Si4713 {
    fn start(&mut self, read: bool) -> bool {
        self.writing = !read;
        if read { self.read_at = 0; } else { self.cmd.clear(); }
        true
    }
    fn write(&mut self, b: u8) -> bool { self.cmd.push(b); true }
    fn read(&mut self) -> u8 {
        let i = self.read_at;
        self.read_at += 1;
        if i == 0 { self.status } else { self.response.get(i - 1).copied().unwrap_or(0) }
    }
    fn stop(&mut self) {
        if self.writing && !self.cmd.is_empty() { self.execute(); }
        self.writing = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn command(d: &mut Si4713, cmd: &[u8], n: usize) -> Vec<u8> {
        d.start(false); for &b in cmd { d.write(b); } d.stop();
        d.start(true); let r = (0..n).map(|_| d.read()).collect(); d.stop(); r
    }
    #[test]
    fn revision_tune_and_measure_as_adafruit_si4713_reads_them() {
        let mut d = Si4713::new();
        assert_eq!(command(&mut d, &[0x01, 0x12, 0x50], 1), [CTS]);
        assert_eq!(command(&mut d, &[0x10, 0], 9)[1], 13, "Adafruit_Si4713::begin wants part 13");
        command(&mut d, &[0x12, 0, 0x22, 0x01, 0x00, 0x05], 1);
        assert_eq!(command(&mut d, &[0x13, 0, 0x22, 0x01], 4)[2..], [0x00, 0x05]);
        command(&mut d, &[0x30, 0, (10230u16 >> 8) as u8, 10230u16 as u8], 1);
        assert_eq!(command(&mut d, &[0x14], 1), [CTS | STCINT], "tuneFM waits for (status & 0x81) == 0x81");
        command(&mut d, &[0x31, 0, 0, 115, 0], 1);
        let s = command(&mut d, &[0x33, 1], 8);
        assert_eq!((u16::from(s[2]) << 8 | u16::from(s[3]), s[5]), (10230, 115));
        assert_eq!(command(&mut d, &[0x14], 1), [CTS], "INTACK cleared STCINT");
        command(&mut d, &[0x32, 0, (STATION >> 8) as u8, STATION as u8, 0], 1);
        assert_eq!(command(&mut d, &[0x14], 1), [0x81], "readTuneMeasure waits for status == 0x81");
        assert_eq!(command(&mut d, &[0x33, 1], 8)[7], 62, "the station's noise level");
        assert_eq!(command(&mut d, &[0x77], 1), [CTS | ERR]);
    }
}
