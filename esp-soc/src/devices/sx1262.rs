//! A Semtech SX1262 LoRa transceiver on a board's SPI bus, byte by byte as the chip sees it
//! between a falling and a rising NSS (SX1261/2 datasheet rev 2.1, section 8.3: an opcode,
//! then parameters; MISO returns the status byte while the opcode and parameters shift in, and
//! data after it). Modelled commands: ReadRegister / WriteRegister (0x1D / 0x0D, 16-bit address,
//! auto-increment), GetStatus (0xC0), GetRssiInst (0x15), SetStandby (0x80), SetFs (0xC1),
//! SetRx (0x82), SetTx (0x83), SetSleep (0x84) and SetRfFrequency (0x86); every other command is
//! accepted and ignored. The LoRa sync word register 0x0740-0x0741 resets to 0x1424 (table 12-1).
//! A falling NSS wakes the chip from sleep (section 13.1.1); RESET low restores the reset state.
//!
//! The output byte depends only on the bytes before it (`next_out`), so a bit-banged master that
//! samples MISO before the first clock edge sees the same bits as GP-SPI.
//!
//! The air is one virtual carrier (`Air`): in RX, GetRssiInst reads its level when the
//! synthesiser (f = RfFreq x 32 MHz / 2^25, section 13.4.1) is within 50 kHz, else the noise floor.
//! shortcut: BUSY is never driven high and DIO1 never rises: no TX done, RX done or CAD
//! interrupts, no packets in the buffer.

const NOISE_DBM: i32 = -110;

/// A carrier on the air: frequency in Hz and level at the antenna in dBm.
#[derive(Clone, Copy, Debug)]
pub struct Air { pub hz: u32, pub dbm: i32 }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mode { Sleep, StandbyRc, StandbyXosc, Fs, Rx, Tx }

#[derive(Clone, Copy, Default, Debug)]
pub struct Sx1262Stats { pub commands: u64, pub rssi_reads: u64 }

pub struct Sx1262 {
    regs: Vec<u8>,
    mode: Mode,
    rf_freq: u32,
    selected: bool,
    /// the frame so far (opcode and the first parameter bytes)
    head: [u8; 4],
    count: usize,
    pub air: Option<Air>,
    pub stats: Sx1262Stats,
}

impl Sx1262 {
    pub fn new(air: Option<Air>) -> Self {
        let mut s = Sx1262 { regs: Vec::new(), mode: Mode::StandbyRc, rf_freq: 0, selected: false, head: [0; 4], count: 0, air, stats: Sx1262Stats::default() };
        s.reset();
        s
    }

    /// The RESET pin held low: the chip restarts in STDBY_RC with its registers at reset values.
    pub fn reset(&mut self) {
        self.regs = vec![0; 0x1000];
        self.regs[0x0740] = 0x14;
        self.regs[0x0741] = 0x24;
        self.mode = Mode::StandbyRc;
        self.count = 0;
    }

    /// Chip select: `true` while NSS is driven low.
    pub fn select(&mut self, low: bool) {
        if low == self.selected { return; }
        if low && self.mode == Mode::Sleep { self.mode = Mode::StandbyRc; }
        if !low { self.finish(); }
        self.selected = low;
        self.count = 0;
    }
    pub fn selected(&self) -> bool { self.selected }

    /// The status byte (section 13.5.1): chip mode in bits 6:4, command status 0.
    fn status(&self) -> u8 {
        (match self.mode { Mode::Sleep => 0, Mode::StandbyRc => 2, Mode::StandbyXosc => 3, Mode::Fs => 4, Mode::Rx => 5, Mode::Tx => 6 }) << 4
    }

    fn addr(&self) -> usize { (usize::from(self.head[1]) << 8 | usize::from(self.head[2])) & 0xfff }

    fn rssi_dbm(&self) -> i32 {
        let hz = ((u64::from(self.rf_freq) * 32_000_000) >> 25) as i64;
        match self.air { Some(a) if self.mode == Mode::Rx && (i64::from(a.hz) - hz).abs() < 50_000 => a.dbm, _ => NOISE_DBM }
    }

    /// The byte MISO carries while the next byte shifts in: it depends only on the bytes before.
    pub fn next_out(&self) -> u8 {
        if !self.selected { return 0xff; }
        let i = self.count;
        match self.head[0] {
            _ if i == 0 => self.status(),
            0x1d if i >= 4 => self.regs[(self.addr() + i - 4) & 0xfff],
            0x15 if i == 2 => (-self.rssi_dbm() * 2).clamp(0, 255) as u8,
            0x1d | 0x15 | 0xc0 | 0x1e if i < 3 => self.status(),
            _ if i == 1 => self.status(),
            _ => 0,
        }
    }

    /// One byte in on MOSI, the byte the chip shifts out on MISO at the same time.
    pub fn exchange(&mut self, mosi: u8) -> u8 {
        if !self.selected { return 0xff; }
        let out = self.next_out();
        let i = self.count;
        if i < self.head.len() { self.head[i] = mosi; }
        if self.head[0] == 0x0d && i >= 3 { let a = (self.addr() + i - 3) & 0xfff; self.regs[a] = mosi; }
        if self.head[0] == 0x86 && (1..=4).contains(&i) { self.rf_freq = self.rf_freq << 8 | u32::from(mosi); }
        if self.head[0] == 0x15 && i == 2 { self.stats.rssi_reads += 1; }
        self.count += 1;
        out
    }

    /// NSS rose: mode-changing commands take effect.
    fn finish(&mut self) {
        if self.count == 0 { return; }
        self.stats.commands += 1;
        match self.head[0] {
            0x80 => self.mode = if self.count > 1 && self.head[1] == 1 { Mode::StandbyXosc } else { Mode::StandbyRc },
            0xc1 => self.mode = Mode::Fs,
            0x82 => self.mode = Mode::Rx,
            0x83 => self.mode = Mode::Tx,
            0x84 => self.mode = Mode::Sleep,
            _ => {}
        }
    }

    pub fn transfer(&mut self, tx: &[u8], rx_len: usize) -> Vec<u8> { super::spi_exchange(tx, rx_len, |b| self.exchange(b)) }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(s: &mut Sx1262, bytes: &[u8]) -> Vec<u8> {
        s.select(true);
        let out = s.transfer(bytes, bytes.len());
        s.select(false);
        out
    }

    #[test]
    fn sync_word_probe_rx_and_rssi_as_a_lora_tool_does() {
        let mut s = Sx1262::new(Some(Air { hz: 868_000_000, dbm: -60 }));
        // LoRaService::probeRadio: ReadRegister 0x0740, two data bytes after the NOP
        assert_eq!(&frame(&mut s, &[0x1d, 0x07, 0x40, 0, 0, 0])[4..], [0x14, 0x24]);
        assert_eq!(frame(&mut s, &[0xc0, 0]), [0x20, 0x20], "STDBY_RC");
        frame(&mut s, &[0x0d, 0x07, 0x40, 0x34, 0x44]);
        assert_eq!(&frame(&mut s, &[0x1d, 0x07, 0x40, 0, 0, 0])[4..], [0x34, 0x44]);
        frame(&mut s, &[0x84, 0x04]);
        assert_eq!(frame(&mut s, &[0xc0, 0])[1], 0x20, "NSS falling wakes it");
        // SetRfFrequency(868 MHz) = 868e6 * 2^25 / 32e6 = 0x36400000, then SetRx
        frame(&mut s, &[0x86, 0x36, 0x40, 0x00, 0x00]);
        assert_eq!(frame(&mut s, &[0x15, 0, 0])[2], 220, "standby: noise floor -110 dBm");
        frame(&mut s, &[0x82, 0xff, 0xff, 0xff]);
        assert_eq!(frame(&mut s, &[0x15, 0, 0]), [0x50, 0x50, 120], "RX on the carrier: -60 dBm");
        frame(&mut s, &[0x86, 0x36, 0x80, 0x00, 0x00]);
        assert_eq!(frame(&mut s, &[0x15, 0, 0])[2], 220, "872 MHz");
        s.reset();
        assert_eq!(&frame(&mut s, &[0x1d, 0x07, 0x40, 0, 0, 0])[4..], [0x14, 0x24]);
    }
}
