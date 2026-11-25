//! A Maxim DS18B20 digital thermometer on a bit-banged 1-Wire bus (datasheet 19-7487 Rev 6),
//! externally powered. The board feeds it the master's drive (`master`: the line pulled low or
//! released, at emulated nanoseconds) and applies what it answers (`take_due`: the chip pulling
//! the line low or letting go). It decodes the time slots as the chip does:
//! - reset: the master holds the line low for at least 480 us; 15-60 us after it lets go the
//!   chip pulls low for 60-240 us (presence pulse);
//! - write slot: the chip samples the line 15-60 us after the master's falling edge, so a short
//!   low (the master's 1) reads 1 and a long low (60-120 us, its 0) reads 0;
//! - read slot: the master pulls low for at least 1 us and lets go; to send a 0 the chip holds
//!   the line low from that edge for up to 60 us (master samples within 15 us), a 1 leaves it.
//!
//! Bytes go least significant bit first. ROM commands: READ ROM 0x33, MATCH ROM 0x55, SKIP ROM
//! 0xCC, SEARCH ROM 0xF0 (ALARM SEARCH 0xEC finds no alarm); function commands: CONVERT T 0x44,
//! READ SCRATCHPAD 0xBE, WRITE SCRATCHPAD 0x4E, COPY SCRATCHPAD 0x48, RECALL E2 0xB8, READ POWER
//! SUPPLY 0xB4.
// shortcut: CONVERT T finishes at once (read slots after it read 1, "done"), not after the
// 750 ms of a 12-bit conversion; a firmware that polls for the end sees it done at once.
use std::collections::VecDeque;

const US: u64 = 1_000;
/// Shorter than the datasheet's 480 us minimum: the board sees edges at tick granularity.
const RESET_MIN: u64 = 400 * US;
/// Within the chip's 15-60 us sampling window (typically 30 us).
const SAMPLE_AT: u64 = 30 * US;
const PRESENCE_WAIT: u64 = 30 * US;
const PRESENCE_LOW: u64 = 120 * US;
/// How long the chip holds a 0 in a read slot, counted from the master's falling edge.
const READ_ZERO_LOW: u64 = 30 * US;

/// Power-up scratchpad: +85 C, TH 75, TL 70, 12-bit configuration (datasheet table 1 and figure 9).
const POWER_UP: [u8; 8] = [0x50, 0x05, 0x4b, 0x46, 0x7f, 0xff, 0x0c, 0x10];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase { Idle, RomCommand, Match(u8), Search(u8), Function, WriteScratchpad(u8) }

pub struct Ds18b20 {
    rom: [u8; 8],
    scratchpad: [u8; 8],
    eeprom: [u8; 3],
    /// What CONVERT T measures, in 1/16 C.
    pub temperature: i16,
    phase: Phase,
    /// Bits the chip answers in the next read slots.
    tx: VecDeque<bool>,
    rx: u8, rx_bits: u8,
    fall: Option<u64>,
    /// When the chip pulls the line low (true) or lets go (false), in emulated ns.
    events: VecDeque<(u64, bool)>,
    pub resets: u64, pub conversions: u64,
}

/// The Dallas/Maxim CRC-8 (x^8 + x^5 + x^4 + 1, least significant bit first) of the ROM code and scratchpad.
pub fn crc8(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |mut crc, &b| {
        let mut b = b;
        for _ in 0..8 { let mix = (crc ^ b) & 1; crc >>= 1; if mix != 0 { crc ^= 0x8c; } b >>= 1; }
        crc
    })
}

impl Ds18b20 {
    /// `serial`: the six-byte serial number; the family code 0x28 and the CRC complete the ROM code.
    pub fn new(serial: [u8; 6], temperature_c16: i16) -> Self {
        let mut rom = [0x28, serial[0], serial[1], serial[2], serial[3], serial[4], serial[5], 0];
        rom[7] = crc8(&rom[..7]);
        Ds18b20 { rom, scratchpad: POWER_UP, eeprom: [0x4b, 0x46, 0x7f], temperature: temperature_c16, phase: Phase::Idle,
                  tx: VecDeque::new(), rx: 0, rx_bits: 0, fall: None, events: VecDeque::new(), resets: 0, conversions: 0 }
    }
    pub fn rom(&self) -> [u8; 8] { self.rom }
    /// The master pulls the line low (`low`) or lets go of it, at `now` ns.
    pub fn master(&mut self, now: u64, low: bool) {
        if low {
            self.fall = Some(now);
            if let Some(bit) = self.tx.pop_front() {
                if !bit { self.events.push_back((now, true)); self.events.push_back((now + READ_ZERO_LOW, false)); }
                self.fall = None;  // a read slot: nothing to sample
            }
            return;
        }
        let Some(fall) = self.fall.take() else { return };
        let low_for = now.saturating_sub(fall);
        if low_for >= RESET_MIN {
            self.resets += 1;
            self.phase = Phase::RomCommand;
            self.tx.clear(); self.rx = 0; self.rx_bits = 0;
            self.events.push_back((now + PRESENCE_WAIT, true));
            self.events.push_back((now + PRESENCE_WAIT + PRESENCE_LOW, false));
        } else {
            self.bit(low_for < SAMPLE_AT);
        }
    }
    /// The chip's next pull or release that is due by `now`.
    pub fn take_due(&mut self, now: u64) -> Option<bool> {
        if self.events.front().is_some_and(|&(t, _)| t <= now) { self.events.pop_front().map(|e| e.1) } else { None }
    }
    pub fn next_event(&self) -> Option<u64> { self.events.front().map(|e| e.0) }

    fn send(&mut self, bytes: &[u8]) { for &b in bytes { for i in 0..8 { self.tx.push_back(b >> i & 1 != 0); } } }
    fn rom_bit(&self, i: u8) -> bool { self.rom[usize::from(i / 8)] >> (i % 8) & 1 != 0 }
    /// One bit written by the master.
    fn bit(&mut self, v: bool) {
        match self.phase {
            Phase::Idle => return,
            Phase::Match(i) => {
                self.phase = if v != self.rom_bit(i) { Phase::Idle } else if i == 63 { Phase::Function } else { Phase::Match(i + 1) };
                return;
            }
            Phase::Search(i) => {
                // the master's direction: a chip whose bit differs leaves the search
                self.phase = if v != self.rom_bit(i) { Phase::Idle } else if i == 63 { Phase::Function } else { Phase::Search(i + 1) };
                if let Phase::Search(n) = self.phase { let b = self.rom_bit(n); self.tx.extend([b, !b]); }
                return;
            }
            _ => {}
        }
        self.rx |= u8::from(v) << self.rx_bits;
        self.rx_bits += 1;
        if self.rx_bits < 8 { return; }
        let byte = self.rx;
        self.rx = 0; self.rx_bits = 0;
        self.byte(byte);
    }
    fn byte(&mut self, b: u8) {
        self.phase = match (self.phase, b) {
            (Phase::RomCommand, 0x33) => { let rom = self.rom; self.send(&rom); Phase::Function }
            (Phase::RomCommand, 0x55) => Phase::Match(0),
            (Phase::RomCommand, 0xcc) => Phase::Function,
            (Phase::RomCommand, 0xf0) => { let b = self.rom_bit(0); self.tx.extend([b, !b]); Phase::Search(0) }
            (Phase::Function, 0x44) => {
                self.conversions += 1;
                self.scratchpad[..2].copy_from_slice(&self.temperature.to_le_bytes());
                Phase::Idle
            }
            (Phase::Function, 0xbe) => {
                let mut s = [0u8; 9];
                s[..8].copy_from_slice(&self.scratchpad);
                s[8] = crc8(&self.scratchpad);
                self.send(&s);
                Phase::Idle
            }
            (Phase::Function, 0x4e) => Phase::WriteScratchpad(0),
            (Phase::WriteScratchpad(n), _) => {
                // TH, TL, configuration; the configuration's unused bits read 1 (datasheet figure 10)
                self.scratchpad[2 + usize::from(n)] = if n == 2 { b | 0x1f } else { b };
                if n == 2 { Phase::Idle } else { Phase::WriteScratchpad(n + 1) }
            }
            (Phase::Function, 0x48) => { self.eeprom.copy_from_slice(&self.scratchpad[2..5]); Phase::Idle }
            (Phase::Function, 0xb8) => { self.scratchpad[2..5].copy_from_slice(&self.eeprom); Phase::Idle }
            // READ POWER SUPPLY: an externally powered chip answers 1, which an idle line reads
            _ => Phase::Idle,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    /// A master with OneWire's (PaulStoffregen) slot timings; `line` is what it samples.
    struct Master { d: Ds18b20, now: u64 }
    impl Master {
        fn low_for(&mut self, us: u64) { self.d.master(self.now, true); self.now += us * US; self.d.master(self.now, false); }
        fn line_low_at(&mut self, t: u64) -> bool {
            let mut low = false;
            while let Some(&(at, l)) = self.d.events.front() { if at > t { break; } low = l; self.d.events.pop_front(); }
            low
        }
        fn reset(&mut self) -> bool { self.low_for(480); let t = self.now + 70 * US; let p = self.line_low_at(t); self.now += 480 * US; self.line_low_at(self.now); p }
        fn write(&mut self, b: u8) { for i in 0..8 { self.low_for(if b >> i & 1 != 0 { 10 } else { 65 }); self.now += 55 * US; } }
        fn read_bit(&mut self) -> bool { let fall = self.now; self.low_for(3); let low = self.line_low_at(fall + 13 * US); self.now = fall + 66 * US; self.line_low_at(self.now); !low }
        fn read(&mut self) -> u8 { (0..8).fold(0, |b, i| b | u8::from(self.read_bit()) << i) }
    }
    #[test]
    fn presence_rom_and_temperature_as_onewire_reads_them() {
        let mut m = Master { d: Ds18b20::new([0x5e, 0x1d, 0x8b, 0x16, 0x03, 0x00], 376), now: 0 };
        assert!(m.reset(), "presence pulse");
        m.write(0x33);
        let rom: Vec<u8> = (0..8).map(|_| m.read()).collect();
        assert_eq!(rom[0], 0x28);
        assert_eq!(crc8(&rom[..7]), rom[7]);
        assert!(m.reset());
        m.write(0xcc); m.write(0xbe);
        let s: Vec<u8> = (0..9).map(|_| m.read()).collect();
        assert_eq!(&s[..2], &[0x50, 0x05], "85 C before a conversion");
        assert_eq!(crc8(&s[..8]), s[8]);
        m.reset(); m.write(0x55); for b in rom.clone() { m.write(b); } m.write(0x44);
        m.reset(); m.write(0x55); for b in rom.clone() { m.write(b); } m.write(0xbe);
        let s: Vec<u8> = (0..9).map(|_| m.read()).collect();
        assert_eq!(i16::from_le_bytes([s[0], s[1]]), 376, "23.5 C");
        assert_eq!(crc8(&s[..8]), s[8]);
        // SEARCH ROM: bit, complement, direction; one chip on the bus never shows a discrepancy
        m.reset(); m.write(0xf0);
        let mut found = [0u8; 8];
        for i in 0..64 {
            let (b, c) = (m.read_bit(), m.read_bit());
            assert_ne!(b, c, "bit {i}");
            m.low_for(if b { 10 } else { 65 }); m.now += 55 * US;
            found[i / 8] |= u8::from(b) << (i % 8);
        }
        assert_eq!(found.to_vec(), rom);
        // a wrong MATCH ROM leaves the chip silent
        m.reset(); m.write(0x55); m.write(0x10); for b in &rom[1..] { m.write(*b); } m.write(0xbe);
        assert_eq!(m.read(), 0xff);
    }
}
