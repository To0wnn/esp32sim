//! The Ethernet MAC: Synopsys DesignWare GMAC (the core of the ESP32, ESP32-P4 and ESP32-S31
//! EMAC), its descriptor DMA, and an RMII PHY on its MDIO bus.
//!
//! Registers: IDF master `soc/esp32s31/register/soc/emac_mac_struct.h` (MAC at +0x0000, PTP at
//! +0x0700) and `emac_dma_struct.h` (DMA at +0x1000); what the driver expects of them:
//! `hal/esp32s31/include/hal/emac_ll.h` and `esp_eth`'s `esp_eth_mac_esp.c`/`emac_hal.c`.
//!
//! The chip calls `run` with its DMA view after a register write or when frames are waiting:
//! transmit descriptors the CPU handed over are sent at once (`take_tx`), received frames
//! (`receive`) go into the next DMA-owned descriptors. There is no wire timing: a frame is sent
//! and delivered the moment the DMA may touch it.
use crate::sdmmc::SdmmcMemory;
use crate::{Device, RegRam, WriteEffect};
use std::collections::VecDeque;

const GMII_ADDR: u32 = 0x10;
const GMII_DATA: u32 = 0x14;
/// `emaccstatus`: the RGMII link as the PHY reports it in-band (bit 0 full duplex, bits 2:1
/// speed, bit 3 link up).
const MAC_LINK_STATUS: u32 = 0xd8;
const DMA: u32 = 0x1000;
const BUS_MODE: u32 = DMA;
const TX_POLL: u32 = DMA + 0x04;
const RX_POLL: u32 = DMA + 0x08;
const RX_BASE: u32 = DMA + 0x0c;
const TX_BASE: u32 = DMA + 0x10;
const STATUS: u32 = DMA + 0x14;
const OP_MODE: u32 = DMA + 0x18;
const INT_EN: u32 = DMA + 0x1c;
const MISSED: u32 = DMA + 0x20;
const TX_CUR_DESC: u32 = DMA + 0x48;
const RX_CUR_DESC: u32 = DMA + 0x4c;
const TX_CUR_BUF: u32 = DMA + 0x50;
const RX_CUR_BUF: u32 = DMA + 0x54;
/// The block spans the MAC/PTP page and the DMA page.
pub const SIZE: u32 = 0x2000;

const BUS_SW_RST: u32 = 1;
const BUS_ALT_DESC: u32 = 1 << 7;
const OP_START_RX: u32 = 1 << 1;
const OP_START_TX: u32 = 1 << 13;
const OP_FLUSH_TX: u32 = 1 << 20;

/// `dmastatus` / `dmain_en` bits.
pub const ST_TX: u32 = 1 << 0;
pub const ST_TX_UNAVAIL: u32 = 1 << 2;
pub const ST_RX: u32 = 1 << 6;
pub const ST_RX_UNAVAIL: u32 = 1 << 7;
const ST_AIS: u32 = 1 << 15;
const ST_NIS: u32 = 1 << 16;
const NORMAL: u32 = ST_TX | ST_TX_UNAVAIL | ST_RX | 1 << 14;
const ABNORMAL: u32 = 0x3fba;
const EN_AIS: u32 = 1 << 15;
const EN_NIS: u32 = 1 << 16;

/// Enhanced descriptor bits (`alt_desc_size`, which IDF always sets).
pub const OWN: u32 = 1 << 31;
pub const TDES0_IC: u32 = 1 << 30;
pub const TDES0_LS: u32 = 1 << 29;
pub const TDES0_FS: u32 = 1 << 28;
pub const TDES0_TER: u32 = 1 << 21;
pub const TDES0_TCH: u32 = 1 << 20;
pub const RDES0_FS: u32 = 1 << 9;
pub const RDES0_LS: u32 = 1 << 8;
pub const RDES1_DIC: u32 = 1 << 31;
pub const RDES1_RER: u32 = 1 << 15;
pub const RDES1_RCH: u32 = 1 << 14;
/// A frame shorter than this goes on the wire padded, as a real MAC receives it.
const MIN_FRAME: usize = 60;
/// Descriptors one frame may span; a ring that never ends is a guest bug, not a hang.
const MAX_DESCS: usize = 64;
/// Frames waiting for receive descriptors; the oldest are dropped past this.
const RX_QUEUE: usize = 64;

/// An IEEE 802.3 clause 22 PHY with autonegotiation always done at 100 Mbit/s full duplex once
/// the cable is in. Its ID is a LAN8720A's, so both IDF's generic and LAN87xx drivers take it.
pub struct Phy {
    pub addr: u8,
    regs: [u16; 32],
    pub link: bool,
}
const BMCR: usize = 0;
const BMSR: usize = 1;
const BMCR_RESET: u16 = 1 << 15;
const BMCR_AN_RESTART: u16 = 1 << 9;
/// 100/10 full and half duplex, autonegotiation able, extended capabilities.
const BMSR_CAPS: u16 = 0x7809;
const BMSR_LINK: u16 = 1 << 2;
const BMSR_AN_DONE: u16 = 1 << 5;
impl Phy {
    pub fn new(addr: u8) -> Self { let mut p = Phy { addr, regs: [0; 32], link: false }; p.reset(); p }
    fn reset(&mut self) {
        self.regs = [0; 32];
        self.regs[BMCR] = 0x3100; // 100 Mbit/s, autonegotiation on, full duplex
        self.regs[2] = 0x0007; self.regs[3] = 0xc0f1; // LAN8720A
        self.regs[4] = 0x01e1; // advertises 100/10 full and half
        self.regs[31] = 0x1058; // LAN87xx PSCSR: autodone, 100BASE-TX full duplex
    }
    pub fn read(&self, reg: u8) -> u16 {
        match reg as usize {
            BMSR => BMSR_CAPS | if self.link { BMSR_LINK | BMSR_AN_DONE } else { 0 },
            // the link partner (the virtual switch) offers what we offer, and acknowledges
            5 => if self.link { 0x4000 | self.regs[4] } else { 0 },
            6 => u16::from(self.link),
            r => self.regs[r & 31],
        }
    }
    pub fn write(&mut self, reg: u8, v: u16) {
        match reg as usize {
            BMCR if v & BMCR_RESET != 0 => self.reset(),
            BMCR => self.regs[BMCR] = v & !BMCR_AN_RESTART,
            BMSR | 2 | 3 | 5 | 6 => {}
            r => self.regs[r & 31] = v,
        }
    }
}

pub struct Emac {
    mac: RegRam,
    dma: RegRam,
    bus_mode: u32,
    op_mode: u32,
    status: u32,
    int_en: u32,
    rx_base: u32, tx_base: u32,
    rx_cur: u32, tx_cur: u32,
    rx_buf: u32, tx_buf: u32,
    missed: u32,
    gmii_addr: u32, gmii_data: u32,
    pub phy: Phy,
    /// frames for the guest, waiting for receive descriptors
    rx: VecDeque<Vec<u8>>,
    /// frames the guest sent, for the network
    tx: Vec<Vec<u8>>,
    /// the DMA has something to look at: a poll demand, a start, a new frame
    kick: bool,
    pub tx_frames: u64, pub rx_frames: u64, pub rx_dropped: u64,
    pub log: bool,
}
impl Default for Emac { fn default() -> Self { Self::new(1) } }

impl Emac {
    pub fn new(phy_addr: u8) -> Self {
        Emac { mac: RegRam::new(), dma: RegRam::new(), bus_mode: 0x0002_0100, op_mode: 0, status: 0, int_en: 0,
               rx_base: 0, tx_base: 0, rx_cur: 0, tx_cur: 0, rx_buf: 0, tx_buf: 0, missed: 0, gmii_addr: 0, gmii_data: 0,
               phy: Phy::new(phy_addr), rx: VecDeque::new(), tx: Vec::new(), kick: false, tx_frames: 0, rx_frames: 0, rx_dropped: 0, log: false }
    }
    /// The DMA software reset: every register to its reset value; the PHY is a separate chip.
    fn sw_reset(&mut self) {
        let (phy, log) = (std::mem::replace(&mut self.phy, Phy::new(0)), self.log);
        let (tx_frames, rx_frames, rx_dropped) = (self.tx_frames, self.rx_frames, self.rx_dropped);
        *self = Emac { phy, log, tx_frames, rx_frames, rx_dropped, ..Emac::new(0) };
    }
    /// The DMA interrupt line (`ETS_SBD_INTR_SOURCE` on the S31).
    pub fn irq(&self) -> bool {
        let s = self.status & self.int_en;
        (self.int_en & EN_NIS != 0 && s & NORMAL != 0) || (self.int_en & EN_AIS != 0 && s & ABNORMAL != 0)
    }
    /// Plug or unplug the cable.
    pub fn set_link(&mut self, up: bool) { self.phy.link = up; }
    /// A frame from the network for the guest (Ethernet II, no FCS).
    pub fn receive(&mut self, frame: Vec<u8>) {
        if self.rx.len() >= RX_QUEUE { self.rx.pop_front(); self.rx_dropped += 1; }
        self.rx.push_back(frame); self.kick = true;
    }
    /// The frames the guest sent since the last call (Ethernet II, no FCS).
    pub fn take_tx(&mut self) -> Vec<Vec<u8>> { std::mem::take(&mut self.tx) }
    /// Whether `run` has work: a kick from a register write or frames waiting to go in.
    pub fn pending(&self) -> bool { self.kick || !self.tx.is_empty() }
    fn desc_step(&self) -> u32 {
        let words = if self.bus_mode & BUS_ALT_DESC != 0 { 8 } else { 4 };
        (words + (self.bus_mode >> 2 & 0x1f)) * 4
    }

    /// Let the DMA move what it may: transmit everything the CPU handed over, then receive
    /// waiting frames while there are descriptors for them. True when the status changed.
    pub fn run(&mut self, mem: &mut impl SdmmcMemory) -> bool {
        self.kick = false;
        let before = self.status;
        if self.op_mode & OP_START_TX != 0 { self.run_tx(mem); }
        if self.op_mode & OP_START_RX != 0 {
            while let Some(frame) = self.rx.front() {
                let frame = frame.clone();
                if !self.deliver(mem, &frame) { break; }
                self.rx.pop_front();
            }
        }
        self.status != before
    }

    fn word(mem: &mut impl SdmmcMemory, addr: u32) -> Option<u32> {
        let mut b = [0u8; 4];
        mem.read(addr, &mut b).ok().map(|_| u32::from_le_bytes(b))
    }

    // shortcut: no checksum insertion (TDES0 CIC); lwIP on IDF computes checksums in software.
    fn run_tx(&mut self, mem: &mut impl SdmmcMemory) {
        let mut frame = Vec::new();
        for _ in 0..MAX_DESCS * 4 {
            let d = self.tx_cur;
            let Some(t0) = Self::word(mem, d) else { return };
            if t0 & OWN == 0 { self.status |= ST_TX_UNAVAIL; return; }
            let (t1, b1, t3) = (Self::word(mem, d + 4).unwrap_or(0), Self::word(mem, d + 8).unwrap_or(0), Self::word(mem, d + 12).unwrap_or(0));
            if t0 & TDES0_FS != 0 { frame.clear(); }
            let mut bufs = vec![(b1, t1 & 0x1fff)];
            if t0 & TDES0_TCH == 0 { bufs.push((t3, t1 >> 16 & 0x1fff)); }
            for (addr, len) in bufs {
                if len == 0 { continue; }
                let mut b = vec![0u8; len as usize];
                if mem.read(addr, &mut b).is_err() { return; }
                frame.extend_from_slice(&b);
                self.tx_buf = addr;
            }
            let _ = mem.write(d, &(t0 & !OWN).to_le_bytes());
            self.tx_cur = if t0 & TDES0_TCH != 0 { t3 } else if t0 & TDES0_TER != 0 { self.tx_base } else { d + self.desc_step() };
            if t0 & TDES0_LS != 0 {
                if self.log { eprintln!("[emac] TX {} bytes desc {d:#010x}", frame.len()); }
                self.tx_frames += 1;
                self.tx.push(std::mem::take(&mut frame));
                if t0 & TDES0_IC != 0 { self.status |= ST_TX; }
            }
        }
    }

    /// One frame into as many DMA-owned descriptors as it needs, or none at all (receive buffer
    /// unavailable: the frame waits for a poll demand or the next one).
    fn deliver(&mut self, mem: &mut impl SdmmcMemory, frame: &[u8]) -> bool {
        let mut data = frame.to_vec();
        if data.len() < MIN_FRAME { data.resize(MIN_FRAME, 0); }
        // shortcut: the FCS is zeros; IDF strips it by length and never checks it.
        data.extend_from_slice(&[0; 4]);
        // Find the descriptors first, so a frame never goes in half.
        let (mut plan, mut d, mut left) = (Vec::new(), self.rx_cur, data.len());
        while left > 0 {
            if plan.len() >= MAX_DESCS { self.rx_dropped += 1; return true; }
            let Some(r0) = Self::word(mem, d) else { return false };
            if r0 & OWN == 0 { self.status |= ST_RX_UNAVAIL; self.missed += 1; return false; }
            let (r1, b1, r3) = (Self::word(mem, d + 4).unwrap_or(0), Self::word(mem, d + 8).unwrap_or(0), Self::word(mem, d + 12).unwrap_or(0));
            let mut bufs = vec![(b1, (r1 & 0x1fff) as usize)];
            if r1 & RDES1_RCH == 0 { bufs.push((r3, (r1 >> 16 & 0x1fff) as usize)); }
            let mut parts = Vec::new();
            for (addr, size) in bufs {
                let n = size.min(left);
                if n > 0 { parts.push((addr, n)); left -= n; }
            }
            let next = if r1 & RDES1_RCH != 0 { r3 } else if r1 & RDES1_RER != 0 { self.rx_base } else { d + self.desc_step() };
            if parts.is_empty() { self.rx_dropped += 1; return true; }
            plan.push((d, r1, parts));
            d = next;
        }
        let (mut off, last) = (0, plan.len() - 1);
        let mut ic = false;
        for (i, (desc, r1, parts)) in plan.iter().enumerate() {
            for &(addr, n) in parts {
                if mem.write(addr, &data[off..off + n]).is_err() { return true; }
                off += n; self.rx_buf = addr;
            }
            let mut r0 = if i == 0 { RDES0_FS } else { 0 };
            if i == last { r0 |= RDES0_LS | (data.len() as u32) << 16; ic = r1 & RDES1_DIC == 0; }
            let _ = mem.write(*desc, &r0.to_le_bytes());
        }
        self.rx_cur = d;
        self.rx_frames += 1;
        if ic { self.status |= ST_RX; }
        if self.log { eprintln!("[emac] RX {} bytes into {} desc from {:#010x}", frame.len(), plan.len(), plan[0].0); }
        true
    }

    fn mdio(&mut self, v: u32) {
        let (write, reg, dev) = (v & 2 != 0, (v >> 6 & 0x1f) as u8, (v >> 11 & 0x1f) as u8);
        let here = dev == self.phy.addr;
        if write { if here { self.phy.write(reg, self.gmii_data as u16); } }
        else { self.gmii_data = if here { u32::from(self.phy.read(reg)) } else { 0xffff }; }
        if self.log { eprintln!("[emac] MDIO {} phy {dev} reg {reg} = {:#06x}", if write { "wr" } else { "rd" }, self.gmii_data & 0xffff); }
    }
}

impl Device for Emac {
    fn read(&mut self, off: u32) -> u32 {
        match off {
            GMII_ADDR => self.gmii_addr & !1, // the MDIO frame is done the moment it starts
            GMII_DATA => self.gmii_data,
            MAC_LINK_STATUS => if self.phy.link { 0b1011 } else { 0 },
            BUS_MODE => self.bus_mode,
            RX_BASE => self.rx_base, TX_BASE => self.tx_base,
            STATUS => {
                let s = self.status;
                let nis = if s & self.int_en & NORMAL != 0 { ST_NIS } else { 0 };
                let ais = if s & self.int_en & ABNORMAL != 0 { ST_AIS } else { 0 };
                // receive state 3 (waiting for a frame), transmit state 6 (suspended, no descriptor)
                let rs = if self.op_mode & OP_START_RX != 0 { 3 << 17 } else { 0 };
                let ts = if self.op_mode & OP_START_TX != 0 { 6 << 20 } else { 0 };
                s | nis | ais | rs | ts
            }
            OP_MODE => self.op_mode & !OP_FLUSH_TX, // the flush is done at once
            INT_EN => self.int_en,
            MISSED => std::mem::take(&mut self.missed) & 0xffff,
            TX_CUR_DESC => self.tx_cur, RX_CUR_DESC => self.rx_cur,
            TX_CUR_BUF => self.tx_buf, RX_CUR_BUF => self.rx_buf,
            o if o >= DMA => self.dma.read(o - DMA),
            o => self.mac.read(o),
        }
    }
    fn write(&mut self, off: u32, v: u32) -> WriteEffect {
        match off {
            GMII_ADDR => { self.gmii_addr = v; if v & 1 != 0 { self.mdio(v); } }
            GMII_DATA => self.gmii_data = v & 0xffff,
            BUS_MODE => if v & BUS_SW_RST != 0 { self.sw_reset() } else { self.bus_mode = v },
            TX_POLL | RX_POLL => self.kick = true,
            RX_BASE => { self.rx_base = v & !3; self.rx_cur = self.rx_base; }
            TX_BASE => { self.tx_base = v & !3; self.tx_cur = self.tx_base; }
            STATUS => self.status &= !(v & 0x1_ffff),
            OP_MODE => { self.op_mode = v; self.kick = true; }
            INT_EN => self.int_en = v,
            o if o >= DMA => self.dma.write(o - DMA, v),
            o => self.mac.write(o, v),
        }
        WriteEffect::NONE
    }
    fn irq_sources(&self) -> u64 { u64::from(self.irq()) }
    fn debug(&mut self, on: bool) { self.log = on; }
}

#[cfg(test)]
mod tests {
    use super::*;
    use emu_core::Fault;

    struct Mem(Vec<u8>);
    const BASE: u32 = 0x1000;
    impl SdmmcMemory for Mem {
        fn read(&mut self, addr: u32, buf: &mut [u8]) -> Result<(), Fault> {
            let o = addr.checked_sub(BASE).ok_or(Fault::Unmapped)? as usize;
            buf.copy_from_slice(self.0.get(o..o + buf.len()).ok_or(Fault::Unmapped)?); Ok(())
        }
        fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), Fault> {
            let o = addr.checked_sub(BASE).ok_or(Fault::Unmapped)? as usize;
            self.0.get_mut(o..o + data.len()).ok_or(Fault::Unmapped)?.copy_from_slice(data); Ok(())
        }
    }
    impl Mem {
        fn w(&mut self, a: u32, v: u32) { SdmmcMemory::write(self, a, &v.to_le_bytes()).unwrap(); }
        fn r(&mut self, a: u32) -> u32 { Emac::word(self, a).unwrap() }
    }

    fn mdio(e: &mut Emac, phy: u32, reg: u32, write: Option<u16>) -> u16 {
        if let Some(v) = write { e.write(GMII_DATA, v.into()); }
        e.write(GMII_ADDR, phy << 11 | reg << 6 | 4 << 2 | u32::from(write.is_some()) << 1 | 1);
        assert_eq!(e.read(GMII_ADDR) & 1, 0);
        e.read(GMII_DATA) as u16
    }

    #[test]
    fn phy_on_mdio_reports_link_after_reset_and_autonegotiation() {
        let mut e = Emac::new(1);
        assert_eq!(mdio(&mut e, 1, 2, None), 0x0007);
        assert_eq!(mdio(&mut e, 5, 2, None), 0xffff, "no PHY at address 5");
        assert_eq!(mdio(&mut e, 1, 1, None) & BMSR_LINK, 0, "no cable yet");
        e.set_link(true);
        mdio(&mut e, 1, 0, Some(BMCR_RESET));
        assert_eq!(mdio(&mut e, 1, 0, None) & BMCR_RESET, 0, "reset self-clears");
        mdio(&mut e, 1, 0, Some(0x1000 | BMCR_AN_RESTART));
        assert_eq!(mdio(&mut e, 1, 0, None), 0x1000);
        let bmsr = mdio(&mut e, 1, 1, None);
        assert_ne!(bmsr & BMSR_LINK, 0); assert_ne!(bmsr & BMSR_AN_DONE, 0);
        assert_eq!(mdio(&mut e, 1, 5, None) & 0x0180, 0x0180, "partner does 100 Mbit/s");
    }

    #[test]
    fn dma_sends_chained_descriptors_and_receives_across_buffers() {
        let mut e = Emac::new(1);
        let mut m = Mem(vec![0; 0x4000]);
        e.write(BUS_MODE, BUS_SW_RST); assert_eq!(e.read(BUS_MODE) & BUS_SW_RST, 0);
        e.write(BUS_MODE, BUS_ALT_DESC);
        // two chained TX descriptors at 0x1000, 0x1020; one frame split 10 + 50 bytes
        for i in 0..60u32 { m.0[0x800 + i as usize] = i as u8; }
        m.w(0x1000, OWN | TDES0_FS | TDES0_TCH); m.w(0x1004, 10); m.w(0x1008, 0x1800); m.w(0x100c, 0x1020);
        m.w(0x1020, OWN | TDES0_LS | TDES0_IC | TDES0_TCH); m.w(0x1024, 50); m.w(0x1028, 0x180a); m.w(0x102c, 0x1000);
        e.write(TX_BASE, 0x1000);
        // three chained 256-byte RX descriptors at 0x1100..
        for i in 0..3u32 {
            let d = 0x1100 + 32 * i;
            m.w(d, OWN); m.w(d + 4, RDES1_RCH | 256); m.w(d + 8, 0x2000 + 256 * i); m.w(d + 12, 0x1100 + 32 * ((i + 1) % 3));
        }
        e.write(RX_BASE, 0x1100);
        e.write(INT_EN, EN_NIS | ST_TX | ST_RX);
        e.write(OP_MODE, OP_START_TX | OP_START_RX);
        assert!(e.run(&mut m));
        assert_eq!(e.take_tx(), vec![(0..60u8).collect::<Vec<_>>()]);
        assert_eq!(m.r(0x1000) & OWN, 0); assert_eq!(m.r(0x1020) & OWN, 0);
        assert!(e.irq()); assert_eq!(e.read(STATUS) & (ST_TX | ST_NIS), ST_TX | ST_NIS);
        e.write(STATUS, ST_TX | ST_TX_UNAVAIL | ST_NIS); assert!(!e.irq());

        let big: Vec<u8> = (0..400).map(|i| i as u8).collect();
        e.receive(big.clone()); e.receive(vec![1; 14]);
        e.run(&mut m);
        let (r0a, r0b, r0c) = (m.r(0x1100), m.r(0x1120), m.r(0x1140));
        assert_eq!(r0a, RDES0_FS);
        assert_eq!(r0b, RDES0_LS | 404 << 16, "two buffers, length with FCS in the last");
        assert_eq!(m.0[0x1000..0x1100], big[..256]); assert_eq!(m.0[0x1100..0x1190], big[256..]);
        assert_eq!(r0c, RDES0_FS | RDES0_LS | 64 << 16, "short frame padded to 60 + FCS");
        assert!(e.irq());
        // the ring is full: the next frame waits for a descriptor and a poll demand
        e.receive(vec![2; 20]); e.run(&mut m);
        assert_ne!(e.read(STATUS) & ST_RX_UNAVAIL, 0); assert_eq!(e.rx.len(), 1);
        m.w(0x1100, OWN); e.write(RX_POLL, 1); assert!(e.pending()); e.run(&mut m);
        assert!(e.rx.is_empty()); assert_eq!(m.r(0x1100), RDES0_FS | RDES0_LS | 64 << 16);
    }
}
