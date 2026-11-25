//! An SD card in SPI mode on a board's bus, byte by byte between a falling and a rising chip
//! select (SD Physical Layer Simplified Specification v9.10, chapter 7): commands are six bytes
//! (0x40 | index, a 32-bit argument, CRC7), answered after one byte (NCR) with R1, R2, R3 or R7;
//! data blocks go as a 0xFE token, the bytes and a CRC16. Implemented: CMD0, 6, 8, 9, 10, 12, 13,
//! 16, 17, 18, 24, 25, 32, 33, 38, 55, 58, 59, ACMD13, 41 and 51, enough for Arduino's SD library
//! (`sd_diskio.cpp`) and IDF's sdspi. The card's storage and its CID and CSD come from
//! `esp_periph::sdmmc::SdCard`: a high-capacity card (block addressing) of the image's size.
//! shortcut: host CRCs are not checked, and the card is never busy for longer than one byte.
use esp_periph::sdmmc::card::BLOCK;
use esp_periph::sdmmc::{SdCard, SdStore};
use std::collections::{HashMap, VecDeque};

enum Rx { Command, Token { lba: u32, multi: bool }, Data { lba: u32, multi: bool, buf: Vec<u8> } }

pub struct SdSpi {
    card: SdCard,
    selected: bool,
    cmd: Vec<u8>,
    out: VecDeque<u8>,
    rx: Rx,
    app: bool,
    idle: bool,
    /// the next block of a CMD18 read, until CMD12
    reading: Option<u32>,
    erase: (u32, u32),
    pub stats: SdSpiStats,
}

#[derive(Clone, Copy, Default, Debug)]
pub struct SdSpiStats { pub commands: u64, pub blocks_read: u64, pub blocks_written: u64 }

/// CRC-16/XMODEM (x^16 + x^12 + x^5 + 1), the data block CRC of SD spec §4.5.
pub fn crc16(data: &[u8]) -> u16 {
    data.iter().fold(0u16, |mut crc, &b| {
        crc ^= u16::from(b) << 8;
        for _ in 0..8 { crc = if crc & 0x8000 != 0 { crc << 1 ^ 0x1021 } else { crc << 1 }; }
        crc
    })
}

impl SdSpi {
    /// A card over `store` (a multiple of 512 KiB, as CSD 2.0 counts it).
    pub fn new(store: Box<dyn SdStore>) -> Result<Self, String> {
        Ok(SdSpi { card: SdCard::new(store)?, selected: false, cmd: Vec::new(), out: VecDeque::new(), rx: Rx::Command, app: false, idle: true, reading: None, erase: (0, 0), stats: SdSpiStats::default() })
    }

    pub fn select(&mut self, low: bool) {
        if low == self.selected { return; }
        self.selected = low;
        // a deselect ends what was on the wire; a started write block is dropped, as on a card
        if !low { self.cmd.clear(); self.out.clear(); self.reading = None; self.rx = Rx::Command; }
    }
    pub fn selected(&self) -> bool { self.selected }

    fn r1(&self) -> u8 { self.idle as u8 }
    fn respond(&mut self, bytes: &[u8]) { self.out.push_back(0xff); self.out.extend(bytes); }
    /// A data block after the response: one byte of access time, the start token, the bytes, the CRC.
    fn data(&mut self, bytes: &[u8]) {
        self.out.push_back(0xff);
        self.out.push_back(0xfe);
        self.out.extend(bytes);
        self.out.extend(crc16(bytes).to_be_bytes());
    }
    fn block(&mut self, lba: u32) -> Option<[u8; BLOCK]> {
        if u64::from(lba) >= self.card.blocks() { return None; }
        let mut buf = [0u8; BLOCK];
        self.card.store().read(u64::from(lba), &mut buf).ok()?;
        self.stats.blocks_read += 1;
        Some(buf)
    }

    /// One byte in on MOSI, the byte the card shifts out on MISO at the same time.
    pub fn exchange(&mut self, mosi: u8) -> u8 {
        if !self.selected { return 0xff; }
        if self.out.is_empty() {
            if let Some(lba) = self.reading {
                match self.block(lba) { Some(b) => { self.data(&b); self.reading = Some(lba + 1); } None => self.reading = None }
            }
        }
        let miso = self.out.pop_front().unwrap_or(0xff);
        match std::mem::replace(&mut self.rx, Rx::Command) {
            Rx::Token { lba, multi } => self.rx = match mosi {
                0xfe | 0xfc => Rx::Data { lba, multi, buf: Vec::with_capacity(BLOCK + 2) },
                // STOP_TRAN after a multiple-block write: one busy byte
                0xfd if multi => { self.out.push_back(0xff); self.out.push_back(0x00); Rx::Command }
                _ => Rx::Token { lba, multi },
            },
            Rx::Data { lba, multi, mut buf } => {
                buf.push(mosi);
                if buf.len() < BLOCK + 2 { self.rx = Rx::Data { lba, multi, buf }; } else {
                    let ok = u64::from(lba) < self.card.blocks() && self.card.store().write(u64::from(lba), buf[..BLOCK].try_into().unwrap()).is_ok();
                    if ok { self.stats.blocks_written += 1; }
                    // data response xxx0_sss1: 010 accepted, 110 write error; then one busy byte
                    self.out.push_back(if ok { 0x05 } else { 0x0d });
                    self.out.push_back(0x00);
                    self.rx = if multi { Rx::Token { lba: lba + 1, multi } } else { Rx::Command };
                }
            }
            Rx::Command => {
                if !self.cmd.is_empty() || mosi & 0xc0 == 0x40 { self.cmd.push(mosi); }
                if self.cmd.len() == 6 { let c = std::mem::take(&mut self.cmd); self.command(&c); }
            }
        }
        miso
    }

    fn command(&mut self, c: &[u8]) {
        self.stats.commands += 1;
        let (index, arg) = (c[0] & 0x3f, u32::from_be_bytes([c[1], c[2], c[3], c[4]]));
        let app = std::mem::take(&mut self.app);
        match (app, index) {
            (true, 41) => { self.idle = false; self.respond(&[0x00]); }
            // SET_CLR_CARD_DETECT (Arduino's SD library drops the CS pull-up after init) and
            // SET_WR_BLK_ERASE_COUNT (before a multiple-block write): accepted, nothing to do
            (true, 42) | (true, 23) => self.respond(&[self.r1()]),
            (true, 51) => { self.respond(&[self.r1()]); self.data(&[0x02, 3 << 4 | 0x5, 0x80, 0x02, 0, 0, 0, 0]); }
            (true, 13) => { self.respond(&[self.r1(), 0x00]); self.data(&[0; 64]); }
            (_, 0) => { self.idle = true; self.reading = None; self.respond(&[0x01]); }
            (_, 8) => self.respond(&[self.r1(), 0x00, 0x00, (arg >> 8) as u8 & 0x0f, arg as u8]),
            (_, 55) => { self.app = true; self.respond(&[self.r1()]); }
            // OCR: powered up, high capacity, 2.7-3.6 V
            (_, 58) => self.respond(&[self.r1(), 0xc0, 0xff, 0x80, 0x00]),
            (_, 59) | (_, 16) => self.respond(&[self.r1()]),
            (_, 9) => { let csd = self.card.csd().to_be_bytes(); self.respond(&[self.r1()]); self.data(&csd); }
            (_, 10) => { let cid = self.card.cid().to_be_bytes(); self.respond(&[self.r1()]); self.data(&cid); }
            (_, 13) => self.respond(&[self.r1(), 0x00]),
            // CMD6: no function switched (default speed only)
            (_, 6) => { self.respond(&[self.r1()]); self.data(&[0; 64]); }
            (_, 17) => match self.block(arg) {
                Some(b) => { self.respond(&[0x00]); self.data(&b); }
                None => self.respond(&[0x40]),   // parameter error: out of range
            },
            (_, 18) => { self.respond(&[0x00]); self.reading = Some(arg); }
            // the stuff byte the host clocks after CMD12, then R1b with one busy byte
            (_, 12) => { self.out.clear(); self.reading = None; self.out.extend([0xff, 0xff, 0x00, 0x00]); }
            (_, 24) | (_, 25) => { self.respond(&[0x00]); self.rx = Rx::Token { lba: arg, multi: index == 25 }; }
            (_, 32) => { self.erase.0 = arg; self.respond(&[0x00]); }
            (_, 33) => { self.erase.1 = arg; self.respond(&[0x00]); }
            (_, 38) => {
                let (lo, hi) = (u64::from(self.erase.0), u64::from(self.erase.1).min(self.card.blocks().saturating_sub(1)));
                if lo <= hi { let _ = self.card.store().erase(lo, hi); }
                self.respond(&[0x00, 0x00]);
            }
            _ => self.respond(&[self.r1() | 0x04]),   // illegal command
        }
    }

    /// One GP-SPI transaction while selected: full duplex when MISO is as long as MOSI, else the
    /// MOSI bytes then `rx_len` bytes read with MOSI idle high.
    pub fn transfer(&mut self, tx: &[u8], rx_len: usize) -> Vec<u8> {
        if rx_len == tx.len() { return tx.iter().map(|&b| self.exchange(b)).collect(); }
        for &b in tx { self.exchange(b); }
        (0..rx_len).map(|_| self.exchange(0xff)).collect()
    }
}

/// A card's blocks kept sparse: a block never written reads as zeros, so a 4 GiB card costs only
/// what its format and the firmware wrote (the browser build has no room for a whole image).
pub struct SparseStore { len: u64, blocks: HashMap<u64, Box<[u8; BLOCK]>> }
impl SparseStore {
    pub fn new(len: u64) -> Self { SparseStore { len, blocks: HashMap::new() } }
    fn put(&mut self, lba: u64, at: usize, bytes: &[u8]) {
        let b = self.blocks.entry(lba).or_insert_with(|| Box::new([0; BLOCK]));
        b[at..at + bytes.len()].copy_from_slice(bytes);
    }
}
impl SdStore for SparseStore {
    fn len(&self) -> u64 { self.len }
    fn read(&mut self, lba: u64, buf: &mut [u8; BLOCK]) -> std::io::Result<()> {
        *buf = self.blocks.get(&lba).map_or([0; BLOCK], |b| **b);
        Ok(())
    }
    fn write(&mut self, lba: u64, data: &[u8; BLOCK]) -> std::io::Result<()> { self.blocks.insert(lba, Box::new(*data)); Ok(()) }
    fn erase(&mut self, lo: u64, hi: u64) -> std::io::Result<()> { self.blocks.retain(|&lba, _| !(lo..=hi).contains(&lba)); Ok(()) }
    fn describe(&self) -> String { format!("{} MiB, {} blocks written", self.len >> 20, self.blocks.len()) }
}

/// A card as a formatter leaves it (the layout of `tests/sd_image.rs`, after Microsoft's FAT32
/// File System Specification 1.03): an MBR with one FAT32 (LBA) partition from LBA 2048, 32 KiB
/// clusters, `label`, and `files` (8.3 names) in the root directory. `bytes` must give FAT32 at
/// least 65525 clusters (2 GiB or more).
pub fn fat32_card(bytes: u64, label: &str, files: &[(&str, &[u8])]) -> SparseStore {
    const PART: u64 = 2048;
    const RESERVED: u32 = 32;
    const SPC: u32 = 64;
    let total = (bytes / 512 - PART) as u32;
    // FAT size: fatgen103, "FAT Type Determination" (TmpVal1 / TmpVal2, rounded up)
    let fat = (total - RESERVED).div_ceil((256 * SPC + 2) / 2);
    let clusters = (total - RESERVED - 2 * fat) / SPC;
    assert!(clusters >= 65525, "{clusters} clusters is not FAT32");
    let data = u64::from(RESERVED + 2 * fat);
    let mut s = SparseStore::new(bytes);
    let le32 = |v: u32| v.to_le_bytes();
    s.put(0, 446, &[0, 0xfe, 0xff, 0xff, 0x0c, 0xfe, 0xff, 0xff]);
    s.put(0, 454, &le32(PART as u32)); s.put(0, 458, &le32(total)); s.put(0, 510, &[0x55, 0xaa]);
    let mut name11 = [b' '; 11];
    for (i, c) in label.bytes().take(11).enumerate() { name11[i] = c.to_ascii_uppercase(); }
    for base in [PART, PART + 6] {
        s.put(base, 0, &[0xeb, 0x58, 0x90]); s.put(base, 3, b"ESP32SIM");
        s.put(base, 11, &512u16.to_le_bytes()); s.put(base, 13, &[SPC as u8]);
        s.put(base, 14, &(RESERVED as u16).to_le_bytes()); s.put(base, 16, &[2]); s.put(base, 21, &[0xf8]);
        s.put(base, 24, &63u16.to_le_bytes()); s.put(base, 26, &255u16.to_le_bytes());
        s.put(base, 28, &le32(PART as u32)); s.put(base, 32, &le32(total)); s.put(base, 36, &le32(fat));
        s.put(base, 44, &le32(2)); s.put(base, 48, &1u16.to_le_bytes()); s.put(base, 50, &6u16.to_le_bytes());
        s.put(base, 64, &[0x80, 0, 0x29]); s.put(base, 67, &le32(0x5d1c_0b17));
        s.put(base, 71, &name11); s.put(base, 82, b"FAT32   "); s.put(base, 510, &[0x55, 0xaa]);
    }
    // the root directory (cluster 2), then each file in its own clusters
    let mut fat_entries = vec![0x0fff_fff8u32, 0x0fff_ffff, 0x0fff_ffff];
    let mut dir = vec![{ let mut e = [0u8; 32]; e[..11].copy_from_slice(&name11); e[11] = 0x08; e }];
    for &(name, content) in files {
        let (stem, ext) = name.split_once('.').unwrap_or((name, ""));
        let mut e = [b' '; 32];
        e[11..].fill(0);
        for (i, c) in stem.bytes().take(8).enumerate() { e[i] = c.to_ascii_uppercase(); }
        for (i, c) in ext.bytes().take(3).enumerate() { e[8 + i] = c.to_ascii_uppercase(); }
        e[11] = 0x20;
        let first = fat_entries.len() as u32;
        let n = content.len().div_ceil(SPC as usize * 512) as u32;
        for k in 0..n { fat_entries.push(if k + 1 == n { 0x0fff_ffff } else { first + k + 1 }); }
        if n > 0 { e[20..22].copy_from_slice(&((first >> 16) as u16).to_le_bytes()); e[26..28].copy_from_slice(&(first as u16).to_le_bytes()); }
        e[28..32].copy_from_slice(&(content.len() as u32).to_le_bytes());
        for (k, chunk) in content.chunks(512).enumerate() { s.put(PART + data + u64::from(first - 2) * u64::from(SPC) + k as u64, 0, chunk); }
        dir.push(e);
    }
    let used = fat_entries.len() as u32 - 2;
    let fat_bytes: Vec<u8> = fat_entries.iter().flat_map(|v| v.to_le_bytes()).collect();
    for copy in 0..2u64 { for (k, chunk) in fat_bytes.chunks(512).enumerate() { s.put(PART + u64::from(RESERVED) + copy * u64::from(fat) + k as u64, 0, chunk); } }
    for base in [PART + 1, PART + 7] {
        s.put(base, 0, &le32(0x4161_5252)); s.put(base, 484, &le32(0x6141_7272));
        s.put(base, 488, &le32(clusters - used)); s.put(base, 492, &le32(fat_entries.len() as u32)); s.put(base, 508, &le32(0xaa55_0000));
    }
    for (i, e) in dir.iter().enumerate() { s.put(PART + data + (i * 32 / 512) as u64, i * 32 % 512, e); }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cmd(s: &mut SdSpi, index: u8, arg: u32) -> Vec<u8> {
        let a = arg.to_be_bytes();
        let mut tx = vec![0x40 | index, a[0], a[1], a[2], a[3], 0x95];
        tx.extend([0xff; 8]);
        s.transfer(&tx, tx.len())[6..].to_vec()
    }
    /// The first byte with bit 7 clear, and what follows it.
    fn r(v: &[u8]) -> &[u8] { &v[v.iter().position(|&b| b & 0x80 == 0).unwrap()..] }

    #[test]
    fn init_read_and_write_as_arduino_sd_does() {
        let mut card = fat32_card(4 << 30, "ESP32SIM", &[("README.TXT", b"hello sd")]);
        let mut mbr = [0u8; BLOCK];
        card.read(0, &mut mbr).unwrap();
        assert_eq!((&mbr[510..512], mbr[450]), (&[0x55, 0xaa][..], 0x0c), "an MBR with a FAT32 LBA partition");
        let mut vbr = [0u8; BLOCK];
        card.read(2048, &mut vbr).unwrap();
        assert_eq!(&vbr[82..90], b"FAT32   ");
        let mut s = SdSpi::new(Box::new(card)).unwrap();
        s.select(true);
        assert_eq!(r(&cmd(&mut s, 0, 0))[0], 0x01, "CMD0: idle");
        assert_eq!(&r(&cmd(&mut s, 8, 0x1aa))[..5], [0x01, 0, 0, 0x01, 0xaa], "CMD8 echo");
        cmd(&mut s, 55, 0);
        assert_eq!(r(&cmd(&mut s, 41, 1 << 30))[0], 0x00, "ACMD41: ready");
        assert_eq!(&r(&cmd(&mut s, 58, 0))[..2], [0x00, 0xc0], "OCR: powered up, CCS");
        // CSD 2.0 for 4 GiB: C_SIZE 8191
        let mut out = cmd(&mut s, 9, 0);
        out.extend(s.transfer(&[0xff; 24], 24));
        let at = out.iter().position(|&b| b == 0xfe).unwrap();
        let csd = &out[at + 1..at + 17];
        assert_eq!(csd[0] >> 6, 1);
        assert_eq!(u32::from_be_bytes([0, csd[7] & 0x3f, csd[8], csd[9]]), 8191);
        assert_eq!(u16::from_be_bytes([out[at + 17], out[at + 18]]), crc16(csd));
        // a block written with CMD24 reads back with CMD17
        assert_eq!(r(&cmd(&mut s, 24, 100))[0], 0x00);
        let mut tx = vec![0xfe]; tx.extend([0xa5; 512]); tx.extend([0, 0]); tx.extend([0xff; 4]);
        let resp = s.transfer(&tx, tx.len());
        assert_eq!(resp[515] & 0x1f, 0x05, "data accepted");
        let mut out = cmd(&mut s, 17, 100);
        out.extend(s.transfer(&[0xff; 520], 520));
        let at = out.iter().position(|&b| b == 0xfe).unwrap();
        assert!(out[at + 1..at + 513].iter().all(|&b| b == 0xa5));
        assert_eq!(s.stats.blocks_written, 1);
    }
}
