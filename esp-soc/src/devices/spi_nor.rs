//! A SPI NOR flash on a board's bus, byte by byte as the chip sees it between a falling and a
//! rising chip select: JEDEC RDID (0x9f), READ (0x03), FAST READ (0x0b), RDSR (0x05), WREN/WRDI
//! (0x06/0x04), READ ID (0x90), RELEASE/RES (0xab), PAGE PROGRAM (0x02), SECTOR/BLOCK ERASE
//! (0x20, 0x52, 0xd8) and CHIP ERASE (0xc7, 0x60), with 24-bit addresses (Winbond W25Q16JV
//! datasheet, sections 8.1-8.2). A program or erase takes effect when chip select rises and
//! needs WREN first, as on the part.
//! shortcut: programs and erases complete at once (BUSY never reads 1); a firmware that times
//! them sees no wait. Add a busy time when a test needs one.

pub struct SpiNor {
    pub mem: Vec<u8>,
    jedec: [u8; 3],
    status: u8,
    selected: bool,
    /// the frame so far: opcode and address bytes, how many bytes went by, the program data
    head: [u8; 5],
    count: usize,
    page: Vec<u8>,
    pub stats: SpiNorStats,
}

#[derive(Clone, Copy, Default, Debug)]
pub struct SpiNorStats { pub frames: u64, pub programs: u64, pub erases: u64 }

impl SpiNor {
    /// An erased part of `size` bytes (a power of two) answering RDID with `jedec`.
    pub fn new(jedec: [u8; 3], size: usize) -> Self {
        SpiNor { mem: vec![0xff; size], jedec, status: 0, selected: false, head: [0; 5], count: 0, page: Vec::new(), stats: SpiNorStats::default() }
    }

    /// Chip select: `true` while it is driven low.
    pub fn select(&mut self, low: bool) {
        if low == self.selected { return; }
        if !low { self.finish(); }
        self.selected = low;
        self.count = 0;
        self.page.clear();
    }
    pub fn selected(&self) -> bool { self.selected }

    fn addr(&self) -> usize { (usize::from(self.head[1]) << 16 | usize::from(self.head[2]) << 8 | usize::from(self.head[3])) & (self.mem.len() - 1) }

    /// One byte in on MOSI, the byte the part shifts out on MISO at the same time.
    pub fn exchange(&mut self, mosi: u8) -> u8 {
        if !self.selected { return 0xff; }
        let i = self.count;
        self.count += 1;
        if i < self.head.len() { self.head[i] = mosi; }
        let op = self.head[0];
        if op == 0x02 && i >= 4 && self.page.len() < 256 { self.page.push(mosi); }
        let n = self.mem.len();
        match op {
            0x9f if i >= 1 => self.jedec[(i - 1) % 3],
            0x05 if i >= 1 => self.status,
            0x03 if i >= 4 => self.mem[(self.addr() + i - 4) & (n - 1)],
            0x0b if i >= 5 => self.mem[(self.addr() + i - 5) & (n - 1)],
            0x90 if i >= 4 => [self.jedec[0], self.device_id()][(i - 4) % 2],
            0xab if i >= 4 => self.device_id(),
            _ => 0xff,
        }
    }

    /// The legacy device ID (0x90/0xab): the capacity byte minus one on Winbond parts (0x15 -> 0x14).
    fn device_id(&self) -> u8 { self.jedec[2].wrapping_sub(1) }

    /// Chip select rose: the frame's command takes effect.
    fn finish(&mut self) {
        if self.count == 0 { return; }
        self.stats.frames += 1;
        let wel = self.status & 2 != 0;
        let erase = |s: &mut Self, size: usize| {
            let a = s.addr() & !(size - 1);
            s.mem[a..a + size].fill(0xff);
            s.stats.erases += 1;
        };
        match self.head[0] {
            0x06 => self.status |= 2,
            0x04 => self.status &= !2,
            0x02 if wel && self.count > 4 => {
                // within the 256-byte page, wrapping as the part does; programming only clears bits
                let base = self.addr() & !0xff;
                let start = self.addr() & 0xff;
                for (k, &b) in self.page.iter().enumerate() { self.mem[base + ((start + k) & 0xff)] &= b; }
                self.stats.programs += 1;
            }
            0x20 if wel && self.count >= 4 => erase(self, 0x1000),
            0x52 if wel && self.count >= 4 => erase(self, 0x8000.min(self.mem.len())),
            0xd8 if wel && self.count >= 4 => erase(self, 0x10000.min(self.mem.len())),
            0xc7 | 0x60 if wel => { self.mem.fill(0xff); self.stats.erases += 1; }
            _ => return,
        }
        // every program or erase clears the write enable latch
        if self.head[0] != 0x06 { self.status &= !2; }
    }

    /// One GP-SPI transaction while selected: full duplex when MISO is as long as MOSI (Arduino's
    /// `SPI.transfer`), else the MOSI bytes then `rx_len` bytes read with MOSI idle high.
    pub fn transfer(&mut self, tx: &[u8], rx_len: usize) -> Vec<u8> {
        if rx_len == tx.len() { return tx.iter().map(|&b| self.exchange(b)).collect(); }
        for &b in tx { self.exchange(b); }
        (0..rx_len).map(|_| self.exchange(0xff)).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(f: &mut SpiNor, bytes: &[u8]) -> Vec<u8> {
        f.select(true);
        let out = bytes.iter().map(|&b| f.exchange(b)).collect();
        f.select(false);
        out
    }

    #[test]
    fn id_program_read_and_erase_as_a_w25q16() {
        let mut f = SpiNor::new([0xef, 0x40, 0x15], 2 << 20);
        assert_eq!(&frame(&mut f, &[0x9f, 0, 0, 0])[1..], [0xef, 0x40, 0x15]);
        assert_eq!(&frame(&mut f, &[0x90, 0, 0, 0, 0, 0])[4..], [0xef, 0x14]);
        // a program without WREN does nothing
        frame(&mut f, &[0x02, 0, 0x10, 0, 0x12]);
        assert_eq!(f.mem[0x1000], 0xff);
        frame(&mut f, &[0x06]);
        assert_eq!(frame(&mut f, &[0x05, 0])[1], 2, "WEL set");
        frame(&mut f, &[0x02, 0, 0x10, 0xfe, 0x12, 0x34, 0x56]);
        assert_eq!(frame(&mut f, &[0x05, 0])[1], 0, "WEL cleared by the program");
        assert_eq!(&f.mem[0x10fe..0x1100], [0x12, 0x34], "the page wraps");
        assert_eq!(f.mem[0x1000], 0x56);
        assert_eq!(&frame(&mut f, &[0x03, 0, 0x10, 0xfe, 0, 0])[4..], [0x12, 0x34]);
        assert_eq!(&frame(&mut f, &[0x0b, 0, 0x10, 0xff, 0, 0])[5..], [0x34]);
        frame(&mut f, &[0x06]);
        frame(&mut f, &[0x20, 0, 0x10, 0x80]);
        assert_eq!(f.mem[0x1000], 0xff, "the 4 KiB sector around the address is erased");
        // one byte per transaction, as Arduino's SPI.transfer sends them under a GPIO chip select
        f.select(true);
        let id: Vec<u8> = [0x9f, 0, 0, 0].iter().flat_map(|&b| f.transfer(&[b], 1)).collect();
        f.select(false);
        assert_eq!(&id[1..], [0xef, 0x40, 0x15]);
        assert_eq!(f.transfer(&[0x9f], 1), [0xff], "not selected: MISO floats high");
    }
}
