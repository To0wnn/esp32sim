//! An NXP PN532 NFC controller on I2C (address 0x24) with an NTAG213 tag on its antenna (PN532
//! user manual UM0701-02, NTAG213/215/216 datasheet Rev 3.2). The host writes normal information
//! frames (00 00 FF LEN LCS D4 command data DCS 00, §6.2.1.1); the chip answers with an ACK frame
//! (00 00 FF 00 FF 00) and then a response frame (D5, command + 1). An I2C read starts with a
//! status byte, 0x01 when a frame is ready (§6.2.4); a read of the status byte alone leaves the
//! frame for the next read, as Adafruit_PN532's `isready` polls it before `readdata`.
//! Commands: GetFirmwareVersion (PN532 v1.6), SAMConfiguration, RFConfiguration,
//! InListPassiveTarget at 106 kbps type A (the tag: ATQA 0044, SAK 00, 7-byte UID) and
//! InDataExchange with the tag's READ (0x30, four pages, rolling over) and WRITE (0xA2).
// shortcut: other baud rates (FeliCa, type B) find no tag, other commands answer with no data,
// and the tag has no password or lock bits; extend when a firmware needs them.
use std::collections::VecDeque;
use esp_periph::i2c::I2cDevice;

pub const PN532_ADDRESS: u8 = 0x24;
/// NTAG213: 45 pages of 4 bytes (datasheet §8.5).
const PAGES: usize = 45;
const ACK: [u8; 6] = [0x00, 0x00, 0xff, 0x00, 0xff, 0x00];

pub struct Pn532 {
    rx: Vec<u8>, writing: bool,
    frames: VecDeque<Vec<u8>>,
    /// Bytes read in the current read transfer, the status byte included.
    read_at: usize,
    pub uid: [u8; 7],
    pub tag: [u8; PAGES * 4],
    pub commands: u64,
}

impl Pn532 {
    pub fn new(uid: [u8; 7]) -> Self {
        let mut tag = [0u8; PAGES * 4];
        // pages 0-2: UID with its two check bytes (cascade tag 0x88), internal byte, lock bytes
        tag[..3].copy_from_slice(&uid[..3]);
        tag[3] = 0x88 ^ uid[0] ^ uid[1] ^ uid[2];
        tag[4..8].copy_from_slice(&uid[3..]);
        tag[8] = uid[3] ^ uid[4] ^ uid[5] ^ uid[6];
        tag[9] = 0x48;
        // page 3: capability container, NDEF version 1.0, 144 bytes of data (0x12 = NTAG213)
        tag[12..16].copy_from_slice(&[0xe1, 0x10, 0x12, 0x00]);
        // page 4: an empty NDEF message TLV and the terminator TLV
        tag[16..20].copy_from_slice(&[0x03, 0x00, 0xfe, 0x00]);
        Pn532 { rx: Vec::new(), writing: false, frames: VecDeque::new(), read_at: 0, uid, tag, commands: 0 }
    }
    /// A response frame: preamble, start code, length and its checksum, D5, data, data checksum.
    fn respond(&mut self, command: u8, data: &[u8]) {
        let len = (data.len() + 2) as u8;
        let mut f = vec![0x00, 0x00, 0xff, len, len.wrapping_neg(), 0xd5, command.wrapping_add(1)];
        f.extend_from_slice(data);
        let sum = f[5..].iter().fold(0u8, |s, &b| s.wrapping_add(b));
        f.extend([sum.wrapping_neg(), 0x00]);
        self.frames.push_back(f);
    }
    fn frame(&mut self) {
        let f = std::mem::take(&mut self.rx);
        // 00 00 FF LEN LCS D4 ...: length checksum and direction checked; a bad frame gets no ACK
        if f.len() < 8 || f[..3] != [0x00, 0x00, 0xff] || f[3].wrapping_add(f[4]) != 0 || f[5] != 0xd4 { return; }
        let len = usize::from(f[3]);
        if f.len() < 5 + len || len < 2 { return; }
        let (command, args) = (f[6], &f[7..5 + len]);
        self.commands += 1;
        self.frames.clear();
        self.frames.push_back(ACK.to_vec());
        let data = match (command, args) {
            (0x02, _) => vec![0x32, 0x01, 0x06, 0x07],      // IC PN532, firmware 1.6, ISO 14443A/B and 18092
            (0x4a, [_, 0x00, ..]) => {
                let mut d = vec![1, 1, 0x00, 0x44, 0x00, 7]; // one target: Tg 1, SENS_RES, SEL_RES, NFCID1 length
                d.extend_from_slice(&self.uid);
                d
            }
            (0x4a, _) => vec![0],                            // no FeliCa or type B target
            (0x40, [1, 0x30, page, ..]) => {
                let mut d = vec![0x00];                      // status OK, then 16 bytes from `page`
                d.extend((0..16).map(|i| self.tag[(usize::from(*page) * 4 + i) % self.tag.len()]));
                d
            }
            (0x40, [1, 0xa2, page, data @ ..]) if usize::from(*page) < PAGES && data.len() >= 4 && *page >= 4 => {
                let p = usize::from(*page) * 4;
                self.tag[p..p + 4].copy_from_slice(&data[..4]);
                vec![0x00]
            }
            (0x40, _) => vec![0x14],                         // a tag command the model does not do: authentication error
            _ => vec![],
        };
        self.respond(command, &data);
    }
}

impl I2cDevice for Pn532 {
    fn start(&mut self, read: bool) -> bool {
        self.writing = !read;
        if read { self.read_at = 0; } else { self.rx.clear(); }
        true
    }
    fn write(&mut self, b: u8) -> bool { self.rx.push(b); true }
    fn read(&mut self) -> u8 {
        let i = self.read_at;
        self.read_at += 1;
        match (i, self.frames.front()) {
            (0, f) => u8::from(f.is_some()),
            (_, Some(f)) => f.get(i - 1).copied().unwrap_or(0),
            (_, None) => 0,
        }
    }
    fn stop(&mut self) {
        if self.writing { if !self.rx.is_empty() { self.frame(); } }
        else if self.read_at > 1 { self.frames.pop_front(); }
        self.writing = false;
        self.read_at = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn command(d: &mut Pn532, cmd: &[u8]) -> Vec<u8> {
        let len = cmd.len() as u8 + 1;
        let sum = cmd.iter().fold(0xd4u8, |s, &b| s.wrapping_add(b));
        let mut f = vec![0, 0, 0xff, len, len.wrapping_neg(), 0xd4];
        f.extend_from_slice(cmd);
        f.extend([sum.wrapping_neg(), 0]);
        d.start(false); for b in f { d.write(b); } d.stop();
        let mut read = |n: usize| { d.start(true); let v: Vec<u8> = (0..n).map(|_| d.read()).collect(); d.stop(); v };
        assert_eq!(read(1), [1], "ready");
        assert_eq!(read(7)[1..], ACK);
        let r = read(40);
        assert_eq!(r[0], 1);
        assert_eq!(r[1..4], [0, 0, 0xff]);
        let len = usize::from(r[4]);
        assert_eq!(r[4].wrapping_add(r[5]), 0);
        assert_eq!(r[6..7 + len].iter().fold(0u8, |s, &b| s.wrapping_add(b)), 0, "data checksum");
        r[8..6 + len].to_vec()
    }
    #[test]
    fn firmware_version_tag_detection_and_page_reads() {
        let uid = [0x04, 0xa2, 0x24, 0x5e, 0x31, 0x6f, 0x80];
        let mut d = Pn532::new(uid);
        assert_eq!(command(&mut d, &[0x02]), [0x32, 0x01, 0x06, 0x07]);
        assert_eq!(command(&mut d, &[0x14, 0x01, 0x14, 0x01]), []);
        let t = command(&mut d, &[0x4a, 1, 0]);
        assert_eq!(t[..6], [1, 1, 0x00, 0x44, 0x00, 7]);
        assert_eq!(t[6..], uid);
        assert_eq!(command(&mut d, &[0x4a, 1, 1]), [0], "no FeliCa tag");
        let p = command(&mut d, &[0x40, 1, 0x30, 3]);
        assert_eq!(p[..5], [0x00, 0xe1, 0x10, 0x12, 0x00], "status, then the NTAG213 capability container");
        assert_eq!(p.len(), 17);
        assert_eq!(command(&mut d, &[0x40, 1, 0xa2, 5, 1, 2, 3, 4]), [0x00]);
        assert_eq!(command(&mut d, &[0x40, 1, 0x30, 5])[1..5], [1, 2, 3, 4]);
        let rollover = command(&mut d, &[0x40, 1, 0x30, 44]);
        assert_eq!(rollover[5..9], uid[..3].iter().copied().chain([0x88 ^ uid[0] ^ uid[1] ^ uid[2]]).collect::<Vec<_>>()[..]);
    }
}
