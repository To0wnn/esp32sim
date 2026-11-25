//! TWAI-FD: Espressif's CAN / CAN-FD controller, the CTU CAN FD IP (ESP32-S31: DR_REG_TWAIFD0_BASE
//! 0x20394000, TWAIFD1 +0x1000). Registers from IDF 6.1.1 soc/esp32s31/register/soc/twaifd_reg.h
//! and twaifd_struct.h; the driver sequence from esp_hal_twai/twai_hal_v2.c, twaifd_ll.h and
//! esp_driver_twai/esp_twai_onchip.c.
//! Modelled: soft reset, enable (integration ends at once: error active, FCSI), the interrupt
//! status/enable/mask registers, four TXT buffers with their commands, states and priorities,
//! the 128-word RX buffer read word by word through RX_DATA, the mask filters A-C and the range
//! filter, ACK errors with the retransmit limit, TEC/REC and the fault states, the traffic
//! counters, self-test (STM), internal loopback (ILBP) and bus monitoring (BMM) modes.
//! The bus is a callback: a frame leaves the controller as a `CanFrame`, and whatever node is
//! attached ACKs it and answers with frames of its own (`Bus`).
//! shortcut: a frame takes no bus time (it completes at the next tick) and bit timing, the
//! timestamp timer, arbitration and bit/stuff/CRC errors are not modelled; add bus time when a
//! firmware measures it.
use std::collections::VecDeque;

/// One CAN or CAN-FD frame on the bus. `data` holds the payload as many bytes as the DLC gives
/// (a classic DLC above 8 carries 8; a remote frame carries none).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct CanFrame {
    pub id: u32,
    /// 29-bit identifier
    pub ext: bool,
    /// CAN-FD frame (FDF) and its bit-rate switch (BRS)
    pub fd: bool,
    pub brs: bool,
    /// remote frame (classic only)
    pub rtr: bool,
    pub data: Vec<u8>,
}

/// CAN-FD payload lengths of DLC 9-15.
const FD_LEN: [usize; 7] = [12, 16, 20, 24, 32, 48, 64];
impl CanFrame {
    pub fn dlc_len(dlc: u32, fd: bool) -> usize {
        match dlc { 0..=8 => dlc as usize, _ if fd => FD_LEN[(dlc - 9) as usize], _ => 8 }
    }
    /// The smallest DLC that carries `len` bytes.
    pub fn len_dlc(len: usize) -> u32 { if len <= 8 { len as u32 } else { 9 + FD_LEN.iter().position(|&l| len <= l).unwrap_or(6) as u32 } }
}

/// What the rest of the bus does with a frame: `None` when no other node is there to ACK it,
/// else the frames the other nodes send back.
pub type Bus<'a> = dyn FnMut(&CanFrame) -> Option<Vec<CanFrame>> + 'a;

// Register offsets (twaifd_reg.h)
const DEVICE_ID: u32 = 0x00;
const MODE: u32 = 0x04;
const STATUS: u32 = 0x08;
const COMMAND: u32 = 0x0c;
const INT_STAT: u32 = 0x10;
const INT_ENA_SET: u32 = 0x14;
const INT_ENA_CLR: u32 = 0x18;
const INT_MASK_SET: u32 = 0x1c;
const INT_MASK_CLR: u32 = 0x20;
const EWL_ERP_FAULT: u32 = 0x2c;
const REC_TEC: u32 = 0x30;
const FILTER_A_MASK: u32 = 0x3c;
const FILTER_RAN_LOW: u32 = 0x54;
const FILTER_RAN_HIGH: u32 = 0x58;
const FILTER_CONTROL: u32 = 0x5c;
const RX_MEM_INFO: u32 = 0x60;
const RX_POINTERS: u32 = 0x64;
const RX_STATUS: u32 = 0x68;
const RX_DATA: u32 = 0x6c;
const TX_STATUS: u32 = 0x70;
const TX_COMMAND: u32 = 0x74;
const TX_PRIORITY: u32 = 0x78;
const ERR_CAPT: u32 = 0x7c;
const RX_FR_CTR: u32 = 0x84;
const TX_FR_CTR: u32 = 0x88;
/// TXT buffer n's 20 words (format, identifier, timestamp low/high, 16 data words) at 0x100 * (n + 1)
const TXT_BASE: u32 = 0x100;

// MODE_SETTINGS bits
const RST: u32 = 1 << 0;
const BMM: u32 = 1 << 1;
const STM: u32 = 1 << 2;
const AFM: u32 = 1 << 3;
const RTRLE: u32 = 1 << 16;
const ILBP: u32 = 1 << 21;
const ENA: u32 = 1 << 22;
/// Reset value: FDE, RXBAM, TBFBO
const MODE_RESET: u32 = 1 << 4 | 1 << 9 | 1 << 25;

// INT_STAT bits
pub const RXI: u32 = 1 << 0;
pub const TXI: u32 = 1 << 1;
pub const DOI: u32 = 1 << 3;
pub const FCSI: u32 = 1 << 4;
pub const BEI: u32 = 1 << 6;
pub const RBNEI: u32 = 1 << 10;
pub const TXBHCI: u32 = 1 << 11;

// COMMAND bits
const RRB: u32 = 1 << 2;
const CDO: u32 = 1 << 3;
const ERCRST: u32 = 1 << 4;
const RXFCRST: u32 = 1 << 5;
const TXFCRST: u32 = 1 << 6;

// TXT buffer states (TX_STATUS, 4 bits each)
pub const TXT_RDY: u8 = 0x1;
pub const TXT_TRAN: u8 = 0x2;
pub const TXT_TOK: u8 = 0x4;
pub const TXT_ERR: u8 = 0x6;
pub const TXT_ABT: u8 = 0x7;
pub const TXT_ETY: u8 = 0x8;
/// TXT_BUFFER_COUNT's reset value: the S31 has 4 (the struct reserves 8 cells).
const TXT_BUFFERS: usize = 4;
/// RX_BUFF_SIZE in 32-bit words
const RX_WORDS: usize = 128;
/// ERR_TYPE 3: acknowledge error (twaifd_ll.h TWAIFD_LL_ERR_ACK_ERR)
const ERR_ACK: u32 = 3;

pub struct TwaiFd {
    /// plain read/write words: timing, filters, priorities, TXT buffers, timer registers
    regs: Box<[u32; 0x400]>,
    int_stat: u32,
    int_ena: u32,
    int_mask: u32,
    tec: u32,
    rec: u32,
    bus_off: bool,
    /// overrun flag (STATUS DOR) until CDO
    overrun: bool,
    /// stored frames, each its words (format, identifier, timestamp low/high, data)
    rx: VecDeque<Vec<u32>>,
    /// words of the front frame already read through RX_DATA
    rx_read: usize,
    tx_state: [u8; TXT_BUFFERS],
    /// failed attempts of each buffer's frame, against RTRTH
    tx_tries: [u32; TXT_BUFFERS],
    rx_frames: u32,
    tx_frames: u32,
}

impl Default for TwaiFd { fn default() -> Self { Self::new() } }

impl TwaiFd {
    pub fn new() -> Self {
        let mut t = TwaiFd { regs: Box::new([0; 0x400]), int_stat: 0, int_ena: 0, int_mask: 0, tec: 0, rec: 0, bus_off: false, overrun: false,
            rx: VecDeque::new(), rx_read: 0, tx_state: [TXT_ETY; TXT_BUFFERS], tx_tries: [0; TXT_BUFFERS], rx_frames: 0, tx_frames: 0 };
        t.set_reg(MODE, MODE_RESET);
        // EW_LIMIT 96, ERP_LIMIT 128; filter A open for all four frame types
        t.set_reg(EWL_ERP_FAULT, 96 | 128 << 8);
        t.set_reg(FILTER_CONTROL, 0xf);
        t
    }

    fn reg(&self, off: u32) -> u32 { self.regs[(off as usize & 0xfff) / 4] }
    fn set_reg(&mut self, off: u32, v: u32) { self.regs[(off as usize & 0xfff) / 4] = v; }
    fn mode(&self) -> u32 { self.reg(MODE) }
    pub fn enabled(&self) -> bool { self.mode() & ENA != 0 }

    /// The interrupt line: a pending status bit that is enabled.
    pub fn irq(&self) -> bool { self.int_stat & self.int_ena != 0 }

    fn raise(&mut self, bits: u32) { self.int_stat |= bits & !self.int_mask; }

    fn passive(&self) -> bool { self.tec >= (self.reg(EWL_ERP_FAULT) >> 8 & 0xff).max(1) || self.rec >= (self.reg(EWL_ERP_FAULT) >> 8 & 0xff).max(1) }

    fn fault_bits(&self) -> u32 {
        if self.bus_off { 1 << 18 } else if self.passive() { 1 << 17 } else { 1 << 16 }
    }

    pub fn read(&mut self, off: u32) -> u32 {
        match off {
            // DEVICE_ID 0xCAFD, version 2.4 (twaifd_struct.h defaults)
            DEVICE_ID => 0x0204_cafd,
            STATUS => {
                let tx_free = self.tx_state.iter().any(|&s| s != TXT_RDY && s != TXT_TRAN);
                let ewl = self.tec.max(self.rec) >= (self.reg(EWL_ERP_FAULT) & 0xff);
                u32::from(!self.rx.is_empty()) | u32::from(self.overrun) << 1 | u32::from(tx_free) << 2 | u32::from(ewl) << 6
                    | u32::from(self.tx_state.iter().all(|&s| s != TXT_TRAN)) << 7 | 7 << 16
            }
            COMMAND => 0,
            INT_STAT => self.int_stat,
            INT_ENA_SET | INT_ENA_CLR => self.int_ena,
            INT_MASK_SET | INT_MASK_CLR => self.int_mask,
            EWL_ERP_FAULT => self.reg(EWL_ERP_FAULT) & 0xffff | self.fault_bits(),
            REC_TEC => self.rec | self.tec << 16,
            FILTER_CONTROL => self.reg(FILTER_CONTROL) & 0xffff | 0xf << 16,
            RX_MEM_INFO => RX_WORDS as u32 | (self.rx_free() as u32) << 16,
            RX_POINTERS => 0,
            RX_STATUS => {
                u32::from(self.rx.is_empty()) | u32::from(self.rx_free() == 0) << 1 | u32::from(self.rx_read > 0) << 2
                    | (self.rx.len() as u32 & 0x7ff) << 4 | self.reg(RX_STATUS) & 1 << 16
            }
            RX_DATA => self.pop_rx_word(),
            TX_STATUS => self.tx_state.iter().enumerate().fold(0, |w, (i, &s)| w | u32::from(s) << (4 * i)),
            TX_COMMAND => (TXT_BUFFERS as u32) << 16,
            ERR_CAPT => self.reg(ERR_CAPT),
            RX_FR_CTR => self.rx_frames,
            TX_FR_CTR => self.tx_frames,
            _ => self.reg(off),
        }
    }

    pub fn write(&mut self, off: u32, value: u32) {
        match off {
            MODE => {
                if value & RST != 0 { *self = Self::new(); return; }
                let was = self.enabled();
                self.set_reg(MODE, value);
                // Integration (11 recessive bits) ends at once: the fault state machine leaves
                // its reset state, which twai_hal_start waits for (twaifd_ll_waiting_state_change).
                if !was && self.enabled() { self.raise(FCSI); }
            }
            COMMAND => {
                if value & RRB != 0 { self.rx.clear(); self.rx_read = 0; }
                if value & CDO != 0 { self.overrun = false; }
                if value & ERCRST != 0 {
                    let before = self.fault_bits();
                    self.tec = 0; self.rec = 0; self.bus_off = false;
                    if self.fault_bits() != before { self.raise(FCSI); }
                }
                if value & RXFCRST != 0 { self.rx_frames = 0; }
                if value & TXFCRST != 0 { self.tx_frames = 0; }
            }
            INT_STAT => self.int_stat &= !value,
            INT_ENA_SET => self.int_ena |= value & 0xfff,
            INT_ENA_CLR => self.int_ena &= !value,
            INT_MASK_SET => self.int_mask |= value & 0xfff,
            INT_MASK_CLR => self.int_mask &= !value,
            TX_COMMAND => {
                for i in (0..TXT_BUFFERS).filter(|i| value >> (8 + i) & 1 != 0) {
                    let s = &mut self.tx_state[i];
                    match value & 7 {
                        // TXCR: Empty, TX OK, Failed or Aborted -> Ready
                        2 if matches!(*s, TXT_ETY | TXT_TOK | TXT_ERR | TXT_ABT) => { *s = TXT_RDY; self.tx_tries[i] = 0; }
                        // TXCE: a finished buffer -> Empty
                        1 if matches!(*s, TXT_TOK | TXT_ERR | TXT_ABT) => *s = TXT_ETY,
                        // TXCA: a waiting (or retransmitting) frame is aborted
                        4 if matches!(*s, TXT_RDY | TXT_TRAN) => { *s = TXT_ABT; self.raise(TXBHCI); }
                        _ => {}
                    }
                }
            }
            STATUS | REC_TEC | RX_MEM_INFO | RX_POINTERS | RX_DATA | TX_STATUS | ERR_CAPT | RX_FR_CTR | TX_FR_CTR | DEVICE_ID => {}
            _ => self.set_reg(off, value),
        }
    }

    fn rx_free(&self) -> usize { RX_WORDS - self.rx.iter().map(Vec::len).sum::<usize>() + self.rx_read }

    fn pop_rx_word(&mut self) -> u32 {
        let Some(frame) = self.rx.front() else { return 0 };
        let w = frame[self.rx_read];
        self.rx_read += 1;
        if self.rx_read == frame.len() { self.rx.pop_front(); self.rx_read = 0; }
        w
    }

    /// A TXT buffer is waiting for the bus.
    pub fn tx_pending(&self) -> bool { self.enabled() && !self.bus_off && self.mode() & BMM == 0 && self.tx_state.contains(&TXT_RDY) }

    /// Send every Ready buffer, highest TX_PRIORITY first (the lower index on a tie), over `bus`.
    pub fn run_tx(&mut self, bus: &mut Bus) {
        while self.tx_pending() {
            let prio = |i: usize| self.reg(TX_PRIORITY) >> (4 * i) & 7;
            let Some(i) = (0..TXT_BUFFERS).filter(|&i| self.tx_state[i] == TXT_RDY).max_by_key(|&i| (prio(i), std::cmp::Reverse(i))) else { break };
            let frame = self.txt_frame(i);
            let answer = bus(&frame);
            let mode = self.mode();
            if answer.is_some() || mode & STM != 0 {
                self.tx_state[i] = TXT_TOK;
                self.tx_frames = self.tx_frames.wrapping_add(1);
                self.tec = self.tec.saturating_sub(1);
                self.raise(TXI | TXBHCI);
                if mode & ILBP != 0 { self.receive(&frame); }
                for f in answer.unwrap_or_default() { self.receive(&f); }
            } else {
                // No ACK: an acknowledge error. An error-passive transmitter's ACK error does not
                // raise its TEC (ISO 11898-1 fault confinement), so a lone node never goes bus off.
                let before = self.fault_bits();
                if !self.passive() { self.tec += 8; }
                self.set_reg(ERR_CAPT, ERR_ACK << 5);
                self.raise(BEI);
                if self.fault_bits() != before { self.raise(FCSI); }
                self.tx_tries[i] += 1;
                if mode & RTRLE != 0 && self.tx_tries[i] > (mode >> 17 & 0xf) {
                    self.tx_state[i] = TXT_ERR;
                    self.raise(TXBHCI);
                } else {
                    // shortcut: an unlimited retransmission stays in progress (as on a bus with no
                    // other node) without repeating the error each tick.
                    self.tx_state[i] = TXT_TRAN;
                }
            }
        }
    }

    fn txt_frame(&self, i: usize) -> CanFrame {
        let base = TXT_BASE * (i as u32 + 1);
        let format = self.reg(base);
        let ident = self.reg(base + 4);
        let (ext, rtr, fd) = (format & 1 << 6 != 0, format & 1 << 5 != 0, format & 1 << 7 != 0);
        let len = if rtr && !fd { 0 } else { CanFrame::dlc_len(format & 0xf, fd) };
        let data = (0..len).map(|b| (self.reg(base + 16 + (b as u32 & !3)) >> (8 * (b % 4))) as u8).collect();
        CanFrame { id: if ext { ident & 0x1fff_ffff } else { ident >> 18 & 0x7ff }, ext, fd, brs: fd && format & 1 << 9 != 0, rtr: rtr && !fd, data }
    }

    /// The identifier word as the TXT/RX buffers and the filters hold it: a base ID in bits 28:18.
    fn ident(f: &CanFrame) -> u32 { if f.ext { f.id & 0x1fff_ffff } else { (f.id & 0x7ff) << 18 } }

    /// The acceptance filters (MODE AFM): filter A-C match `ident & mask == value & mask`, the range
    /// filter `low <= ident <= high`, each for the frame types its FILTER_CONTROL bits enable.
    fn accepted(&self, f: &CanFrame) -> bool {
        if self.mode() & AFM == 0 { return true; }
        let kind = usize::from(f.ext) | usize::from(f.fd) << 1;
        let ctrl = self.reg(FILTER_CONTROL);
        let id = Self::ident(f);
        let mask_hit = (0..3).any(|n| {
            let (mask, val) = (self.reg(FILTER_A_MASK + 8 * n), self.reg(FILTER_A_MASK + 4 + 8 * n));
            ctrl >> (4 * n as usize + kind) & 1 != 0 && id & mask == val & mask
        });
        mask_hit || ctrl >> (12 + kind) & 1 != 0 && (self.reg(FILTER_RAN_LOW)..=self.reg(FILTER_RAN_HIGH)).contains(&id)
    }

    /// A frame from the bus: stored when it passes the filters (and the RTR drop, FDRF) and fits.
    pub fn receive(&mut self, f: &CanFrame) {
        if !self.enabled() || self.bus_off || !self.accepted(f) || f.rtr && self.mode() & 1 << 26 != 0 { return; }
        let dlc = if f.rtr { f.data.len().min(8) as u32 } else { CanFrame::len_dlc(f.data.len()) };
        let len = if f.rtr { 0 } else { CanFrame::dlc_len(dlc, f.fd) };
        let mut bytes = f.data.clone();
        bytes.resize(len, 0);
        let mut words = vec![0, Self::ident(f), 0, 0];
        words.extend(bytes.chunks(4).map(|c| c.iter().rev().fold(0u32, |w, &b| w << 8 | u32::from(b))));
        // FRAME_FORMAT_W: DLC, RTR, IDE, FDF, BRS, and RWCNT = the words after this one
        words[0] = dlc | u32::from(f.rtr) << 5 | u32::from(f.ext) << 6 | u32::from(f.fd) << 7 | u32::from(f.brs) << 9 | ((words.len() as u32 - 1) << 11);
        if words.len() > self.rx_free() { self.overrun = true; self.raise(DOI); return; }
        self.rx.push_back(words);
        self.rx_frames = self.rx_frames.wrapping_add(1);
        self.rec = self.rec.saturating_sub(1);
        self.raise(RXI | RBNEI);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// twai_hal_init / twai_hal_start: reset, mode, interrupts, then enable waits for FCSI.
    fn started() -> TwaiFd {
        let mut t = TwaiFd::new();
        t.write(MODE, RST);
        assert_eq!(t.read(TX_COMMAND) >> 16 & 0xf, 4, "four TXT buffers");
        t.write(MODE, MODE_RESET | AFM | RTRLE);   // fail_retry_cnt 0: one shot
        t.write(INT_ENA_SET, TXBHCI | RBNEI | BEI | FCSI);
        t.write(MODE, t.mode() | ENA);
        assert_ne!(t.read(INT_STAT) & FCSI, 0, "integration done");
        assert_eq!(t.read(EWL_ERP_FAULT) & 7 << 16, 1 << 16, "error active");
        t.write(INT_STAT, t.int_stat);
        t
    }

    /// twaifd_ll_format_frame_header / mount_tx_buffer / set_tx_buffer_cmd(TXCR) for buffer `i`.
    fn queue(t: &mut TwaiFd, i: u32, id: u32, data: &[u8]) {
        let base = TXT_BASE * (i + 1);
        t.write(base, data.len() as u32);
        t.write(base + 4, id << 18);
        for (n, c) in data.chunks(4).enumerate() { t.write(base + 16 + 4 * n as u32, c.iter().rev().fold(0, |w, &b| w << 8 | u32::from(b))); }
        t.write(TX_COMMAND, 2 | 1 << (8 + i));
    }

    /// twaifd_ll_get_rx_frame: the format word, then RWCNT more.
    fn read_frame(t: &mut TwaiFd) -> Vec<u32> {
        let format = t.read(RX_DATA);
        let mut w = vec![format];
        for _ in 0..format >> 11 & 0x1f { w.push(t.read(RX_DATA)); }
        w
    }

    #[test]
    fn an_acknowledged_frame_completes_and_the_answer_is_received() {
        let mut t = started();
        queue(&mut t, 0, 0x7df, &[2, 1, 0x0c, 0x55, 0x55, 0x55, 0x55, 0x55]);
        assert!(t.tx_pending());
        let mut seen = Vec::new();
        t.run_tx(&mut |f: &CanFrame| { seen.push(f.clone()); Some(vec![CanFrame { id: 0x7e8, data: vec![4, 0x41, 0x0c, 0x0d, 0x48, 0xaa, 0xaa, 0xaa], ..Default::default() }]) });
        assert_eq!(seen, [CanFrame { id: 0x7df, data: vec![2, 1, 0x0c, 0x55, 0x55, 0x55, 0x55, 0x55], ..Default::default() }]);
        assert_eq!(t.read(TX_STATUS) & 0xf, u32::from(TXT_TOK));
        assert!(t.irq(), "TXBHCI and RBNEI are enabled");
        assert_eq!(t.read(INT_STAT) & (TXBHCI | RBNEI), TXBHCI | RBNEI);
        assert_eq!(t.read(RX_STATUS) >> 4 & 0x7ff, 1, "one frame stored");
        let w = read_frame(&mut t);
        assert_eq!((w.len(), w[0] & 0xf, w[1] >> 18), (6, 8, 0x7e8));
        assert_eq!(w[4..], [0x0d0c_4104, 0xaaaa_aa48]);
        assert_eq!(t.read(RX_STATUS) & 1, 1, "empty again");
        // the ISR clears the buffer to Empty
        t.write(TX_COMMAND, 1 | 1 << 8);
        assert_eq!(t.read(TX_STATUS) & 0xf, u32::from(TXT_ETY));
        assert_eq!((t.read(TX_FR_CTR), t.read(RX_FR_CTR)), (1, 1));
    }

    #[test]
    fn no_ack_fails_after_the_retransmit_limit_and_raises_tec() {
        let mut t = started();
        queue(&mut t, 1, 0x123, &[0xaa]);
        t.run_tx(&mut |_: &CanFrame| None);
        assert_eq!(t.read(TX_STATUS) >> 4 & 0xf, u32::from(TXT_ERR));
        assert_eq!(t.read(INT_STAT) & (BEI | TXBHCI), BEI | TXBHCI);
        assert_eq!(t.read(ERR_CAPT) >> 5 & 7, ERR_ACK);
        assert_eq!(t.read(REC_TEC) >> 16, 8);
    }

    #[test]
    fn filters_and_internal_loopback() {
        let mut t = started();
        // filter A accepts only classic base IDs 0x100-0x10f
        t.write(MODE, t.mode() & !ENA);
        t.write(FILTER_A_MASK, 0x7f0 << 18);
        t.write(FILTER_A_MASK + 4, 0x100 << 18);
        t.write(FILTER_CONTROL, 1);
        t.write(MODE, t.mode() | ENA | STM | ILBP);
        queue(&mut t, 0, 0x105, &[1, 2, 3]);
        queue(&mut t, 1, 0x205, &[4]);
        t.run_tx(&mut |_: &CanFrame| None);
        assert_eq!(t.read(TX_STATUS) & 0xff, u32::from(TXT_TOK) * 0x11, "self test: no ACK needed");
        assert_eq!(t.read(RX_STATUS) >> 4 & 0x7ff, 1, "only 0x105 passes filter A");
        assert_eq!(read_frame(&mut t)[1] >> 18, 0x105);
        // an FD frame of 20 bytes keeps DLC 11
        t.receive(&CanFrame { id: 0x101, fd: true, brs: true, data: vec![7; 20], ..Default::default() });
        assert_eq!(t.read(RX_STATUS) >> 4 & 0x7ff, 0, "FD base frames are not enabled in filter A");
        t.write(FILTER_CONTROL, 1 | 4);
        t.receive(&CanFrame { id: 0x101, fd: true, brs: true, data: vec![7; 20], ..Default::default() });
        let w = read_frame(&mut t);
        assert_eq!((w[0] & 0xf, w[0] >> 7 & 1, w[0] >> 9 & 1, w.len()), (11, 1, 1, 9));
    }
}
