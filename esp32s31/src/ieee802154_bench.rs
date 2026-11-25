//! A virtual Zigbee network on the s31-devkit bench, for the 802.15.4 radio
//! (`esp_periph::ieee802154`): a coordinator (short 0x0000) and a sensor (short 0x0001) on
//! PAN 0x1A2B, channel 15 (2425 MHz, between WiFi channels 1 and 6).
//!
//! - Every 250 ms the sensor sends a data frame to the coordinator (no acknowledgement request,
//!   so nobody waits on the chip).
//! - A beacon request (MAC command 0x07) the chip sends on channel 15 gets the coordinator's
//!   beacon 2 ms later, as an active scan expects. The beacon carries a Zigbee payload
//!   (protocol 0, stack profile 2, association permitted).
//!
//! The chip hears a frame only while its radio listens on channel 15; the SoC delivers it.
//! What the chip sends is counted, on any channel.

/// The bench network's channel, PAN and the coordinator's extended PAN id (on-air order).
pub const CHANNEL: u8 = 15;
pub const PAN: u16 = 0x1a2b;
const EXT_PAN: [u8; 8] = [0x53, 0x31, 0x50, 0x41, 0x4e, 0x00, 0x00, 0x00];
/// What the chip measures from the bench's radios.
pub const RSSI_DBM: i8 = -48;
const DATA_PERIOD_NS: u64 = 250_000_000;
const BEACON_DELAY_NS: u64 = 2_000_000;

#[derive(Default)]
pub struct BenchNet {
    next_data_ns: u64,
    data_seq: u8,
    beacon_seq: u8,
    /// a beacon answering a beacon request, due at this time
    beacon_due: Option<u64>,
    /// frames the bench put on the air, beacons among them
    pub sent: u64, pub beacons: u64,
    /// frames the chip sent, those on the bench's channel, and the last one
    pub heard: u64, pub heard_on_channel: u64, pub last_heard: Vec<u8>,
}

impl BenchNet {
    pub fn chip_tx(&mut self, ns: u64, channel: u8, frame: &[u8]) {
        self.heard += 1;
        self.last_heard = frame.to_vec();
        if channel != CHANNEL { return; }
        self.heard_on_channel += 1;
        // shortcut: a MAC command is recognised by its frame type and its last byte (the command
        // id), not by walking its addressing fields; a beacon request has no payload after it.
        if frame.len() >= 4 && frame[0] & 7 == 3 && frame[frame.len() - 1] == 0x07 { self.beacon_due = Some(ns + BEACON_DELAY_NS); }
    }

    pub fn next_frame(&mut self, ns: u64) -> Option<(u64, u8, Vec<u8>, i8)> {
        if let Some(t) = self.beacon_due.filter(|&t| t <= ns) {
            self.beacon_due = None;
            self.beacon_seq = self.beacon_seq.wrapping_add(1);
            self.sent += 1; self.beacons += 1;
            return Some((t, CHANNEL, self.beacon(), RSSI_DBM));
        }
        if self.next_data_ns > ns { return None; }
        // a chip that ran without looking for a while hears only the latest report
        let t = ns - (ns - self.next_data_ns) % DATA_PERIOD_NS;
        self.next_data_ns = t + DATA_PERIOD_NS;
        self.data_seq = self.data_seq.wrapping_add(1);
        self.sent += 1;
        let [p0, p1] = PAN.to_le_bytes();
        // FCF 0x8841: data, PAN ID compression, short destination and source addresses
        let mut f = vec![0x41, 0x88, self.data_seq, p0, p1, 0x00, 0x00, 0x01, 0x00];
        f.extend_from_slice(b"temp=21.5C");
        Some((t, CHANNEL, f, RSSI_DBM))
    }

    /// FCF 0x8000: a beacon from the coordinator's short address; superframe 0xCFFF (beacon and
    /// superframe order 15, PAN coordinator, association permitted), no GTS, no pending addresses.
    fn beacon(&self) -> Vec<u8> {
        let [p0, p1] = PAN.to_le_bytes();
        let mut f = vec![0x00, 0x80, self.beacon_seq, p0, p1, 0x00, 0x00, 0xff, 0xcf, 0x00, 0x00];
        // Zigbee beacon payload: protocol id 0, stack profile 2 / protocol version 2,
        // router and end-device capacity, the extended PAN id, TX offset 0xFFFFFF, update id 0
        f.extend_from_slice(&[0x00, 0x22, 0x84]);
        f.extend_from_slice(&EXT_PAN);
        f.extend_from_slice(&[0xff, 0xff, 0xff, 0x00]);
        f
    }

    pub fn report(&self) -> String {
        format!("802.15.4 bench (channel {CHANNEL}, PAN {PAN:#06x}): sent {} frames ({} beacons); heard {} frames from the chip, {} on channel {CHANNEL}{}",
            self.sent, self.beacons, self.heard, self.heard_on_channel,
            if self.last_heard.is_empty() { String::new() } else { format!(", last {:02x?}", self.last_heard) })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp_periph::ieee802154::parse_header;

    #[test]
    fn reports_every_250_ms_and_answers_a_beacon_request() {
        let mut n = BenchNet::default();
        let (t, ch, f, _) = n.next_frame(0).unwrap();
        assert_eq!((t, ch), (0, CHANNEL));
        let h = parse_header(&f).unwrap();
        assert_eq!((h.frame_type, h.dst_pan, h.dst_short), (1, Some(PAN), Some(0)));
        assert!(n.next_frame(DATA_PERIOD_NS - 1).is_none());
        // a chip that looked 1.1 s later hears one report, the one that started at 1.0 s
        assert_eq!(n.next_frame(1_100_000_000).unwrap().0, 1_000_000_000);
        assert!(n.next_frame(1_100_000_000).is_none());

        n.chip_tx(1_200_000_000, 11, &[0x03, 0x08, 0x01, 0xff, 0xff, 0xff, 0xff, 0x07]);
        assert!(n.next_frame(1_249_000_000).is_none(), "a request on channel 11 is not heard");
        n.chip_tx(1_200_000_000, CHANNEL, &[0x03, 0x08, 0x01, 0xff, 0xff, 0xff, 0xff, 0x07]);
        let (t, _, f, _) = n.next_frame(1_202_000_000).unwrap();
        assert_eq!(t, 1_202_000_000);
        let h = parse_header(&f).unwrap();
        assert_eq!((h.frame_type, h.src_pan), (0, Some(PAN)));
        assert_eq!((n.beacons, n.heard, n.heard_on_channel), (1, 2, 1));
    }
}
