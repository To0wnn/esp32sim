//! Boards around the ESP32-S31: the S31-DevKitC with a test bench (`DevKit`), and Espressif's ESP-Mosaico (board revision v1.2), wiring
//! from esp-mosaico-bsp (`esp_mosaico.h`) and the Wokwi board-esp-mosaico docs: a CO5300 480 x 480
//! AMOLED on GP-SPI2 in QSPI mode (CLK 42, CS 50, D0-D3 36/51/35/9, TE 43), its RESET on GPIO 44
//! (shared with the touch controller's), and the display supply enabled by GPIO 60 driven low.
//! Modelled: the panel and its reset and power gates, the CST9220 touch on I2C0 (0x5A, SDA 56,
//! SCL 3, INT 6), the AI button on GPIO 7 and the Boot key on GPIO 61, the vibration motor on GPIO 8, the PWR_SW shutdown line on GPIO 57, the BQ27220 fuel gauge (0x55), the BMI270 IMU (0x69) and two BMM150 magnetometers (0x11, 0x12) on I2C0, and the panel's TE output on GPIO 43: a 60 Hz frame with a 1 ms high blanking pulse.
//! Audio: the ES8311 codec (0x19) on I2C0 as a register file with its chip ID, serving the factory
//! firmware's `audio_dac` and `audio_adc` both (no ES7210 is addressed), full duplex on I2S0 (the factory firmware logs BCLK 37, WS 49, DOUT 52, DIN 40) with the
//! speaker amplifier enabled by GPIO 45; what I2S0 transmits is the run's audio (`--wav`).
use esp_soc::devices::co5300::Co5300;
use esp_soc::{Board, BoardModel, NoBoard};

pub fn make_board(name: &str) -> Option<Board> {
    match name {
        "none" | "bare" => Some(Box::new(NoBoard)),
        "mosaico" | "esp-mosaico" => Some(Box::new(Mosaico::new())),
        "s31-devkit" | "devkitc" => Some(Box::new(DevKit::default())),
        "s31-devkit-can" => Some(Box::new(DevKit { can: Some(esp_soc::devices::Mcp2515::new()), ..DevKit::default() })),
        _ => None,
    }
}

/// The ESP32-S31-DevKitC with a test bench on its header: the board itself has the Boot key on
/// GPIO 61 and an addressable RGB LED (WS2812) on GPIO 60, which the RMT drives. The bench is what
/// a bus-tool firmware finds when it probes: a 24C02 EEPROM at 0x50 on I2C0, as on a
/// breadboard, a jumper from UART1 TX to UART1 RX, and a W25Q16 SPI NOR flash (2 MiB, JEDEC EF 40 15)
/// on GP-SPI2/3 with its chip select on GPIO 10, and a 4 GiB FAT32 SD card in
/// SPI mode on the same bus with its chip select on GPIO 9. The S31 routes no I2C, UART
/// or SPI data pins, so these answer on whichever pads the firmware picks; the flash's CS is a pin.
pub struct DevKit {
    /// the bench's SPI NOR flash, selected while `BENCH_FLASH_CS` is low
    pub flash: esp_soc::devices::SpiNor,
    /// the bench's SD card in SPI mode, selected while `BENCH_SD_CS` is low
    pub sd: esp_soc::devices::SdSpi,
    /// the RGB LED's colour (R, G, B) and how often a frame set it
    led: [[u8; 3]; 1], led_updates: u64,
    /// RMT frames per pin and the first WS2812 pixel (R, G, B) of the last one, for the end-of-run report
    rmt_frames: std::collections::BTreeMap<u8, (u64, Option<[u8; 3]>, usize)>,
    /// the 1-Wire thermometer, the emulated time (ns) and its pulls not yet applied to the pad
    pub thermometer: esp_soc::devices::Ds18b20,
    onewire_edges: Vec<esp_soc::board::BoardEdge>,
    onewire_released: bool,
    /// the bench's radios and, on `s31-devkit-can`, the MCP2515 that takes the flash's CS 10
    pub nrf24: esp_soc::devices::Nrf24,
    pub cc1101: esp_soc::devices::Cc1101,
    pub sx1262: esp_soc::devices::Sx1262,
    pub can: Option<esp_soc::devices::Mcp2515>,
    /// the SX1262 probe's bit-banged SPI (`BENCH_SX1262_PROBE`): MOSI and SCK levels, bits in,
    /// the MISO level driven, and MISO edges and releases not yet delivered
    probe: (bool, bool, u8, u8, Option<bool>),
    probe_edges: Vec<esp_soc::board::BoardEdge>,
    probe_release: Vec<u8>,
    now_ns: u64,
    /// the virtual Zigbee network on channel 15 (`ieee802154_bench.rs`)
    pub zigbee: crate::ieee802154_bench::BenchNet,
    /// CAN frames the bench node acknowledged, and how many of them were OBD-II requests it answered
    can_frames: u64,
    obd_answers: u64,
}
impl Default for DevKit {
    fn default() -> Self {
        let readme: &[u8] = b"esp32sim bench SD card: a 4 GiB SDHC card, FAT32, on the S31-DevKitC test bench.\r\n";
        let card = esp_soc::devices::sd_spi::fat32_card(4 << 30, "ESP32SIM", &[("README.TXT", readme)]);
        DevKit {
            flash: esp_soc::devices::SpiNor::new([0xef, 0x40, 0x15], 2 << 20),
            sd: esp_soc::devices::SdSpi::new(Box::new(card)).expect("a 4 GiB card is valid"),
            led: Default::default(), led_updates: 0, rmt_frames: Default::default(),
            thermometer: esp_soc::devices::Ds18b20::new(BENCH_DS18B20_SERIAL, 376),
            onewire_edges: Vec::new(), onewire_released: false,
            nrf24: esp_soc::devices::Nrf24::new(Some(esp_soc::devices::nrf24::Air { channel: BENCH_NRF24_CHANNEL, address: [0xe7; 5], text: "esp32sim bench".into(), period_ns: 250_000_000 })),
            cc1101: esp_soc::devices::Cc1101::new(Some(esp_soc::devices::cc1101::Air { mhz: 433.92, dbm: -35.0 })),
            sx1262: esp_soc::devices::Sx1262::new(Some(esp_soc::devices::sx1262::Air { hz: 868_000_000, dbm: -60 })),
            can: None, probe: (false, false, 0, 0, None), probe_edges: Vec::new(), probe_release: Vec::new(), now_ns: 0,
            zigbee: Default::default(), can_frames: 0, obd_answers: 0,
        }
    }
}
/// The bench flash's chip select.
pub const BENCH_FLASH_CS: u8 = 10;
/// The bench SD card's chip select.
pub const BENCH_SD_CS: u8 = 9;
/// The DevKitC's addressable RGB LED.
pub const DEVKIT_RGB_PIN: u8 = 60;
/// The 24C02 on the bench: 256 bytes behind a one-byte word address, read and written sequentially.
pub const BENCH_EEPROM: u8 = 0x50;
/// The bench's 1-Wire bus.
pub const BENCH_ONEWIRE: u8 = 6;
/// The DS18B20's serial number; its ROM code is 28 5E 1D 8B 16 03 00 and the CRC.
pub const BENCH_DS18B20_SERIAL: [u8; 6] = [0x5e, 0x1d, 0x8b, 0x16, 0x03, 0x00];
/// The NTAG213's UID on the PN532 (NXP manufacturer code 04).
pub const BENCH_NTAG_UID: [u8; 7] = [0x04, 0xa2, 0x24, 0x5e, 0x31, 0x6f, 0x80];
/// The bench nRF24L01+'s CSN and CE,
/// and the channel its transmitter sends on (2476 MHz, the RF24 library's default channel).
pub const BENCH_NRF24_CSN: u8 = 38;
pub const BENCH_NRF24_CE: u8 = 15;
pub const BENCH_NRF24_CHANNEL: u8 = 76;
/// The bench CC1101's CS; a carrier sits on 433.92 MHz.
pub const BENCH_CC1101_CS: u8 = 39;
/// The bench SX1262's NSS and RESET; BUSY (5) and DIO1 (6) stay undriven and read low. A carrier
/// sits on 868 MHz.
pub const BENCH_SX1262_CS: u8 = 40;
pub const BENCH_SX1262_RESET: u8 = 4;
/// Some LoRa tools read the sync word by bit-banging SPI on GPIO before they start the driver:
/// SCK, MOSI and MISO on these pins.
pub const BENCH_SX1262_PROBE: [u8; 3] = [48, 47, 41];
impl BoardModel for DevKit {
    fn name(&self) -> &'static str { if self.can.is_some() { "s31-devkit-can" } else { "s31-devkit" } }
    fn uses_gpio_edges(&self) -> bool { false }
    fn input_levels(&self) -> Vec<(u8, bool)> { vec![(PIN_BOOT_BUTTON, true)] }
    fn gpio_changes(&mut self, changes: &[(u8, bool)]) {
        for &(pin, level) in changes {
            if pin == BENCH_FLASH_CS { if let Some(c) = &mut self.can { c.select(!level); } else { self.flash.select(!level); } }
            if pin == BENCH_SD_CS { self.sd.select(!level); }
            if pin == BENCH_ONEWIRE { self.thermometer.master(self.now_ns, !level); }
            if pin == BENCH_NRF24_CSN { self.nrf24.select(!level); }
            if pin == BENCH_NRF24_CE { self.nrf24.set_ce(level); }
            if pin == BENCH_CC1101_CS { self.cc1101.select(!level); }
            if pin == BENCH_SX1262_RESET && !level { self.sx1262.reset(); }
            self.probe_pin(pin, level);
        }
    }
    fn pull_ups(&self) -> u64 { 1 << BENCH_ONEWIRE }
    fn advance_to(&mut self, now: esp_soc::board::VirtualCycle) {
        self.now_ns = now;
        self.nrf24.advance_to(now);
        while let Some(low) = self.thermometer.take_due(now) {
            if low { self.onewire_edges.push(esp_soc::board::BoardEdge { cycle: now, pin: BENCH_ONEWIRE, level: false }); }
            else { self.onewire_released = true; }
        }
    }
    fn take_edges(&mut self) -> Vec<esp_soc::board::BoardEdge> {
        let mut edges = std::mem::take(&mut self.onewire_edges);
        edges.extend(std::mem::take(&mut self.probe_edges));
        edges
    }
    fn released_inputs(&mut self) -> Vec<u8> {
        let mut rel = std::mem::take(&mut self.probe_release);
        if std::mem::take(&mut self.onewire_released) { rel.push(BENCH_ONEWIRE); }
        rel
    }
    fn next_deadline(&self) -> Option<esp_soc::board::VirtualCycle> {
        self.thermometer.next_event()
    }
    /// Both GP-SPI hosts reach the bench's bus; only a selected flash answers, else MISO floats high.
    fn spi_transfer(&mut self, _host: u8, tx: &[u8], rx_len: usize) -> Vec<u8> {
        if self.flash.selected() { self.flash.transfer(tx, rx_len) }
        else if self.sd.selected() { self.sd.transfer(tx, rx_len) }
        else if let Some(can) = self.can.as_mut().filter(|c| c.selected()) { can.transfer(tx, rx_len) }
        else if self.nrf24.selected() { self.nrf24.transfer(tx, rx_len) }
        else if self.cc1101.selected() { self.cc1101.transfer(tx, rx_len) }
        else if self.sx1262.selected() { self.sx1262.transfer(tx, rx_len) }
        else { vec![0xff; rx_len] }
    }
    fn named_pin(&self, name: &str) -> Option<u8> { matches!(name, "boot" | "button" | "btn1").then_some(PIN_BOOT_BUTTON) }
    /// The bench's jumper from UART1 TX to UART1 RX: what a firmware sends on UART1 it reads back.
    fn uart_loopback(&self, n: usize) -> bool { n == 1 }
    // shortcut: a 24C02 page write is not limited to 8 bytes and has no write cycle time; a firmware
    // that polls for the write to finish sees it done at once.
    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn esp_periph::i2c::I2cDevice>)> {
        vec![
            (0, BENCH_EEPROM, Box::new(esp_periph::i2c::Reg8Device::new("24c02", &[]))),
            (0, esp_soc::devices::PN532_ADDRESS, Box::new(esp_soc::devices::Pn532::new(BENCH_NTAG_UID))),
            (0, esp_soc::devices::SI4713_ADDRESS, Box::new(esp_soc::devices::Si4713::default())),
            (0, esp_soc::devices::MAX17048_ADDRESS, Box::new(esp_soc::devices::max17048(3850, 76))),
        ]
    }
    /// A WS2812 frame on the RGB LED's pin: its first 24 bits are the LED's G, R, B, most
    /// significant bit first; a shorter frame leaves the LED as it was.
    fn rmt_frame(&mut self, pin: u8, bits: &[bool]) {
        let byte = |i: usize| bits[i * 8..i * 8 + 8].iter().fold(0u8, |b, &x| b << 1 | x as u8);
        let pixel = (bits.len() >= 24).then(|| [byte(1), byte(0), byte(2)]);
        let e = self.rmt_frames.entry(pin).or_default();
        e.0 += 1;
        e.2 = bits.len();
        if pixel.is_some() { e.1 = pixel; }
        if let (DEVKIT_RGB_PIN, Some(p)) = (pin, pixel) { self.led[0] = p; self.led_updates += 1; }
    }
    fn leds(&self) -> Option<(&[[u8; 3]], u64)> { Some((&self.led, self.led_updates)) }
    /// The bench's CAN node on TWAI0, at whatever bitrate the firmware picks: it ACKs every frame,
    /// answers an OBD-II request (`obd_answer`) as an engine ECU, and echoes any other frame on
    /// the next identifier (0x123 comes back as 0x124, with the same payload and format).
    fn can_frame(&mut self, n: usize, f: &CanFrame) -> Option<Vec<CanFrame>> {
        if n != 0 { return None; }
        self.can_frames += 1;
        if let Some(answer) = obd_answer(f) { self.obd_answers += 1; return Some(vec![answer]); }
        if obd_request(f) { return Some(Vec::new()); }
        let id = (f.id + 1) & if f.ext { 0x1fff_ffff } else { 0x7ff };
        Some(vec![CanFrame { id, ..f.clone() }])
    }
    fn ieee802154_tx(&mut self, ns: u64, channel: u8, frame: &[u8]) { self.zigbee.chip_tx(ns, channel, frame); }
    fn ieee802154_rx(&mut self, ns: u64) -> Option<(u64, u8, Vec<u8>, i8)> { self.zigbee.next_frame(ns) }
    fn report(&self) -> String {
        let s = self.flash.stats;
        let d = self.sd.stats;
        let (n, c, x) = (self.nrf24.stats, self.cc1101.stats, self.sx1262.stats);
        let mut flash = format!("[emu] s31-devkit: bench flash {} frames, {} programs, {} erases; SD card {} commands, {} blocks read, {} written; CAN node {} frames, {} OBD-II answers", s.frames, s.programs, s.erases, d.commands, d.blocks_read, d.blocks_written, self.can_frames, self.obd_answers);
        flash += &format!("\n[emu] s31-devkit: nRF24 {} frames, {} packets received, {} sent; CC1101 {} frames, {} strobes, {} RSSI reads; SX1262 {} commands, {} RSSI reads",
                          n.frames, n.received, n.sent, c.frames, c.strobes, c.rssi_reads, x.commands, x.rssi_reads);
        if let Some(m) = &self.can {
            let t = m.stats;
            flash += &format!("\n[emu] s31-devkit: MCP2515 {} frames, {} sends ({} looped back, {} unacknowledged), mode {}", t.frames, t.sent, t.looped_back, t.unacknowledged, m.mode());
        }
        if self.zigbee.sent + self.zigbee.heard > 0 {
            flash += &format!("\n[emu] s31-devkit: {}", self.zigbee.report());
        }
        if self.rmt_frames.is_empty() { return flash; }
        let pins: Vec<String> = self.rmt_frames.iter().map(|(p, (n, px, len))| format!("GPIO {p}: {n} frames (last {len} bits), first pixel {px:?}")).collect();
        format!("{flash}\n[emu] s31-devkit: RGB LED {:?} after {} frames; RMT {}", self.led[0], self.led_updates, pins.join("; "))
    }
}

impl DevKit {
    /// The SX1262 seen through a bit-banged probe (SPI mode 0, MSB first): the chip
    /// puts each bit on MISO after NSS falls and after each falling SCK, and samples MOSI on the
    /// rising one. MISO is released when NSS rises, so the pad is free for other modes.
    fn probe_pin(&mut self, pin: u8, level: bool) {
        let [sck, mosi, miso] = BENCH_SX1262_PROBE;
        if pin == BENCH_SX1262_CS {
            self.sx1262.select(!level);
            self.probe.2 = 0; self.probe.3 = 0;
            self.probe_release.retain(|&p| p != miso);
            if level { if self.probe.4.take().is_some() { self.probe_release.push(miso); } return; }
        } else if !self.sx1262.selected() {
            return;
        } else if pin == mosi {
            self.probe.0 = level; return;
        } else if pin == sck {
            let rising = level && !self.probe.1;
            let falling = !level && self.probe.1;
            self.probe.1 = level;
            if rising { self.probe.2 = self.probe.2 << 1 | u8::from(self.probe.0); self.probe.3 += 1; }
            if !falling { return; }
            if self.probe.3 == 8 { self.sx1262.exchange(self.probe.2); self.probe.2 = 0; self.probe.3 = 0; }
        } else {
            return;
        }
        let bit = self.sx1262.next_out() >> (7 - self.probe.3) & 1 != 0;
        if self.probe.4 != Some(bit) {
            self.probe.4 = Some(bit);
            self.probe_edges.push(esp_soc::board::BoardEdge { cycle: self.now_ns, pin: miso, level: bit });
        }
    }
}

use esp_periph::twaifd::CanFrame;

/// A classic single-frame OBD-II request on the functional ID 0x7DF or the ECU's physical ID 0x7E0.
fn obd_request(f: &CanFrame) -> bool {
    !f.ext && !f.fd && matches!(f.id, 0x7df | 0x7e0) && f.data.len() >= 2 && (1..=7).contains(&f.data[0])
}

/// The bench ECU's OBD-II (SAE J1979) answer from 0x7E8, padded with 0xAA: mode 01 PIDs 00
/// (supported: 05, 0C, 0D), 05 (coolant 90 degrees C), 0C (850 rpm), 0D (42 km/h); mode 03 (no
/// stored DTCs) and mode 04 (cleared). Other requests get no answer, as from an ECU that does not
/// support them.
// shortcut: no ISO-TP multi-frame answers (mode 09 VIN); add flow control when a firmware reads one.
fn obd_answer(f: &CanFrame) -> Option<CanFrame> {
    if !obd_request(f) { return None; }
    let pid = f.data.get(2).copied().unwrap_or(0);
    let payload: &[u8] = match (f.data[1], pid) {
        (0x01, 0x00) => &[0x41, 0x00, 0x08, 0x18, 0x00, 0x00],
        (0x01, 0x05) => &[0x41, 0x05, 90 + 40],
        (0x01, 0x0c) => &[0x41, 0x0c, 0x0d, 0x48],
        (0x01, 0x0d) => &[0x41, 0x0d, 42],
        (0x03, _) => &[0x43, 0x00],
        (0x04, _) => &[0x44],
        _ => return None,
    };
    let mut data = vec![payload.len() as u8];
    data.extend_from_slice(payload);
    data.resize(8, 0xaa);
    Some(CanFrame { id: 0x7e8, data, ..Default::default() })
}

pub const PIN_LCD_RESET: u8 = 44;
pub const PIN_LCD_POWER_N: u8 = 60;
pub const PIN_LCD_TE: u8 = 43;
/// The speaker amplifier's enable (the factory firmware's `gpio_pa_control`), active high.
pub const PIN_PA: u8 = 45;
/// The ES8311 DAC on I2C0: the factory firmware addresses it as 0x32 (8-bit), CE high.
/// Chip ID registers 0xFD/0xFE read 0x83/0x11 as on the other boards' ES8311.
pub const ES8311_ADDRESS: u8 = 0x19;
/// CST9220 INT, active low: low while a finger is down, and for 1 ms after it lifts so the
/// driver reads the release report.
pub const PIN_TOUCH_INT: u8 = 6;
/// The AI button on top (esp-mosaico-bsp `esp_mosaico.h` BSP_BUTTON_AI_GPIO; the factory
/// firmware's board manager calls it `button_power`: single click sleep/back, long press chat).
/// Active low (BSP_BUTTON_ACTIVE_LEVEL 0) with the board's pull-up.
pub const PIN_AI_BUTTON: u8 = 7;
/// The Boot key: BSP_BUTTON_BOOT_GPIO (the factory firmware's `button_boot`), active low. These
/// two are the board's only GPIO keys: the BSP, esp-brookesia's esp_mosaico_v1_2 board files and
/// esp-mosaico-claw read no other button, and there is no power key on a GPIO (power is
/// switched outside the chip; the chip can only request shutdown on `PIN_POWER_SWITCH`).
pub const PIN_BOOT_BUTTON: u8 = 61;
/// The vibration motor: BSP_MOTOR_GPIO, on high (BSP_MOTOR_ON_LEVEL 1). The BSP drives it from
/// LEDC PWM by default (CONFIG_BSP_MOTOR_ENABLE_PWM: low-speed timer 0, 10 bits, 5 kHz, channel
/// 0) or as a plain output. Modelled as on or off; an undriven pin reads off (`pull_downs`: a
/// motor driver's gate pull-down is assumed, the schematic is not public).
pub const PIN_MOTOR: u8 = 8;
/// PWR_SW: BSP_POWER_SWITCH_GPIO, open drain; `bsp_power_set_shutdown(true)` drives it low
/// (BSP_POWER_SWITCH_ASSERT_LEVEL 0) to ask the power circuit to switch the board off, and
/// releases it to high impedance otherwise. Reported, not acted on.
pub const PIN_POWER_SWITCH: u8 = 57;
/// What the web page's indicator shows while the motor runs.
const MOTOR_COLOUR: [u8; 3] = [255, 160, 0];
/// The BMI270 IMU (`imu_sensor`) on I2C0: the factory firmware's board-manager entry
/// {"imu_sensor", "custom", "bmi270", addr 0x69, "i2c_master"} (SDO high).
pub const BMI270_ADDRESS: u8 = esp_soc::devices::BMI270_ADDRESS;
/// Two BMM150 magnetometers on I2C0, directly on the bus (not behind the BMI270's AUX
/// interface): `magnetometer_1` and `magnetometer_2` list 8-bit addresses 0x22 and 0x24, which
/// the firmware logs as yaml_addr and shifts to the 7-bit i2c_addr.
pub const BMM150_ADDRESSES: [u8; 2] = [0x11, 0x12];
const TOUCH_RELEASE_NS: u64 = 1_000_000;
/// CO5300 TE: one frame every 1/60 s, high for the first 1 ms (the blanking interval).
const TE_PERIOD_NS: u64 = 16_666_667;
const TE_HIGH_NS: u64 = 1_000_000;

/// The ESP-Mosaico v1.2.
pub struct Mosaico {
    /// the CO5300; its frame is what the glass shows (480 x 480, no column offset in the BSP)
    pub panel: Co5300,
    /// the supply enable (GPIO 60, active low) and the reset line (GPIO 44, active low)
    pub powered: bool,
    pub in_reset: bool,
    /// the speaker amplifier's enable (GPIO 45)
    pub amp: bool,
    pub gpio_events: u64,
    /// transactions the panel could not take: unpowered or held in reset
    pub dropped: u64,
    /// the finger the CST9220 reports
    pub touch: std::sync::Arc<std::sync::Mutex<crate::cst9220::Touch>>,
    /// the storage NAND on GP-SPI3 (the BSP's MOSAICO_NAND)
    pub nand: crate::spi_nand::SpiNand,
    /// TE: the level now and the edges not yet delivered (`advance_to` takes emulated nanoseconds)
    te_level: bool,
    te_now: u64,
    /// INT level and, after a release, when it goes back high
    touch_int_low: bool,
    touch_release_at: Option<u64>,
    te_edges: Vec<esp_soc::board::BoardEdge>,
    /// Scripted motion for the BMI270 and the BMM150s (clock: emulated nanoseconds, stored by `advance_to`)
    pub imu: std::sync::Arc<esp_soc::devices::imu_motion::ImuMotion>,
    /// The vibration motor: running, how often it started, how long it ran (emulated ns, up to
    /// the last start when running), and its one-LED indicator for `leds()`.
    pub motor: bool,
    pub motor_starts: u64,
    motor_ns: u64,
    motor_since: u64,
    motor_led: [[u8; 3]; 1],
    motor_updates: u64,
    /// PWR_SW asserted (driven low): the firmware asked for power off.
    pub shutdown_requested: bool,
}
impl Default for Mosaico { fn default() -> Self { Self::new() } }
impl Mosaico {
    pub const WIDTH: usize = 480;
    pub const HEIGHT: usize = 480;
    pub fn new() -> Self {
        Mosaico { panel: Co5300::new(Self::WIDTH, Self::HEIGHT, 0), powered: false, in_reset: false, amp: false, gpio_events: 0, dropped: 0, touch: Default::default(), nand: crate::spi_nand::SpiNand::new(), te_level: false, te_now: 0, touch_int_low: false, touch_release_at: None, te_edges: Vec::new(), imu: Default::default(),
                  motor: false, motor_starts: 0, motor_ns: 0, motor_since: 0, motor_led: [[0; 3]], motor_updates: 0, shutdown_requested: false }
    }
    /// Emulated nanoseconds the motor has run, including the current run.
    pub fn motor_ns(&self) -> u64 { self.motor_ns + if self.motor { self.te_now.saturating_sub(self.motor_since) } else { 0 } }
    fn set_motor(&mut self, on: bool) {
        if on == self.motor { return; }
        if on { self.motor_starts += 1; self.motor_since = self.te_now; } else { self.motor_ns += self.te_now.saturating_sub(self.motor_since); }
        self.motor = on;
        self.motor_led = [if on { MOTOR_COLOUR } else { [0; 3] }];
        self.motor_updates += 1;
    }
}
impl BoardModel for Mosaico {
    fn name(&self) -> &'static str { "mosaico" }
    fn uses_gpio_edges(&self) -> bool { false }
    fn gpio_changes(&mut self, changes: &[(u8, bool)]) {
        for &(pin, level) in changes {
            self.gpio_events += 1;
            match pin {
                PIN_LCD_POWER_N => { if level && self.powered { self.panel.reset(); } self.powered = !level; }
                PIN_LCD_RESET => { if !level { self.panel.reset(); } self.in_reset = !level; }
                PIN_PA => self.amp = level,
                PIN_MOTOR => self.set_motor(level),
                PIN_POWER_SWITCH => self.shutdown_requested = !level,
                _ => {}
            }
        }
    }
    fn gpio_events(&self) -> u64 { self.gpio_events }
    fn pull_downs(&self) -> u64 { 1 << PIN_MOTOR }
    /// The motor's indicator: one "LED", lit while it runs.
    fn leds(&self) -> Option<(&[[u8; 3]], u64)> { Some((&self.motor_led, self.motor_updates)) }
    fn input_levels(&self) -> Vec<(u8, bool)> { vec![(PIN_LCD_TE, self.te_level), (PIN_TOUCH_INT, !self.touch_int_low), (PIN_AI_BUTTON, true), (PIN_BOOT_BUTTON, true)] }
    /// The S31 machine passes emulated time in nanoseconds. TE runs only while the panel is powered
    /// and out of reset; a jump over several frames yields the level at `now` (one edge pair at most).
    fn advance_to(&mut self, now: esp_soc::board::VirtualCycle) {
        self.te_now = now;
        self.imu.cycle.store(now, std::sync::atomic::Ordering::Relaxed);
        if self.touch_release_at.is_some_and(|t| now >= t) {
            self.touch_release_at = None; self.touch_int_low = false;
            self.te_edges.push(esp_soc::board::BoardEdge { cycle: now, pin: PIN_TOUCH_INT, level: true });
        }
        let level = self.powered && !self.in_reset && now % TE_PERIOD_NS < TE_HIGH_NS;
        if level != self.te_level {
            self.te_level = level;
            self.te_edges.push(esp_soc::board::BoardEdge { cycle: now, pin: PIN_LCD_TE, level });
        }
    }
    fn named_pin(&self, name: &str) -> Option<u8> {
        match name { "ai" | "power" | "btn1" | "button" => Some(PIN_AI_BUTTON), "boot" | "btn2" => Some(PIN_BOOT_BUTTON), _ => None }
    }
    fn take_edges(&mut self) -> Vec<esp_soc::board::BoardEdge> { std::mem::take(&mut self.te_edges) }
    /// The next TE edge, in the same emulated nanoseconds `advance_to` takes, so an idle machine
    /// wakes for it instead of jumping over the 1 ms blanking pulse.
    fn next_deadline(&self) -> Option<esp_soc::board::VirtualCycle> {
        if !self.powered || self.in_reset { return None; }
        let now = self.te_now;
        let phase = now % TE_PERIOD_NS;
        let te = now - phase + if phase < TE_HIGH_NS { TE_HIGH_NS } else { TE_PERIOD_NS };
        Some(self.touch_release_at.map_or(te, |t| t.min(te)))
    }
    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn esp_periph::i2c::I2cDevice>)> {
        vec![(0, crate::cst9220::ADDRESS, Box::new(crate::cst9220::Cst9220::new(self.touch.clone(), Self::WIDTH as u16, Self::HEIGHT as u16))),
             (0, crate::bq27220::ADDRESS, Box::new(crate::bq27220::Bq27220::new())),
             (0, ES8311_ADDRESS, Box::new(esp_periph::i2c::Reg8Device::new("es8311", &[(0xfd, 0x83), (0xfe, 0x11)]))),
             (0, BMI270_ADDRESS, Box::new(esp_soc::devices::Bmi270::new(self.imu.clone(), 1_000_000_000))),
             (0, BMM150_ADDRESSES[0], Box::new(esp_soc::devices::Bmm150::with_motion(Default::default(), self.imu.clone(), 1_000_000_000))),
             (0, BMM150_ADDRESSES[1], Box::new(esp_soc::devices::Bmm150::with_motion(Default::default(), self.imu.clone(), 1_000_000_000)))]
    }
    fn set_imu_motion(&mut self, mode: u32) -> bool { self.imu.mode.store(mode, std::sync::atomic::Ordering::Relaxed); true }
    fn imu_motion(&self) -> u32 { self.imu.mode.load(std::sync::atomic::Ordering::Relaxed) }
    fn touch(&mut self, x: u16, y: u16, down: bool) {
        *self.touch.lock().unwrap() = crate::cst9220::Touch { down, x, y };
        // A new report: INT falls (again, for a move while down) and, on release, rises 1 ms later.
        if self.touch_int_low { self.te_edges.push(esp_soc::board::BoardEdge { cycle: self.te_now, pin: PIN_TOUCH_INT, level: true }); }
        self.touch_int_low = true;
        self.te_edges.push(esp_soc::board::BoardEdge { cycle: self.te_now, pin: PIN_TOUCH_INT, level: false });
        self.touch_release_at = (!down).then_some(self.te_now + TOUCH_RELEASE_NS);
    }
    fn spi_transfer(&mut self, host: u8, tx: &[u8], rx_len: usize) -> Vec<u8> {
        if host == 3 { return self.nand.transaction(tx, rx_len); }
        if host == 2 {
            if self.powered && !self.in_reset { self.panel.transaction(tx); } else { self.dropped += 1; }
        }
        vec![0xff; rx_len]
    }
    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> {
        Some((Self::WIDTH as u32, Self::HEIGHT as u32, self.panel.frame.clone(), self.panel.pixels_written))
    }
    fn display_version(&self) -> u64 { self.panel.pixels_written }
    fn display_frames(&self) -> u64 { self.panel.frames }
    fn report(&self) -> String {
        let n = self.nand.stats;
        format!("[emu] mosaico: CO5300 {} RAMWR, {} pixels; power {}, reset {}; {} transactions dropped; gpio events {}; speaker amp {}\n[emu] mosaico: NAND {} page reads, {} programs, {} erases, {} refused\n[emu] mosaico: motor {}, {} starts, {} ms on; PWR_SW {}",
                self.panel.frames, self.panel.pixels_written, if self.powered { "on" } else { "off" },
                if self.in_reset { "held" } else { "released" }, self.dropped, self.gpio_events, if self.amp { "on" } else { "off" }, n.reads, n.programs, n.erases, n.refused,
                if self.motor { "on" } else { "off" }, self.motor_starts, self.motor_ns() / 1_000_000, if self.shutdown_requested { "asserted (shutdown requested)" } else { "released" })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn devkit_thermometer_answers_a_reset_with_a_presence_pulse_on_gpio6() {
        let mut b = DevKit::default();
        assert_eq!(b.pull_ups(), 1 << BENCH_ONEWIRE);
        b.advance_to(1_000_000);
        b.gpio_changes(&[(BENCH_ONEWIRE, false)]);
        b.advance_to(1_480_000);
        b.gpio_changes(&[(BENCH_ONEWIRE, true)]);
        assert!(b.take_edges().is_empty() && b.released_inputs().is_empty());
        let pull = b.next_deadline().unwrap();
        b.advance_to(pull);
        assert_eq!(b.take_edges(), [esp_soc::board::BoardEdge { cycle: pull, pin: BENCH_ONEWIRE, level: false }]);
        b.advance_to(b.next_deadline().unwrap());
        assert_eq!(b.released_inputs(), [BENCH_ONEWIRE]);
        assert_eq!(b.next_deadline(), None);
    }

    #[test]
    fn the_bench_can_node_answers_obd_and_echoes_other_frames() {
        let mut b = DevKit::default();
        let req = |id, data: &[u8]| CanFrame { id, data: data.to_vec(), ..Default::default() };
        let rpm = b.can_frame(0, &req(0x7df, &[2, 1, 0x0c, 0x55, 0x55, 0x55, 0x55, 0x55])).unwrap();
        assert_eq!(rpm, [req(0x7e8, &[4, 0x41, 0x0c, 0x0d, 0x48, 0xaa, 0xaa, 0xaa])], "(0x0d * 256 + 0x48) / 4 = 850 rpm");
        assert_eq!(b.can_frame(0, &req(0x7df, &[1, 3, 0x55])).unwrap()[0].data[..2], [2, 0x43]);
        assert_eq!(b.can_frame(0, &req(0x7df, &[2, 9, 2])), Some(vec![]), "acknowledged, not answered");
        let fd = CanFrame { id: 0x1abc_def0, ext: true, fd: true, brs: true, data: vec![1; 12], ..Default::default() };
        assert_eq!(b.can_frame(0, &fd), Some(vec![CanFrame { id: 0x1abc_def1, ..fd.clone() }]));
        assert_eq!(b.can_frame(1, &fd), None, "nothing on TWAI1's bus");
        assert!(b.report().contains("CAN node 4 frames, 2 OBD-II answers"), "{}", b.report());
    }

    #[test]
    fn panel_takes_commands_only_when_powered_and_out_of_reset() {
        let mut b = Mosaico::new();
        let ramwr = [0x32, 0, 0x2c, 0, 0x12, 0x34];
        b.spi_transfer(2, &ramwr, 0);
        assert_eq!(b.panel.pixels_written, 0);
        assert_eq!(b.dropped, 1);
        b.gpio_changes(&[(PIN_LCD_POWER_N, false), (PIN_LCD_RESET, false)]);
        b.spi_transfer(2, &ramwr, 0);
        assert_eq!(b.panel.pixels_written, 0, "held in reset");
        b.gpio_changes(&[(PIN_LCD_RESET, true)]);
        b.spi_transfer(2, &ramwr, 0);
        assert_eq!(b.panel.frame[0], 0x1234);
    }

    #[test]
    fn motor_runs_while_gpio8_is_high_and_lights_its_indicator() {
        let mut b = Mosaico::new();
        assert_eq!(b.pull_downs(), 1 << PIN_MOTOR, "an undriven motor pin reads off");
        assert_eq!(b.leds(), Some((&[[0, 0, 0]][..], 0)));
        b.advance_to(1_000_000);
        b.gpio_changes(&[(PIN_MOTOR, true)]);
        assert!(b.motor);
        assert_eq!(b.leds(), Some((&[MOTOR_COLOUR][..], 1)));
        b.gpio_changes(&[(PIN_MOTOR, true)]);
        assert_eq!(b.leds().unwrap().1, 1, "no change, no update");
        b.advance_to(201_000_000);
        assert_eq!(b.motor_ns(), 200_000_000, "running: counted up to now");
        b.gpio_changes(&[(PIN_MOTOR, false)]);
        b.advance_to(500_000_000);
        assert_eq!((b.motor, b.motor_starts, b.motor_ns()), (false, 1, 200_000_000));
        assert!(b.report().contains("motor off, 1 starts, 200 ms on; PWR_SW released"), "{}", b.report());
        b.gpio_changes(&[(PIN_POWER_SWITCH, false)]);
        assert!(b.report().contains("PWR_SW asserted"), "{}", b.report());
    }

    #[test]
    fn keys_by_name() {
        let b = Mosaico::new();
        assert_eq!(b.named_pin("ai"), Some(7));
        assert_eq!(b.named_pin("boot"), Some(61));
        assert!(b.input_levels().contains(&(PIN_AI_BUTTON, true)) && b.input_levels().contains(&(PIN_BOOT_BUTTON, true)), "released: high");
    }
}
