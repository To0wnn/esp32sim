//! The SAR ADC at DR_REG_ADC_BASE (0x20815000) in the one-shot mode IDF 6.1.1's `adc_oneshot`
//! driver uses (esp_hal_ana_conv/esp32s31 adc_ll.h): the channel goes into SARn_PATT_TAB1
//! (bits 23:20), SARn_TRIGGER_START (CTRL0/CTRL1 bit 1) converts it, SARn_DONE_INT_RAW (INT_RAW
//! bit 31 for ADC1, 30 for ADC2) says so, and SARn_DATA_STATUS holds the code. ADC1 channels 0-7
//! are GPIO 42-49, ADC2's start at GPIO 50 (soc/adc_channel.h). The voltage on a pin comes from
//! `esp_periph::AnalogInputs` (a script's `adc`, the CLI's `--adc-raw`, the page).
//! shortcut: a conversion completes at the trigger, with the C6's 12-bit curve at 12 dB (the
//! S31's own curve-fitting coefficients are not public); no continuous/DMA mode, thresholds or
//! filters, and no interrupt line (the driver polls INT_RAW).
use esp_periph::AnalogInputs;

pub const BASE: u32 = 0x2081_5000;
const CTRL0: u32 = 0x00;
const CTRL1: u32 = 0x04;
const PATT_TAB1: [u32; 2] = [0x1c, 0x2c];
const DATA_STATUS: [u32; 2] = [0x40, 0x44];
const INT_RAW: u32 = 0x68;
const INT_ST: u32 = 0x6c;
const INT_ENA: u32 = 0x64;
const INT_CLR: u32 = 0x70;
/// SARn_DONE_INT_RAW
const DONE: [u32; 2] = [1 << 31, 1 << 30];
/// The first GPIO of each unit's channels.
const FIRST_GPIO: [u8; 2] = [42, 50];

pub struct Adc {
    regs: Box<[u32; 0x100]>,
    int_raw: u32,
    pub analog: AnalogInputs,
    pub conversions: u64,
}

impl Adc {
    pub fn new(cpu_hz: u64) -> Self { Adc { regs: Box::new([0; 0x100]), int_raw: 0, analog: AnalogInputs::new(cpu_hz), conversions: 0 } }

    pub fn contains(address: u32) -> bool { (BASE..BASE + 0x400).contains(&address) }

    pub fn read(&self, off: u32) -> u32 {
        match off {
            INT_RAW => self.int_raw,
            INT_ST => self.int_raw & self.regs[(INT_ENA / 4) as usize],
            // TRIGGER_START/STOP are pulses: they read 0
            CTRL0 | CTRL1 => self.regs[(off / 4) as usize] & !3,
            _ => self.regs[((off & 0x3ff) / 4) as usize],
        }
    }

    /// A word written at `off`; `now` is the bus cycle, for a source that changes over time.
    pub fn write(&mut self, off: u32, value: u32, now: u64) {
        match off {
            INT_CLR => self.int_raw &= !value,
            INT_RAW | INT_ST => {}
            CTRL0 | CTRL1 => {
                self.regs[(off / 4) as usize] = value & !3;
                if value & 2 != 0 { self.convert(((off - CTRL0) / 4) as usize, now); }
            }
            _ => self.regs[((off & 0x3ff) / 4) as usize] = value,
        }
    }

    fn convert(&mut self, unit: usize, now: u64) {
        let channel = (self.regs[(PATT_TAB1[unit] / 4) as usize] >> 20) & 0xf;
        let pin = FIRST_GPIO[unit] + channel as u8;
        let code = self.analog.convert(pin, now, |v| esp_periph::sar_adc::voltage_code(v, 3, esp_periph::sar_adc::Calibration::C6));
        self.regs[(DATA_STATUS[unit] / 4) as usize] = code & 0x1_ffff;
        self.int_raw |= DONE[unit];
        self.conversions += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The register sequence of `adc_oneshot_read` on ADC1 channel 3 (GPIO 45), as adc_ll.h writes it.
    #[test]
    fn a_oneshot_conversion_reads_the_pin_voltage() {
        let mut a = Adc::new(320_000_000);
        a.analog.set(45, esp_periph::AnalogSource::Const(1.65));
        a.write(PATT_TAB1[0], (3 << 2) << 18, 0);
        a.write(INT_CLR, DONE[0], 0);
        a.write(CTRL0, 1 << 2 | 2, 0);   // SW trigger mode, TRIGGER_START
        assert_ne!(a.read(INT_RAW) & DONE[0], 0, "done");
        let code = a.read(DATA_STATUS[0]);
        assert!((1500..2600).contains(&code), "about half scale, got {code}");
        assert_eq!(a.read(CTRL0) & 2, 0, "START reads back 0");
        a.write(INT_CLR, DONE[0], 0);
        assert_eq!(a.read(INT_RAW) & DONE[0], 0);
        // a raw count set on a pin is what the next conversion returns
        assert!(a.analog.set_raw(45, 1234));
        a.write(CTRL0, 1 << 2 | 2, 0);
        assert_eq!(a.read(DATA_STATUS[0]), 1234);
    }
}
