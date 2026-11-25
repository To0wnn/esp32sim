//! A Maxim MAX17048 battery fuel gauge on I2C (address 0x36, datasheet 19-6171 Rev 7): 16-bit
//! big-endian registers at even addresses behind a register pointer, so a byte-wide register
//! file reads them as the gauge does. VCELL is 78.125 uV per bit, SOC 1/256 % per bit.
// shortcut: a full, idle cell (no ModelGauge dynamics, no alerts); VCELL and SOC stay as set.
use esp_periph::i2c::Reg8Device;

pub const MAX17048_ADDRESS: u8 = 0x36;

/// The gauge with a cell at `millivolts` and `percent` charge.
pub fn max17048(millivolts: u32, percent: u8) -> Reg8Device {
    let vcell = (millivolts * 64 / 5) as u16;
    let mut regs = vec![(0x04, percent), (0x05, 0)];
    for (reg, value) in [(0x02, vcell), (0x08, 0x0012), (0x0a, 0x8030), (0x0c, 0x971c), (0x14, 0x00ff), (0x18, 0x9600), (0x1a, 0x0100), (0xfe, 0xffff)] {
        // VCELL, VERSION, HIBRT, CONFIG, VALRT, VRESET/ID, STATUS, CMD: power-up values (table 2)
        regs.extend([(reg, (value >> 8) as u8), (reg + 1, value as u8)]);
    }
    Reg8Device::new("max17048", &regs)
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp_periph::i2c::I2cDevice;
    fn word(d: &mut Reg8Device, reg: u8) -> u16 {
        d.start(false); d.write(reg); d.start(true);
        u16::from(d.read()) << 8 | u16::from(d.read())
    }
    #[test]
    fn version_vcell_and_soc_as_a_driver_decodes_them() {
        let mut d = max17048(3850, 76);
        assert_eq!(word(&mut d, 0x08) & 0xfff0, 0x0010, "max17048IsVersion");
        assert_eq!(u32::from(word(&mut d, 0x02)) * 5 / 64, 3850, "max17048VcellToMv");
        assert_eq!((word(&mut d, 0x04) + 128) >> 8, 76, "max17048SocToPercent");
    }
}
