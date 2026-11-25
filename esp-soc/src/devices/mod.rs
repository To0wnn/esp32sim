//! Device models that are not tied to a chip: the things a board wires to the pins, fed with
//! what the SoC model already decoded (GPIO levels, SPI bytes, RMT bit streams). A board owns
//! them and adds what is module-specific — pin numbers, the visible window of a glass, where
//! each LED of a chain sits physically.
pub mod bmi270;
pub mod bmm150;
pub mod cc1101;
pub mod co5300;
pub mod dcs_panel;
pub mod ds18b20;
pub mod ft5x06;
pub mod imu_motion;
pub mod max17048;
pub mod mcp2515;
pub mod nrf24;
pub mod pn532;
pub mod sd_spi;
pub mod si4713;
pub mod spi_bitbang;
pub mod spi_nor;
pub mod sx1262;
pub mod ws2812;

pub use bmi270::{Bmi270, BMI270_ADDRESS};
pub use bmm150::{Bmm150, Bmm150Trim};
pub use cc1101::Cc1101;
pub use dcs_panel::DcsPanel;
pub use ds18b20::Ds18b20;
pub use ft5x06::{Ft5x06, Ft5x06Points, FT5X06_ADDRESS};
pub use max17048::{max17048, MAX17048_ADDRESS};
pub use mcp2515::Mcp2515;
pub use nrf24::Nrf24;
pub use pn532::{Pn532, PN532_ADDRESS};
pub use sd_spi::SdSpi;
pub use si4713::{Si4713, SI4713_ADDRESS};
pub use spi_bitbang::SpiBitBang;
pub use spi_nor::SpiNor;
pub use sx1262::Sx1262;
pub use ws2812::Ws2812Chain;

/// One GP-SPI transaction through a device that exchanges a byte at a time: full duplex when MISO
/// is as long as MOSI (Arduino's `SPI.transfer`), else the MOSI bytes then `rx_len` bytes read
/// with MOSI idle high.
pub fn spi_exchange(tx: &[u8], rx_len: usize, mut exchange: impl FnMut(u8) -> u8) -> Vec<u8> {
    if rx_len == tx.len() { return tx.iter().map(|&b| exchange(b)).collect(); }
    for &b in tx { exchange(b); }
    (0..rx_len).map(|_| exchange(0xff)).collect()
}
