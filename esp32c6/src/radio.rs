//! The C6's IEEE 802.15.4 MAC at 0x600A3000: the shared model in `esp_periph::ieee802154`,
//! which the S31 mounts too. `Ieee802154::new` counts the C6's 160 MHz CPU cycles.
pub use esp_periph::ieee802154::*;
