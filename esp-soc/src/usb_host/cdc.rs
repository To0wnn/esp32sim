//! A CDC-ACM console on the USB-OTG port: a host that enumerates the device once it connects,
//! raises DTR and RTS on its first CDC-ACM interface, then keeps one bulk transfer in flight. The
//! device's output joins the machine's `usb` console stream, and the page's (or a script's) serial
//! input goes out on the bulk OUT endpoint. This is how a terminal reaches a TinyUSB console, such as
//! Arduino's `ARDUINO_USB_MODE=0` builds, where USB-Serial/JTAG is silent.
//!
//! Enumeration reuses the blocking `UsbHost` and runs the machine itself (about 170 ms of emulated
//! time). It therefore runs only from `Machine::usb_cdc_service`, between run slices, never from
//! inside `Machine::run`.
use super::{setup_packet, UsbHost};
use crate::machine::Machine;
use crate::soc::{Soc, SocBus};
use esp_periph::usb_otg::{Outcome, Request};

#[derive(Debug)]
enum State {
    /// VBUS on, waiting for the device to pull D+ up
    Connect,
    /// enumerated: the data interface's bulk endpoints, and whether the transfer in flight is OUT
    Open { in_ep: u8, out_ep: u8, mps: u16, sending: bool },
    /// enumeration failed; tried again once the device disconnects
    Failed,
}

#[derive(Debug)]
pub struct CdcConsole {
    state: State,
    /// typed input not sent yet
    pub out: Vec<u8>,
    /// device output not taken into the console yet
    pub rx: Vec<u8>,
    /// what happened, for the page's log (taken by the front-end)
    pub events: Vec<String>,
}

impl Default for CdcConsole {
    fn default() -> Self { CdcConsole { state: State::Connect, out: Vec::new(), rx: Vec::new(), events: Vec::new() } }
}

/// The first CDC-ACM interface and its data interface's bulk endpoints (IN, OUT, packet size).
fn cdc_endpoints(cfg: &super::ConfigInfo) -> Option<(u8, u8, u8, u16)> {
    let comm = cfg.interfaces.iter().find(|f| f.class == 0x02 && f.subclass == 0x02)?.number;
    let data = cfg.interfaces.iter().filter(|f| f.class == 0x0a).flat_map(|f| f.endpoints.iter());
    let bulk: Vec<_> = data.filter(|e| e.attributes & 3 == 2).collect();
    let i = bulk.iter().find(|e| e.address & 0x80 != 0)?;
    let o = bulk.iter().find(|e| e.address & 0x80 == 0)?;
    Some((comm, i.address, o.address, i.mps))
}

impl CdcConsole {
    pub fn is_open(&self) -> bool { matches!(self.state, State::Open { .. }) }

    /// Move the console on by what the link finished since the last call.
    pub fn service<S: Soc>(&mut self, m: &mut Machine<S>) {
        match self.state {
            State::Connect => {
                let Some(l) = m.bus.usb_link() else { return };
                // the Attach's completion, or a transfer's from before a disconnect
                if !l.busy() { let _ = l.take_completion(); }
                let p = l.port();
                if !p.power {
                    if !l.busy() { let _ = l.submit(Request::Attach); }
                    return;
                }
                if l.busy() { return; }
                if !p.connected { return; }
                let limit = m.max_cycles;
                m.max_cycles = u64::MAX;
                let r = Self::open(m);
                m.max_cycles = limit;
                match r {
                    Ok((in_ep, out_ep, mps)) => {
                        self.events.push(format!("USB CDC console up: bulk IN {in_ep:02x}, OUT {out_ep:02x}, {mps} bytes"));
                        self.state = State::Open { in_ep, out_ep, mps, sending: false };
                    }
                    Err(e) => { self.events.push(format!("USB CDC console: {e}")); self.state = State::Failed; }
                }
            }
            State::Failed => {
                if m.bus.usb_link().is_none_or(|l| !l.port().connected) { self.state = State::Connect; }
            }
            State::Open { in_ep, out_ep, mps, ref mut sending } => {
                let Some(l) = m.bus.usb_link() else { self.state = State::Connect; return };
                if let Some(c) = l.take_completion() {
                    match c.outcome {
                        Outcome::Done(d) => { if !*sending { self.rx.extend_from_slice(&d); } }
                        Outcome::Disconnected | Outcome::NotRunning => {
                            self.events.push("USB CDC console: the device went away".into());
                            self.state = State::Connect;
                            return;
                        }
                        Outcome::Stall(_) | Outcome::Babble(_) => self.events.push(format!("USB CDC console: {:?} on a bulk transfer", c.outcome)),
                        // a cancelled IN carried nothing: it asks for one packet
                        Outcome::Cancelled => {}
                    }
                    *sending = false;
                }
                if l.busy() {
                    // An IN that waits for output gives way to typed input.
                    if self.out.is_empty() || *sending { return; }
                    l.cancel();
                    let _ = l.take_completion();
                }
                let r = if self.out.is_empty() {
                    l.submit(Request::BulkIn { ep: in_ep & 15, len: mps as usize, mps })
                } else {
                    let n = self.out.len().min(mps as usize);
                    let data: Vec<u8> = self.out.drain(..n).collect();
                    *sending = true;
                    l.submit(Request::BulkOut { ep: out_ep & 15, data, mps, zlp: false })
                };
                if let Err(e) = r { self.events.push(format!("USB CDC console: {e}")); }
            }
        }
    }

    /// Enumerate, then SET_LINE_CODING 115200 8N1 and SET_CONTROL_LINE_STATE (DTR | RTS), CDC
    /// PSTN 1.2 §6.3.10 and §6.3.12, as the CLI's `cdc` command does.
    fn open<S: Soc>(m: &mut Machine<S>) -> Result<(u8, u8, u16), String> {
        let mut h = UsbHost::new(m);
        h.enumerate(10_000_000_000).map_err(|e| e.to_string())?;
        let cfg = h.config.clone().ok_or("no configuration")?;
        let (comm, in_ep, out_ep, mps) = cdc_endpoints(&cfg).ok_or("the device has no CDC-ACM interface")?;
        let mut coding = 115_200u32.to_le_bytes().to_vec();
        coding.extend([0, 0, 8]);
        h.control(setup_packet(0x21, 0x20, 0, comm as u16, 7), coding).map_err(|e| e.to_string())?;
        h.control(setup_packet(0x21, 0x22, 3, comm as u16, 0), Vec::new()).map_err(|e| e.to_string())?;
        Ok((in_ep, out_ep, mps))
    }
}
