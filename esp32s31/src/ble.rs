//! Virtual BLE controller for the NimBLE host of ESP-IDF v6.1 (`--ble`).
//!
//! The S31's native controller initialises without the radio: its OS abstraction, HCI buffer
//! pools and msys mbuf pools are plain software. Only its link layer needs the radio, so the
//! HCI transport is cut at the host boundary instead of replacing the controller (as on the C6).
//! Commands and ACL from the host go to `esp_soc::ble::peer::Session`. Events and ACL come back
//! through a guest FreeRTOS task that exists only as hook addresses: each step calls a guest
//! function with the return address on the next hook, so no code is patched into flash.
use crate::bus::{SRAM_BASE, SRAM_SIZE};
use crate::cache::PSRAM_VADDR;
use crate::soc::SocBus;
use emu_core::{Bus, Core};
use esp_soc::ble::peer::Session;
use riscv_rv32::Cpu;
use std::collections::HashMap;

/// Hook offsets inside `hci_driver_host_callback_register`. Its entry is always intercepted, so
/// these bytes never run natively.
const TASK: u32 = 4;
const REGISTERED: u32 = 8;
const PDPASS: u32 = 1;
/// `r_ble_hci_trans_buf_alloc` type used by the native controller for events to the host.
const EVENT_BUFFER: u32 = 3;
const PACKET_MAX: usize = 260;

#[derive(Clone, Copy)]
enum Hook { Register, Registered, Task, Command, Acl }

#[derive(Default)]
enum Step {
    /// The task's first entry: it still needs a scratch buffer for ACL payloads.
    #[default]
    Start,
    Scratch,
    Poll,
    Event(Vec<u8>),
    Acl(Vec<u8>),
    Appended(u32, u32),
    Delivered,
    /// A packet whose buffer allocation failed, tried again after a tick.
    Retry(u8, Vec<u8>),
}

#[derive(Default)]
pub struct Ble {
    pub session: Session,
    pub hooks: Vec<u32>,
    kinds: HashMap<u32, Hook>,
    symbols: HashMap<&'static str, u32>,
    base: u32,
    dummy_callback: Option<u32>,
    callback: u32,
    register_return: u32,
    task_created: bool,
    scratch: u32,
    step: Step,
    /// `[ble]` lines: stderr, or kept for the browser build (`BleLog::keep`)
    pub lines: esp_soc::ble::log::BleLog,
}

impl Ble {
    pub fn enable(&mut self, symbols: &HashMap<String, u32>, register_size: u32, idf_version: &str) -> Result<(), String> {
        let get = |name: &str| symbols.get(name).copied().ok_or_else(|| format!("BLE requires ELF symbol {name}"));
        let base = get("hci_driver_host_callback_register")?;
        if !(idf_version.starts_with("v6.1.") || idf_version.starts_with("6.1.")) {
            return Err(format!("S31 BLE requires esp_app_desc.idf_ver v6.1.x, got {idf_version:?}"));
        }
        if base & 1 != 0 || register_size < REGISTERED + 2 { return Err("BLE callback register has no room for the task hooks".into()); }
        let mut next = Self { base, dummy_callback: symbols.get("ble_transport_dummy_host_recv_cb").copied(), ..Self::default() };
        next.kinds.insert(base, Hook::Register);
        next.kinds.insert(base + TASK, Hook::Task);
        next.kinds.insert(base + REGISTERED, Hook::Registered);
        next.kinds.insert(get("ble_transport_to_ll_cmd_impl")?, Hook::Command);
        next.kinds.insert(get("ble_transport_to_ll_acl_impl")?, Hook::Acl);
        for name in [
            "r_ble_hci_trans_buf_alloc",
            "r_ble_hci_trans_buf_free",
            "r_os_msys_get_pkthdr",
            "r_os_mbuf_append",
            "r_os_mbuf_free_chain",
            "malloc",
            "xTaskCreatePinnedToCore",
            "vTaskDelay",
        ] { next.symbols.insert(name, get(name)?); }
        next.hooks = next.kinds.keys().copied().collect();
        next.lines = std::mem::take(&mut self.lines);
        *self = next;
        Ok(())
    }
    /// A chip reset: the guest task and its buffers are gone.
    pub fn reset(&mut self) {
        self.session.reset();
        self.callback = 0;
        self.task_created = false;
        self.scratch = 0;
        self.step = Step::Start;
    }
    pub fn command(&mut self, command: &str, cycles: u64) -> Result<(), String> {
        if self.hooks.is_empty() { return Err("BLE requires --ble and the application ELF".into()); }
        self.session.advance_to(cycles);
        let result = self.session.command(command);
        self.log(cycles);
        result
    }
    /// The devices around the station (`--ble-nearby`), replacing the ones there were.
    pub fn set_nearby(&mut self, list: Vec<esp_soc::ble::peer::Nearby>, cycles: u64) -> Result<(), String> {
        if self.hooks.is_empty() { return Err("BLE requires --ble and the application ELF".into()); }
        self.session.advance_to(cycles);
        self.session.set_nearby(list);
        self.log(cycles);
        Ok(())
    }
    fn log(&mut self, cycles: u64) {
        for line in self.session.drain_log() { self.lines.note(format!("t={:.6}s {line}", cycles as f64 / crate::periph::CPU_HZ as f64)); }
    }
}

/// Internal SRAM or mapped PSRAM: where the guest heap hands out buffers.
fn ram(addr: u32, len: usize) -> bool {
    let Some(end) = addr.checked_add(len as u32) else { return false };
    (addr >= SRAM_BASE && end <= SRAM_BASE + SRAM_SIZE as u32) || (addr >= PSRAM_VADDR && end <= PSRAM_VADDR + (32 << 20))
}
fn packet(bus: &mut SocBus, addr: u32, len: usize) -> Result<Vec<u8>, String> {
    if len > PACKET_MAX || !ram(addr, len) { return Err("BLE invalid packet pointer or length".into()); }
    (0..len).map(|i| bus.read8_unpriced(addr + i as u32).map_err(|_| "BLE unmapped packet".into())).collect()
}
fn store(bus: &mut SocBus, addr: u32, bytes: &[u8]) -> Result<(), String> {
    bytes.iter().enumerate().try_for_each(|(i, &b)| bus.write8(addr + i as u32, b)).map_err(|_| "BLE unmapped buffer".into())
}
/// An os_mbuf chain as one H4 ACL packet (the same NimBLE layout as on the C6).
fn acl(bus: &mut SocBus, mut mbuf: u32) -> Result<Vec<u8>, String> {
    let mut bytes = vec![2];
    let mut seen = Vec::new();
    let mut total = 0;
    while mbuf != 0 {
        if !ram(mbuf, 24) || seen.contains(&mbuf) || seen.len() == 16 { return Err("BLE invalid mbuf chain".into()); }
        if seen.is_empty() {
            if bus.read8_unpriced(mbuf + 5).map_err(|_| "BLE mbuf header")? < 8 { return Err("BLE ACL requires a packet header".into()); }
            total = bus.read16_unpriced(mbuf + 16).map_err(|_| "BLE mbuf length")? as usize;
            if total > PACKET_MAX { return Err("BLE ACL exceeds controller packet size".into()); }
        }
        seen.push(mbuf);
        let data = bus.read32_unpriced(mbuf).map_err(|_| "BLE mbuf data")?;
        let len = bus.read16_unpriced(mbuf + 6).map_err(|_| "BLE mbuf length")? as usize;
        if len > total.saturating_sub(bytes.len() - 1) { return Err("BLE inconsistent mbuf lengths".into()); }
        bytes.extend(packet(bus, data, len)?);
        mbuf = bus.read32_unpriced(mbuf + 12).map_err(|_| "BLE mbuf next")?;
    }
    if bytes.len() != total + 1 || total < 4 { return Err("BLE incomplete ACL packet".into()); }
    Ok(bytes)
}
fn retire(cpu: &mut Cpu) {
    cpu.insn_count += 1;
    cpu.retired_count += 1;
    cpu.cycle_count += 1;
}
/// Call a guest function that returns to the hook at `back`.
fn call(cpu: &mut Cpu, target: u32, args: &[u32], back: u32) {
    cpu.x[10..10 + args.len()].copy_from_slice(args);
    cpu.x[1] = back;
    cpu.pc = target;
    retire(cpu);
}
/// Jump to a guest function that returns to the hooked function's caller.
fn tail(cpu: &mut Cpu, target: u32, args: &[u32]) {
    cpu.x[10..10 + args.len()].copy_from_slice(args);
    cpu.pc = target;
    retire(cpu);
}
fn task_call(cpu: &mut Cpu, bus: &mut SocBus, name: &str, args: &[u32]) {
    let back = bus.ble.base + TASK;
    call(cpu, bus.ble.symbols[name], args, back);
}
fn delay(cpu: &mut Cpu, bus: &mut SocBus) {
    bus.ble.step = Step::Poll;
    task_call(cpu, bus, "vTaskDelay", &[1]);
}
fn receive(cpu: &mut Cpu, bus: &mut SocBus, mut bytes: Vec<u8>) {
    let kind = bytes.remove(0);
    if bytes.len() > PACKET_MAX || !matches!(kind, 2 | 4) {
        bus.ble.lines.note(format!("controller packet type {kind} of {} bytes dropped", bytes.len()));
        delay(cpu, bus);
    } else if kind == 4 {
        bus.ble.step = Step::Event(bytes);
        task_call(cpu, bus, "r_ble_hci_trans_buf_alloc", &[EVENT_BUFFER]);
    } else {
        let len = bytes.len() as u32;
        bus.ble.step = Step::Acl(bytes);
        task_call(cpu, bus, "r_os_msys_get_pkthdr", &[len, 0]);
    }
}
fn retry(cpu: &mut Cpu, bus: &mut SocBus, kind: u8, bytes: Vec<u8>) {
    bus.ble.lines.note("buffer allocation failed; retrying".into());
    bus.ble.step = Step::Retry(kind, bytes);
    task_call(cpu, bus, "vTaskDelay", &[1]);
}
fn deliver(cpu: &mut Cpu, bus: &mut SocBus, kind: u32, buffer: u32, len: u32) {
    if bus.ble.callback == 0 {
        // The host deinitialised while the packet waited for its buffer.
        bus.ble.step = Step::Poll;
        task_call(cpu, bus, if kind == 4 { "r_ble_hci_trans_buf_free" } else { "r_os_mbuf_free_chain" }, &[buffer]);
        return;
    }
    bus.ble.step = Step::Delivered;
    let back = bus.ble.base + TASK;
    call(cpu, bus.ble.callback, &[kind, buffer, len], back);
}
/// One step of the guest task; `result` is what the previous step's call returned.
fn task_step(cpu: &mut Cpu, bus: &mut SocBus) {
    let result = cpu.x[10];
    match std::mem::take(&mut bus.ble.step) {
        Step::Start => {
            bus.ble.step = Step::Scratch;
            task_call(cpu, bus, "malloc", &[PACKET_MAX as u32]);
        }
        Step::Scratch if ram(result, PACKET_MAX) => {
            bus.ble.scratch = result;
            delay(cpu, bus);
        }
        Step::Scratch => {
            bus.ble.lines.note("task scratch allocation failed; retrying".into());
            bus.ble.step = Step::Start;
            task_call(cpu, bus, "vTaskDelay", &[1]);
        }
        Step::Event(bytes) if ram(result, bytes.len()) => {
            // The pool's block size leads its descriptor; the buffer follows a 4-byte pool pointer.
            let capacity = bus.read32_unpriced(result - 4).and_then(|pool| bus.read32_unpriced(pool)).unwrap_or(0).saturating_sub(4);
            if bytes.len() > capacity as usize {
                bus.ble.lines.note(format!("event of {} bytes exceeds the {capacity}-byte buffer; dropped", bytes.len()));
                bus.ble.step = Step::Poll;
                task_call(cpu, bus, "r_ble_hci_trans_buf_free", &[result]);
            } else if let Err(error) = store(bus, result, &bytes) {
                bus.ble.lines.note(error);
                bus.ble.step = Step::Poll;
                task_call(cpu, bus, "r_ble_hci_trans_buf_free", &[result]);
            } else { deliver(cpu, bus, 4, result, bytes.len() as u32); }
        }
        Step::Acl(bytes) if ram(result, 24) && bus.ble.scratch != 0 => {
            let (scratch, len) = (bus.ble.scratch, bytes.len() as u32);
            match store(bus, scratch, &bytes) {
                Ok(()) => {
                    bus.ble.step = Step::Appended(result, len);
                    task_call(cpu, bus, "r_os_mbuf_append", &[result, scratch, len]);
                }
                Err(error) => {
                    bus.ble.lines.note(error);
                    bus.ble.step = Step::Poll;
                    task_call(cpu, bus, "r_os_mbuf_free_chain", &[result]);
                }
            }
        }
        Step::Event(bytes) => retry(cpu, bus, 4, bytes),
        Step::Acl(bytes) => retry(cpu, bus, 2, bytes),
        Step::Retry(kind, mut bytes) => {
            bytes.insert(0, kind);
            receive(cpu, bus, bytes);
        }
        Step::Appended(mbuf, len) if result == 0 => deliver(cpu, bus, 2, mbuf, len),
        Step::Appended(mbuf, _) => {
            bus.ble.lines.note(format!("ACL append failed: {result}"));
            bus.ble.step = Step::Poll;
            task_call(cpu, bus, "r_os_mbuf_free_chain", &[mbuf]);
        }
        Step::Delivered => {
            if result != 0 { bus.ble.lines.note(format!("host receive callback failed: {result}")); }
            // Let the host finish processing before the next asynchronous event.
            delay(cpu, bus);
        }
        Step::Poll => match bus.ble.session.pop_packet() {
            Some(bytes) if bus.ble.callback != 0 && bus.ble.scratch != 0 => receive(cpu, bus, bytes),
            Some(_) => {
                bus.ble.lines.note("controller packet dropped: no host callback".into());
                delay(cpu, bus);
            }
            None => delay(cpu, bus),
        },
    }
}

pub fn intercept(cpu: &mut Cpu, bus: &mut SocBus) -> bool {
    let Some(&hook) = bus.ble.kinds.get(&cpu.pc) else { return false };
    bus.ble.session.advance_to(bus.cycles);
    let arg = cpu.x[10];
    match hook {
        Hook::Register => {
            if Some(arg) == bus.ble.dummy_callback {
                // ble_transport_ll_deinit: the host is gone; a later init starts with HCI Reset.
                bus.ble.callback = 0;
                bus.ble.session.reset();
            } else { bus.ble.callback = arg; }
            if bus.ble.task_created || bus.ble.callback == 0 {
                cpu.return_from_stub(bus, 0);
            } else {
                bus.ble.register_return = cpu.x[1];
                let (task, back) = (bus.ble.base + TASK, bus.ble.base + REGISTERED);
                call(cpu, bus.ble.symbols["xTaskCreatePinnedToCore"], &[task, 0, 4096, 0, 23, 0, 0], back);
            }
        }
        Hook::Registered => {
            let created = arg == PDPASS;
            if created { bus.ble.task_created = true; } else { bus.ble.lines.note(format!("controller task creation failed: {arg:#x}")); }
            cpu.x[1] = bus.ble.register_return;
            cpu.return_from_stub(bus, if created { 0 } else { 0x101 });
        }
        Hook::Task => task_step(cpu, bus),
        Hook::Command => {
            let bytes = packet(bus, arg, 3).and_then(|header| packet(bus, arg, 3 + header[2] as usize));
            send(bus, bytes.map(|mut p| { p.insert(0, 1); p }));
            // The host's own failure path frees a command buffer with ble_transport_free; it returns 0.
            tail(cpu, bus.ble.symbols["r_ble_hci_trans_buf_free"], &[arg]);
        }
        Hook::Acl => {
            let bytes = acl(bus, arg);
            send(bus, bytes);
            tail(cpu, bus.ble.symbols["r_os_mbuf_free_chain"], &[arg]);
        }
    }
    true
}
fn send(bus: &mut SocBus, bytes: Result<Vec<u8>, String>) {
    match bytes {
        Ok(bytes) => {
            if bus.debug.has("ble") { eprintln!("[ble] tx {bytes:02x?}"); }
            bus.ble.session.send(&bytes);
            bus.ble.log(bus.cycles);
        }
        Err(error) => bus.ble.lines.note(error),
    }
}
