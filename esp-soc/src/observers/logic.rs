//! The page's logic analyzer: the GPIO edges the bus reports (`Wants::GPIO`, as `--vcd` takes
//! them), queued for `Machine::web_push` to stream as `{"t":"logic"}` messages. The queue is
//! bounded, so a page that stops reading cannot grow the emulator's memory: past `KEEP` edges the
//! oldest go, and the next message says how many.
use crate::observe::{Observer, Wants};
use crate::soc::Soc;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

/// Edges not sent yet (cycle, pin, level) and how many were dropped since the last message.
#[derive(Default)]
pub struct LogicQueue { pub edges: VecDeque<(u64, u8, bool)>, pub dropped: u64 }
pub type LogicEdges = Arc<Mutex<LogicQueue>>;

pub const KEEP: usize = 1 << 16;

pub struct LogicTap(pub LogicEdges);
impl LogicTap {
    pub fn push(&self, cycle: u64, pin: u8, level: bool) {
        let Ok(mut q) = self.0.lock() else { return };
        if q.edges.len() >= KEEP { q.edges.pop_front(); q.dropped += 1; }
        q.edges.push_back((cycle, pin, level));
    }
}

impl<S: Soc> Observer<S> for LogicTap {
    fn name(&self) -> &'static str { "logic" }
    fn wants(&self) -> Wants { Wants::GPIO }
    fn on_gpio(&mut self, cycle: u64, pin: u8, level: bool) { self.push(cycle, pin, level); }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_queue_keeps_the_newest_edges_and_counts_the_rest() {
        let q: LogicEdges = Default::default();
        let tap = LogicTap(q.clone());
        for i in 0..KEEP as u64 + 3 { tap.push(i, 5, i % 2 == 0); }
        let q = q.lock().unwrap();
        assert_eq!((q.edges.len(), q.dropped), (KEEP, 3));
        assert_eq!(q.edges.front().unwrap().0, 3, "the oldest went first");
    }
}
