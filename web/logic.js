// The run page's logic analyzer: the GPIO output edges the emulator reports, drawn per pin over a
// window of emulated time, and saved as a VCD file for PulseView or GTKWave. Capture starts when
// the panel opens: the page sends {"t":"logic","on":true}, and the emulator then streams
//   {"t":"logic","hz":<cpu Hz>,"dropped":<n>,"e":[cycle,pin,level, ...]}
// with every display push. Edges are emulated CPU cycles; the page keeps the newest KEEP of them.
(function () {
  'use strict';
  const KEEP = 200000;
  const ROW = 22, LEFT = 64;
  let edges = [], hz = 0, dropped = 0, last = 0, capturing = false, windowNs = 10e6;
  const $ = id => document.getElementById(id);

  function build() {
    const card = document.createElement('div');
    card.className = 'card'; card.id = 'logicpanel';
    card.innerHTML = `<details id="logicdetails"><summary class="row" style="cursor:pointer;list-style:revert"><strong>Logic analyzer and analog input</strong><span class="st" id="logicstat">GPIO outputs over time; opening the panel starts the capture</span></summary>
      <div class="row" style="margin-top:8px">
        <label class="st">window <select id="logicwin"><option value="1e6">1 ms</option><option value="10e6" selected>10 ms</option><option value="100e6">100 ms</option><option value="1e9">1 s</option><option value="10e9">10 s</option></select></label>
        <label class="st">pins <input type="text" id="logicpins" placeholder="all seen, or e.g. 4,5,15" style="width:140px"></label>
        <button class="act" id="logicclear">clear</button><button class="act" id="logicvcd">download VCD</button></div>
      <div class="row" style="margin-top:8px"><span class="st">analog input</span>
        <label class="st">GPIO <input type="number" id="adcpin" value="42" min="0" max="63" style="width:56px"></label>
        <input type="range" id="adcv" min="0" max="3.3" step="0.01" value="1.65" style="flex:1;min-width:120px">
        <span class="st" id="adcvtext">1.65 V</span></div>
      <canvas id="logiccanvas" width="760" height="60" style="margin-top:8px;width:100%;background:#0b0d10;border-radius:8px"></canvas>
    </details>`;
    const right = document.querySelector('section.right');
    (right || document.body).appendChild(card);
    $('logicdetails').addEventListener('toggle', () => setCapture($('logicdetails').open));
    $('logicwin').onchange = () => { windowNs = Number($('logicwin').value); draw(); };
    $('logicpins').oninput = draw;
    $('logicclear').onclick = () => { edges = []; dropped = 0; draw(); };
    $('logicvcd').onclick = downloadVcd;
    // the voltage the chip's ADC reads on that pin (S31: ADC1 is GPIO 42-49, ADC2 from 50)
    const adc = () => { const v = Number($('adcv').value); $('adcvtext').textContent = v.toFixed(2) + ' V'; if (typeof send === 'function') send({ t: 'adc', pin: Number($('adcpin').value), v }); };
    $('adcv').oninput = adc; $('adcpin').onchange = adc;
  }

  function setCapture(on) {
    capturing = on;
    if (typeof send === 'function') send({ t: 'logic', on });
  }

  const ns = c => hz ? c * 1e9 / hz : 0;

  function pins() {
    const want = $('logicpins').value.split(/[\s,]+/).filter(Boolean).map(Number).filter(n => Number.isInteger(n));
    const seen = [...new Set(edges.map(e => e[1]))].sort((a, b) => a - b);
    return want.length ? want : seen;
  }

  function draw() {
    const cv = $('logiccanvas'); if (!cv) return;
    const rows = pins();
    cv.height = Math.max(1, rows.length) * ROW + 20;
    const g = cv.getContext('2d'), w = cv.width;
    g.clearRect(0, 0, w, cv.height);
    g.font = '11px ui-monospace,Menlo,monospace';
    const end = ns(last), start = end - windowNs, x = t => LEFT + (t - start) / windowNs * (w - LEFT - 4);
    g.fillStyle = '#8b949e';
    g.fillText(`${(windowNs / 1e6).toLocaleString()} ms ending at ${(end / 1e6).toFixed(3)} ms`, LEFT, cv.height - 6);
    if (!rows.length) { g.fillText('no GPIO output has changed yet', LEFT, 14); return; }
    rows.forEach((pin, i) => {
      const top = i * ROW + 4, hi = top + 2, lo = top + ROW - 6;
      g.fillStyle = '#c9d1d9'; g.fillText('GPIO ' + pin, 4, top + 13);
      const mine = edges.filter(e => e[1] === pin);
      // the level when the window opens: the last edge before it, else unknown (drawn mid-height)
      let level = null;
      for (const e of mine) { if (ns(e[0]) <= start) level = e[2]; else break; }
      g.strokeStyle = '#3fb950'; g.lineWidth = 1.5; g.beginPath();
      let px = LEFT, py = level === null ? (hi + lo) / 2 : level ? hi : lo;
      g.moveTo(px, py);
      for (const e of mine) {
        const t = ns(e[0]); if (t <= start) continue;
        const ex = x(t), ey = e[2] ? hi : lo;
        g.lineTo(ex, py); g.lineTo(ex, ey); px = ex; py = ey;
      }
      g.lineTo(w - 4, py); g.stroke();
    });
  }

  function downloadVcd() {
    const rows = [...new Set(edges.map(e => e[1]))].sort((a, b) => a - b);
    const id = pin => 'g' + pin;
    let out = '$timescale 1ns $end\n$scope module esp32sim $end\n';
    for (const p of rows) out += `$var wire 1 ${id(p)} gpio${p} $end\n`;
    out += '$upscope $end\n$enddefinitions $end\n';
    let prev = -1;
    for (const e of edges) {
      const t = Math.round(ns(e[0]));
      if (t !== prev) { out += '#' + t + '\n'; prev = t; }
      out += (e[2] ? '1' : '0') + id(e[1]) + '\n';
    }
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([out], { type: 'text/plain' }));
    a.download = 'esp32sim-gpio.vcd'; a.click(); URL.revokeObjectURL(a.href);
  }

  window.Logic = {
    msg(m) {
      if (!capturing) return;
      hz = m.hz || hz; dropped += m.dropped || 0;
      const e = m.e || [];
      for (let i = 0; i + 2 < e.length; i += 3) edges.push([e[i], e[i + 1], !!e[i + 2]]);
      if (edges.length > KEEP) { dropped += edges.length - KEEP; edges = edges.slice(-KEEP); }
      if (edges.length) last = Math.max(last, edges[edges.length - 1][0]);
      $('logicstat').textContent = `${edges.length.toLocaleString()} edges on ${new Set(edges.map(e => e[1])).size} pins` + (dropped ? `, ${dropped.toLocaleString()} dropped` : '');
      draw();
    },
  };
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', build); else build();
})();
