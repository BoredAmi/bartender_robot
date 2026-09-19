"""The teach pendant's browser UI, as one self-contained page.

Inlined as a string rather than shipped as a file on purpose: it removes the
whole class of "works from source, 404s from the install" path-resolution bugs,
and there is nothing here worth a build step. No CDN either -- a tool that
drives a robot should not stop working because the network is down.
"""

PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Teach Pendant</title>
<style>
  :root {
    --bg: #14161a; --panel: #1d2026; --line: #2c313a; --fg: #e6e8ec;
    --dim: #8b94a3; --accent: #4da3ff; --ok: #3fb950; --warn: #d29922;
    --bad: #f85149; --btn: #272c35; --btn-hi: #333a46;
  }
  @media (prefers-color-scheme: light) {
    :root:not([data-theme="dark"]) {
      --bg: #f4f5f7; --panel: #fff; --line: #dfe2e8; --fg: #1b1f27;
      --dim: #5d6676; --btn: #eceef2; --btn-hi: #dfe3ea;
    }
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; background: var(--bg); color: var(--fg);
    font: 14px/1.45 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
  }
  header {
    display: flex; align-items: center; gap: 14px; flex-wrap: wrap;
    padding: 10px 16px; border-bottom: 1px solid var(--line);
    background: var(--panel); position: sticky; top: 0; z-index: 5;
  }
  header h1 { font-size: 15px; margin: 0; font-weight: 600; letter-spacing: .3px; }
  .dot { width: 9px; height: 9px; border-radius: 50%; background: var(--bad); }
  .dot.live { background: var(--ok); }
  .dot.busy { background: var(--warn); }
  .path { color: var(--dim); font-size: 12px; margin-left: auto;
          overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
  main {
    display: grid; gap: 14px; padding: 14px 16px;
    grid-template-columns: repeat(auto-fit, minmax(310px, 1fr));
    align-items: start; max-width: 1500px;
  }
  section {
    background: var(--panel); border: 1px solid var(--line);
    border-radius: 8px; padding: 12px 14px;
  }
  h2 {
    font-size: 11px; text-transform: uppercase; letter-spacing: .09em;
    color: var(--dim); margin: 0 0 10px; font-weight: 600;
  }
  table { width: 100%; border-collapse: collapse; }
  td { padding: 3px 0; white-space: nowrap; }
  td.name { color: var(--dim); }
  td.val { text-align: right; font-variant-numeric: tabular-nums; }
  td.deg { text-align: right; color: var(--dim); width: 74px;
           font-variant-numeric: tabular-nums; }
  button {
    background: var(--btn); color: var(--fg); border: 1px solid var(--line);
    border-radius: 5px; padding: 5px 9px; cursor: pointer;
    font: inherit; font-size: 13px;
  }
  button:hover:not(:disabled) { background: var(--btn-hi); }
  button:active:not(:disabled) { transform: translateY(1px); }
  button:disabled { opacity: .38; cursor: not-allowed; }
  button.go { background: var(--accent); border-color: var(--accent); color: #08131f; }
  button.danger:hover:not(:disabled) {
    background: var(--bad); border-color: var(--bad); color: #fff;
  }
  .jog { display: grid; grid-template-columns: 52px 1fr 1fr; gap: 5px; align-items: center; }
  .jog .lbl { color: var(--dim); font-size: 12px; }
  .steps { display: flex; gap: 5px; flex-wrap: wrap; margin-bottom: 10px; }
  .steps button.sel { background: var(--accent); border-color: var(--accent); color: #08131f; }
  .sub { color: var(--dim); font-size: 11px; margin: 12px 0 6px; }
  .row { display: flex; gap: 6px; align-items: center; }
  .row + .row { margin-top: 6px; }
  input[type=text], input[type=number] {
    background: var(--bg); color: var(--fg); border: 1px solid var(--line);
    border-radius: 5px; padding: 5px 8px; font: inherit; font-size: 13px;
    min-width: 0; flex: 1;
  }
  .pt { border-top: 1px solid var(--line); padding: 7px 0; }
  .pt:first-of-type { border-top: 0; }
  .pt .top { display: flex; gap: 6px; align-items: center; }
  .pt .nm { font-weight: 600; flex: 1; overflow: hidden;
            text-overflow: ellipsis; white-space: nowrap; }
  .pt .note { color: var(--dim); font-size: 12px; margin-top: 2px; }
  .empty { color: var(--dim); font-size: 12px; padding: 6px 0; }
  pre#log {
    margin: 0; background: var(--bg); border: 1px solid var(--line);
    border-radius: 6px; padding: 9px 11px; height: 190px; overflow-y: auto;
    font-size: 12px; white-space: pre-wrap; word-break: break-word;
  }
  .warnbar {
    background: var(--warn); color: #1b1200; padding: 6px 16px;
    font-size: 12.5px; font-weight: 600;
  }
  .hint { color: var(--dim); font-size: 11px; margin-top: 9px; }
  @media (max-width: 640px) { main { padding: 12px; } }
</style>
</head>
<body>
<header>
  <span class="dot" id="dot"></span>
  <h1>Teach Pendant</h1>
  <span id="status" style="color:var(--dim);font-size:12px"></span>
  <span class="path" id="file"></span>
</header>
<div class="warnbar" id="safetybar" hidden>
  Collision checking is OFF - Cartesian jogs are not checked against anything.
</div>

<main>
  <section>
    <h2>Arm</h2>
    <div id="arms"></div>
    <div class="hint">
      Everything except Go acts on the selected arm, and jog axes are in
      <span id="armframe">that arm's</span> base frame - the two arms do not
      share an origin. Go always uses the arm its point was taught on.
    </div>
  </section>

  <section>
    <h2>State <span class="path" id="armlabel"></span></h2>
    <table id="joints"></table>
    <div class="sub"><span id="flangename">tool0</span> in
      <span id="framename">base_link</span></div>
    <table id="pose"></table>
    <div class="sub">tool tip (<span id="toolname">tool0</span>)</div>
    <table id="tip"></table>
    <div class="sub">gripper</div>
    <table id="grip"></table>
  </section>

  <section>
    <h2>Tool centre point</h2>
    <div id="tools"></div>
    <div class="hint">
      Rotation jogs turn about the selected tip and hold it still. Pick a
      bottle's spout after grasping it and <code>jog ry</code> tilts the bottle
      around the spout instead of swinging the spout through an arc.
      Translation jogs are the same whatever is selected.
    </div>
  </section>

  <section>
    <h2>Jog</h2>
    <div class="steps" id="steps"></div>
    <div class="jog" id="jogjoints"></div>
    <div class="sub">base frame (mm)</div>
    <div class="jog" id="jogbase"></div>
    <div class="sub">tool frame (mm) - tz is the approach axis</div>
    <div class="jog" id="jogtool"></div>
    <div class="sub">rotate about base axis (deg)</div>
    <div class="jog" id="jogrot"></div>
    <div class="hint">
      Jogs larger than the limits are refused, not clamped.
      Arrow keys jog the base Z (up/down) and Y (left/right) axes.
    </div>
  </section>

  <section>
    <h2>Gripper &amp; safety</h2>
    <div class="row">
      <button id="gopen" style="flex:1">Open</button>
      <button id="gclose" style="flex:1">Close</button>
    </div>
    <div class="row">
      <input type="number" id="gpos" value="0.5" step="0.05" min="0" max="0.8">
      <button id="gset">Close to</button>
    </div>
    <div class="sub">cartesian jog collision checking</div>
    <div class="row">
      <button id="safety" style="flex:1"></button>
    </div>
    <div class="hint">
      Off is for teaching a point that really is in contact. It stays off until
      you turn it back on.
    </div>
  </section>

  <section>
    <h2>Save current pose</h2>
    <div class="row"><input type="text" id="pname" placeholder="point name"></div>
    <div class="row"><input type="text" id="pnote" placeholder="note (optional)"></div>
    <div class="row">
      <button class="go" id="save" style="flex:1">Save</button>
      <button id="resave" style="flex:1">Overwrite</button>
    </div>
    <div class="hint">Written to disk immediately, not on exit.</div>
  </section>

  <section>
    <h2>Points</h2>
    <div id="points"></div>
  </section>

  <section>
    <h2>Log</h2>
    <pre id="log"></pre>
    <div class="row" style="margin-top:8px">
      <input type="text" id="cmd" placeholder="type any pendant command, e.g. jog tz 15">
      <button id="send">Run</button>
    </div>
  </section>
</main>

<script>
const $ = s => document.querySelector(s);
const MM = [1, 5, 10, 25, 50];
const DEG = [1, 5, 15, 30];
let stepMM = 10, stepDeg = 5, busy = false;

function esc(s) {
  return String(s).replace(/[&<>"']/g, c => (
    {'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
}

async function run(cmd) {
  if (busy) return;
  busy = true; paint();
  try {
    const r = await fetch('api/command', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({cmd})
    });
    const j = await r.json();
    if (j.output) appendLog(j.output);
  } catch (e) {
    appendLog('  lost contact with the pendant: ' + e);
  } finally {
    busy = false; poll();
  }
}

function appendLog(text) {
  const el = $('#log');
  el.textContent += (el.textContent ? '\n' : '') + text.replace(/\n+$/, '');
  el.scrollTop = el.scrollHeight;
}

// -- build the jog grids ----------------------------------------------------
function jogRow(host, label, axis, unit) {
  const lbl = document.createElement('span');
  lbl.className = 'lbl'; lbl.textContent = label;
  const minus = document.createElement('button');
  const plus = document.createElement('button');
  minus.textContent = '-'; plus.textContent = '+';
  minus.dataset.axis = plus.dataset.axis = axis;
  minus.dataset.unit = plus.dataset.unit = unit;
  minus.dataset.sign = '-1'; plus.dataset.sign = '1';
  for (const b of [minus, plus]) {
    b.className = 'jogbtn';
    b.onclick = () => {
      const s = (unit === 'deg' ? stepDeg : stepMM) * Number(b.dataset.sign);
      run(`jog ${axis} ${s}`);
    };
  }
  host.append(lbl, minus, plus);
}

const JOINTS = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'];
JOINTS.forEach(j => jogRow($('#jogjoints'), j, j, 'deg'));
['x', 'y', 'z'].forEach(a => jogRow($('#jogbase'), a, a, 'mm'));
['tx', 'ty', 'tz'].forEach(a => jogRow($('#jogtool'), a, a, 'mm'));
['rx', 'ry', 'rz'].forEach(a => jogRow($('#jogrot'), a, a, 'deg'));

function paintSteps() {
  const host = $('#steps');
  host.textContent = '';
  MM.forEach(v => {
    const b = document.createElement('button');
    b.textContent = v + 'mm';
    if (v === stepMM) b.className = 'sel';
    b.onclick = () => { stepMM = v; paintSteps(); };
    host.append(b);
  });
  DEG.forEach(v => {
    const b = document.createElement('button');
    b.textContent = v + '°';
    if (v === stepDeg) b.className = 'sel';
    b.onclick = () => { stepDeg = v; paintSteps(); };
    host.append(b);
  });
}
paintSteps();

// -- wiring -----------------------------------------------------------------
$('#gopen').onclick = () => run('open');
$('#gclose').onclick = () => run('close');
$('#gset').onclick = () => run('close ' + Number($('#gpos').value));
$('#send').onclick = () => {
  const v = $('#cmd').value.trim();
  if (v) { appendLog('teach> ' + v); run(v); $('#cmd').value = ''; }
};
$('#cmd').onkeydown = e => { if (e.key === 'Enter') $('#send').click(); };

function saveWith(verb) {
  const n = $('#pname').value.trim();
  if (!n) { appendLog('  give the point a name first'); return; }
  run(`${verb} ${JSON.stringify(n)} ${$('#pnote').value.trim()}`.trim());
  $('#pname').value = ''; $('#pnote').value = '';
}
$('#save').onclick = () => saveWith('save');
$('#resave').onclick = () => saveWith('resave');

document.addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT' || busy) return;
  const map = {ArrowUp: ['z', 1], ArrowDown: ['z', -1],
               ArrowLeft: ['y', 1], ArrowRight: ['y', -1]};
  if (map[e.key]) {
    e.preventDefault();
    run(`jog ${map[e.key][0]} ${stepMM * map[e.key][1]}`);
  }
});

// -- rendering --------------------------------------------------------------
function paint(s) {
  const dis = busy;
  document.querySelectorAll('button').forEach(b => {
    if (!b.classList.contains('sel') && b.parentElement.id !== 'steps') {
      b.disabled = dis;
    }
  });
  $('#dot').className = 'dot' + (dis ? ' busy' : (s && s.connected ? ' live' : ''));
  if (!s) return;

  $('#status').textContent = dis ? 'moving...'
    : (s.connected ? 'ready' : 'waiting for /joint_states');
  $('#file').textContent = s.file;
  $('#safetybar').hidden = s.safety;
  $('#safety').textContent = s.safety ? 'ON - click to disable' : 'OFF - click to enable';
  $('#safety').onclick = () => run('safety ' + (s.safety ? 'off' : 'on'));

  $('#joints').innerHTML = s.joints.map(j =>
    `<tr><td class="name">${esc(j.short)}</td>` +
    `<td class="val">${j.rad.toFixed(4)}</td>` +
    `<td class="deg">${j.deg.toFixed(2)}°</td></tr>`).join('')
    || '<tr><td class="empty">no /joint_states yet</td></tr>';

  $('#pose').innerHTML = s.pose
    ? ['x', 'y', 'z'].map((a, i) =>
        `<tr><td class="name">${a}</td><td class="val">` +
        `${s.pose.xyz[i].toFixed(4)}</td></tr>`).join('') +
      `<tr><td class="name">quat</td><td class="val">` +
      `${s.pose.quat_xyzw.map(v => v.toFixed(3)).join(' ')}</td></tr>`
    : '<tr><td class="empty">no /compute_fk - is move_group up?</td></tr>';

  $('#armlabel').textContent = s.arm_label || '';
  $('#flangename').textContent = s.arm === 'b' ? 'b_tool0' : 'tool0';
  $('#framename').textContent = s.frame || '';
  $('#armframe').textContent = s.frame || "that arm's";

  $('#arms').innerHTML = '';
  (s.arms || []).forEach(a => {
    const d = document.createElement('div');
    d.className = 'pt';
    d.innerHTML = `<div class="top"><span class="nm">${esc(a.label)}</span></div>`;
    const pick = document.createElement('button');
    pick.textContent = a.key === s.arm ? 'driving' : 'Select';
    pick.disabled = dis || a.key === s.arm;
    if (a.key === s.arm) pick.className = 'go';
    pick.onclick = () => run('arm ' + JSON.stringify(a.key));
    d.querySelector('.top').append(pick);
    $('#arms').append(d);
  });

  $('#toolname').textContent = s.tool;
  $('#tip').innerHTML = s.tip
    ? ['x', 'y', 'z'].map((a, i) =>
        `<tr><td class="name">${a}</td><td class="val">` +
        `${s.tip[i].toFixed(4)}</td></tr>`).join('')
    : '<tr><td class="empty">needs /compute_fk</td></tr>';

  $('#tools').innerHTML = '';
  s.tools.forEach(t => {
    const d = document.createElement('div');
    d.className = 'pt';
    d.innerHTML = `<div class="top"><span class="nm">${esc(t.name)}</span>` +
                  `<span style="color:var(--dim);font-size:12px">` +
                  `${(t.reach * 1000).toFixed(0)}mm</span></div>` +
                  (t.note ? `<div class="note">${esc(t.note)}</div>` : '');
    const pick = document.createElement('button');
    pick.textContent = t.name === s.tool ? 'selected' : 'Select';
    pick.disabled = dis || t.name === s.tool;
    if (t.name === s.tool) pick.className = 'go';
    pick.onclick = () => run('tool ' + JSON.stringify(t.name));
    d.querySelector('.top').prepend(pick);
    $('#tools').append(d);
  });

  $('#grip').innerHTML = s.gripper === null
    ? '<tr><td class="empty">not published</td></tr>'
    : `<tr><td class="name">knuckle</td><td class="val">` +
      `${s.gripper.toFixed(4)} rad</td></tr>`;

  $('#points').innerHTML = s.points.length ? '' : '<div class="empty">none yet</div>';
  s.points.forEach(p => {
    const d = document.createElement('div');
    d.className = 'pt';
    // The arm is on every row, not only the ones that differ from the
    // selection: Go uses the point's own arm whatever is selected, so "which
    // arm will this move" has to be answerable without looking anywhere else.
    const tag = p.arm && p.arm !== '?'
      ? ` <span class="note">arm ${esc(p.arm.toUpperCase())}</span>` : '';
    d.innerHTML = `<div class="top"><span class="nm">${esc(p.name)}</span>${tag}</div>` +
                  (p.note ? `<div class="note">${esc(p.note)}</div>` : '');
    const go = document.createElement('button');
    go.textContent = 'Go'; go.className = 'go'; go.disabled = dis;
    go.onclick = () => run('goto ' + JSON.stringify(p.name));
    const del = document.createElement('button');
    del.textContent = 'Delete'; del.className = 'danger'; del.disabled = dis;
    del.onclick = () => {
      if (confirm(`Delete point "${p.name}"?`)) run('rm ' + JSON.stringify(p.name));
    };
    d.querySelector('.top').append(go, del);
    $('#points').append(d);
  });

}

async function poll() {
  try {
    const s = await (await fetch('api/state')).json();
    paint(s);
  } catch (e) {
    $('#dot').className = 'dot';
    $('#status').textContent = 'pendant not responding';
  }
}
poll();
setInterval(() => { if (!busy) poll(); }, 500);
</script>
</body>
</html>
"""
