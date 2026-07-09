#pragma once
#include <Arduino.h>

const char HTML_PAGE[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Stepper Motor Controller Dashboard</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;500;600;700&display=swap');
*, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
:root {
  --bg: #080b11;
  --surface: rgba(255, 255, 255, 0.03);
  --surface-hover: rgba(255, 255, 255, 0.06);
  --border: rgba(255, 255, 255, 0.08);
  --cyan: #00f0ff;
  --green: #39ff14;
  --red: #ff073a;
  --amber: #ffaa00;
  --text: #e2e8f0;
  --sub: #718096;
}
body {
  background: var(--bg);
  color: var(--text);
  font-family: 'Outfit', sans-serif;
  min-height: 100vh;
  display: flex;
  flex-direction: column;
  align-items: center;
  padding: 2rem 1rem;
  gap: 2rem;
}
header { text-align: center; }
h1 { font-size: clamp(1.5rem, 4vw, 2.2rem); font-weight: 700; letter-spacing: 0.03em; }
header p { color: var(--sub); font-size: 0.9rem; margin-top: 0.4rem; }
.dot {
  display: inline-block;
  width: 9px;
  height: 9px;
  border-radius: 50%;
  background: var(--red);
  margin-right: 6px;
  box-shadow: 0 0 8px var(--red);
  transition: all 0.3s ease;
}
.dot.active {
  background: var(--green);
  box-shadow: 0 0 8px var(--green);
  animation: blink 1.5s infinite;
}
@keyframes blink { 0%, 100% { opacity: 1; } 50% { opacity: 0.4; } }

.container {
  display: flex;
  flex-direction: row;
  flex-wrap: wrap;
  justify-content: center;
  gap: 2rem;
  max-width: 1100px;
  width: 100%;
}

.column {
  flex: 1;
  min-width: 320px;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.panel {
  background: var(--surface);
  border: 1px solid var(--border);
  border-radius: 1.2rem;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1.2rem;
  backdrop-filter: blur(10px);
}

.panel h2 {
  font-size: 1.1rem;
  font-weight: 600;
  color: var(--sub);
  text-transform: uppercase;
  letter-spacing: 0.08em;
  border-bottom: 1px solid var(--border);
  padding-bottom: 0.5rem;
  margin-bottom: 0.2rem;
}

.status-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1rem;
}

.status-card {
  background: rgba(0, 0, 0, 0.2);
  border: 1px solid var(--border);
  border-radius: 0.8rem;
  padding: 1rem;
  text-align: center;
}
.status-card label {
  font-size: 0.75rem;
  color: var(--sub);
  text-transform: uppercase;
  display: block;
  margin-bottom: 0.3rem;
}
.status-card .val {
  font-size: 1.4rem;
  font-weight: 700;
}
.status-card.pos .val { color: var(--cyan); text-shadow: 0 0 10px rgba(0,240,255,0.2); }
.status-card.speed .val { color: var(--green); text-shadow: 0 0 10px rgba(57,255,20,0.2); }

/* Rotating Visualizer */
.visualizer-container {
  display: flex;
  justify-content: center;
  align-items: center;
  padding: 1rem;
}
.motor-svg {
  width: 160px;
  height: 160px;
  filter: drop-shadow(0 0 15px rgba(0, 240, 255, 0.15));
}
.rotor-group {
  transition: transform 0.1s linear;
  transform-origin: 80px 80px;
}

/* Form Controls */
.control-group {
  display: flex;
  flex-direction: column;
  gap: 0.4rem;
}
.control-row {
  display: flex;
  gap: 0.8rem;
  align-items: center;
}
label.field-lbl {
  font-size: 0.85rem;
  font-weight: 500;
  color: var(--text);
}
input[type="number"], input[type="range"], select {
  background: rgba(0, 0, 0, 0.3);
  border: 1px solid var(--border);
  border-radius: 0.5rem;
  padding: 0.6rem 0.8rem;
  color: var(--text);
  font-family: inherit;
  font-size: 0.95rem;
  outline: none;
  transition: border-color 0.2s;
  width: 100%;
}
input[type="number"]:focus, select:focus {
  border-color: var(--cyan);
}
input[type="range"] {
  padding: 0.2rem 0;
  cursor: pointer;
}

/* Buttons */
.btn {
  background: var(--surface-hover);
  border: 1px solid var(--border);
  color: var(--text);
  font-family: inherit;
  font-weight: 600;
  border-radius: 0.6rem;
  padding: 0.7rem 1.2rem;
  cursor: pointer;
  transition: all 0.2s ease;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 0.5rem;
  text-transform: uppercase;
  font-size: 0.85rem;
}
.btn:hover {
  background: var(--text);
  color: var(--bg);
}
.btn-primary {
  background: var(--cyan);
  color: var(--bg);
  border: none;
}
.btn-primary:hover {
  background: #00d6e6;
  box-shadow: 0 0 12px rgba(0, 240, 255, 0.4);
}
.btn-danger {
  background: var(--red);
  color: #fff;
  border: none;
  font-size: 1rem;
  padding: 0.9rem;
}
.btn-danger:hover {
  background: #e00030;
  box-shadow: 0 0 15px rgba(255, 7, 58, 0.5);
}

/* Toggle Switch */
.switch {
  position: relative;
  display: inline-block;
  width: 52px;
  height: 28px;
}
.switch input { opacity: 0; width: 0; height: 0; }
.slider {
  position: absolute;
  cursor: pointer;
  top: 0; left: 0; right: 0; bottom: 0;
  background-color: rgba(255,255,255,0.1);
  transition: .3s;
  border-radius: 28px;
  border: 1px solid var(--border);
}
.slider:before {
  position: absolute;
  content: "";
  height: 20px; width: 20px;
  left: 3px; bottom: 3px;
  background-color: var(--sub);
  transition: .3s;
  border-radius: 50%;
}
input:checked + .slider {
  background-color: rgba(57,255,20,0.15);
  border-color: var(--green);
}
input:checked + .slider:before {
  transform: translateX(24px);
  background-color: var(--green);
  box-shadow: 0 0 8px var(--green);
}

footer {
  font-size: 0.75rem;
  color: var(--sub);
  margin-top: 1rem;
}
</style>
</head>
<body>

<header>
  <h1><span class="dot" id="stateDot"></span>Stepper Motor Test Bench</h1>
  <p>Real-time stepper drive validation &bull; ESP32 WiFi Access Point</p>
</header>

<div class="container">
  <!-- Left Column: Motor Visualization and Status -->
  <div class="column">
    <div class="panel">
      <h2>Visualizer</h2>
      <div class="visualizer-container">
        <!-- SVG of Stepper Motor Face and Shaft -->
        <svg class="motor-svg" viewBox="0 0 160 160">
          <!-- Motor Body Outer Frame (NEMA 17 Profile) -->
          <rect x="15" y="15" width="130" height="130" rx="15" fill="#1b2230" stroke="var(--border)" stroke-width="2"/>
          <!-- Four corner mounting screw holes -->
          <circle cx="28" cy="28" r="5" fill="#0d1117" stroke="var(--border)" stroke-width="1"/>
          <circle cx="132" cy="28" r="5" fill="#0d1117" stroke="var(--border)" stroke-width="1"/>
          <circle cx="28" cy="132" r="5" fill="#0d1117" stroke="var(--border)" stroke-width="1"/>
          <circle cx="132" cy="132" r="5" fill="#0d1117" stroke="var(--border)" stroke-width="1"/>
          <!-- Circular center collar -->
          <circle cx="80" cy="80" r="42" fill="#2d3748" stroke="var(--border)" stroke-width="2"/>
          <!-- D-Shaft / Rotor group that rotates -->
          <g class="rotor-group" id="rotor">
            <!-- Inner bearing/shaft -->
            <circle cx="80" cy="80" r="16" fill="#4a5568" stroke="#718096" stroke-width="2"/>
            <!-- D-Flat indicator line (or spoke indicator) -->
            <line x1="80" y1="80" x2="80" y2="52" stroke="var(--cyan)" stroke-width="4" stroke-linecap:="round"/>
            <circle cx="80" cy="54" r="4.5" fill="var(--cyan)" />
            <!-- Other minor spokes -->
            <line x1="80" y1="80" x2="108" y2="80" stroke="rgba(255,255,255,0.2)" stroke-width="2"/>
            <line x1="80" y1="80" x2="52" y2="80" stroke="rgba(255,255,255,0.2)" stroke-width="2"/>
            <line x1="80" y1="80" x2="80" y2="108" stroke="rgba(255,255,255,0.2)" stroke-width="2"/>
          </g>
        </svg>
      </div>
    </div>

    <div class="panel">
      <h2>Real-time Feedback</h2>
      <div class="status-grid">
        <div class="status-card pos">
          <label>Position</label>
          <div class="val" id="curPos">0</div>
          <span style="font-size:0.75rem; color:var(--sub)">steps</span>
        </div>
        <div class="status-card speed">
          <label>Speed</label>
          <div class="val" id="curSpeed">0.0</div>
          <span style="font-size:0.75rem; color:var(--sub)">steps/s</span>
        </div>
        <div class="status-card">
          <label>Driver Output</label>
          <div class="val" id="driverState" style="color:var(--red); font-size:1.1rem; padding-top:0.3rem;">DISABLED</div>
        </div>
        <div class="status-card">
          <label>Active Mode</label>
          <div class="val" id="activeMode" style="color:var(--amber); font-size:1.1rem; padding-top:0.3rem;">POSITION</div>
        </div>
      </div>
      <button class="btn btn-danger" onclick="triggerStop()">EMERGENCY STOP</button>
    </div>
  </div>

  <!-- Right Column: Control Panel -->
  <div class="column">
    <div class="panel">
      <h2>Power & Drive</h2>
      <div style="display:flex; justify-content:space-between; align-items:center;">
        <div>
          <span style="font-weight:600; display:block;">Enable Stepper Driver</span>
          <span style="font-size:0.8rem; color:var(--sub)">Energizes stepper coils</span>
        </div>
        <label class="switch">
          <input type="checkbox" id="driverEnable" onchange="toggleEnable(this.checked)">
          <span class="slider"></span>
        </label>
      </div>
    </div>

    <div class="panel">
      <h2>Motion Profile</h2>
      
      <!-- Mode Selection -->
      <div class="control-group">
        <label class="field-lbl">Operation Mode</label>
        <select id="modeSelect" onchange="updateMode(this.value)">
          <option value="0">Absolute Position Control</option>
          <option value="1">Continuous Speed Control</option>
          <option value="2">Position Sweep Test</option>
        </select>
      </div>

      <!-- Max Speed -->
      <div class="control-group">
        <div style="display:flex; justify-content:space-between;">
          <label class="field-lbl">Target Speed</label>
          <span style="font-size:0.85rem; color:var(--green);" id="speedVal">200 steps/s</span>
        </div>
        <input type="range" id="speedSlider" min="-1500" max="1500" value="200" step="10" oninput="updateSpeedSlider(this.value)" onchange="sendSpeed(this.value)">
      </div>

      <!-- Acceleration -->
      <div class="control-group">
        <div style="display:flex; justify-content:space-between;">
          <label class="field-lbl">Acceleration</label>
          <span style="font-size:0.85rem; color:var(--cyan);" id="accelVal">500 steps/s²</span>
        </div>
        <input type="range" id="accelSlider" min="50" max="3000" value="500" step="50" oninput="updateAccelSlider(this.value)" onchange="sendAccel(this.value)">
      </div>

      <!-- S-Curve Profile Panel -->
      <div class="control-group" style="gap:0.8rem;">
        <label class="field-lbl" style="color:var(--cyan); letter-spacing:0.05em;">&#9670; S-Curve Profile</label>
        <div style="display:grid; grid-template-columns: 1fr 1fr; gap:0.8rem;">
          <div>
            <span style="font-size:0.75rem; color:var(--sub); display:block; margin-bottom:0.3rem;">Starting Pulse (steps/s)</span>
            <input type="number" id="startingPulseInput" value="50" min="1" max="500" onchange="sendStartingPulse(this.value)">
          </div>
          <div>
            <span style="font-size:0.75rem; color:var(--sub); display:block; margin-bottom:0.3rem;">Min Pulse (steps/s)</span>
            <input type="number" id="minPulseInput" value="20" min="1" max="500" onchange="sendMinPulse(this.value)">
          </div>
        </div>
        <div style="background:rgba(0,240,255,0.04); border:1px solid rgba(0,240,255,0.12); border-radius:0.6rem; padding:0.65rem 0.8rem; font-size:0.78rem; color:var(--sub);">
          <strong style="color:var(--cyan);">S-Curve Motion:</strong> Sinusoidal acceleration/deceleration. Motor ramps from <em>Starting Pulse</em> to max speed and decelerates to <em>Min Pulse</em> at target using a &frac12;(1&minus;cos(&pi;t)) curve.
        </div>
      </div>

      <!-- Mode-Specific Input Panels -->
      <!-- Position Mode Settings -->
      <div id="positionPanel" class="control-group" style="gap: 0.8rem;">
        <label class="field-lbl">Target Position (steps)</label>
        <div class="control-row">
          <input type="number" id="targetPosInput" value="0" style="flex:1;">
          <button class="btn btn-primary" onclick="sendPosition()">MOVE</button>
        </div>
      </div>

      <!-- Sweep Mode Settings -->
      <div id="sweepPanel" class="control-group" style="display:none; gap: 0.8rem;">
        <label class="field-lbl">Sweep Settings</label>
        <div style="display:grid; grid-template-columns: 1fr 1fr; gap:0.8rem;">
          <div>
            <span style="font-size:0.75rem; color:var(--sub)">Min Position</span>
            <input type="number" id="sweepMin" value="-200">
          </div>
          <div>
            <span style="font-size:0.75rem; color:var(--sub)">Max Position</span>
            <input type="number" id="sweepMax" value="200">
          </div>
        </div>
        <button class="btn btn-primary" onclick="sendSweepLimits()">UPDATE SWEEP</button>
      </div>

    </div>
  </div>
</div>

<footer>Connected via softAP &bull; 10 Hz refresh rate</footer>

<script>
const rotor = document.getElementById('rotor');
const curPos = document.getElementById('curPos');
const curSpeed = document.getElementById('curSpeed');
const driverState = document.getElementById('driverState');
const activeMode = document.getElementById('activeMode');
const driverEnable = document.getElementById('driverEnable');
const stateDot = document.getElementById('stateDot');
const modeSelect = document.getElementById('modeSelect');

const positionPanel = document.getElementById('positionPanel');
const sweepPanel = document.getElementById('sweepPanel');

function updateSpeedSlider(v) {
  document.getElementById('speedVal').textContent = `${v} steps/s`;
}

function updateAccelSlider(v) {
  document.getElementById('accelVal').textContent = `${v} steps/s\u00b2`;
}

function updateModePanels(mode) {
  positionPanel.style.display = (mode === '0') ? 'flex' : 'none';
  sweepPanel.style.display = (mode === '2') ? 'flex' : 'none';
}

async function apiCall(endpoint) {
  try {
    const res = await fetch(endpoint);
    return res.ok;
  } catch (e) {
    console.error("API call failed:", e);
    return false;
  }
}

function toggleEnable(checked) {
  apiCall(`/control?en=${checked ? 1 : 0}`);
}

function updateMode(modeVal) {
  updateModePanels(modeVal);
  apiCall(`/control?mode=${modeVal}`);
}

function sendSpeed(speedVal) {
  apiCall(`/control?speed=${speedVal}`);
}

function sendAccel(accelVal) {
  apiCall(`/control?accel=${accelVal}`);
}

function sendMinPulse(val) {
  apiCall(`/control?minPulse=${val}`);
}

function sendStartingPulse(val) {
  apiCall(`/control?startingPulse=${val}`);
}

function sendPosition() {
  const p = document.getElementById('targetPosInput').value;
  apiCall(`/control?pos=${p}`);
}

function sendSweepLimits() {
  const minVal = document.getElementById('sweepMin').value;
  const maxVal = document.getElementById('sweepMax').value;
  apiCall(`/control?sweepMin=${minVal}&sweepMax=${maxVal}`);
}

function triggerStop() {
  apiCall(`/control?stop=1`);
}

let _lastMinPulse = null;
let _lastStartPulse = null;
let _lastAccel = null;

async function poll() {
  try {
    const res = await fetch('/data');
    if (!res.ok) return;
    const d = await res.json();

    // Update Status Cards
    curPos.textContent = d.pos;
    curSpeed.textContent = d.speed.toFixed(1);

    if (d.en) {
      driverState.textContent = "ENABLED";
      driverState.style.color = "var(--green)";
      stateDot.classList.add('active');
      driverEnable.checked = true;
    } else {
      driverState.textContent = "DISABLED";
      driverState.style.color = "var(--red)";
      stateDot.classList.remove('active');
      driverEnable.checked = false;
    }

    const modes = ["POSITION", "SPEED", "SWEEP"];
    activeMode.textContent = modes[d.mode] || "UNKNOWN";
    modeSelect.value = d.mode;
    updateModePanels(d.mode.toString());

    // Sync S-Curve inputs (only when user is not focused on them)
    const mpEl = document.getElementById('minPulseInput');
    const spEl = document.getElementById('startingPulseInput');
    const accelEl = document.getElementById('accelSlider');

    if (document.activeElement !== mpEl && d.minPulse !== _lastMinPulse) {
      mpEl.value = d.minPulse.toFixed(0);
      _lastMinPulse = d.minPulse;
    }
    if (document.activeElement !== spEl && d.startingPulse !== _lastStartPulse) {
      spEl.value = d.startingPulse.toFixed(0);
      _lastStartPulse = d.startingPulse;
    }
    if (document.activeElement !== accelEl && d.accel !== _lastAccel) {
      accelEl.value = d.accel.toFixed(0);
      document.getElementById('accelVal').textContent = `${d.accel.toFixed(0)} steps/s\u00b2`;
      _lastAccel = d.accel;
    }

    // Rotate SVG Shaft (1.8 deg per step for 200 steps/rev)
    const angle = (d.pos % 200) * 1.8;
    rotor.style.transform = `rotate(${angle}deg)`;

  } catch(e) {}
}

setInterval(poll, 100);
poll();
</script>
</body>
</html>)rawhtml";
