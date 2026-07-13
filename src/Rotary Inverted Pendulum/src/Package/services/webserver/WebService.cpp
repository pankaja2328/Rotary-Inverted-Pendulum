#include "WebService.h"

static const char* HTML_TEMPLATE PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Rotary Inverted Pendulum - Pro Console</title>
  <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;600;800&display=swap" rel="stylesheet">
  <style>
    :root {
      --bg: #070913;
      --panel: rgba(255, 255, 255, 0.02);
      --panel-hover: rgba(255, 255, 255, 0.04);
      --border: rgba(255, 255, 255, 0.07);
      --text: #f3f4f6;
      --text-muted: #9ca3af;
      --primary: #3b82f6;
      --primary-glow: rgba(59, 130, 246, 0.3);
      --accent: #10b981;
      --accent-glow: rgba(16, 185, 129, 0.25);
      --danger: #ef4444;
      --danger-glow: rgba(239, 68, 68, 0.25);
    }
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      font-family: 'Outfit', sans-serif;
    }
    body {
      background-color: var(--bg);
      color: var(--text);
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 2rem 1rem;
      background-image: radial-gradient(circle at top right, rgba(59, 130, 246, 0.08), transparent 45%),
                        radial-gradient(circle at bottom left, rgba(16, 185, 129, 0.04), transparent 45%);
    }
    .container {
      width: 100%;
      max-width: 1000px;
    }
    header {
      text-align: center;
      margin-bottom: 2rem;
    }
    h1 {
      font-size: 2.2rem;
      font-weight: 800;
      background: linear-gradient(135deg, #fff 40%, var(--primary) 100%);
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
      margin-bottom: 0.3rem;
    }
    .subtitle {
      color: var(--text-muted);
      font-weight: 300;
      font-size: 1rem;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(310px, 1fr));
      gap: 1.25rem;
      margin-bottom: 1.5rem;
    }
    .card {
      background: var(--panel);
      backdrop-filter: blur(12px);
      -webkit-backdrop-filter: blur(12px);
      border: 1px solid var(--border);
      border-radius: 20px;
      padding: 1.5rem;
      transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
    }
    .card:hover {
      border-color: rgba(255, 255, 255, 0.12);
      transform: translateY(-2px);
    }
    .card-title {
      font-size: 0.95rem;
      font-weight: 600;
      color: var(--text-muted);
      margin-bottom: 1.25rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      border-bottom: 1px solid var(--border);
      padding-bottom: 0.5rem;
    }
    .dial-container {
      margin: 10px auto;
      position: relative;
      width: 130px;
      height: 130px;
    }
    .dial {
      width: 100%;
      height: 100%;
      transition: transform 0.1s linear;
    }
    .status-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-top: 1rem;
    }
    .status-item {
      display: flex;
      flex-direction: column;
    }
    .status-label {
      font-size: 0.75rem;
      color: var(--text-muted);
      text-transform: uppercase;
      letter-spacing: 0.08em;
      margin-bottom: 0.25rem;
    }
    .status-value {
      font-size: 1.6rem;
      font-weight: 600;
      color: #fff;
    }
    .tag {
      display: inline-block;
      padding: 0.25rem 0.75rem;
      border-radius: 9999px;
      font-size: 0.75rem;
      font-weight: 600;
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }
    .tag-waiting { background: #581c87; color: #f3e8ff; }
    .tag-ready { background: #1e3a8a; color: #dbeafe; }
    .tag-active { background: var(--accent-glow); color: #34d399; border: 1px solid rgba(16, 185, 129, 0.2); }
    .tag-stopped { background: #7f1d1d; color: #fee2e2; }

    .control-row {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 0.75rem;
      margin-bottom: 1rem;
    }
    .btn {
      padding: 0.75rem;
      border: none;
      border-radius: 10px;
      font-weight: 600;
      font-size: 0.9rem;
      cursor: pointer;
      transition: all 0.2s;
      display: flex;
      justify-content: center;
      align-items: center;
      gap: 0.5rem;
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }
    .btn-primary {
      background: var(--primary);
      color: #fff;
      box-shadow: 0 4px 10px var(--primary-glow);
    }
    .btn-primary:hover {
      background: #2563eb;
      box-shadow: 0 6px 15px var(--primary-glow);
    }
    .btn-danger {
      background: var(--danger);
      color: #fff;
      box-shadow: 0 4px 10px var(--danger-glow);
    }
    .btn-danger:hover {
      background: #dc2626;
      box-shadow: 0 6px 15px var(--danger-glow);
    }
    .btn-secondary {
      background: rgba(255,255,255,0.06);
      color: #fff;
      border: 1px solid var(--border);
    }
    .btn-secondary:hover {
      background: rgba(255,255,255,0.12);
    }
    .btn-full {
      grid-column: span 2;
    }
    .tuning-group {
      background: rgba(0, 0, 0, 0.25);
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 0.75rem;
      margin-bottom: 0.75rem;
    }
    .tuning-title {
      font-size: 0.8rem;
      font-weight: 600;
      color: var(--primary);
      margin-bottom: 0.5rem;
      text-transform: uppercase;
    }
    .input-row {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 0.5rem;
    }
    .input-item {
      display: flex;
      flex-direction: column;
    }
    .input-item label {
      font-size: 0.65rem;
      color: var(--text-muted);
      margin-bottom: 0.2rem;
    }
    .input-item input {
      background: rgba(0,0,0,0.4);
      border: 1px solid var(--border);
      color: #fff;
      padding: 0.4rem;
      border-radius: 6px;
      font-size: 0.85rem;
      text-align: center;
    }
    .input-item input:focus {
      outline: none;
      border-color: var(--primary);
    }
    .polarity-card {
      display: flex;
      flex-direction: column;
      gap: 0.75rem;
    }
    .polarity-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      background: rgba(255, 255, 255, 0.01);
      padding: 0.5rem 0.75rem;
      border-radius: 8px;
      border: 1px solid var(--border);
    }
    .polarity-label {
      font-size: 0.85rem;
    }
  </style>
</head>
<body>
  <div class="container">
    <header>
      <h1>Rotary Inverted Pendulum</h1>
      <p class="subtitle">State Machine Control Center</p>
    </header>

    <div class="grid">
      <!-- Live Monitoring -->
      <div class="card">
        <h2 class="card-title">System Status</h2>
        <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 1.25rem;">
          <span class="status-label">Active State</span>
          <span id="state-tag" class="tag tag-waiting">Unknown</span>
        </div>
        <div class="dial-container">
          <svg id="dial-svg" class="dial" viewBox="0 0 100 100">
            <circle cx="50" cy="50" r="46" fill="#0b0f19" stroke="#3b82f6" stroke-width="3"/>
            <line x1="50" y1="50" x2="50" y2="15" stroke="#ef4444" stroke-width="1.5" stroke-dasharray="2" />
            <polygon points="48,50 52,50 50,16" fill="#3b82f6" id="needle" />
          </svg>
        </div>
        <div class="status-row">
          <div class="status-item">
            <span class="status-label">Pendulum Angle</span>
            <span class="status-value" id="pend-angle">0.0°</span>
          </div>
          <div class="status-item" style="text-align: right;">
            <span class="status-label">Velocity</span>
            <span class="status-value" id="pend-vel" style="color: #60a5fa;">0°/s</span>
          </div>
        </div>
        <div class="status-row" style="border-top: 1px solid var(--border); padding-top: 0.75rem; margin-top: 0.75rem;">
          <div class="status-item">
            <span class="status-label">Arm Position</span>
            <span class="status-value" id="arm-steps" style="font-size: 1.3rem;">0 stp</span>
          </div>
          <div class="status-item" style="text-align: right;">
            <span class="status-label">Motor Rate</span>
            <span class="status-value" id="motor-rate" style="font-size: 1.3rem; color: #10b981;">0 stp/s</span>
          </div>
        </div>
      </div>

      <!-- Action Panel -->
      <div class="card">
        <h2 class="card-title">System Actions</h2>
        <div class="control-row">
          <button class="btn btn-primary" onclick="sendCmd('S')">Swing Up</button>
          <button class="btn btn-secondary" onclick="sendCmd('U')">Catch Up</button>
          <button class="btn btn-secondary" onclick="sendCmd('D')">Down PID</button>
          <button class="btn btn-secondary" onclick="sendCmd('H')">Set Center</button>
          <button class="btn btn-secondary" onclick="sendCmd('Z')">Set Down Zero</button>
          <button class="btn btn-secondary" onclick="sendCmd('R')">Reset Ctrl</button>
          <button class="btn btn-danger btn-full" onclick="sendCmd('X')">Emergency Stop (X)</button>
        </div>
        <div style="margin-top: 1rem; border-top: 1px solid var(--border); padding-top: 1rem;">
          <span class="status-label" style="display: block; margin-bottom: 0.5rem;">Encoder Calibration</span>
          <div class="control-row">
            <button class="btn btn-secondary" onclick="sendCmd('C')">Start Cal</button>
            <button class="btn btn-secondary" onclick="sendCmd('N')">Next Point</button>
          </div>
        </div>
      </div>

      <!-- PID / Polarity Tuning -->
      <div class="card">
        <h2 class="card-title">Tuning &amp; Polarity</h2>
        <div class="tuning-group">
          <div class="tuning-title">Upright PID (Balancing)</div>
          <div class="input-row">
            <div class="input-item"><label>Kp</label><input type="number" id="up-kp" step="0.1" onchange="sendTuning()"></div>
            <div class="input-item"><label>Ki</label><input type="number" id="up-ki" step="0.01" onchange="sendTuning()"></div>
            <div class="input-item"><label>Kd</label><input type="number" id="up-kd" step="0.01" onchange="sendTuning()"></div>
          </div>
        </div>
        <div class="tuning-group">
          <div class="tuning-title">Catching PD</div>
          <div class="input-row" style="grid-template-columns: repeat(2, 1fr);">
            <div class="input-item"><label>Kp</label><input type="number" id="catch-kp" step="0.1" onchange="sendTuning()"></div>
            <div class="input-item"><label>Kd</label><input type="number" id="catch-kd" step="0.01" onchange="sendTuning()"></div>
          </div>
        </div>
        <div class="tuning-group">
          <div class="tuning-title">Downward PID</div>
          <div class="input-row">
            <div class="input-item"><label>Kp</label><input type="number" id="down-kp" step="0.1" onchange="sendTuning()"></div>
            <div class="input-item"><label>Ki</label><input type="number" id="down-ki" step="0.01" onchange="sendTuning()"></div>
            <div class="input-item"><label>Kd</label><input type="number" id="down-kd" step="0.01" onchange="sendTuning()"></div>
          </div>
        </div>
        <div class="polarity-card">
          <div class="polarity-row">
            <span class="polarity-label">Swing Polar</span>
            <button class="btn btn-secondary" style="padding: 0.25rem 0.5rem; font-size: 0.8rem;" onclick="sendCmd('K')">Reverse (K)</button>
          </div>
          <div class="polarity-row">
            <span class="polarity-label">Up Polar</span>
            <button class="btn btn-secondary" style="padding: 0.25rem 0.5rem; font-size: 0.8rem;" onclick="sendCmd('V')">Reverse (V)</button>
          </div>
          <div class="polarity-row">
            <span class="polarity-label">Down Polar</span>
            <button class="btn btn-secondary" style="padding: 0.25rem 0.5rem; font-size: 0.8rem;" onclick="sendCmd('B')">Reverse (B)</button>
          </div>
        </div>
      </div>
    </div>
  </div>

  <script>
    function sendCmd(char) {
      fetch('/control?cmd=' + char)
        .then(r => r.text())
        .then(txt => console.log('Command sent:', char, txt));
    }

    function sendTuning() {
      const params = new URLSearchParams({
        action: 'tuning',
        up_kp: document.getElementById('up-kp').value,
        up_ki: document.getElementById('up-ki').value,
        up_kd: document.getElementById('up-kd').value,
        catch_kp: document.getElementById('catch-kp').value,
        catch_kd: document.getElementById('catch-kd').value,
        down_kp: document.getElementById('down-kp').value,
        down_ki: document.getElementById('down-ki').value,
        down_kd: document.getElementById('down-kd').value
      });
      fetch('/control?' + params.toString())
        .then(r => r.text())
        .then(txt => console.log('Tuning updated:', txt));
    }

    function updateStatus() {
      fetch('/status')
        .then(res => res.json())
        .then(data => {
          document.getElementById('pend-angle').innerText = data.angle.toFixed(1) + '°';
          document.getElementById('pend-vel').innerText = data.velocity.toFixed(0) + '°/s';
          document.getElementById('arm-steps').innerText = data.arm_steps + ' stp';
          document.getElementById('motor-rate').innerText = data.motor_rate.toFixed(0) + ' stp/s';

          // Rotate pointer
          document.getElementById('needle').style.transform = `rotate(${data.angle}deg)`;
          document.getElementById('needle').style.transformOrigin = "50px 50px";

          // Tag class and label
          const tag = document.getElementById('state-tag');
          tag.innerText = data.state;
          tag.className = 'tag';
          if (data.state === 'READY') {
            tag.classList.add('tag-ready');
          } else if (data.state === 'STOPPED') {
            tag.classList.add('tag-stopped');
          } else if (data.state === 'WAITING_ZERO') {
            tag.classList.add('tag-waiting');
          } else {
            tag.classList.add('tag-active');
          }

          // Populate inputs if not focused
          const inputs = {
            'up-kp': data.up_kp, 'up-ki': data.up_ki, 'up-kd': data.up_kd,
            'catch-kp': data.catch_kp, 'catch-kd': data.catch_kd,
            'down-kp': data.down_kp, 'down-ki': data.down_ki, 'down-kd': data.down_kd
          };
          for (let id in inputs) {
            let el = document.getElementById(id);
            if (el && document.activeElement !== el) {
              el.value = inputs[id];
            }
          }
        })
        .catch(err => console.error(err));
    }

    setInterval(updateStatus, 150);
    updateStatus();
  </script>
</body>
</html>
)rawliteral";

WebService::WebService(PendulumController& controller)
  : _controller(controller), _server(80) {}

void WebService::begin() {
  _server.on("/", HTTP_GET, std::bind(&WebService::handleRoot, this));
  _server.on("/status", HTTP_GET, std::bind(&WebService::handleStatus, this));
  _server.on("/control", HTTP_GET, std::bind(&WebService::handleControl, this));
  _server.begin();
}

void WebService::handle() {
  _server.handleClient();
}

void WebService::handleRoot() {
  _server.send(200, "text/html", HTML_TEMPLATE);
}

void WebService::handleStatus() {
  String json = "{";
  json += "\"angle\":" + String(_controller.getPendulumAngleDeg(), 2) + ",";
  json += "\"velocity\":" + String(_controller.getPendulumVelocityDegS(), 2) + ",";
  json += "\"arm_steps\":" + String(_controller.getArmPositionSteps()) + ",";
  json += "\"motor_rate\":" + String(_controller.getMotorRateStepsS(), 1) + ",";
  json += "\"state\":\"" + String(_controller.getStateName()) + "\",";
  json += "\"swing_polarity\":" + String(_controller.getSwingPolarity()) + ",";
  json += "\"up_polarity\":" + String(_controller.getUpPolarity()) + ",";
  json += "\"down_polarity\":" + String(_controller.getDownPolarity()) + ",";
  json += "\"up_kp\":" + String(_controller.getUpKp(), 2) + ",";
  json += "\"up_ki\":" + String(_controller.getUpKi(), 2) + ",";
  json += "\"up_kd\":" + String(_controller.getUpKd(), 2) + ",";
  json += "\"catch_kp\":" + String(_controller.getCatchKp(), 2) + ",";
  json += "\"catch_kd\":" + String(_controller.getCatchKd(), 2) + ",";
  json += "\"down_kp\":" + String(_controller.getDownKp(), 2) + ",";
  json += "\"down_ki\":" + String(_controller.getDownKi(), 2) + ",";
  json += "\"down_kd\":" + String(_controller.getDownKd(), 2);
  json += "}";
  _server.send(200, "application/json", json);
}

void WebService::handleControl() {
  if (_server.hasArg("cmd")) {
    String cmd = _server.arg("cmd");
    if (cmd.length() > 0) {
      _controller.processCommand(cmd[0]);
      _server.send(200, "text/plain", "Command Executed");
      return;
    }
  }

  if (_server.hasArg("action") && _server.arg("action") == "tuning") {
    float up_kp = _server.arg("up_kp").toFloat();
    float up_ki = _server.arg("up_ki").toFloat();
    float up_kd = _server.arg("up_kd").toFloat();
    float catch_kp = _server.arg("catch_kp").toFloat();
    float catch_kd = _server.arg("catch_kd").toFloat();
    float down_kp = _server.arg("down_kp").toFloat();
    float down_ki = _server.arg("down_ki").toFloat();
    float down_kd = _server.arg("down_kd").toFloat();

    _controller.setUpGains(up_kp, up_ki, up_kd);
    _controller.setCatchGains(catch_kp, catch_kd);
    _controller.setDownGains(down_kp, down_ki, down_kd);

    _server.send(200, "text/plain", "Gains Tuned");
    return;
  }

  _server.send(400, "text/plain", "Bad Request");
}
