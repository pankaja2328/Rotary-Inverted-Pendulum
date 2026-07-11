#include "WebService.h"

static const char* HTML_TEMPLATE = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Stepper Control Center</title>
  <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;600;800&display=swap" rel="stylesheet">
  <style>
    :root {
      --bg: #0b0f19;
      --panel: rgba(255, 255, 255, 0.03);
      --border: rgba(255, 255, 255, 0.08);
      --text: #f3f4f6;
      --text-muted: #9ca3af;
      --primary: #3b82f6;
      --primary-glow: rgba(59, 130, 246, 0.5);
      --accent: #10b981;
      --accent-glow: rgba(16, 185, 129, 0.3);
      --danger: #ef4444;
      --danger-glow: rgba(239, 68, 68, 0.3);
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
      background-image: radial-gradient(circle at top right, rgba(59, 130, 246, 0.12), transparent 45%),
                        radial-gradient(circle at bottom left, rgba(16, 185, 129, 0.06), transparent 45%);
    }
    .container {
      width: 100%;
      max-width: 900px;
    }
    header {
      text-align: center;
      margin-bottom: 2.5rem;
    }
    h1 {
      font-size: 2.5rem;
      font-weight: 800;
      background: linear-gradient(135deg, #fff 40%, var(--primary) 100%);
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
      margin-bottom: 0.5rem;
    }
    .subtitle {
      color: var(--text-muted);
      font-weight: 300;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
      gap: 1.5rem;
      margin-bottom: 2rem;
    }
    .card {
      background: var(--panel);
      backdrop-filter: blur(16px);
      -webkit-backdrop-filter: blur(16px);
      border: 1px solid var(--border);
      border-radius: 24px;
      padding: 1.5rem;
      transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
    }
    .card:hover {
      border-color: rgba(255, 255, 255, 0.15);
      transform: translateY(-2px);
    }
    .status-value {
      font-size: 2rem;
      font-weight: 600;
      margin-top: 0.5rem;
      color: #fff;
    }
    .status-label {
      font-size: 0.85rem;
      color: var(--text-muted);
      text-transform: uppercase;
      letter-spacing: 0.08em;
    }
    .control-group {
      margin-bottom: 1.25rem;
    }
    label {
      display: block;
      font-size: 0.875rem;
      color: var(--text-muted);
      margin-bottom: 0.5rem;
    }
    input[type="number"], select {
      width: 100%;
      background: rgba(0, 0, 0, 0.3);
      border: 1px solid var(--border);
      padding: 0.75rem 1rem;
      border-radius: 12px;
      color: #fff;
      font-size: 1rem;
      transition: border-color 0.2s;
    }
    input[type="number"]:focus, select:focus {
      outline: none;
      border-color: var(--primary);
      box-shadow: 0 0 10px var(--primary-glow);
    }
    .btn {
      width: 100%;
      padding: 0.875rem;
      border: none;
      border-radius: 12px;
      font-weight: 600;
      font-size: 1rem;
      cursor: pointer;
      transition: all 0.2s;
      display: flex;
      justify-content: center;
      align-items: center;
      gap: 0.5rem;
    }
    .btn-primary {
      background: var(--primary);
      color: #fff;
      box-shadow: 0 4px 14px var(--primary-glow);
    }
    .btn-primary:hover {
      background: #2563eb;
      box-shadow: 0 6px 20px var(--primary-glow);
    }
    .btn-danger {
      background: var(--danger);
      color: #fff;
      box-shadow: 0 4px 14px var(--danger-glow);
    }
    .btn-danger:hover {
      background: #dc2626;
      box-shadow: 0 6px 20px var(--danger-glow);
    }
    .btn-secondary {
      background: transparent;
      border: 1px solid var(--border);
      color: var(--text);
    }
    .btn-secondary:hover {
      background: rgba(255, 255, 255, 0.05);
    }
    .flex-row {
      display: flex;
      gap: 1rem;
    }
    .badge {
      display: inline-block;
      padding: 0.25rem 0.75rem;
      border-radius: 9999px;
      font-size: 0.75rem;
      font-weight: 600;
      text-transform: uppercase;
    }
    .badge-idle { background: rgba(156, 163, 175, 0.2); color: #9ca3af; }
    .badge-accel { background: rgba(59, 130, 246, 0.2); color: #3b82f6; animation: pulse 2.5s infinite; }
    .badge-constant { background: rgba(16, 185, 129, 0.2); color: #10b981; }
    .badge-decel { background: rgba(245, 158, 11, 0.2); color: #f59e0b; }
    .badge-stopping { background: rgba(239, 68, 68, 0.2); color: #ef4444; }
    @keyframes pulse {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.5; }
    }
  </style>
</head>
<body>
  <div class="container">
    <header>
      <h1>Stepper Motor Control Center</h1>
      <p class="subtitle">WiFi AP Telemetry & Control Dashboard</p>
    </header>

    <!-- Telemetry Cards -->
    <div class="grid">
      <div class="card">
        <div class="status-label">Current Position</div>
        <div class="status-value" id="val-pos">--</div>
      </div>
      <div class="card">
        <div class="status-label">Current Speed</div>
        <div class="status-value" id="val-speed">--</div>
      </div>
      <div class="card">
        <div class="status-label">Status & State</div>
        <div class="status-value">
          <span id="val-state" class="badge badge-idle">UNKNOWN</span>
          <span id="val-enabled" style="font-size: 1rem; margin-left: 0.5rem; color: var(--text-muted);">(--)</span>
        </div>
      </div>
    </div>

    <!-- Control Forms -->
    <div class="grid">
      <!-- Movement config -->
      <div class="card" style="grid-column: span 2;">
        <h2 style="font-size: 1.25rem; font-weight: 600; margin-bottom: 1.25rem;">Command Generator</h2>
        <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 1rem;">
          <div class="control-group">
            <label for="input-steps">Steps</label>
            <input type="number" id="input-steps" value="200" min="1">
          </div>
          <div class="control-group">
            <label for="input-dir">Direction</label>
            <select id="input-dir">
              <option value="1">CCW (Forward)</option>
              <option value="0">CW (Reverse)</option>
            </select>
          </div>
          <div class="control-group">
            <label for="input-min">Min Interval (20uS Ticks)</label>
            <input type="number" id="input-min" value="50" min="1" title="Minimum interval between step pulses. 50 ticks = 1000 uS = 1000 steps/s">
          </div>
          <div class="control-group">
            <label for="input-start">Start Interval (20uS Ticks)</label>
            <input type="number" id="input-start" value="250" min="1" title="Starting step interval for acceleration. 250 ticks = 5000 uS = 200 steps/s">
          </div>
          <div class="control-group" style="grid-column: span 2;">
            <label for="input-accel">Acceleration steps</label>
            <input type="number" id="input-accel" value="100" min="0" title="Number of steps to accelerate/decelerate. Use 0 for constant speed.">
          </div>
        </div>

        <div class="flex-row" style="margin-top: 1rem;">
          <button class="btn btn-primary" onclick="sendMove()">Start Action</button>
          <button class="btn btn-danger" onclick="sendStop()">STOP</button>
        </div>
      </div>

      <!-- General Device Actions -->
      <div class="card">
        <h2 style="font-size: 1.25rem; font-weight: 600; margin-bottom: 1.25rem;">Coil Enable Control</h2>
        <div style="display: flex; flex-direction: column; gap: 1rem;">
          <button class="btn btn-secondary" onclick="sendEnable(true)">Enable Driver Coils</button>
          <button class="btn btn-secondary" style="border-color: rgba(239, 68, 68, 0.4); color: #fca5a5;" onclick="sendEnable(false)">Disable Coils</button>
        </div>
      </div>
    </div>
  </div>

  <script>
    function updateStatus() {
      fetch('/status')
        .then(r => r.json())
        .then(data => {
          document.getElementById('val-pos').innerText = data.position + ' steps';
          document.getElementById('val-speed').innerText = data.speed.toFixed(1) + ' steps/s';
          
          const stateEl = document.getElementById('val-state');
          stateEl.className = 'badge';
          stateEl.classList.add('badge-' + data.mode.toLowerCase());
          stateEl.innerText = data.mode;
          
          document.getElementById('val-enabled').innerText = data.enabled ? 'ENABLED' : 'DISABLED';
          document.getElementById('val-enabled').style.color = data.enabled ? 'var(--accent)' : 'var(--text-muted)';
        })
        .catch(err => console.error('Status fetch error:', err));
    }

    function sendMove() {
      const steps = document.getElementById('input-steps').value;
      const dir = document.getElementById('input-dir').value;
      const min_interval = document.getElementById('input-min').value;
      const start_interval = document.getElementById('input-start').value;
      const accel = document.getElementById('input-accel').value;

      fetch(`/control?action=move&steps=${steps}&dir=${dir}&min_interval=${min_interval}&start_interval=${start_interval}&accel=${accel}`)
        .then(r => r.text())
        .then(txt => console.log(txt));
    }

    function sendStop() {
      fetch('/control?action=stop')
        .then(r => r.text())
        .then(txt => console.log(txt));
    }

    function sendEnable(val) {
      fetch(`/control?action=enable&value=${val ? 1 : 0}`)
        .then(r => r.text())
        .then(txt => console.log(txt));
    }

    setInterval(updateStatus, 150);
    updateStatus();
  </script>
</body>
</html>
)rawliteral";

WebService::WebService(StepperMotor& stepper) : _stepper(stepper), _server(80) {}

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
  json += "\"position\":" + String(_stepper.getCurrentPosition()) + ",";
  json += "\"speed\":" + String(_stepper.getCurrentSpeed(), 1) + ",";
  
  String modeStr;
  switch (_stepper.getMode()) {
    case StepperMode::IDLE:     modeStr = "IDLE"; break;
    case StepperMode::ACCEL:    modeStr = "ACCEL"; break;
    case StepperMode::CONSTANT: modeStr = "CONSTANT"; break;
    case StepperMode::DECEL:    modeStr = "DECEL"; break;
    case StepperMode::STOPPING: modeStr = "STOPPING"; break;
    default:                    modeStr = "UNKNOWN"; break;
  }
  
  json += "\"mode\":\"" + modeStr + "\",";
  json += "\"enabled\":" + String(_stepper.isEnabled() ? "true" : "false");
  json += "}";
  
  _server.send(200, "application/json", json);
}

void WebService::handleControl() {
  String action = _server.arg("action");
  if (action == "move") {
    uint32_t steps = _server.arg("steps").toInt();
    bool direction = _server.arg("dir").toInt() != 0;
    uint32_t min_interval = _server.arg("min_interval").toInt();
    uint32_t start_interval = _server.arg("start_interval").toInt();
    uint32_t accel = _server.arg("accel").toInt();
    
    _stepper.moveRelative(steps, direction, min_interval, start_interval, accel);
    _server.send(200, "text/plain", "OK");
  } 
  else if (action == "stop") {
    _stepper.stop();
    _server.send(200, "text/plain", "Stopped");
  } 
  else if (action == "enable") {
    bool val = _server.arg("value").toInt() != 0;
    _stepper.setEnable(val);
    _server.send(200, "text/plain", val ? "Enabled" : "Disabled");
  } 
  else {
    _server.send(400, "text/plain", "Bad Request");
  }
}
