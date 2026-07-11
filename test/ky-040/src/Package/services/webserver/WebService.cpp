#include "WebService.h"

static const char HTML_PAGE[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>KY-040 Encoder Status</title>
  <style>
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; text-align: center; margin-top: 50px; background-color: #0f172a; color: #f8fafc; }
    .card { background: #1e293b; padding: 30px; border-radius: 16px; box-shadow: 0 10px 25px -5px rgba(0,0,0,0.3); display: inline-block; min-width: 320px; }
    h1 { color: #38bdf8; font-size: 1.8em; margin-bottom: 20px; }
    .value { font-size: 2.2em; font-weight: bold; margin: 15px 0; color: #f1f5f9; }
    .direction { font-size: 1.3em; color: #38bdf8; min-height: 1.5em; font-weight: 500; }
    .dial-container { margin: 30px auto; position: relative; width: 160px; height: 160px; }
    .dial { width: 100%; height: 100%; transition: transform 0.2s cubic-bezier(0.25, 0.8, 0.25, 1); }
    .button-state { font-size: 1em; color: #94a3b8; margin-top: 15px; }
    .btn-indicator { display: inline-block; padding: 4px 12px; border-radius: 9999px; font-weight: 600; font-size: 0.85em; transition: all 0.2s; }
    .btn-released { background: #334155; color: #cbd5e1; }
    .btn-pressed { background: #ef4444; color: #ffffff; box-shadow: 0 0 12px #ef4444; }
  </style>
  <script>
    let stepsPerRev = 80; // Full quadrature: 20 detents x 4 transitions = 80 steps/rev
    function updateData() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          document.getElementById('pos').innerText = data.position;
          
          let dirText = "Stationary";
          if (data.delta > 0) dirText = "Clockwise \u21BB";
          else if (data.delta < 0) dirText = "Counter-Clockwise \u21BA";
          
          document.getElementById('dir').innerText = dirText;
          
          // Button state class toggle
          const btnEl = document.getElementById('btn');
          if (data.button) {
            btnEl.innerText = "PRESSED";
            btnEl.className = "btn-indicator btn-pressed";
          } else {
            btnEl.innerText = "RELEASED";
            btnEl.className = "btn-indicator btn-released";
          }
          
          // Calculate angle: 360 degrees / 80 steps = 4.5 degrees per quadrature step
          let angle = data.position * 4.5;
          document.getElementById('dial-svg').style.transform = `rotate(${angle}deg)`;
        })
        .catch(err => console.error(err));
    }
    setInterval(updateData, 100); // Fetch every 100ms for responsiveness
  </script>
</head>
<body>
  <div class="card">
    <h1>KY-040 Rotary Encoder</h1>
    
    <div class="dial-container">
      <!-- SVG encoder knob graphics -->
      <svg id="dial-svg" class="dial" viewBox="0 0 100 100">
        <circle cx="50" cy="50" r="46" fill="#0f172a" stroke="#38bdf8" stroke-width="4"/>
        <circle cx="50" cy="50" r="38" fill="#1e293b"/>
        <!-- Notch/Indicator dot -->
        <circle cx="50" cy="22" r="6" fill="#38bdf8"/>
        <!-- Decorative ribs -->
        <line x1="50" y1="50" x2="50" y2="10" stroke="#334155" stroke-dasharray="2,6" stroke-width="2"/>
        <line x1="50" y1="50" x2="10" y2="50" stroke="#334155" stroke-dasharray="2,6" stroke-width="2"/>
        <line x1="50" y1="50" x2="90" y2="50" stroke="#334155" stroke-dasharray="2,6" stroke-width="2"/>
        <line x1="50" y1="50" x2="50" y2="90" stroke="#334155" stroke-dasharray="2,6" stroke-width="2"/>
      </svg>
    </div>

    <div class="value">Position: <span id="pos">0</span></div>
    <div class="direction" id="dir">Stationary</div>
    <div class="button-state">
      Button: <span id="btn" class="btn-indicator btn-released">RELEASED</span>
    </div>
  </div>
</body>
</html>
)=====";

WebService::WebService(KY040& encoder) : _server(80), _encoder(encoder) {}

void WebService::begin() {
  _server.on("/", HTTP_GET, [this]() { _handleRoot(); });
  _server.on("/data", HTTP_GET, [this]() { _handleData(); });
  _server.onNotFound([this]() { _handleNotFound(); });
  _server.begin();
}

void WebService::handle() {
  _server.handleClient();
}

void WebService::_handleRoot() {
  _server.send(200, "text/html", HTML_PAGE);
}

void WebService::_handleData() {
  long delta = _encoder.getDelta(); // Calling this will reset the delta internally in our class
  long pos = _encoder.getPosition();
  bool btn = _encoder.isButtonPressed();

  String json = "{";
  json += "\"position\":" + String(pos) + ",";
  json += "\"delta\":" + String(delta) + ",";
  json += "\"button\":" + String(btn ? "true" : "false");
  json += "}";

  _server.send(200, "application/json", json);
}

void WebService::_handleNotFound() {
  _server.send(404, "text/plain", "Not Found");
}
