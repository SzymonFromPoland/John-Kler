#include "dashboard.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

static AsyncWebServer server(80);
static uint16_t *_dist;
static int *_mode;
static volatile float *_kp, *_kd, *_speed, *_mspeed, *_targetYaw;
static volatile bool *_calibrateFlag;

static const char HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body { font-family: sans-serif; padding: 16px; max-width: 500px; margin: auto; }
  h2 { margin-bottom: 8px; }
  .modes { display: flex; gap: 8px; margin-bottom: 16px; }
  .modes button { flex: 1; padding: 10px; font-size: 16px; cursor: pointer; border: 2px solid #ccc; background: #f0f0f0; border-radius: 6px; }
  .modes button.active { background: #333; color: #fff; border-color: #333; }
  .params { display: grid; grid-template-columns: 80px 1fr 50px; gap: 6px 10px; align-items: center; margin-bottom: 16px; }
  .params label { font-weight: bold; }
  .params input[type=range] { width: 100%; }
  .params span { text-align: right; font-family: monospace; }
  .btn-cal { flex: 1; width: 100%; padding: 10px; font-size: 16px; cursor: pointer; border: 2px solid #ccc; background: #f0f0f0; border-radius: 6px; }
  .btn-cal:active { background: #333; color: #fff; border-color: #333; }
</style>
</head>
<body>
<h2>Johnatan-Kler</h2>

<div class="modes">
  <button id="m1" onclick="setMode(1)">Mode 1</button>
  <button id="m2" onclick="setMode(2)">Mode 2</button>
  <button id="m3" onclick="setMode(3)">Mode 3</button>
</div>

<div class="params">

  <label>Kp</label>
  <input type="range" id="kp" min="0" max="1000" step="1" oninput="document.getElementById('kp_v').innerText=this.value; sendParams()">
  <span id="kp_v">-</span>

  <label>Kd</label>
  <input type="range" id="kd" min="0" max="1000" step="1" oninput="document.getElementById('kd_v').innerText=this.value; sendParams()">
  <span id="kd_v">-</span>

  <label>Speed</label>
  <input type="range" id="sp" min="0" max="100" step="1" oninput="document.getElementById('sp_v').innerText=this.value; sendParams()">
  <span id="sp_v">-</span>

  <label>Target Yaw</label>
  <input type="range" id="ty" min="-180" max="180" step="1" oninput="document.getElementById('ty_v').innerText=this.value; sendParams()">
  <span id="ty_v">0</span>
</div>

<button class="btn-cal" onclick="calibrate()">CALIBRATE MPU</button>

<script>
function setMode(m) {
  fetch('/mode?v=' + m);
}
function calibrate() {
  fetch('/calibrate');
}
function sendParams() {
  const ty = document.getElementById('ty').value;
  const kp = document.getElementById('kp').value;
  const kd = document.getElementById('kd').value;
  const sp = document.getElementById('sp').value;
  fetch(`/params?ty=${ty}&kp=${kp}&kd=${kd}&sp=${sp}`);
}
function update() {
  fetch('/data').then(r=>r.json()).then(d=>{
    if (document.getElementById('kp_v').innerText === '-') {
        document.getElementById('ty').value = d.ty;
        document.getElementById('ty_v').innerText = d.ty;
        document.getElementById('kp').value = d.kp;
        document.getElementById('kp_v').innerText = d.kp;
        document.getElementById('kd').value = d.kd;
        document.getElementById('kd_v').innerText = d.kd;
        document.getElementById('sp').value = d.sp;
        document.getElementById('sp_v').innerText = d.sp;
    }
  });
}
setInterval(update, 500);
</script>
</body>
</html>)HTML";

static void dashTask(void *param)
{
  WiFi.softAP("Johnatan-Kler", "12345678");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req)   
            { req->send(200, "text/html", HTML); });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *req)
            {
    char buf[512];
    snprintf(buf, sizeof(buf),
      "{\"mode\":%d,\"kp\":%.1f,\"kd\":%.1f,\"sp\":%.1f,\"ty\":%.1f}",
      *_mode, *_kp, *_kd, *_speed, *_targetYaw);
    req->send(200, "application/json", buf); });

  server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *req)
            {
    *_calibrateFlag = true;
    req->send(200, "text/plain", "calibrating"); });

  server.on("/params", HTTP_GET, [](AsyncWebServerRequest *req)
            {
    if (req->hasParam("ty")) *_targetYaw = req->getParam("ty")->value().toFloat();
    if (req->hasParam("kp")) *_kp = req->getParam("kp")->value().toFloat();
    if (req->hasParam("kd")) *_kd = req->getParam("kd")->value().toFloat();
    if (req->hasParam("sp")) *_speed = req->getParam("sp")->value().toFloat();
    req->send(200, "text/plain", "ok"); });

  server.begin();
  vTaskDelete(NULL);
}

void startDashboard(uint16_t *dist, int *mode, float *kp, float *kd, float *speed, float *mspeed, float *targetYaw, bool *calibrateFlag)
{
  _dist = dist;
  _mode = mode;
  _kp = kp;
  _kd = kd;
  _speed = speed;
  _mspeed = mspeed;
  _targetYaw = targetYaw;
  _calibrateFlag = calibrateFlag;
  xTaskCreatePinnedToCore(dashTask, "dashboard", 8192, NULL, 1, NULL, 1);
}
