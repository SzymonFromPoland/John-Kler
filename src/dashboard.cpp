#include "dashboard.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

static AsyncWebServer server(80);
static uint16_t *_dist;
static int *_mode;
static volatile float *_kp;
static volatile float *_kd;
static volatile float *_speed;

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
  table { width: 100%; border-collapse: collapse; margin-bottom: 16px; }
  td, th { border: 1px solid #ccc; padding: 6px 10px; text-align: center; }
  th { background: #f0f0f0; }
  .bar-cell { padding: 4px 6px; }
  .bar-wrap { background: #eee; border-radius: 4px; height: 16px; }
  .bar { background: #333; height: 16px; border-radius: 4px; transition: width 0.1s; }
  .ut-yes { color: green; font-weight: bold; }
  .ut-no  { color: #aaa; }
  .params { display: grid; grid-template-columns: 80px 1fr 50px; gap: 6px 10px; align-items: center; margin-bottom: 16px; }
  .params label { font-weight: bold; }
  .params input[type=range] { width: 100%; }
  .params span { text-align: right; font-family: monospace; }
  .apply-btn { width: 100%; padding: 10px; font-size: 16px; cursor: pointer; border: none; background: #333; color: #fff; border-radius: 6px; }
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
  <input type="range" id="kp" min="0" max="300" step="1" oninput="document.getElementById('kp_v').innerText=this.value; sendParams()">
  <span id="kp_v">-</span>

  <label>Kd</label>
  <input type="range" id="kd" min="0" max="200" step="1" oninput="document.getElementById('kd_v').innerText=this.value; sendParams()">
  <span id="kd_v">-</span>

  <label>Speed</label>
  <input type="range" id="sp" min="0" max="100" step="1" oninput="document.getElementById('sp_v').innerText=this.value; sendParams()">
  <span id="sp_v">-</span>
</div>
<br>

<table>
  <tr><th>#</th><th>Distance (mm)</th><th>Bar</th><th>Under</th></tr>
  <script>for(let i=0;i<9;i++) document.write(`<tr><td>${i}</td><td id="d${i}">-</td><td class="bar-cell"><div class="bar-wrap"><div class="bar" id="b${i}" style="width:0%"></div></div></td><td id="u${i}">-</td></tr>`);</script>
</table>

<script>
const THRESHOLD = 300;
function setMode(m) {
  fetch('/mode?v=' + m);
  [1,2,3].forEach(i => document.getElementById('m'+i).classList.toggle('active', i===m));
}
function sendParams() {
  const kp = document.getElementById('kp').value;
  const kd = document.getElementById('kd').value;
  const sp = document.getElementById('sp').value;
  fetch('/params?kp=' + kp + '&kd=' + kd + '&sp=' + sp);
}
function update() {
  fetch('/data').then(r=>r.json()).then(d=>{
    d.dist.forEach((v,i)=>{
      document.getElementById('d'+i).innerText = v;
      document.getElementById('b'+i).style.width = Math.max(0, 100 - Math.round(v/THRESHOLD*100)) + '%';
      const u = document.getElementById('u'+i);
      u.innerText = v < THRESHOLD ? '●' : '○';
      u.className = v < THRESHOLD ? 'ut-yes' : 'ut-no';
    });
    [1,2,3].forEach(i => document.getElementById('m'+i).classList.toggle('active', i===d.mode));
    if (document.getElementById('kp_v').innerText === '-') {
      document.getElementById('kp').value = d.kp;
      document.getElementById('kp_v').innerText = d.kp;
      document.getElementById('kd').value = d.kd;
      document.getElementById('kd_v').innerText = d.kd;
      document.getElementById('sp').value = d.sp;
      document.getElementById('sp_v').innerText = d.sp;
    }
  });
}
setInterval(update, 100);
update();
</script>
</body>
</html>)HTML";

static void dashTask(void *param)
{
    WiFi.softAP("Johnatan-Kler", "12345678");

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send(200, "text/html", HTML);
    });

    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *req) {
        char buf[300];
        snprintf(buf, sizeof(buf),
            "{\"dist\":[%d,%d,%d,%d,%d,%d,%d,%d,%d],\"mode\":%d,\"kp\":%.1f,\"kd\":%.1f,\"sp\":%.1f}",
            _dist[0], _dist[1], _dist[2], _dist[3], _dist[4],
            _dist[5], _dist[6], _dist[7], _dist[8], *_mode,
            *_kp, *_kd, *_speed);
        req->send(200, "application/json", buf);
    });

    server.on("/mode", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (req->hasParam("v"))
            *_mode = req->getParam("v")->value().toInt();
        req->send(200, "text/plain", "ok");
    });

    server.on("/params", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (req->hasParam("kp"))
            *_kp = req->getParam("kp")->value().toFloat();
        if (req->hasParam("kd"))
            *_kd = req->getParam("kd")->value().toFloat();
        if (req->hasParam("sp"))
            *_speed = req->getParam("sp")->value().toFloat();
        req->send(200, "text/plain", "ok");
    });

    server.begin();
    vTaskDelete(NULL);
}

void startDashboard(uint16_t *dist, int *mode, float *kp, float *kd, float *speed)
{
    _dist = dist;
    _mode = mode;
    _kp = kp;
    _kd = kd;
    _speed = speed;
    xTaskCreatePinnedToCore(dashTask, "dashboard", 8192, NULL, 1, NULL, 0);
}
