/*
  Bench Test - Minimal WASD control over ESP32 AP
  For debugging Wi-Fi latency and physical car agility.

  Connect to WiFi "KartBench" / "kartbench", open http://192.168.4.1
  Hardware: steering servo GPIO 18, ESC GPIO 19
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// -- Pins --
#define STEER_PIN 18
#define ESC_PIN   19

// -- Servo limits --
#define STEER_LEFT_US  1250
#define STEER_CENTER_US 1750
#define STEER_RIGHT_US 2250
#define ESC_NEUTRAL_US 1500
#define ESC_FWD_MAX_US 2000
#define ESC_REV_MAX_US 1000

// -- Failsafe --
#define CMD_TIMEOUT_MS 500

WebServer server(80);
Servo steer;
Servo esc;

int throttle = 0;   // -100..100
int steering = 0;   // -100..100
uint32_t lastCmd = 0;

int clamp(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }

int pctToUs(int pct, int negUs, int midUs, int posUs) {
  pct = clamp(pct, -100, 100);
  return pct >= 0
    ? midUs + (posUs - midUs) * pct / 100
    : midUs + (midUs - negUs) * pct / 100;
}

void drive() {
  steer.writeMicroseconds(pctToUs(steering, STEER_LEFT_US, STEER_CENTER_US, STEER_RIGHT_US));
  esc.writeMicroseconds(pctToUs(throttle, ESC_REV_MAX_US, ESC_NEUTRAL_US, ESC_FWD_MAX_US));
}

// ---- Web page ----
const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Bench</title>
<style>
*{margin:0;box-sizing:border-box}
body{background:#111;color:#eee;font-family:monospace;display:flex;flex-direction:column;align-items:center;justify-content:center;height:100vh}
h1{font-size:1.4rem;margin-bottom:12px;color:#0df}
.row{display:flex;gap:24px;margin:8px 0}
.v{font-size:2rem;min-width:100px;text-align:center}
.l{font-size:.8rem;color:#888;text-align:center}
.keys{display:grid;grid-template-columns:repeat(3,48px);gap:4px;margin:16px 0}
.k{width:48px;height:48px;display:flex;align-items:center;justify-content:center;border:2px solid #333;border-radius:8px;font-size:1.2rem;color:#666}
.k.on{border-color:#0df;color:#0df;background:#0df2}
#lat{color:#fa0;font-size:1.1rem;margin-top:10px}
.bar-wrap{width:220px;height:14px;background:#222;border-radius:7px;overflow:hidden;border:1px solid #333}
.bar{height:100%;transition:width 40ms;border-radius:7px}
.bt{background:#0df}.bs{background:#f55}
#help{color:#555;font-size:.7rem;margin-top:16px}
</style></head><body>
<h1>BENCH TEST</h1>

<div class="row">
  <div><div class="l">THROTTLE</div><div class="v" id="tv">0</div>
    <div class="bar-wrap"><div class="bar bt" id="tb"></div></div></div>
  <div><div class="l">STEER</div><div class="v" id="sv">0</div>
    <div class="bar-wrap"><div class="bar bt" id="sb"></div></div></div>
</div>

<div class="keys">
  <div></div><div class="k" id="kw">W</div><div></div>
  <div class="k" id="ka">A</div><div class="k" id="ks">S</div><div class="k" id="kd">D</div>
</div>

<div id="lat">RTT: --</div>
<div style="margin-top:10px"><label style="color:#888;font-size:.8rem">POWER <span id="pv">100</span>%</label><br>
<input type="range" id="pwr" min="0" max="100" value="100" style="width:220px"></div>
<div id="help">WASD drive | Q/E trim steer center | slider sets power</div>

<script>
let th=0,st=0,trim=0,keys={};
const tv=document.getElementById('tv'),sv=document.getElementById('sv');
const tb=document.getElementById('tb'),sb=document.getElementById('sb');
const lat=document.getElementById('lat');
const pwr=document.getElementById('pwr'),pv=document.getElementById('pv');
const kels={w:'kw',a:'ka',s:'ks',d:'kd'};
pwr.addEventListener('input',()=>{pv.textContent=pwr.value;calc();});

function ui(){
  tv.textContent=th;sv.textContent=st;
  tb.style.width=Math.abs(th)+'%';
  tb.style.background=th<0?'#f55':'#0df';
  tb.style.marginLeft=th<0?(100-Math.abs(th))+'%':'0';
  sb.style.width=Math.abs(st)+'%';
  sb.style.background=st<0?'#f55':'#0df';
  sb.style.marginLeft=st<0?(100-Math.abs(st))+'%':'0';
  for(let k in kels){
    document.getElementById(kels[k]).classList.toggle('on',!!keys[k]);
  }
}

let inflight=false;
function send(){
  if(inflight)return;
  inflight=true;
  let t0=performance.now();
  fetch('/cmd?th='+th+'&st='+st+'&t='+Date.now())
    .then(r=>r.text()).then(()=>{
      lat.textContent='RTT: '+(performance.now()-t0).toFixed(1)+' ms';
    }).catch(()=>{lat.textContent='RTT: LOST';})
    .finally(()=>{inflight=false;});
}

function calc(){
  th=0;st=0;
  let p=parseInt(pwr.value);
  if(keys.w)th=p;
  if(keys.s)th=-p;
  if(keys.a)st=-100;
  if(keys.d)st=100;
  ui();
}

document.addEventListener('keydown',e=>{
  let k=e.key.toLowerCase();
  if(k=='q'){trim=Math.max(trim-2,-100);return;}
  if(k=='e'){trim=Math.min(trim+2,100);return;}
  if('wasd'.includes(k)){e.preventDefault();keys[k]=true;calc();}
});
document.addEventListener('keyup',e=>{
  let k=e.key.toLowerCase();
  if('wasd'.includes(k)){e.preventDefault();keys[k]=false;calc();}
});

setInterval(send,50);
ui();
</script></body></html>
)HTML";

// ---- Handlers ----
void handleRoot() { server.send_P(200, "text/html", PAGE); }

void handleCmd() {
  if (server.hasArg("th")) throttle = clamp(server.arg("th").toInt(), -100, 100);
  if (server.hasArg("st")) steering = clamp(server.arg("st").toInt(), -100, 100);
  lastCmd = millis();
  drive();
  server.send(200, "text/plain", "ok");
}

// ---- Setup / Loop ----
void setup() {
  Serial.begin(115200);

  steer.setPeriodHertz(50);
  esc.setPeriodHertz(50);
  steer.attach(STEER_PIN, 500, 2500);
  esc.attach(ESC_PIN, 500, 2500);
  steer.writeMicroseconds(STEER_CENTER_US);
  esc.writeMicroseconds(ESC_NEUTRAL_US);

  Serial.println("Arming ESC...");
  delay(3000);

  WiFi.softAP("KartBench", "kartbench");
  Serial.print("AP ready: http://");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.begin();
}

void loop() {
  server.handleClient();
  if (millis() - lastCmd > CMD_TIMEOUT_MS) {
    throttle = 0;
    steering = 0;
    drive();
  }
}
