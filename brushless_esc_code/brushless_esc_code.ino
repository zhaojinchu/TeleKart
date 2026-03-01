/*
  ESP32 WiFi RC Car Controller - V2

  Features:
  - ESP32 hosts WiFi AP
  - Web UI with:
      * Steering slider
      * Throttle slider
      * Live steering trim
      * Throttle strength control
      * Keyboard WASD control
  - Failsafe auto stop
  - ESC arming delay

  Hardware:
  - HS-311 steering on GPIO 18
  - ESC signal on GPIO 19
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// ================== PINS ==================
static const int STEER_PIN = 18;
static const int ESC_PIN   = 19;

// ================== WIFI ==================
const char* AP_SSID = "ESP32-CAR";
const char* AP_PASS = "drivecar123";

// ================== RC SIGNAL SETTINGS ==================
static const int STEER_LEFT_US   = 1100;
static const int STEER_CENTER_US = 1500;
static const int STEER_RIGHT_US  = 1900;

static const int ESC_NEUTRAL_US  = 1500;
static const int ESC_REV_MAX_US  = 1300;
static const int ESC_FWD_MAX_US  = 1700;

static const bool ESC_BIDIRECTIONAL = true;

// ================== DEFAULT CALIBRATION ==================
// Your tested straight value was 73
int steeringTrim = 73;        // -100..100 scale

static const uint32_t COMMAND_TIMEOUT_MS = 700;

// ================== OBJECTS ==================
WebServer server(80);
Servo steerServo;
Servo escServo;

volatile int targetThrottlePct = 0;
volatile int targetSteerPct    = 0;
uint32_t lastCommandMs = 0;
bool inFailsafe = false;

// ==========================================================
// =================== UTILITY FUNCTIONS ====================
// ==========================================================

int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

int mapPercentToUsSymmetric(int pct, int negUs, int zeroUs, int posUs) {
  pct = clampInt(pct, -100, 100);

  if (pct >= 0) {
    return zeroUs + ((posUs - zeroUs) * pct) / 100;
  } else {
    return zeroUs + ((zeroUs - negUs) * pct) / 100;
  }
}

// Remap so that user 0 = steeringTrim
int remapSteerUserPct(int userPct) {
  userPct = clampInt(userPct, -100, 100);

  if (userPct >= 0) {
    return steeringTrim
         + ((100 - steeringTrim) * userPct) / 100;
  } else {
    return steeringTrim
         + ((steeringTrim + 100) * userPct) / 100;
  }
}

void applyOutputs(int throttlePct, int steerPct) {

  throttlePct = clampInt(throttlePct, -100, 100);
  steerPct    = clampInt(steerPct, -100, 100);

  int steerCal = remapSteerUserPct(steerPct);

  int steerUs = mapPercentToUsSymmetric(
      steerCal, STEER_LEFT_US, STEER_CENTER_US, STEER_RIGHT_US);

  steerServo.writeMicroseconds(steerUs);

  int escUs;

  if (ESC_BIDIRECTIONAL) {
    escUs = mapPercentToUsSymmetric(
        throttlePct, ESC_REV_MAX_US, ESC_NEUTRAL_US, ESC_FWD_MAX_US);
  } else {
    int t = throttlePct < 0 ? 0 : throttlePct;
    escUs = ESC_NEUTRAL_US
          + ((ESC_FWD_MAX_US - ESC_NEUTRAL_US) * t) / 100;
  }

  escServo.writeMicroseconds(escUs);
}

// ==========================================================
// ====================== WEB UI ============================
// ==========================================================

const char MAIN_PAGE[] = R"HTML(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32 Car V2</title>
<style>
body { font-family: Arial; margin:20px; max-width:700px; }
.card { border:1px solid #ddd; padding:15px; border-radius:12px; margin-bottom:15px;}
button { padding:10px; margin:5px; }
input[type=range] { width:100%; }
.value { font-weight:bold; }
</style>
</head>
<body>

<h2>ESP32 Car V2</h2>

<div class="card">
Throttle <span id="thVal" class="value">0</span>
<input id="th" type="range" min="-100" max="100" value="0">
</div>

<div class="card">
Steering <span id="stVal" class="value">0</span>
<input id="st" type="range" min="-100" max="100" value="0">
</div>

<div class="card">
Steering Trim <span id="trimVal" class="value">73</span>
<input id="trim" type="range" min="-100" max="100" value="73">
</div>

<div class="card">
Throttle Strength <span id="powVal" class="value">70</span>
<input id="power" type="range" min="10" max="100" value="70">
</div>

<div class="card">
<button id="stopBtn" onclick="stopNow()">STOP</button>
</div>

<script>
const thEl = document.getElementById("th");
const stEl = document.getElementById("st");
const trimEl = document.getElementById("trim");
const powerEl = document.getElementById("power");
const thValEl = document.getElementById("thVal");
const stValEl = document.getElementById("stVal");
const trimValEl = document.getElementById("trimVal");
const powValEl = document.getElementById("powVal");
const stopBtnEl = document.getElementById("stopBtn");

let throttle = 0;
let steer = 0;
let trim = 73;
let power = 70;
let stopLatched = false;
const keys = {};

function toInt(value, fallback) {
  const parsed = parseInt(value, 10);
  return Number.isNaN(parsed) ? fallback : parsed;
}

function send() {
  fetch("/cmd?th=" + throttle + "&st=" + steer + "&trim=" + trim)
    .catch(err => console.log("fetch error:", err));
}

function updateUI() {
  thValEl.innerText = throttle;
  stValEl.innerText = steer;
  trimValEl.innerText = trim;
  powValEl.innerText = power;
}

function syncStopButton() {
  stopBtnEl.innerText = stopLatched ? "RESUME" : "STOP";
}

function releaseStopLatch() {
  if (!stopLatched) return;
  stopLatched = false;
  syncStopButton();
}

function applyStoppedState() {
  throttle = 0;
  steer = 0;
  thEl.value = "0";
  stEl.value = "0";
  updateUI();
  send();
}

function stopNow() {
  stopLatched = !stopLatched;
  syncStopButton();

  if (stopLatched) {
    applyStoppedState();
  } else {
    send();
  }
}

thEl.oninput = (event) => {
  releaseStopLatch();
  throttle = toInt(event.target.value, 0);
  updateUI();
  send();
};

stEl.oninput = (event) => {
  releaseStopLatch();
  steer = toInt(event.target.value, 0);
  updateUI();
  send();
};

trimEl.oninput = (event) => {
  trim = toInt(event.target.value, 73);
  updateUI();
  send();
};

powerEl.oninput = (event) => {
  power = toInt(event.target.value, 70);
  updateUI();
};

window.addEventListener("keydown", (event) => {
  const key = event.key.toLowerCase();
  if (!["w", "a", "s", "d"].includes(key)) return;
  event.preventDefault();
  keys[key] = true;
  updateFromKeys();
});

window.addEventListener("keyup", (event) => {
  const key = event.key.toLowerCase();
  if (!["w", "a", "s", "d"].includes(key)) return;
  event.preventDefault();
  keys[key] = false;
  updateFromKeys();
});

function updateFromKeys() {
  releaseStopLatch();
  throttle = 0;
  steer = 0;

  if (keys.w) throttle = power;
  if (keys.s) throttle = -power;
  if (keys.a) steer = -70;
  if (keys.d) steer = 70;

  thEl.value = String(throttle);
  stEl.value = String(steer);
  updateUI();
  send();
}

syncStopButton();
updateUI();
</script>

</body>
</html>
)HTML";

// ==========================================================
// ==================== WEB HANDLERS ========================
// ==========================================================

void handleRoot() {
  server.send(200, "text/html", MAIN_PAGE);
}

void handleCmd() {

  if (server.hasArg("th"))
    targetThrottlePct = clampInt(server.arg("th").toInt(), -100, 100);

  if (server.hasArg("st"))
    targetSteerPct = clampInt(server.arg("st").toInt(), -100, 100);

  if (server.hasArg("trim"))
    steeringTrim = clampInt(server.arg("trim").toInt(), -100, 100);

  lastCommandMs = millis();
  inFailsafe = false;

  applyOutputs(targetThrottlePct, targetSteerPct);

  server.send(200, "text/plain", "OK");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ==========================================================
// ====================== SETUP =============================
// ==========================================================

void setup() {

  Serial.begin(115200);

  steerServo.setPeriodHertz(50);
  escServo.setPeriodHertz(50);

  steerServo.attach(STEER_PIN, 500, 2500);
  escServo.attach(ESC_PIN, 500, 2500);

  steerServo.writeMicroseconds(STEER_CENTER_US);
  escServo.writeMicroseconds(ESC_NEUTRAL_US);

  Serial.println("Arming ESC...");
  delay(3000);

  WiFi.softAP(AP_SSID, AP_PASS);

  Serial.print("Connect to WiFi: ");
  Serial.println(AP_SSID);
  Serial.print("Open: http://");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.onNotFound(handleNotFound);
  server.begin();

  lastCommandMs = millis();
}

// ==========================================================
// ====================== LOOP ==============================
// ==========================================================

void loop() {

  server.handleClient();

  uint32_t now = millis();

  if ((now - lastCommandMs) > COMMAND_TIMEOUT_MS) {
    if (!inFailsafe) {
      applyOutputs(0, 0);
      inFailsafe = true;
      Serial.println("Failsafe STOP");
    }
  }
}
