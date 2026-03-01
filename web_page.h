#pragma once
#include <pgmspace.h>

// Put the entire HTML page here.
// NOTE: This is V6 (controller select + center + trim + shaping).
// If you want, we can add the steering smoothing patch later.
static const char MAIN_PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32 Car V6</title>
<style>
  body { font-family: Arial; margin:20px; max-width:900px; }
  .card { border:1px solid #ddd; padding:15px; border-radius:12px; margin-bottom:15px;}
  button { padding:10px; margin:5px; }
  input[type=range] { width:100%; }
  .value { font-weight:bold; }
  .small { color:#666; font-size: 12px; }
  .row { display:flex; gap:10px; flex-wrap:wrap; align-items:flex-end; }
  .row > * { flex: 1 1 180px; }
  input[type=number] { width: 100%; padding:8px; }
  select { width: 100%; padding:8px; }
  label { display:block; margin-bottom:6px; }
  .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace; }
</style>
</head>
<body>

<h2>ESP32 Car V6</h2>
<div class="small">Connect to WiFi: <b>ESP32-CAR</b>, then open <b>http://192.168.4.1/</b></div>

<div class="card">
  <div class="row">
    <div>
      <label>Controller</label>
      <select id="gpSelect"></select>
      <div class="small" id="gpInfo">(no controller)</div>
    </div>
    <div>
      <button onclick="refreshGamepads()">Refresh list</button>
      <button id="stopBtn" onclick="stopNow()">STOP</button>
    </div>
  </div>
</div>

<div class="card">
  Throttle <span id="thVal" class="value">0</span>
  <input id="th" type="range" min="-100" max="100" value="0">
</div>

<div class="card">
  Steering <span id="stVal" class="value">0</span>
  <input id="st" type="range" min="-100" max="100" value="0">
</div>

<div class="card">
  <b>Steering neutral (adjust anytime)</b>
  <div class="small">Use <b>Center</b> for big correction, <b>Trim</b> for small correction.</div>

  <div style="margin-top:10px;">
    Center (µs) <span id="centerVal" class="value">2000</span>
    <input id="center" type="range" min="1200" max="2400" value="2000">
  </div>

  <div style="margin-top:10px;">
    Trim <span id="trimVal" class="value">0</span>
    <input id="trim" type="range" min="-100" max="100" value="0">
  </div>

  <div class="row" style="margin-top:10px;">
    <button onclick="recenter()">Recenter (bake trim into center)</button>
  </div>
</div>

<div class="card">
  <b>Pedals setup</b>
  <div class="small">
    Default: <b>B7 throttle</b>, <b>B6 brake</b>.
  </div>

  <div class="row" style="margin-top:10px;">
    <div>
      <label>Pedal Source</label>
      <select id="pedalSource">
        <option value="buttons" selected>Buttons (B6/B7 triggers)</option>
        <option value="axes">Axes</option>
      </select>
    </div>

    <div>
      <label>Throttle Button (B7)</label>
      <input id="gasBtn" type="number" min="0" max="31" value="7">
    </div>

    <div>
      <label>Brake Button (B6)</label>
      <input id="brakeBtn" type="number" min="0" max="31" value="6">
    </div>

    <div>
      <label>Reverse enabled?</label>
      <select id="enableReverse">
        <option value="yes" selected>Yes (brake -> reverse)</option>
        <option value="no">No (throttle only)</option>
      </select>
    </div>
  </div>

  <div class="row" style="margin-top:10px;">
    <div>
      <label>Throttle Axis (if using Axes)</label>
      <input id="gasAxis" type="number" min="0" max="15" value="2">
    </div>
    <div>
      <label>Brake Axis (if using Axes)</label>
      <input id="brakeAxis" type="number" min="0" max="15" value="5">
    </div>
    <div>
      <label><input id="invGas" type="checkbox"> Invert Throttle</label>
    </div>
    <div>
      <label><input id="invBrake" type="checkbox"> Invert Brake</label>
    </div>
  </div>

  <div class="small mono" id="pedalStatus" style="margin-top:10px;"></div>
</div>

<div class="card">
  <b>Throttle sensitivity</b>
  <div class="small">
    Deadband ignores tiny movement, Expo makes low end gentler, Min Start helps overcome resistance.
  </div>

  <div class="row" style="margin-top:10px;">
    <div>
      Deadband (%) <span id="dbVal" class="value">5</span>
      <input id="deadband" type="range" min="0" max="20" value="5">
    </div>
    <div>
      Expo (×10) <span id="expoVal" class="value">20</span>
      <input id="expo10" type="range" min="10" max="40" value="20">
    </div>
    <div>
      Min Start (%) <span id="msVal" class="value">18</span>
      <input id="minstart" type="range" min="0" max="40" value="18">
    </div>
  </div>
</div>

<div class="card">
  Throttle Strength (keyboard) <span id="powVal" class="value">70</span>
  <input id="power" type="range" min="10" max="100" value="70">
</div>

<div class="card">
  <b>Debug</b>
  <label><input id="dbg" type="checkbox"> Print raw axes/buttons to console</label>
</div>

<script>
const STEER_TRIM_US_PER_UNIT_JS = 3;
const SEND_INTERVAL_MS = 40;
const STEER_WHEEL_DEADZONE = 0.08;
const STEER_ZERO_LOCK = 2;
const STEER_FILTER_ALPHA = 0.25;

const thEl = document.getElementById("th");
const stEl = document.getElementById("st");
const trimEl = document.getElementById("trim");
const centerEl = document.getElementById("center");
const powerEl = document.getElementById("power");
const deadbandEl = document.getElementById("deadband");
const expo10El = document.getElementById("expo10");
const minStartEl = document.getElementById("minstart");
const dbgEl = document.getElementById("dbg");
const gpSelectEl = document.getElementById("gpSelect");
const gpInfoEl = document.getElementById("gpInfo");
const pedalSourceEl = document.getElementById("pedalSource");
const gasBtnEl = document.getElementById("gasBtn");
const brakeBtnEl = document.getElementById("brakeBtn");
const enableReverseEl = document.getElementById("enableReverse");
const gasAxisEl = document.getElementById("gasAxis");
const brakeAxisEl = document.getElementById("brakeAxis");
const invGasEl = document.getElementById("invGas");
const invBrakeEl = document.getElementById("invBrake");
const pedalStatusEl = document.getElementById("pedalStatus");
const thValEl = document.getElementById("thVal");
const stValEl = document.getElementById("stVal");
const trimValEl = document.getElementById("trimVal");
const centerValEl = document.getElementById("centerVal");
const powValEl = document.getElementById("powVal");
const dbValEl = document.getElementById("dbVal");
const expoValEl = document.getElementById("expoVal");
const msValEl = document.getElementById("msVal");
const stopBtnEl = document.getElementById("stopBtn");

let throttle = 0;
let steer = 0;
let trim = 0;
let centerUs = 2000;
let power = 70;

const keys = {};
let lastSend = 0;
let selectedGamepadIndex = null;
let stopLatched = false;
let filteredWheel = 0;
let hasSentState = false;
let lastSentThrottle = 0;
let lastSentSteer = 0;
let lastSentTrim = 0;
let lastSentCenterUs = 2000;

let pedalSource = "buttons";
let gasBtnIndex = 7;
let brakeBtnIndex = 6;
let enableReverse = true;

let gasAxisIndex = 2;
let brakeAxisIndex = 5;
let invertGas = false;
let invertBrake = false;

let TH_DEADBAND = 5;
let TH_EXPO = 2.0;
let TH_MIN_START = 18;

function clamp(v, lo, hi){ return Math.max(lo, Math.min(hi, v)); }
function deadzone(v, dz){ return Math.abs(v) < dz ? 0 : v; }

function toInt(value, fallback) {
  const parsed = parseInt(value, 10);
  return Number.isNaN(parsed) ? fallback : parsed;
}

function shapeSignedThrottlePct(raw) {
  raw = clamp(Math.round(raw), -100, 100);
  const absRaw = Math.abs(raw);
  if (absRaw <= TH_DEADBAND) return 0;

  const sign = raw < 0 ? -1 : 1;
  let scaled = (absRaw - TH_DEADBAND) / (100 - TH_DEADBAND);
  scaled = clamp(scaled, 0, 1);

  let shaped = Math.pow(scaled, TH_EXPO);
  let out = Math.round(shaped * 100);

  if (TH_MIN_START > 0 && out > 0 && out < TH_MIN_START) out = TH_MIN_START;
  return sign * clamp(out, 0, 100);
}

function buttonToPct(gp, idx, invertFlag){
  const button = gp.buttons && gp.buttons[idx] ? gp.buttons[idx].value : 0;
  let pct = clamp(Math.round(button * 100), 0, 100);
  if (invertFlag) pct = 100 - pct;
  return pct;
}

function axisToPctAdaptive(value, invertFlag){
  if (value === undefined || value === null || Number.isNaN(value)) return 0;
  let pct;
  if (value >= 0 && value <= 1) pct = value * 100;
  else pct = ((value + 1) * 0.5) * 100;
  pct = clamp(Math.round(pct), 0, 100);
  if (invertFlag) pct = 100 - pct;
  return pct;
}

function updateUI() {
  thValEl.innerText = throttle;
  stValEl.innerText = steer;
  trimValEl.innerText = trim;
  centerValEl.innerText = centerUs;
  powValEl.innerText = power;
  dbValEl.innerText = TH_DEADBAND;
  expoValEl.innerText = Math.round(TH_EXPO * 10);
  msValEl.innerText = TH_MIN_START;
}

function syncStopButton() {
  stopBtnEl.innerText = stopLatched ? "RESUME" : "STOP";
}

function releaseStopLatch() {
  if (!stopLatched) return;
  stopLatched = false;
  syncStopButton();
}

function send(extraQuery = "", force = false) {
  const stateChanged =
    !hasSentState ||
    throttle !== lastSentThrottle ||
    steer !== lastSentSteer ||
    trim !== lastSentTrim ||
    centerUs !== lastSentCenterUs ||
    extraQuery !== "";

  if (!force && !stateChanged) return;

  const now = performance.now();
  if (!force && (now - lastSend) < SEND_INTERVAL_MS) return;
  lastSend = now;

  const query =
    "/cmd?th=" + throttle +
    "&st=" + steer +
    "&trim=" + trim +
    "&center=" + centerUs +
    extraQuery;

  hasSentState = true;
  lastSentThrottle = throttle;
  lastSentSteer = steer;
  lastSentTrim = trim;
  lastSentCenterUs = centerUs;

  fetch(query).catch(err => console.log("fetch error:", err));
}

function applyStoppedState(forceSend) {
  throttle = 0;
  steer = 0;
  filteredWheel = 0;
  thEl.value = "0";
  stEl.value = "0";
  updateUI();
  if (forceSend) send("", true);
}

function stopNow() {
  stopLatched = !stopLatched;
  syncStopButton();

  if (stopLatched) {
    applyStoppedState(true);
  } else {
    send("", true);
  }
}

function recenter() {
  centerUs = clamp(centerUs + trim * STEER_TRIM_US_PER_UNIT_JS, 1200, 2400);
  trim = 0;
  centerEl.value = String(centerUs);
  trimEl.value = "0";
  updateUI();
  send("&recenter=1", true);
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
  trim = toInt(event.target.value, 0);
  updateUI();
  send();
};

centerEl.oninput = (event) => {
  centerUs = toInt(event.target.value, 2000);
  updateUI();
  send();
};

powerEl.oninput = (event) => {
  power = toInt(event.target.value, 70);
  updateUI();
};

deadbandEl.oninput = (event) => {
  TH_DEADBAND = toInt(event.target.value, 5);
  updateUI();
};

expo10El.oninput = (event) => {
  TH_EXPO = toInt(event.target.value, 20) / 10.0;
  updateUI();
};

minStartEl.oninput = (event) => {
  TH_MIN_START = toInt(event.target.value, 18);
  updateUI();
};

function syncPedalVars() {
  pedalSource = pedalSourceEl.value;
  gasBtnIndex = toInt(gasBtnEl.value, 7);
  brakeBtnIndex = toInt(brakeBtnEl.value, 6);
  enableReverse = (enableReverseEl.value === "yes");
  gasAxisIndex = toInt(gasAxisEl.value, 2);
  brakeAxisIndex = toInt(brakeAxisEl.value, 5);
  invertGas = invGasEl.checked;
  invertBrake = invBrakeEl.checked;

  pedalStatusEl.innerText =
    "Pedals: source=" + pedalSource +
    " | throttle=" + (pedalSource === "buttons" ? ("B" + gasBtnIndex) : ("axis " + gasAxisIndex)) +
    " | brake=" + (pedalSource === "buttons" ? ("B" + brakeBtnIndex) : ("axis " + brakeAxisIndex)) +
    " | reverse=" + (enableReverse ? "ON" : "OFF");
}

["pedalSource", "gasBtn", "brakeBtn", "enableReverse", "gasAxis", "brakeAxis", "invGas", "invBrake"].forEach((id) => {
  document.getElementById(id).addEventListener("change", syncPedalVars);
});

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

function refreshGamepads() {
  gpSelectEl.innerHTML = "";
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  const connectedPads = [];

  for (let i = 0; i < pads.length; i++) {
    const pad = pads[i];
    if (!pad) continue;

    connectedPads.push(pad);

    const option = document.createElement("option");
    option.value = String(pad.index);
    option.text = "[" + pad.index + "] " + pad.id;
    gpSelectEl.appendChild(option);
  }

  if (connectedPads.length === 0) {
    const option = document.createElement("option");
    option.value = "";
    option.text = "No controllers detected";
    gpSelectEl.appendChild(option);
    selectedGamepadIndex = null;
  } else {
    const selectedStillPresent = connectedPads.some((pad) => pad.index === selectedGamepadIndex);
    if (!selectedStillPresent) {
      selectedGamepadIndex = connectedPads[0].index;
    }
    gpSelectEl.value = String(selectedGamepadIndex);
  }

  updateGpInfo();
}

function updateGpInfo() {
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  const gp = (selectedGamepadIndex !== null) ? pads[selectedGamepadIndex] : null;
  gpInfoEl.innerText = gp ? ("(using: " + gp.id + " @ index " + gp.index + ")") : "(no controller)";
}

gpSelectEl.addEventListener("change", (event) => {
  selectedGamepadIndex = event.target.value === "" ? null : toInt(event.target.value, 0);
  filteredWheel = 0;
  updateGpInfo();
});

window.addEventListener("gamepadconnected", refreshGamepads);
window.addEventListener("gamepaddisconnected", refreshGamepads);

let WHEEL_AXIS_INDEX = 0;

function poll() {
  if (stopLatched) {
    filteredWheel = 0;
    requestAnimationFrame(poll);
    return;
  }

  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  const gp = (selectedGamepadIndex !== null) ? pads[selectedGamepadIndex] : null;
  if (!gp) {
    filteredWheel = 0;
    requestAnimationFrame(poll);
    return;
  }

  if (dbgEl.checked) {
    const axes = (gp.axes || []).map((value) => Number(value.toFixed(3)));
    const buttons = (gp.buttons || []).map((button) => Number(button.value.toFixed(2)));
    console.log("axes:", axes, "buttons:", buttons);
  }

  let wheel = gp.axes[WHEEL_AXIS_INDEX] ?? 0;
  wheel = deadzone(wheel, STEER_WHEEL_DEADZONE);
  filteredWheel += (wheel - filteredWheel) * STEER_FILTER_ALPHA;
  if (Math.abs(filteredWheel) < 0.01) filteredWheel = 0;

  steer = clamp(Math.round(filteredWheel * 100), -100, 100);
  if (Math.abs(steer) <= STEER_ZERO_LOCK) steer = 0;

  syncPedalVars();

  let raw = 0;
  if (pedalSource === "buttons") {
    const gasPct = buttonToPct(gp, gasBtnIndex, invertGas);
    const brakePct = buttonToPct(gp, brakeBtnIndex, invertBrake);
    raw = enableReverse ? (gasPct - brakePct) : gasPct;
  } else {
    const gasPct = axisToPctAdaptive(gp.axes[gasAxisIndex], invertGas);
    const brakePct = axisToPctAdaptive(gp.axes[brakeAxisIndex], invertBrake);
    raw = enableReverse ? (gasPct - brakePct) : gasPct;
  }

  const shaped = shapeSignedThrottlePct(raw);
  throttle = enableReverse ? shaped : clamp(shaped, 0, 100);

  stEl.value = String(steer);
  thEl.value = String(throttle);
  updateUI();
  send();

  requestAnimationFrame(poll);
}

syncPedalVars();
syncStopButton();
refreshGamepads();
updateUI();
poll();
</script>

</body>
</html>
)HTML";
