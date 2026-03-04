#pragma once

#include <pgmspace.h>

static const char MAIN_PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>TeleKart Setup</title>
<style>
  :root {
    color-scheme: light;
    --bg: #f2efe8;
    --card: #fffaf0;
    --line: #d4c7ad;
    --ink: #1d2428;
    --muted: #5f6b72;
    --accent: #0f6a73;
    --warn: #a63c1e;
  }
  body {
    margin: 0;
    background: radial-gradient(circle at top left, #fff9ec, var(--bg) 55%);
    color: var(--ink);
    font-family: "Avenir Next", "Segoe UI", sans-serif;
  }
  main {
    max-width: 980px;
    margin: 0 auto;
    padding: 24px;
  }
  h1 {
    margin: 0 0 8px;
    font-size: 2rem;
  }
  p {
    color: var(--muted);
  }
  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
    gap: 16px;
  }
  .card {
    background: var(--card);
    border: 1px solid var(--line);
    border-radius: 16px;
    padding: 18px;
    box-shadow: 0 10px 30px rgba(46, 56, 61, 0.08);
  }
  .card h2 {
    margin-top: 0;
    font-size: 1.1rem;
  }
  label {
    display: block;
    margin: 12px 0 4px;
    font-size: 0.92rem;
    color: var(--muted);
  }
  input, textarea, button {
    width: 100%;
    box-sizing: border-box;
    border-radius: 12px;
    border: 1px solid var(--line);
    padding: 10px 12px;
    font: inherit;
  }
  input, textarea {
    background: #fffdf7;
  }
  button {
    margin-top: 12px;
    cursor: pointer;
    background: var(--accent);
    border: 0;
    color: white;
    font-weight: 600;
  }
  button.alt {
    background: #3f4d54;
  }
  button.warn {
    background: var(--warn);
  }
  .row {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
  }
  .tiny {
    font-size: 0.82rem;
    color: var(--muted);
  }
  pre {
    background: #12181b;
    color: #d9edf1;
    border-radius: 14px;
    padding: 12px;
    overflow: auto;
    min-height: 180px;
  }
  .status {
    margin-top: 12px;
    color: var(--muted);
    min-height: 1.2em;
  }
  .metric {
    margin-top: 10px;
  }
  .metric-head {
    display: flex;
    justify-content: space-between;
    align-items: baseline;
    font-size: 0.85rem;
    color: var(--muted);
    margin-bottom: 4px;
  }
  .meter {
    height: 10px;
    border-radius: 999px;
    background: #e6dcc9;
    overflow: hidden;
  }
  .meter-fill {
    height: 100%;
    width: 0%;
    background: var(--accent);
    transition: width 120ms linear;
  }
  .state-row {
    display: flex;
    gap: 10px;
    flex-wrap: wrap;
    margin-top: 12px;
  }
  .badge {
    padding: 4px 10px;
    border-radius: 999px;
    background: #e9e2d2;
    color: var(--ink);
    font-size: 0.82rem;
    font-weight: 600;
  }
  .badge.on {
    background: var(--accent);
    color: white;
  }
  .badge.warn {
    background: var(--warn);
    color: white;
  }
  @media (max-width: 640px) {
    .row {
      grid-template-columns: 1fr;
    }
  }
</style>
</head>
<body>
<main>
  <h1>TeleKart Control Plane</h1>
  <p>This page is for setup, calibration, diagnostics, and emergency override. Primary driving is handled by the native controller app over CMU-DEVICE.</p>

  <div class="grid">
    <section class="card">
      <h2>Status</h2>
      <button class="alt" onclick="refreshAll()">Refresh Now</button>
      <pre id="statusView">Loading...</pre>
      <div class="status" id="statusMsg"></div>
    </section>

    <section class="card">
      <h2>Drive Monitor</h2>
      <div class="metric">
        <div class="metric-head"><span>Throttle Pedal Input</span><span id="throttleCmdValue">0.0%</span></div>
        <div class="meter"><div class="meter-fill" id="throttleCmdBar"></div></div>
      </div>
      <div class="metric">
        <div class="metric-head"><span>Brake Pedal Input</span><span id="brakeCmdValue">0.0%</span></div>
        <div class="meter"><div class="meter-fill" id="brakeCmdBar"></div></div>
      </div>
      <div class="metric">
        <div class="metric-head"><span>Steering Command</span><span id="steerCmdValue">0.0%</span></div>
        <div class="meter"><div class="meter-fill" id="steerCmdBar"></div></div>
      </div>
      <div class="metric">
        <div class="metric-head"><span>Throttle Output</span><span id="throttleOutValue">0.0%</span></div>
        <div class="meter"><div class="meter-fill" id="throttleOutBar"></div></div>
      </div>
      <div class="state-row">
        <span class="badge" id="reverseReqBadge">Reverse Requested: OFF</span>
        <span class="badge" id="reverseDriveBadge">Reverse Engaged: OFF</span>
        <span class="badge" id="estopReqBadge">E-Stop Request: OFF</span>
      </div>
    </section>

    <section class="card">
      <h2>Network</h2>
      <label for="ssid">SSID</label>
      <input id="ssid" value="CMU-DEVICE">

      <label for="password">Password / Token</label>
      <input id="password" type="password" placeholder="Leave unchanged to keep current value">

      <label for="vehicleName">Vehicle Name</label>
      <input id="vehicleName" value="telekart-01">

      <label for="sharedKey">Shared Auth Key</label>
      <input id="sharedKey" placeholder="Required for pairing">

      <label><input id="fallbackAp" type="checkbox" checked style="width:auto;margin-right:8px;">Enable fallback setup AP</label>

      <button onclick="saveNetwork()">Save Network Config</button>
      <div class="status" id="networkMsg"></div>
    </section>

    <section class="card">
      <h2>Calibration</h2>
      <p class="tiny">Center shifts the steering neutral point in microseconds. Trim is a quick fine offset around that center.</p>
      <label for="steeringTrim">Steering Trim</label>
      <input id="steeringTrim" type="number" min="-100" max="100" value="0">

      <label for="steerCenterUs">Steering Center (us)</label>
      <input id="steerCenterUs" type="number" min="1200" max="2400" value="2000">

      <div class="row">
        <div>
          <label for="steerLeftRangePct">Left Range (%)</label>
          <input id="steerLeftRangePct" type="number" min="0" max="180" value="100">
        </div>
        <div>
          <label for="steerRightRangePct">Right Range (%)</label>
          <input id="steerRightRangePct" type="number" min="0" max="180" value="100">
        </div>
      </div>
      <div class="tiny">Range above 100% pushes each side to full lock sooner and can help compensate side-to-side mismatch.</div>

      <button onclick="saveCalibration()">Save Calibration</button>
      <button class="alt" onclick="recenterCalibration()">Recenter (apply trim into center)</button>
      <button class="alt" onclick="centerWheelsNow()">Center Wheels Now</button>
      <button class="alt" onclick="clearEstop()">Clear E-Stop</button>
      <button class="warn" onclick="triggerEstop()">Trigger E-Stop</button>
      <div class="status" id="calMsg"></div>
    </section>

    <section class="card">
      <h2>Debug Drive</h2>
      <p class="tiny">This is a bench-only direct HTTP control path. It is not the primary driving mode, and it is blocked while a paired controller session is active.</p>

      <div class="row">
        <div>
          <label for="debugThrottle">Throttle (%)</label>
          <input id="debugThrottle" type="range" min="-100" max="100" value="0">
        </div>
        <div>
          <label for="debugSteer">Steer (%)</label>
          <input id="debugSteer" type="range" min="-100" max="100" value="0">
        </div>
      </div>

      <div class="row">
        <div class="tiny">Throttle: <span id="debugThrottleValue">0</span></div>
        <div class="tiny">Steer: <span id="debugSteerValue">0</span></div>
      </div>

      <button onclick="sendDebug()">Send Debug Command</button>
      <button class="alt" onclick="stopDebug()">Stop Debug Output</button>
      <div class="status" id="debugMsg"></div>
    </section>
  </div>
</main>

<script>
const statusViewEl = document.getElementById("statusView");
const statusMsgEl = document.getElementById("statusMsg");
const networkMsgEl = document.getElementById("networkMsg");
const calMsgEl = document.getElementById("calMsg");
const debugMsgEl = document.getElementById("debugMsg");
const throttleCmdValueEl = document.getElementById("throttleCmdValue");
const throttleCmdBarEl = document.getElementById("throttleCmdBar");
const brakeCmdValueEl = document.getElementById("brakeCmdValue");
const brakeCmdBarEl = document.getElementById("brakeCmdBar");
const steerCmdValueEl = document.getElementById("steerCmdValue");
const steerCmdBarEl = document.getElementById("steerCmdBar");
const throttleOutValueEl = document.getElementById("throttleOutValue");
const throttleOutBarEl = document.getElementById("throttleOutBar");
const reverseReqBadgeEl = document.getElementById("reverseReqBadge");
const reverseDriveBadgeEl = document.getElementById("reverseDriveBadge");
const estopReqBadgeEl = document.getElementById("estopReqBadge");

const ssidEl = document.getElementById("ssid");
const passwordEl = document.getElementById("password");
const vehicleNameEl = document.getElementById("vehicleName");
const sharedKeyEl = document.getElementById("sharedKey");
const fallbackApEl = document.getElementById("fallbackAp");
const steeringTrimEl = document.getElementById("steeringTrim");
const steerCenterUsEl = document.getElementById("steerCenterUs");
const steerLeftRangePctEl = document.getElementById("steerLeftRangePct");
const steerRightRangePctEl = document.getElementById("steerRightRangePct");
const debugThrottleEl = document.getElementById("debugThrottle");
const debugSteerEl = document.getElementById("debugSteer");
const debugThrottleValueEl = document.getElementById("debugThrottleValue");
const debugSteerValueEl = document.getElementById("debugSteerValue");
let debugRepeatId = null;
const STEER_TRIM_US_PER_UNIT = 3;

async function getJson(url) {
  const response = await fetch(url, { cache: "no-store" });
  if (!response.ok) {
    throw new Error(await response.text());
  }
  return response.json();
}

async function postJson(url, payload) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });

  if (!response.ok) {
    throw new Error(await response.text());
  }

  return response.json();
}

function renderStatus(status) {
  statusViewEl.textContent = JSON.stringify(status, null, 2);
}

function syncDebugLabels() {
  debugThrottleValueEl.textContent = debugThrottleEl.value;
  debugSteerValueEl.textContent = debugSteerEl.value;
}

function syncTextInput(input, value) {
  if (document.activeElement !== input) {
    input.value = value;
  }
}

function syncCheckbox(input, checked) {
  if (document.activeElement !== input) {
    input.checked = !!checked;
  }
}

function clampInt(value, lo, hi) {
  return Math.max(lo, Math.min(hi, value));
}

function clampFloat(value, lo, hi) {
  return Math.max(lo, Math.min(hi, value));
}

function setUnsignedMeter(barEl, valueEl, valuePct) {
  const clamped = clampFloat(valuePct, 0, 100);
  barEl.style.width = clamped.toFixed(1) + "%";
  barEl.style.background = "var(--accent)";
  valueEl.textContent = clamped.toFixed(1) + "%";
}

function setSignedMeter(barEl, valueEl, valuePct) {
  const clamped = clampFloat(valuePct, -100, 100);
  barEl.style.width = Math.abs(clamped).toFixed(1) + "%";
  barEl.style.background = clamped < 0 ? "var(--warn)" : "var(--accent)";
  valueEl.textContent = clamped.toFixed(1) + "%";
}

function setBadge(el, label, on, warn) {
  el.textContent = label + ": " + (on ? "ON" : "OFF");
  el.classList.toggle("on", !!on && !warn);
  el.classList.toggle("warn", !!on && !!warn);
}

function updateDriveMonitor(status) {
  setUnsignedMeter(throttleCmdBarEl, throttleCmdValueEl, Number(status.input_throttle_cmd_pct ?? 0));
  setUnsignedMeter(brakeCmdBarEl, brakeCmdValueEl, Number(status.input_brake_cmd_pct ?? 0));
  setSignedMeter(steerCmdBarEl, steerCmdValueEl, Number(status.input_steer_cmd_pct ?? 0));
  setSignedMeter(throttleOutBarEl, throttleOutValueEl, Number(status.current_throttle_pct ?? 0));
  setBadge(reverseReqBadgeEl, "Reverse Requested", !!status.reverse_requested, false);
  setBadge(reverseDriveBadgeEl, "Reverse Engaged", !!status.reverse_drive, false);
  setBadge(estopReqBadgeEl, "E-Stop Request", !!status.estop_requested, true);
}

function calibrationRangeValue(input, fallback) {
  const parsed = Number(input.value);
  return clampInt(Number.isFinite(parsed) ? parsed : fallback, 0, 180);
}

async function refreshAll() {
  try {
    const [status, config] = await Promise.all([
      getJson("/api/status"),
      getJson("/api/config"),
    ]);

    renderStatus(status);
    updateDriveMonitor(status);
    syncTextInput(ssidEl, config.ssid || "");
    syncTextInput(vehicleNameEl, config.vehicle_name || "");
    syncTextInput(sharedKeyEl, config.shared_key || "");
    syncCheckbox(fallbackApEl, config.fallback_ap_enabled);
    syncTextInput(steeringTrimEl, String(config.steering_trim ?? 0));
    syncTextInput(steerCenterUsEl, String(config.steer_center_us ?? 2000));
    syncTextInput(steerLeftRangePctEl, String(config.steer_left_range_pct ?? 100));
    syncTextInput(steerRightRangePctEl, String(config.steer_right_range_pct ?? 100));
    statusMsgEl.textContent = "Status refreshed at " + new Date().toLocaleTimeString();
  } catch (error) {
    statusMsgEl.textContent = "Refresh failed: " + error.message;
  }
}

async function saveNetwork() {
  networkMsgEl.textContent = "Saving...";
  try {
    await postJson("/api/network", {
      ssid: ssidEl.value,
      password: passwordEl.value,
      vehicle_name: vehicleNameEl.value,
      shared_key: sharedKeyEl.value,
      fallback_ap_enabled: fallbackApEl.checked,
    });
    passwordEl.value = "";
    networkMsgEl.textContent = "Network config saved. The ESP32 is reconnecting.";
    setTimeout(refreshAll, 1200);
  } catch (error) {
    networkMsgEl.textContent = "Save failed: " + error.message;
  }
}

async function saveCalibration() {
  calMsgEl.textContent = "Saving...";
  try {
    const leftRange = calibrationRangeValue(steerLeftRangePctEl, 100);
    const rightRange = calibrationRangeValue(steerRightRangePctEl, 100);
    steerLeftRangePctEl.value = String(leftRange);
    steerRightRangePctEl.value = String(rightRange);
    await postJson("/api/config", {
      steering_trim: Number(steeringTrimEl.value),
      steer_center_us: Number(steerCenterUsEl.value),
      steer_left_range_pct: leftRange,
      steer_right_range_pct: rightRange,
    });
    calMsgEl.textContent = "Calibration saved.";
    refreshAll();
  } catch (error) {
    calMsgEl.textContent = "Save failed: " + error.message;
  }
}

async function recenterCalibration() {
  calMsgEl.textContent = "Recentering...";
  const trim = Number(steeringTrimEl.value);
  const center = Number(steerCenterUsEl.value);
  const safeTrim = Number.isFinite(trim) ? trim : 0;
  const safeCenter = Number.isFinite(center) ? center : 2000;
  const nextCenter = clampInt(
    Math.round(safeCenter + (safeTrim * STEER_TRIM_US_PER_UNIT)),
    1200,
    2400
  );
  const leftRange = calibrationRangeValue(steerLeftRangePctEl, 100);
  const rightRange = calibrationRangeValue(steerRightRangePctEl, 100);
  steerLeftRangePctEl.value = String(leftRange);
  steerRightRangePctEl.value = String(rightRange);

  try {
    await postJson("/api/config", {
      steering_trim: 0,
      steer_center_us: nextCenter,
      steer_left_range_pct: leftRange,
      steer_right_range_pct: rightRange,
    });
    steeringTrimEl.value = "0";
    steerCenterUsEl.value = String(nextCenter);
    calMsgEl.textContent = "Recentered and saved.";
    refreshAll();
  } catch (error) {
    calMsgEl.textContent = "Recenter failed: " + error.message;
  }
}

async function centerWheelsNow() {
  try {
    await postJson("/api/center_steering", {});
    calMsgEl.textContent = "Center command sent.";
  } catch (error) {
    calMsgEl.textContent = "Center failed: " + error.message;
  }
}

async function triggerEstop() {
  try {
    await postJson("/api/estop", { clear: false });
    calMsgEl.textContent = "E-stop latched.";
    refreshAll();
  } catch (error) {
    calMsgEl.textContent = "E-stop failed: " + error.message;
  }
}

async function clearEstop() {
  try {
    await postJson("/api/estop", { clear: true });
    calMsgEl.textContent = "E-stop cleared.";
    refreshAll();
  } catch (error) {
    calMsgEl.textContent = "Clear failed: " + error.message;
  }
}

async function sendDebug() {
  try {
    const throttle = Number(debugThrottleEl.value);
    const steer = Number(debugSteerEl.value);
    const url = "/cmd?th=" + encodeURIComponent(throttle) + "&st=" + encodeURIComponent(steer);
    const response = await fetch(url, { cache: "no-store" });
    if (!response.ok) {
      throw new Error(await response.text());
    }
    debugMsgEl.textContent = "Debug command sent.";

    if (debugRepeatId !== null) {
      clearInterval(debugRepeatId);
      debugRepeatId = null;
    }

    if (throttle !== 0 || steer !== 0) {
      debugRepeatId = setInterval(() => {
        fetch(url, { cache: "no-store" })
          .then((repeatResponse) => {
            if (!repeatResponse.ok && debugRepeatId !== null) {
              clearInterval(debugRepeatId);
              debugRepeatId = null;
            }
          })
          .catch(() => {});
      }, 80);
    }
  } catch (error) {
    debugMsgEl.textContent = "Debug send failed: " + error.message;
  }
}

async function stopDebug() {
  if (debugRepeatId !== null) {
    clearInterval(debugRepeatId);
    debugRepeatId = null;
  }
  debugThrottleEl.value = "0";
  debugSteerEl.value = "0";
  syncDebugLabels();
  await sendDebug();
}

debugThrottleEl.addEventListener("input", syncDebugLabels);
debugSteerEl.addEventListener("input", syncDebugLabels);

syncDebugLabels();
refreshAll();
setInterval(refreshAll, 1000);
</script>
</body>
</html>
)HTML";
