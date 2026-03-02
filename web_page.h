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
      <label for="steeringTrim">Steering Trim</label>
      <input id="steeringTrim" type="number" min="-100" max="100" value="0">

      <label for="steerCenterUs">Steering Center (us)</label>
      <input id="steerCenterUs" type="number" min="1200" max="2400" value="2000">

      <button onclick="saveCalibration()">Save Calibration</button>
      <button class="alt" onclick="clearEstop()">Clear E-Stop</button>
      <button class="warn" onclick="triggerEstop()">Trigger E-Stop</button>
      <div class="status" id="calMsg"></div>
    </section>

    <section class="card">
      <h2>Debug Drive</h2>
      <p class="tiny">This is a bench-only direct HTTP control path. It is not the primary driving mode.</p>

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

const ssidEl = document.getElementById("ssid");
const passwordEl = document.getElementById("password");
const vehicleNameEl = document.getElementById("vehicleName");
const sharedKeyEl = document.getElementById("sharedKey");
const fallbackApEl = document.getElementById("fallbackAp");
const steeringTrimEl = document.getElementById("steeringTrim");
const steerCenterUsEl = document.getElementById("steerCenterUs");
const debugThrottleEl = document.getElementById("debugThrottle");
const debugSteerEl = document.getElementById("debugSteer");
const debugThrottleValueEl = document.getElementById("debugThrottleValue");
const debugSteerValueEl = document.getElementById("debugSteerValue");
let debugRepeatId = null;

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

async function refreshAll() {
  try {
    const [status, config] = await Promise.all([
      getJson("/api/status"),
      getJson("/api/config"),
    ]);

    renderStatus(status);
    ssidEl.value = config.ssid || "";
    vehicleNameEl.value = config.vehicle_name || "";
    fallbackApEl.checked = !!config.fallback_ap_enabled;
    steeringTrimEl.value = config.steering_trim ?? 0;
    steerCenterUsEl.value = config.steer_center_us ?? 2000;
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
    sharedKeyEl.value = "";
    networkMsgEl.textContent = "Network config saved. The ESP32 is reconnecting.";
    setTimeout(refreshAll, 1200);
  } catch (error) {
    networkMsgEl.textContent = "Save failed: " + error.message;
  }
}

async function saveCalibration() {
  calMsgEl.textContent = "Saving...";
  try {
    await postJson("/api/config", {
      steering_trim: Number(steeringTrimEl.value),
      steer_center_us: Number(steerCenterUsEl.value),
    });
    calMsgEl.textContent = "Calibration saved.";
    refreshAll();
  } catch (error) {
    calMsgEl.textContent = "Save failed: " + error.message;
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
    await fetch(url, { cache: "no-store" });
    debugMsgEl.textContent = "Debug command sent.";

    if (debugRepeatId !== null) {
      clearInterval(debugRepeatId);
      debugRepeatId = null;
    }

    if (throttle !== 0 || steer !== 0) {
      debugRepeatId = setInterval(() => {
        fetch(url, { cache: "no-store" }).catch(() => {});
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
