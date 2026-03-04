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
    color-scheme: dark;
    --bg0: #05070d;
    --bg1: #0b1220;
    --bg2: #101726;
    --card: rgba(14, 21, 36, 0.86);
    --line: rgba(129, 157, 198, 0.26);
    --ink: #eaf3ff;
    --muted: #95a8c5;
    --accent: #00d9ff;
    --accent-2: #68ff94;
    --warn: #ff5f6d;
    --amber: #ffb347;
    --shadow: 0 18px 40px rgba(0, 0, 0, 0.45);
  }
  body {
    margin: 0;
    min-height: 100vh;
    background:
      radial-gradient(1400px 600px at -12% -18%, rgba(0, 217, 255, 0.16), transparent 62%),
      radial-gradient(1200px 540px at 110% -10%, rgba(255, 179, 71, 0.15), transparent 64%),
      linear-gradient(160deg, var(--bg0), var(--bg1) 42%, var(--bg2));
    color: var(--ink);
    font-family: "Rajdhani", "Bahnschrift", "Trebuchet MS", sans-serif;
  }
  main {
    max-width: 1240px;
    margin: 0 auto;
    padding: 22px;
  }
  h1 {
    margin: 0;
    font-size: 2.2rem;
    letter-spacing: 0.06em;
    text-transform: uppercase;
  }
  p {
    color: var(--muted);
    margin-top: 8px;
  }
  .hero {
    display: grid;
    grid-template-columns: 2fr 1fr;
    gap: 14px;
    margin-bottom: 16px;
  }
  .hero-card {
    border: 1px solid var(--line);
    border-radius: 18px;
    padding: 16px 18px;
    background: linear-gradient(145deg, rgba(12, 18, 31, 0.92), rgba(8, 13, 23, 0.88));
    box-shadow: var(--shadow);
  }
  .hero-note {
    color: var(--muted);
    font-size: 0.9rem;
  }
  .hero-badges {
    display: flex;
    gap: 8px;
    flex-wrap: wrap;
    margin-top: 10px;
  }
  .hero-badge {
    border: 1px solid var(--line);
    background: rgba(0, 217, 255, 0.08);
    color: var(--ink);
    font-size: 0.78rem;
    letter-spacing: 0.04em;
    text-transform: uppercase;
    border-radius: 999px;
    padding: 5px 10px;
  }
  .grid {
    display: grid;
    grid-template-columns: 1.2fr 1fr;
    gap: 14px;
  }
  .card {
    background: var(--card);
    border: 1px solid var(--line);
    border-radius: 16px;
    padding: 18px;
    box-shadow: var(--shadow);
    backdrop-filter: blur(4px);
  }
  .card h2 {
    margin-top: 0;
    margin-bottom: 10px;
    font-size: 1.08rem;
    letter-spacing: 0.05em;
    text-transform: uppercase;
  }
  label {
    display: block;
    margin: 12px 0 4px;
    font-size: 0.9rem;
    letter-spacing: 0.03em;
    text-transform: uppercase;
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
    background: rgba(8, 14, 25, 0.88);
    color: var(--ink);
  }
  button {
    margin-top: 12px;
    cursor: pointer;
    background: linear-gradient(90deg, #00b8ff, #00d9ff 52%, #3de8ff);
    border: 0;
    color: #031019;
    font-weight: 700;
    letter-spacing: 0.04em;
    text-transform: uppercase;
    transition: transform 120ms ease, filter 120ms ease;
  }
  button:hover {
    transform: translateY(-1px);
    filter: brightness(1.08);
  }
  button.alt {
    background: linear-gradient(90deg, #293a58, #364f76);
    color: #d8e5ff;
  }
  button.warn {
    background: linear-gradient(90deg, #9f2132, var(--warn));
    color: #ffe8ea;
  }
  .row {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
  }
  .tiny {
    font-size: 0.8rem;
    color: var(--muted);
  }
  pre {
    background: #060b16;
    color: #d5ecff;
    border: 1px solid rgba(84, 126, 173, 0.4);
    border-radius: 14px;
    padding: 12px;
    overflow: auto;
    min-height: 240px;
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
    height: 12px;
    border-radius: 999px;
    background: rgba(68, 89, 122, 0.52);
    overflow: hidden;
    border: 1px solid rgba(122, 144, 178, 0.25);
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
    background: rgba(67, 86, 118, 0.58);
    color: var(--ink);
    font-size: 0.82rem;
    font-weight: 600;
    border: 1px solid rgba(125, 151, 186, 0.26);
  }
  .badge.on {
    background: rgba(0, 217, 255, 0.26);
    color: #d9faff;
  }
  .badge.warn {
    background: var(--warn);
    color: white;
  }
  .stretch {
    grid-column: span 2;
  }
  @media (max-width: 640px) {
    .row {
      grid-template-columns: 1fr;
    }
  }
  @media (max-width: 980px) {
    .hero {
      grid-template-columns: 1fr;
    }
    .grid {
      grid-template-columns: 1fr;
    }
    .stretch {
      grid-column: auto;
    }
  }
</style>
</head>
<body>
<main>
  <section class="hero">
    <div class="hero-card">
      <h1>TeleKart Race Control</h1>
      <p>Live pit-lane console for telemetry, setup, and emergency override. Primary steering/throttle still comes from the native controller app.</p>
      <div class="hero-badges">
        <span class="hero-badge">100 Hz Command Link</span>
        <span class="hero-badge">HMAC Pairing</span>
        <span class="hero-badge">Telemetry Feedback</span>
      </div>
    </div>
    <div class="hero-card">
      <div class="hero-note">Quick Checklist</div>
      <p class="tiny">1. Pair controller first.</p>
      <p class="tiny">2. Confirm pedal monitor bars move as expected.</p>
      <p class="tiny">3. Tune center/range and save calibration once.</p>
    </div>
  </section>

  <div class="grid">
    <section class="card stretch">
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
        <span class="badge" id="noBrakeModeBadge">No-Brake Mode: ON</span>
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
      <label><input id="noBrakeMode" type="checkbox" checked style="width:auto;margin-right:8px;">Brake Pedal Reverses (No Brake Decel)</label>

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
const noBrakeModeBadgeEl = document.getElementById("noBrakeModeBadge");

const ssidEl = document.getElementById("ssid");
const passwordEl = document.getElementById("password");
const vehicleNameEl = document.getElementById("vehicleName");
const sharedKeyEl = document.getElementById("sharedKey");
const fallbackApEl = document.getElementById("fallbackAp");
const steeringTrimEl = document.getElementById("steeringTrim");
const steerCenterUsEl = document.getElementById("steerCenterUs");
const steerLeftRangePctEl = document.getElementById("steerLeftRangePct");
const steerRightRangePctEl = document.getElementById("steerRightRangePct");
const noBrakeModeEl = document.getElementById("noBrakeMode");
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
  setBadge(noBrakeModeBadgeEl, "No-Brake Mode", !!status.no_brake_mode, false);
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
    syncCheckbox(noBrakeModeEl, config.no_brake_mode);
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
      no_brake_mode: noBrakeModeEl.checked,
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
      no_brake_mode: noBrakeModeEl.checked,
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
