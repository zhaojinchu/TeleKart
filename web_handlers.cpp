#include "web_handlers.h"
#include "app_state.h"
#include "config.h"
#include "rc_output.h"
#include "web_page.h"

static void handleRoot() {
  // Send HTML from flash (PROGMEM)
  server.send_P(200, "text/html", MAIN_PAGE);
}

static void handleCmd() {
  if (server.hasArg("th"))
    targetThrottlePct = clampInt(server.arg("th").toInt(), -100, 100);

  if (server.hasArg("st"))
    targetSteerPct = clampInt(server.arg("st").toInt(), -100, 100);

  if (server.hasArg("trim"))
    steeringTrim = clampInt(server.arg("trim").toInt(), -100, 100);

  if (server.hasArg("center"))
    steerCenterUs = clampInt(server.arg("center").toInt(), 1200, 2400);

  if (server.hasArg("recenter")) {
    int r = server.arg("recenter").toInt();
    if (r == 1) {
      steerCenterUs = clampInt(steerCenterUs + steeringTrim * STEER_TRIM_US_PER_UNIT, 1200, 2400);
      steeringTrim = 0;
    }
  }

  lastCommandMs = millis();
  inFailsafe = false;

  applyOutputs(targetThrottlePct, targetSteerPct);

  server.send(200, "text/plain", "OK");
}

static void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

void web_handlers_init() {
  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.onNotFound(handleNotFound);
  server.begin();
}
