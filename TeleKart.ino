#include <WiFi.h>

#include "config.h"
#include "app_state.h"
#include "rc_output.h"
#include "web_handlers.h"

void setup() {
  Serial.begin(115200);

  rc_output_init();

  Serial.println("Arming ESC...");
  delay(3000);

  WiFi.softAP(AP_SSID, AP_PASS);

  Serial.print("Connect to WiFi: ");
  Serial.println(AP_SSID);
  Serial.print("Open: http://");
  Serial.println(WiFi.softAPIP());

  web_handlers_init();

  lastCommandMs = millis();
}

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
