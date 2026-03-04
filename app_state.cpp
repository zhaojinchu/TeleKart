#include "app_state.h"

#include "config.h"

WebServer server(HTTP_PORT);
Servo steerServo;
Servo escServo;
WiFiUDP controlUdp;

NetworkConfig networkConfig = {
  String(DEFAULT_STA_SSID),
  String(DEFAULT_STA_PASSWORD),
  String(DEFAULT_VEHICLE_NAME),
  String(DEFAULT_SHARED_KEY),
  DEFAULT_FALLBACK_AP_ON,
  false,
  IPAddress(0, 0, 0, 0),
};

ControlSession controlSession = {
  0,
  0,
  0,
  0,
  0,
  0,
  false,
  IPAddress(0, 0, 0, 0),
};

DriveInput driveInput = {
  0,
  0,
  0,
  0,
  0,
  0,
};

VehicleState vehicleState = {
  0.0f,
  0.0f,
  0.0f,
  0.0f,
  0.0f,
  0,
  false,
  false,
  false,
  0,
};

DiagnosticsState diagnosticsState = {
  false,
  false,
  false,
  false,
  false,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
};

int steeringTrim = 0;
int steerCenterUs = 2000;
int steerLeftRangePct = DEFAULT_STEER_LEFT_RANGE_PCT;
int steerRightRangePct = DEFAULT_STEER_RIGHT_RANGE_PCT;
int appliedSteerUs = 2000;

uint32_t lastCommandMs = 0;
uint32_t lastNetworkAttemptMs = 0;
bool udpListening = false;

volatile int manualThrottlePct = 0;
volatile int manualSteerPct = 0;
bool manualOverrideActive = false;
