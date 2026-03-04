#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>
#include <IPAddress.h>
#include <WebServer.h>
#include <WiFiUdp.h>

#include "control_protocol.h"

struct NetworkConfig {
  String staSsid;
  String staPassword;
  String vehicleName;
  String sharedKey;
  bool fallbackApEnabled;
  bool trustedControllerEnabled;
  IPAddress trustedControllerIp;
};

struct ControlSession {
  uint32_t sessionId;
  uint32_t createdAtMs;
  uint32_t expiresAtMs;
  uint32_t controllerNonce;
  uint32_t lastSequence;
  uint16_t controllerPort;
  bool valid;
  IPAddress controllerIp;
};

struct DriveInput {
  int steeringCmd;   // -1000..1000
  int throttleCmd;   // 0..1000
  int brakeCmd;      // 0..1000
  uint8_t flags;
  uint32_t clientTimeMs;
  uint32_t receivedAtMs;
};

struct VehicleState {
  float virtualSpeedPct;
  float filteredSteerInput;
  float targetSteerPct;
  float currentSteerPct;
  float filteredThrottlePct;
  int currentThrottlePct;
  bool reverseDrive;
  bool reverseArmed;
  bool braking;
  uint32_t reverseArmSinceMs;
};

struct DiagnosticsState {
  bool wifiConnected;
  bool fallbackApActive;
  bool inFailsafe;
  bool estopLatched;
  bool cameraLinkState;
  uint16_t failsafeCount;
  uint16_t droppedPacketCount;
  uint16_t authFailureCount;
  uint32_t telemetrySequence;
  uint32_t lastPacketAgeMs;
  uint32_t lastTelemetryMs;
  uint32_t lastControlLoopMs;
  uint32_t lastOutputLoopMs;
  int wifiRssi;
};

// Shared runtime objects
extern WebServer server;
extern Servo steerServo;
extern Servo escServo;
extern WiFiUDP controlUdp;

// Shared configs and state
extern NetworkConfig networkConfig;
extern ControlSession controlSession;
extern DriveInput driveInput;
extern VehicleState vehicleState;
extern DiagnosticsState diagnosticsState;

// Calibration state
extern int steeringTrim;   // -100..100
extern int steerCenterUs;  // microseconds center
extern int steerLeftRangePct;   // 0..180, scales center->left travel
extern int steerRightRangePct;  // 0..180, scales center->right travel
extern int appliedSteerUs; // last steering pulse written

// Loop timing
extern uint32_t lastCommandMs;
extern uint32_t lastNetworkAttemptMs;
extern bool udpListening;

// Debug/manual control path for the browser UI
extern volatile int manualThrottlePct;  // -100..100
extern volatile int manualSteerPct;     // -100..100
extern bool manualOverrideActive;
