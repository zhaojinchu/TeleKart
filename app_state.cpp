#include "app_state.h"

// Server + servos
WebServer server(80);
Servo steerServo;
Servo escServo;

// Command targets
volatile int targetThrottlePct = 0;
volatile int targetSteerPct    = 0;

// Defaults (you can change these anytime from the UI)
int steeringTrim = 0;
int steerCenterUs = 2000;
int appliedSteerUs = 2000;

// Failsafe
uint32_t lastCommandMs = 0;
bool inFailsafe = false;
