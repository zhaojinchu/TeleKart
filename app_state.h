#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// Global objects (defined once in app_state.cpp)
extern WebServer server;
extern Servo steerServo;
extern Servo escServo;

// Live state updated by the webpage
extern volatile int targetThrottlePct;  // -100..100
extern volatile int targetSteerPct;     // -100..100

extern int steeringTrim;                // -100..100
extern int steerCenterUs;               // microseconds center (adjustable live)
extern int appliedSteerUs;              // last steering pulse actually sent

// Failsafe state
extern uint32_t lastCommandMs;
extern bool inFailsafe;
