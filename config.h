#pragma once

// ================== PINS ==================
static const int STEER_PIN = 18;
static const int ESC_PIN   = 19;

// ================== WIFI ==================
static const char* AP_SSID = "ESP32-CAR";
static const char* AP_PASS = "drivecar123";

// ================== STEERING (SERVO) SETTINGS ==================
// Safe-ish defaults; you can adjust center live from the web UI
static const int STEER_MIN_US = 1100;
static const int STEER_MAX_US = 2500;

// Trim: -100..100 -> microseconds shift (must match JS constant)
static const int STEER_TRIM_US_PER_UNIT = 3;

// ================== ESC SETTINGS ==================
static const int ESC_NEUTRAL_US = 1500;
static const int ESC_REV_MAX_US = 1300;
static const int ESC_FWD_MAX_US = 1700;
static const bool ESC_BIDIRECTIONAL = true;

// ================== BEHAVIOR ==================
static const uint32_t COMMAND_TIMEOUT_MS = 700;