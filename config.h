#pragma once

#include <Arduino.h>

// ================== PINS ==================
static const int STEER_PIN = 18;
static const int ESC_PIN   = 19;

// Optional setup button. Leave disabled by default because many ESP32 dev boards
// use GPIO0 as the boot strap pin.
static const int SETUP_BUTTON_PIN = -1;

// ================== NETWORK DEFAULTS ==================
static const char* DEFAULT_STA_SSID       = "CMU-DEVICE";
static const char* DEFAULT_STA_PASSWORD   = "";
static const char* DEFAULT_VEHICLE_NAME   = "telekart-01";
static const char* DEFAULT_SHARED_KEY     = "changeme-telekart";
static const char* FALLBACK_AP_SSID       = "TeleKart-Setup";
static const char* FALLBACK_AP_PASS       = "telekart-setup";
static const bool  DEFAULT_FALLBACK_AP_ON = true;

// ================== NETWORK PORTS ==================
static const uint16_t HTTP_PORT    = 80;
static const uint16_t CONTROL_PORT = 4210;

// ================== STEERING (SERVO) SETTINGS ==================
static const int STEER_MIN_US = 1100;
static const int STEER_MAX_US = 2500;
static const int STEER_TRIM_US_PER_UNIT = 3;
static const int DEFAULT_STEER_LEFT_RANGE_PCT = 100;
static const int DEFAULT_STEER_RIGHT_RANGE_PCT = 100;

// ================== ESC SETTINGS ==================
static const int ESC_NEUTRAL_US = 1500;
static const int ESC_REV_MAX_US = 1300;
static const int ESC_FWD_MAX_US = 1700;
static const bool ESC_BIDIRECTIONAL = true;

// ================== TIMING ==================
static const uint32_t COMMAND_TIMEOUT_MS         = 120;
static const uint32_t CONTROL_LOOP_INTERVAL_MS   = 10;
static const uint32_t OUTPUT_LOOP_INTERVAL_MS    = 20;
static const uint32_t TELEMETRY_INTERVAL_MS      = 50;
static const uint32_t NETWORK_CONNECT_TIMEOUT_MS = 15000;
static const uint32_t NETWORK_RETRY_INTERVAL_MS  = 10000;
static const uint32_t SESSION_TTL_MS             = 300000;
static const uint32_t REVERSE_ARM_HOLD_MS        = 400;

// ================== DRIVING MODEL DEFAULTS ==================
static const float STEER_INPUT_DEADZONE         = 0.03f;
static const float STEER_FILTER_ALPHA           = 0.20f;
static const float STEER_SPEED_REDUCTION_MAX    = 0.45f;
static const float STEER_RATE_LOW_SPEED_PCT_S   = 700.0f;
static const float STEER_RATE_HIGH_SPEED_PCT_S  = 280.0f;
static const float THROTTLE_FILTER_ALPHA        = 0.18f;
static const float THROTTLE_DEADBAND            = 0.04f;
static const float THROTTLE_EXPO                = 1.8f;
static const float THROTTLE_LAUNCH_RATE_PCT_S   = 220.0f;
static const float BRAKE_STRENGTH               = 0.35f;
static const float DRAG_PER_SECOND              = 18.0f;
static const float ACCEL_PER_SECOND             = 32.0f;
static const float BRAKE_DECEL_PER_SECOND       = 70.0f;
static const float REVERSE_ALLOWED_SPEED_PCT    = 3.0f;
