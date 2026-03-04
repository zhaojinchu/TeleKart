#include "drive_control.h"

#include <math.h>

#include "app_state.h"
#include "config.h"
#include "control_protocol.h"
#include "rc_output.h"

static float clampFloat(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

static float moveToward(float current, float target, float maxStep) {
  if (target > current + maxStep) return current + maxStep;
  if (target < current - maxStep) return current - maxStep;
  return target;
}

static float lerpFloat(float a, float b, float t) {
  return a + (b - a) * clampFloat(t, 0.0f, 1.0f);
}

static float shapePedal(float raw) {
  raw = clampFloat(raw, 0.0f, 1.0f);
  if (raw <= THROTTLE_DEADBAND) return 0.0f;

  float scaled = (raw - THROTTLE_DEADBAND) / (1.0f - THROTTLE_DEADBAND);
  scaled = clampFloat(scaled, 0.0f, 1.0f);
  return powf(scaled, THROTTLE_EXPO);
}

static void resetDriveState() {
  vehicleState.filteredThrottlePct = 0.0f;
  vehicleState.filteredSteerInput = 0.0f;
  vehicleState.targetSteerPct = 0.0f;
  vehicleState.currentSteerPct = 0.0f;
  vehicleState.currentThrottlePct = 0;
  vehicleState.reverseDrive = false;
  vehicleState.reverseArmed = false;
  vehicleState.braking = false;
  vehicleState.reverseArmSinceMs = 0;
}

void drive_control_init() {
  diagnosticsState.estopLatched = false;
  diagnosticsState.inFailsafe = false;
  lastCommandMs = millis();
  diagnosticsState.lastControlLoopMs = lastCommandMs;
  diagnosticsState.lastOutputLoopMs = lastCommandMs;
  resetDriveState();
}

void drive_control_set_manual(int throttlePct, int steerPct) {
  manualThrottlePct = clampInt(throttlePct, -100, 100);
  manualSteerPct = clampInt(steerPct, -100, 100);
  manualOverrideActive = true;
  lastCommandMs = millis();
  diagnosticsState.inFailsafe = false;
}

void drive_control_release_manual() {
  manualOverrideActive = false;
}

void drive_control_trigger_estop() {
  diagnosticsState.estopLatched = true;
  manualOverrideActive = false;
  resetDriveState();
  applyOutputs(0, 0);
}

void drive_control_clear_estop() {
  diagnosticsState.estopLatched = false;
  lastCommandMs = millis();
}

void drive_control_poll(uint32_t now) {
  if ((now - diagnosticsState.lastControlLoopMs) < CONTROL_LOOP_INTERVAL_MS) {
    return;
  }

  float dt = (now - diagnosticsState.lastControlLoopMs) / 1000.0f;
  diagnosticsState.lastControlLoopMs = now;

  if (dt <= 0.0f) {
    dt = CONTROL_LOOP_INTERVAL_MS / 1000.0f;
  }

  bool timedOut = (now - lastCommandMs) > COMMAND_TIMEOUT_MS;
  if (timedOut && !diagnosticsState.inFailsafe) {
    diagnosticsState.inFailsafe = true;
    diagnosticsState.failsafeCount++;
  } else if (!timedOut) {
    diagnosticsState.inFailsafe = false;
  }

  float rawSteer = 0.0f;
  float rawThrottle = 0.0f;
  float rawBrake = 0.0f;
  bool reverseRequested = false;

  if (!diagnosticsState.estopLatched && !diagnosticsState.inFailsafe) {
    if (manualOverrideActive) {
      rawSteer = clampFloat(static_cast<float>(manualSteerPct) / 100.0f, -1.0f, 1.0f);

      float manualThrottle = static_cast<float>(manualThrottlePct) / 100.0f;
      if (manualThrottle >= 0.0f) {
        rawThrottle = clampFloat(manualThrottle, 0.0f, 1.0f);
      } else {
        rawBrake = clampFloat(-manualThrottle, 0.0f, 1.0f);
        reverseRequested = true;
      }
    } else {
      rawSteer = clampFloat(static_cast<float>(driveInput.steeringCmd) / 1000.0f, -1.0f, 1.0f);
      rawThrottle = clampFloat(static_cast<float>(driveInput.throttleCmd) / 1000.0f, 0.0f, 1.0f);
      rawBrake = clampFloat(static_cast<float>(driveInput.brakeCmd) / 1000.0f, 0.0f, 1.0f);
      reverseRequested = (driveInput.flags & CONTROL_FLAG_REVERSE_REQ) != 0;

      if ((driveInput.flags & CONTROL_FLAG_ESTOP) != 0) {
        diagnosticsState.estopLatched = true;
      }
    }
  }

  if (diagnosticsState.estopLatched) {
    rawSteer = 0.0f;
    rawThrottle = 0.0f;
    rawBrake = 0.0f;
    reverseRequested = false;
    resetDriveState();
  }

  float speedNorm = clampFloat(vehicleState.virtualSpeedPct / 100.0f, 0.0f, 1.0f);

  if (fabsf(rawSteer) < STEER_INPUT_DEADZONE) {
    rawSteer = 0.0f;
  }
  vehicleState.filteredSteerInput += (rawSteer - vehicleState.filteredSteerInput) * STEER_FILTER_ALPHA;
  if (fabsf(vehicleState.filteredSteerInput) < 0.005f) {
    vehicleState.filteredSteerInput = 0.0f;
  }

  float steerScale = 1.0f - (STEER_SPEED_REDUCTION_MAX * speedNorm);
  vehicleState.targetSteerPct = vehicleState.filteredSteerInput * (100.0f * steerScale);

  float steerRatePctPerSecond = lerpFloat(STEER_RATE_LOW_SPEED_PCT_S, STEER_RATE_HIGH_SPEED_PCT_S, speedNorm);
  float steerStep = steerRatePctPerSecond * dt;
  vehicleState.currentSteerPct = moveToward(vehicleState.currentSteerPct, vehicleState.targetSteerPct, steerStep);

  float shapedThrottle = shapePedal(rawThrottle);
  vehicleState.filteredThrottlePct += (shapedThrottle - vehicleState.filteredThrottlePct) * THROTTLE_FILTER_ALPHA;
  if (vehicleState.filteredThrottlePct < 0.01f) {
    vehicleState.filteredThrottlePct = 0.0f;
  }

  float accelTerm = rawThrottle * ACCEL_PER_SECOND * dt;
  float dragTerm = DRAG_PER_SECOND * dt;
  float brakeTerm = noBrakeMode ? 0.0f : (rawBrake * BRAKE_DECEL_PER_SECOND * dt);
  vehicleState.virtualSpeedPct += accelTerm;
  vehicleState.virtualSpeedPct -= dragTerm;
  vehicleState.virtualSpeedPct -= brakeTerm;
  vehicleState.virtualSpeedPct = clampFloat(vehicleState.virtualSpeedPct, 0.0f, 100.0f);

  int desiredThrottlePct = 0;
  vehicleState.braking = false;

  if (!diagnosticsState.estopLatched && !diagnosticsState.inFailsafe) {
    if (!vehicleState.reverseDrive) {
      if (vehicleState.filteredThrottlePct > 0.0f) {
        vehicleState.reverseArmed = false;
        desiredThrottlePct = clampInt(static_cast<int>(lroundf(vehicleState.filteredThrottlePct * 100.0f)), 0, 100);
      } else if (rawBrake > THROTTLE_DEADBAND) {
        if (vehicleState.virtualSpeedPct > REVERSE_ALLOWED_SPEED_PCT) {
          if (!noBrakeMode) {
            vehicleState.braking = true;
            float brakePct = clampFloat(rawBrake * BRAKE_STRENGTH, 0.0f, 0.35f);
            desiredThrottlePct = -clampInt(static_cast<int>(lroundf(brakePct * 100.0f)), 0, 35);
          } else {
            desiredThrottlePct = 0;
            vehicleState.reverseArmed = false;
          }
        } else {
          desiredThrottlePct = 0;
          if (reverseRequested) {
            if (!vehicleState.reverseArmed) {
              vehicleState.reverseArmed = true;
              vehicleState.reverseArmSinceMs = now;
            }

            if ((now - vehicleState.reverseArmSinceMs) >= REVERSE_ARM_HOLD_MS) {
              vehicleState.reverseDrive = true;
              vehicleState.reverseArmed = false;
            }
          } else {
            vehicleState.reverseArmed = false;
          }
        }
      } else {
        vehicleState.reverseArmed = false;
      }
    }

    if (vehicleState.reverseDrive) {
      if (vehicleState.filteredThrottlePct > 0.0f && vehicleState.virtualSpeedPct <= REVERSE_ALLOWED_SPEED_PCT) {
        vehicleState.reverseDrive = false;
        desiredThrottlePct = 0;
      } else {
        float reversePedal = rawBrake;
        if (reversePedal <= THROTTLE_DEADBAND && reverseRequested) {
          reversePedal = rawThrottle;
        }

        float shapedReverse = shapePedal(reversePedal);
        desiredThrottlePct = -clampInt(static_cast<int>(lroundf(shapedReverse * 65.0f)), 0, 65);
      }
    }
  }

  float throttleStep = THROTTLE_LAUNCH_RATE_PCT_S * dt;
  float nextThrottle = moveToward(
      static_cast<float>(vehicleState.currentThrottlePct),
      static_cast<float>(desiredThrottlePct),
      throttleStep);
  vehicleState.currentThrottlePct = clampInt(static_cast<int>(lroundf(nextThrottle)), -100, 100);

  if ((now - diagnosticsState.lastOutputLoopMs) >= OUTPUT_LOOP_INTERVAL_MS) {
    diagnosticsState.lastOutputLoopMs = now;
    applyOutputs(vehicleState.currentThrottlePct, static_cast<int>(lroundf(vehicleState.currentSteerPct)));
  }
}
