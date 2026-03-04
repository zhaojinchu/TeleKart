#include "rc_output.h"
#include "config.h"
#include "app_state.h"

int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static int mapPercentToUsSymmetric(int pct, int negUs, int zeroUs, int posUs) {
  pct = clampInt(pct, -100, 100);
  if (pct >= 0) return zeroUs + ((posUs - zeroUs) * pct) / 100;
  else          return zeroUs + ((zeroUs - negUs) * pct) / 100;
}

static const int STEER_OUTPUT_HOLD_US = 8;
static const int STEER_OUTPUT_MAX_STEP_US = 40;

void rc_output_init() {
  steerServo.setPeriodHertz(50);
  escServo.setPeriodHertz(50);

  steerServo.attach(STEER_PIN, 500, 2500);
  escServo.attach(ESC_PIN, 500, 2500);

  appliedSteerUs = clampInt(steerCenterUs, STEER_MIN_US, STEER_MAX_US);
  steerServo.writeMicroseconds(appliedSteerUs);
  escServo.writeMicroseconds(ESC_NEUTRAL_US);
}

void applyOutputs(int throttlePct, int steerPct) {
  throttlePct = clampInt(throttlePct, -100, 100);
  steerPct    = clampInt(steerPct, -100, 100);

  // -------- Steering: center + per-side range scaling --------
  int centerUs = steerCenterUs + steeringTrim * STEER_TRIM_US_PER_UNIT;
  centerUs = clampInt(centerUs, STEER_MIN_US + 20, STEER_MAX_US - 20);

  int leftAvail  = centerUs - STEER_MIN_US;
  int rightAvail = STEER_MAX_US - centerUs;
  int leftTravelUs = (leftAvail * clampInt(steerLeftRangePct, 0, 100)) / 100;
  int rightTravelUs = (rightAvail * clampInt(steerRightRangePct, 0, 100)) / 100;

  int targetSteerUs = centerUs;
  if (steerPct >= 0) {
    targetSteerUs = centerUs + (rightTravelUs * steerPct) / 100;
  } else {
    targetSteerUs = centerUs + (leftTravelUs * steerPct) / 100;
  }
  targetSteerUs = clampInt(targetSteerUs, STEER_MIN_US, STEER_MAX_US);

  int steerDeltaUs = targetSteerUs - appliedSteerUs;
  if (abs(steerDeltaUs) > STEER_OUTPUT_HOLD_US) {
    steerDeltaUs = clampInt(steerDeltaUs, -STEER_OUTPUT_MAX_STEP_US, STEER_OUTPUT_MAX_STEP_US);
    appliedSteerUs = clampInt(appliedSteerUs + steerDeltaUs, STEER_MIN_US, STEER_MAX_US);
    steerServo.writeMicroseconds(appliedSteerUs);
  }

  // -------- ESC --------
  int escUs;
  if (ESC_BIDIRECTIONAL) {
    escUs = mapPercentToUsSymmetric(throttlePct, ESC_REV_MAX_US, ESC_NEUTRAL_US, ESC_FWD_MAX_US);
  } else {
    int t = throttlePct < 0 ? 0 : throttlePct;
    escUs = ESC_NEUTRAL_US + ((ESC_FWD_MAX_US - ESC_NEUTRAL_US) * t) / 100;
  }
  escServo.writeMicroseconds(escUs);
}
