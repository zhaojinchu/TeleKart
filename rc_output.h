#pragma once
#include <Arduino.h>

// Small helpers
int clampInt(int v, int lo, int hi);

// Hardware output functions
void rc_output_init();
void applyOutputs(int throttlePct, int steerPct);
void center_steering_now();
