#pragma once

#include <Arduino.h>

void drive_control_init();
void drive_control_poll(uint32_t now);
void drive_control_set_manual(int throttlePct, int steerPct);
void drive_control_release_manual();
void drive_control_trigger_estop();
void drive_control_clear_estop();
