#pragma once

#include <Arduino.h>
#include <IPAddress.h>

void control_link_init();
void control_link_poll(uint32_t now);
void control_link_reset_session();

bool control_link_pair(
    const String& vehicleName,
    uint32_t timestampMs,
    uint32_t controllerNonce,
    uint16_t controllerPort,
    uint32_t providedTag,
    const IPAddress& remoteIp,
    String& responseJson);

uint32_t control_link_compute_pair_tag(
    const String& vehicleName,
    uint32_t timestampMs,
    uint32_t controllerNonce,
    uint16_t controllerPort);
