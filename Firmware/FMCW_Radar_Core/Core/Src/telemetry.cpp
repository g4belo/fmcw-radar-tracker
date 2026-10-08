/*
 * telemetry.cpp
 *
 *  Created on: Oct 8, 2026
 *      Author: gh0st
 */




#include "telemetry.h"

void TelemetryTransmitter::send(float tx, float ty, float tz,
								float ex, float ey, float ez,
								float pan, float tilt) {
    packet.header[0] = 0xAA;
    packet.header[1] = 0x55;
    packet.true_x = tx; packet.true_y = ty; packet.true_z = tz;
    packet.est_x = ex;  packet.est_y = ey;  packet.est_z = ez;
    packet.pan_deg = pan; packet.tilt_deg = tilt;

    uint8_t csum = 0;
    uint8_t* p_bytes = (uint8_t*)&packet;
    for (size_t b = 0; b < sizeof(TelemetryPacket) - 1; b++) {
        csum ^= p_bytes[b];
    }
    packet.checksum = csum;

    CDC_Transmit_FS((uint8_t*)&packet, sizeof(TelemetryPacket));
}
