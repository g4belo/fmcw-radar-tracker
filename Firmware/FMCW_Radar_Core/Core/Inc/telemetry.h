/*
 * telemetry.h
 *
 *  Created on: Oct 8, 2026
 *      Author: gh0st
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_


#pragma once
#include <stdint.h>
#include "usbd_cdc_if.h" // Inclua o USB do CubeMX

#pragma pack(push, 1)
struct TelemetryPacket {
    uint8_t header[2];      // 0xAA, 0x55
    float true_x, true_y, true_z;
    float est_x, est_y, est_z;
    float pan_deg, tilt_deg;
    uint8_t checksum;
};
#pragma pack(pop)

class TelemetryTransmitter {
public:
    TelemetryPacket packet;
    void send(float tx, float ty, float tz,
    		float ex, float ey, float ez,
			float pan, float tilt);
};


#endif /* INC_TELEMETRY_H_ */
