/*
 * target_simulator.cpp
 *
 *  Created on: Oct 8, 2026
 *      Author: gh0st
 */

#include "target_simulator.h"


BallisticTargetSimulator::BallisticTargetSimulator() {
    reset();
}

void BallisticTargetSimulator::reset() {
        x = 11.5f;
        y = 2.5f;
        z = 1.2f;
        vx = -9.5f;
        vy = -1.2f;
        vz = 3.2f;
        gamma = 0.025f;
        g = 9.81f;
    }

void BallisticTargetSimulator::update(float dt) {
    float v_norm = sqrtf(vx * vx + vy * vy + vz * vz);
    if (v_norm < 0.001f) v_norm = 0.001f;

    float ax = -gamma * v_norm * vx;
    float ay = -gamma * v_norm * vy;
    float az = -g - (gamma * v_norm * vz);

    x += vx * dt;
    y += vy * dt;
    z += vz * dt;
    vx += ax * dt;
    vy += ay * dt;
    vz += az * dt;

    if (x <= 1.5f || z <= -2.5f) {
        reset();
    }
}
