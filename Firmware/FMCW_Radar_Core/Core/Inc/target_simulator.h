/*
 * target_simulator.h
 *
 *  Created on: Oct 8, 2026
 *      Author: gh0st
 */

#ifndef INC_TARGET_SIMULATOR_H_
#define INC_TARGET_SIMULATOR_H_


#pragma once
#include <cmath>

class BallisticTargetSimulator {
public:
    float x, y, z;
    float vx, vy, vz;
    float gamma;
    float g;

    BallisticTargetSimulator();
    void reset();
    void update(float dt);
};



#endif /* INC_TARGET_SIMULATOR_H_ */
