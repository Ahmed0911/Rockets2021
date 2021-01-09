/*
 * CommData.h
 *
 *  Created on: Jan 10, 2016
 *      Author: Ivan
 */

#ifndef COMMDATA_H_
#define COMMDATA_H_

#include <stdint.h>

struct SGimbal3kData
{
    uint32_t LoopCounter;
    uint32_t ActiveMode;
    uint16_t PositionPanEncoCNT;
    uint16_t PositionTiltEncoCNT;

    float PositionPanDeg;
    float PositionTiltDeg;

    float AccX; // [m/s^2]
    float AccY; // [m/s^2]
    float AccZ; // [m/s^2]

    // GPS
    double HomeLongitude; // [deg]
    double HomeLatitude; // [deg]
};

struct SGimbal3kCommand
{
    uint32_t Command; // 0 - Disabled, 1 - Manual Ctrl
    float RefPan; // [-1...+1]
    float RefTilt; // [-1...+1]
};

// Ethernet packets
// data[0] = 0x42; // magic codes
// data[1] = 0x24; // magic codes
// data[2] = TYPE; // [0x10 - PING, 0x20 - DATA, 0.30 - Command...]
// data[3] = data....
#endif /* COMMDATA_H_ */
