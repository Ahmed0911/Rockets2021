/*
 * AS5147UEncoder.h
 *
 *  Created on: Jul 3, 202
 *      Author: User
 *      KGX-1 - Gimbal3k Version (dual encoders)
 */

#ifndef DRIVERS_AS5147UENCODER_H_
#define DRIVERS_AS5147UENCODER_H_

#include <stdint.h>

class AS5147EncoDrv {
public:
    bool Init();
    void Update();
    int GetCounter1(); // CNT
    int GetCounter2(); // CNT
    float GetVelocity1(); // CNT/s
    float GetVelocity2(); // CNT/s


private:
    uint16_t ReadReg(uint16_t address);
	//void WriteReg(unsigned char address, unsigned short data);
	void ClearFIFO();

	uint16_t Counter1;
	uint16_t Velocity1;

    uint16_t Counter2;
    uint16_t Velocity2;
};

#endif /* DRIVERS_AS5147UENCODER_H_ */
