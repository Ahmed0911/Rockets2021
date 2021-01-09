/*
 * AS5147UEncoder.cpp
 *
 *  Created on: Apr 14, 2016
 *      Author: User
 */

#include "AS5147UEncoder.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"


extern uint32_t g_ui32SysClock;

// SPI - ENCODER
// ---------
// 1MHz, 20MHz max (read only)
// Freescale SPI
// SPO = 1,SPH = 0 -> FRF_MOTO_MODE_2
// SSI1
// PINS:
// PB5 - CLK
// PE4 - MOSI
// PE5 - MISO
// PN2 - DIO0 - Pan CS
// PN1 - DIO1 - Tilt CS

bool AS5147EncoDrv::Init()
{
    // Enable
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);

    // Keep HOPE in RESET
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_2, 0 );
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_2, 0 );

    // SPI Pins
    GPIOPinConfigure(GPIO_PB5_SSI1CLK);
    GPIOPinConfigure(GPIO_PE4_SSI1XDAT0);
    GPIOPinConfigure(GPIO_PE5_SSI1XDAT1);
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_5);
    GPIOPinTypeSSI(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // Chip Select Pins - CS
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1 ); // default to 1
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2 ); // default to 1
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2);

	// Configure
	SSIConfigSetExpClk(SSI1_BASE, g_ui32SysClock, SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, 5000000, 16);
	// Enable
	SSIEnable(SSI1_BASE);

	// Clear fifo
	ClearFIFO();

	return true;
}


void AS5147EncoDrv::Update()
{
	// read status - Enco1
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0 ); // select 1
    Counter1 = ReadReg(0x3FFC);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1 ); // deselect 1

    // 1us delay
    SysCtlDelay(g_ui32SysClock/3/1000000); // 1 us delay

    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0 ); // select 1
    Velocity1 = ReadReg(0x3FFF);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1 ); // deselect 1

    // read status - Enco2
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0 ); // select 2
    Counter2 = ReadReg(0x3FFC);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2 ); // deselect 2

    // 1us delay
    SysCtlDelay(g_ui32SysClock/3/1000000); // 1 us delay

    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0 ); // select 2
    Velocity2 = ReadReg(0x3FFF);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2 ); // deselect 2
}

int AS5147EncoDrv::GetCounter1()
{
    return Counter1;
}

int AS5147EncoDrv::GetCounter2()
{
    return Counter2;
}

float AS5147EncoDrv::GetVelocity1() // CNT/s
{
    float vel = Velocity1;
    if( vel > 8192 ) vel = -(16384 - vel);

    // scale [1098.6837 cnt/bit]
    float velCNTs = vel * 1098.6837;

    return velCNTs;
}

float AS5147EncoDrv::GetVelocity2() // CNT/s
{
    float vel = Velocity2;
    if( vel > 8192 ) vel = -(16384 - vel);

    // scale [1098.6837 cnt/bit]
    float velCNTs = vel * 1098.6837;

    return velCNTs;
}

/////////////////////
// SPI Functions
/////////////////////
void AS5147EncoDrv::ClearFIFO()
{
	// Clear FIFO
	uint32_t data;
	while(SSIDataGetNonBlocking(SSI1_BASE, &data));
}

uint16_t AS5147EncoDrv::ReadReg(uint16_t address) // return previous value
{
	uint32_t request = address + 0x4000; // 0x4000 - READ FLAG

	SSIDataPut(SSI1_BASE, request);
	while(SSIBusy(SSI1_BASE));

	uint32_t response;
	SSIDataGet(SSI1_BASE, &response); // get data

	return (response & 0x3FFF);
}

/*void AS5147EncoDrv::WriteReg(unsigned char address, unsigned short data)
{
	uint32_t request = (address << 11) + 0x0000 + data; // 0x0000 - WRITE FLAG

	SSIDataPut(SSI2_BASE, request);
	while(SSIBusy(SSI2_BASE));

	volatile uint32_t response;
	SSIDataGet(SSI2_BASE, (uint32_t*)&response); // dummy
}*/
