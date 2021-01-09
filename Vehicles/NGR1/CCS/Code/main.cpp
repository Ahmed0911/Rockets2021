/*
 * main.c
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/fpu.h"

#include "Drivers/DBGLed.h"
#include "Drivers/Timer.h"
#include "Drivers/PWMDrv.h"
#include "Drivers/SerialDriver.h"
#include "Drivers/SBUSComm.h"
#include "Drivers/BaroDrv.h"
#include "Drivers/MPU9250Drv.h"
#include "Drivers/UBloxGPS.h"
#include "Drivers/EtherDriver.h"
#include "Drivers/IMU.h"
#include "Drivers/swupdate.h"

#include "CommData.h"
#include "LLConverter.h"

uint32_t g_ui32SysClock;
extern bool g_swUpdateRequest;

// Drivers
DBGLed dbgLed;
Timer timerLoop;
PWMDrv pwmDrv;
SerialDriver serialU3; // SBUS, RX
SerialDriver serialU5; // external GPS
BaroDrv baroDrv;
SBUSComm sbusRecv;
MPU9250Drv mpu9250Drv;
UBloxGPS gps;
EtherDriver etherDrv;
IMU imu;

// System Objects
LLConverter llConv;

// GPS Port (serialU2->Internal GPS, serialU5->External GPS on Ext Comm.)
#define serialGPS serialU5

// Systick
#define SysTickFrequency 1000
volatile bool SysTickIntHit = false;

// Buffers
#define COMMBUFFERSIZE 1024
BYTE CommBuffer[COMMBUFFERSIZE];

// Global Functions
void InitGPS(void);
void ProcessGPSData(void);
void SendPeriodicDataEth(void);
void ProcessCommand(int cmd, unsigned char* data, int dataSize);

// Global Data
int MainLoopCounter;
float PerfLoopTimeMS;
float PerfCpuTimeMS;
float PerfCpuTimeMSMAX;

float Acc[3];
float Gyro[3];
float Mag[3];

// OFFSETS
#define GYROOFFX -1.004f
#define GYROOFFY 1.344f
#define GYROOFFZ -0.6995f
#define MAGOFFX -10.8451f
#define MAGOFFY 15.3744f
#define MAGOFFZ 155.7038f
#define ATTOFFROLL 1.5f
#define ATTOFFPITCH 1.9f

//double g1 = 0;
//double g2 = 0;
//double g3 = 0;

void main(void)
{
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	FPULazyStackingEnable();

	// Ensure that ext. osc is used!
	SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);

	// set clock
	g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

	// Init
	dbgLed.Init();
	timerLoop.Init();
	pwmDrv.Init();
	// set all PWMs to middle
	pwmDrv.SetWidthUS(0, 1500 );
	pwmDrv.SetWidthUS(1, 1500 );
	pwmDrv.SetWidthUS(2, 1500 );
	pwmDrv.SetWidthUS(3, 1500 );
	pwmDrv.SetWidthUS(4, 1500 );
	pwmDrv.SetWidthUS(5, 1500 );
	serialU3.Init(UART3_BASE, 100000); // SBUS
	serialU5.Init(UART5_BASE, 115200); // Ext. Comm, Ext. GPS - F9
	sbusRecv.Init();
	baroDrv.Init();
	mpu9250Drv.Init();
	etherDrv.Init();
	imu.Init();
	gps.Init();

	// Systick
	SysTickPeriodSet(g_ui32SysClock/SysTickFrequency);
	SysTickIntEnable();
	SysTickEnable();

	// Master INT Enable
	IntMasterEnable();

	while(1)
	{
		timerLoop.Start(); // start timer
		MainLoopCounter++;

		// Check SW Update Request
		if( g_swUpdateRequest)
		{
		    SoftwareUpdateBegin(g_ui32SysClock);
		}

		/////////////////////////////////
		// INPUTS
		/////////////////////////////////
		// SBUS Data
		int rd = serialU3.Read(CommBuffer, COMMBUFFERSIZE); // read data from SBUS Recv [2500 bytes/second, read at least 3x per second for 1k buffer!!!]
		sbusRecv.NewRXPacket(CommBuffer, rd); // process data

		// Baro
		baroDrv.Update(); // [??? us]

		// IMU1
		mpu9250Drv.Update();
		Acc[0] = -mpu9250Drv.Accel[1];
		Acc[1] = -mpu9250Drv.Accel[0];
		Acc[2] = mpu9250Drv.Accel[2];
		Gyro[0] = mpu9250Drv.Gyro[1] - GYROOFFX;
		Gyro[1] = mpu9250Drv.Gyro[0] - GYROOFFY;
		Gyro[2] = -mpu9250Drv.Gyro[2] - GYROOFFZ;
		Mag[0] = mpu9250Drv.Mag[0] - MAGOFFX;
		Mag[1] = mpu9250Drv.Mag[1] - MAGOFFY;
		Mag[2] = mpu9250Drv.Mag[2] - MAGOFFZ;
		imu.Update(Acc[0], Acc[1], Acc[2], Gyro[0], Gyro[1], Gyro[2], Mag[0], Mag[1], Mag[2]); // TODO: CHECK AXES
		// Correct IMU offsets
		imu.Roll -= ATTOFFROLL;
		imu.Pitch -= ATTOFFPITCH;
		//g1 = 0.9999*g1 + 0.0001*Gyro[0];
		//g2 = 0.9999*g2 + 0.0001*Gyro[1];
		//g3 = 0.9999*g3 + 0.0001*Gyro[2];

		// GPS
		rd = serialGPS.Read(CommBuffer, COMMBUFFERSIZE); // read data from GPS
		gps.NewRXPacket(CommBuffer, rd); // process data
		// GPS TODO!!!

		// Process ethernet (RX)
		etherDrv.Process(1000/SysTickFrequency); // 1ms tick


		/////////////////////////////////
		// CTRL STEP
		/////////////////////////////////
		int throttle = sbusRecv.Channels[0];
		int aileron = sbusRecv.Channels[1];
		int elevator = sbusRecv.Channels[2];
		int rudder = sbusRecv.Channels[3];
		int ASwitch = sbusRecv.Channels[4];
		int DSwitch = sbusRecv.Channels[5];
		
		/////////////////////////////////
		// OUTPUTS
		/////////////////////////////////
		pwmDrv.SetWidthUS(0, throttle + 476);
		pwmDrv.SetWidthUS(1, aileron + 476);
		pwmDrv.SetWidthUS(2, elevator + 476);
		pwmDrv.SetWidthUS(3, rudder + 476);
		pwmDrv.SetWidthUS(4, ASwitch + 476);
		pwmDrv.SetWidthUS(5, DSwitch + 476);

		// Ethernet
		SendPeriodicDataEth();

		// DBG LED
		if( MainLoopCounter%10 == 0) dbgLed.Toggle();

		// Get CPU Time
		PerfCpuTimeMS = timerLoop.GetUS()/1000.0f;
		if( PerfCpuTimeMS > PerfCpuTimeMSMAX ) PerfCpuTimeMSMAX = PerfCpuTimeMS;


		// wait next
		while(!SysTickIntHit);
		SysTickIntHit = false;
		// Get total loop time
		PerfLoopTimeMS = timerLoop.GetUS()/1000.0f;
	}
}

void SendPeriodicDataEth()
{
/*    SGimbal3kData data{};
    data.LoopCounter = MainLoopCounter;
    data.ActiveMode = CmdMode;

    // acc
    data.AccX = Acc[0];
    data.AccY = Acc[1];
    data.AccZ = Acc[2];

    // gps
    if( llConv.IsHomeSet())
    {
        double lat;
        double lon;
        llConv.GetHome(lat, lon);
        data.HomeLatitude = lat;
        data.HomeLongitude = lon;
    }

    data.PositionPanEncoCNT = PositionPanEnco;
    data.PositionTiltEncoCNT = PositionTiltEnco;

    data.PositionPanDeg = PositionPanDeg;
    data.PositionTiltDeg = PositionTiltDeg;

    etherDrv.SendPacket(0x20, (char*)&data, sizeof(data));*/
}

void ProcessCommand(int cmd, unsigned char* data, int dataSize)
{
    switch( cmd )
    {
        case 0x30: // Command received
        {
            break;
        }
    }
}

///////////////
// INTERRUPTS
///////////////
extern "C" void UART2IntHandler(void)
{
	//serialU2.IntHandler();
}

extern "C" void UART3IntHandler(void)
{
	serialU3.IntHandler();
}

extern "C" void UART5IntHandler(void)
{
	serialU5.IntHandler();
}

extern "C" void IntGPIOA(void)
{
	//lsm90Drv.MotionINTG();
	//lsm90Drv.MotionINTX();
}

extern "C" void IntGPIOH(void)
{
	//lsm90Drv.MotionINTM();
}

extern "C" void IntGPIOK(void)
{
	mpu9250Drv.MotionINT();
}

extern "C" void IntGPION(void)
{

}

extern "C" void SysTickIntHandler(void)
{
	SysTickIntHit = true;
}
