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
#include "Drivers/MPU9250Drv.h"
#include "Drivers/UBloxGPS.h"
#include "Drivers/EtherDriver.h"
#include "Drivers/AS5147UEncoder.h"
#include "Drivers/swupdate.h"

#include "CommData.h"
#include "LLConverter.h"

uint32_t g_ui32SysClock;
extern bool g_swUpdateRequest;

// Drivers
DBGLed dbgLed;
Timer timerLoop;
PWMDrv pwmDrv;
SerialDriver serialU2; // internal GPS
MPU9250Drv mpu9250Drv;
UBloxGPS gps;
EtherDriver etherDrv;
AS5147EncoDrv as5147Drv;

// System Objects
LLConverter llConv;

// GPS Port (serialU2->Internal GPS, serialU5->External GPS on Ext Comm.)
#define serialGPS serialU2

// Systick
#define SysTickFrequency 100
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

uint16_t PositionPanEnco = 0;
uint16_t PositionTiltEnco = 0;

float PositionPanDeg = 0;
float PositionTiltDeg = 0;

// 0 - disabled
// 1 - manual
// 2 - ....
uint32_t CmdMode = 0;
float RefManualPan = 0; // [-1...1]
float RefManualTilt = 0; // [-1...1]

float RefAutoPanDeg = 0; // [-180...+180]
float RefAutoTiltDeg = 0; // [-90...+90]

// Safety
int32_t ManualCounter = 0; // shutdown system when reaches zero

// Config
#define PANPWMCENTER 1510
#define TILTPWMCENTER 1500

#define PANLEVELDEG 257.4913f
#define TILTLEVELDEG 34.1915f

float GAIN_K = 0.2f; // 0.3+ begins to overshot

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
	pwmDrv.SetWidthUS(0, PANPWMCENTER );
	pwmDrv.SetWidthUS(1, TILTPWMCENTER );
	pwmDrv.SetWidthUS(2, 1500 );
	pwmDrv.SetWidthUS(3, 1500 );
	pwmDrv.SetWidthUS(4, 1500 );
	pwmDrv.SetWidthUS(5, 1500 );

	serialU2.Init(UART2_BASE, 9600); // GPS
	mpu9250Drv.Init();
	InitGPS(); // init GPS
	etherDrv.Init();
    as5147Drv.Init();

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
		// Encoders
		as5147Drv.Update();
		PositionPanEnco = as5147Drv.GetCounter1();
		PositionTiltEnco = as5147Drv.GetCounter2();
		// calculate deg
		float degPan = PositionPanEnco/16384.0f * 360; // [0...359.987]
		float degTilt = PositionTiltEnco/16384.0f * 360; // [0...359.987]
		// center
		degPan -= PANLEVELDEG;
		degTilt -= TILTLEVELDEG;
		// wrap to -180...180
		if( degPan < -180 ) degPan+=360;
		if( degPan > 180 ) degPan-=360;
		if( degTilt < -180 )degTilt+=360;
		if( degTilt > 180 )degTilt-=360;
		PositionPanDeg = degPan;
		PositionTiltDeg = degTilt;



		// IMU1
		mpu9250Drv.Update();
		Acc[0] = -mpu9250Drv.Accel[1];
		Acc[1] = -mpu9250Drv.Accel[0];
		Acc[2] = mpu9250Drv.Accel[2];

		// GPS
		int rd = serialGPS.Read(CommBuffer, COMMBUFFERSIZE); // read data from GPS
		gps.NewRXPacket(CommBuffer, rd); // process data
		if( gps.NumSV >= 6) // set home position
		{
			if( !llConv.IsHomeSet() )
			{
				double lat = gps.Latitude * 1e-7;
				double lon = gps.Longitude * 1e-7;
				llConv.SetHome(lat, lon);
			}
		}


		// Process ethernet (RX)
		etherDrv.Process(1000/SysTickFrequency); // 10ms tick


		/////////////////////////////////
		// CTRL STEP
		/////////////////////////////////
		float outputPWMPan = PANPWMCENTER;
		float outputPWMTilt = TILTPWMCENTER;
		if( CmdMode == 0x01)
		{
		    if( ManualCounter > 0) // safety shutdown for manual control
		    {
		        ManualCounter--;

		        // limit check
		        if( (PositionPanDeg < 175  && RefManualPan > 0) || (PositionPanDeg > -175  && RefManualPan < 0) )
		        {
		            outputPWMPan = PANPWMCENTER + (RefManualPan*150);
		        }

		        if( (PositionTiltDeg < 85  && RefManualTilt > 0) || (PositionTiltDeg > -85  && RefManualTilt < 0) )
		        {
		            outputPWMTilt = TILTPWMCENTER + (RefManualTilt*150);
		        }
		    }
		    else CmdMode = 0x00;
  		}

		if( CmdMode == 0x02 )
		{
		    // AUTO MODE
		    float deltaPan = RefAutoPanDeg - PositionPanDeg;
		    float deltaTilt = RefAutoTiltDeg - PositionTiltDeg;

		    float outputPan = GAIN_K * deltaPan;
		    if( outputPan > 1) outputPan = 1;
		    if( outputPan < -1) outputPan = -1;

		    float outputTilt = GAIN_K * deltaTilt;
            if( outputTilt > 1) outputTilt = 1;
		    if( outputTilt < -1) outputTilt = -1;

		    outputPWMPan = PANPWMCENTER + (outputPan*150);
            outputPWMTilt = TILTPWMCENTER + (outputTilt*150);
		}

		/////////////////////////////////
		// OUTPUTS
		/////////////////////////////////
		pwmDrv.SetWidthUS(0, outputPWMPan );
		pwmDrv.SetWidthUS(1, outputPWMTilt );

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
    SGimbal3kData data{};
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

    etherDrv.SendPacket(0x20, (char*)&data, sizeof(data));
}

void ProcessCommand(int cmd, unsigned char* data, int dataSize)
{
    switch( cmd )
    {
        case 0x30: // Command received
        {
            if( dataSize == sizeof(SGimbal3kCommand))
            {
                SGimbal3kCommand cmdRef;
                memcpy(&cmdRef, data, dataSize);

                if( cmdRef.Command == 0x01)
                {
                    CmdMode = 0x01;
                    RefManualPan = cmdRef.RefPan;
                    RefManualTilt = cmdRef.RefTilt;

                    ManualCounter = 100; // 1 second for shutdown
                }

                if( cmdRef.Command == 0x02)
                {
                    CmdMode = 0x02;
                    RefAutoPanDeg = cmdRef.RefPan;
                    RefAutoTiltDeg = cmdRef.RefTilt;

                    // LIMIT REFs
                    if( RefAutoPanDeg > 175) RefAutoPanDeg = 175;
                    if( RefAutoPanDeg < -175) RefAutoPanDeg = -175;
                    if( RefAutoTiltDeg > 85) RefAutoTiltDeg = 85;
                    if( RefAutoTiltDeg < -85) RefAutoTiltDeg = -85;
                }
            }
            break;
        }
    }
}

void InitGPS(void)
{
	SysCtlDelay(g_ui32SysClock); // Wait Ext. GPS to boot

	gps.Init();
	// send GPS init commands
	int toSend = gps.GenerateMsgCFGPrt(CommBuffer, 57600); // set to 57k
	serialGPS.Write(CommBuffer, toSend);
	SysCtlDelay(g_ui32SysClock/10); // 100ms wait, flush
	serialGPS.Init(UART2_BASE, 57600); // open with 57k (115k doesn't work well??? small int FIFO, wrong INT prio?)
	toSend = gps.GenerateMsgCFGRate(CommBuffer, 100); // 100ms rate, 10Hz
	serialGPS.Write(CommBuffer, toSend);
	toSend = gps.GenerateMsgCFGMsg(CommBuffer, 0x01, 0x07, 1); // NAV-PVT
	serialGPS.Write(CommBuffer, toSend);
	toSend = gps.GenerateMsgCFGMsg(CommBuffer, 0x01, 0x35, 1); // NAV-SAT
	serialGPS.Write(CommBuffer, toSend);
	toSend = gps.GenerateMsgNAV5Msg(CommBuffer, 6, 3); // airborne <1g, 2D/3D mode
	//toSend = m_GPS.GenerateMsgNAV5Msg(CommBuffer, 7, 2); // airborne <2g, 3D mode only
	serialGPS.Write(CommBuffer, toSend);

	// check response
	SysCtlDelay(g_ui32SysClock/10); // 100ms wait, wait response
	int rd = serialGPS.Read(CommBuffer, COMMBUFFERSIZE);
	gps.NewRXPacket(CommBuffer, rd);
}

///////////////
// INTERRUPTS
///////////////
extern "C" void UART2IntHandler(void)
{
	serialU2.IntHandler();
}

extern "C" void UART3IntHandler(void)
{
	//serialU3.IntHandler();
}

extern "C" void UART5IntHandler(void)
{
	//serialU5.IntHandler();
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
