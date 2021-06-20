#pragma once
#include <cstdint>

struct GPSData
{
	uint32_t GPSTime; // GPS time of week [ms]
	uint8_t FixType; // GNSSfix Type
	uint8_t FixFlags; // FIX Flags
	uint8_t NumSV; // number of active satellites
	int32_t Longitude; // 1e-7 [deg]
	int32_t Latitude; // 1e-7 [deg]
	int32_t HeightMSL; // MSL [mm]
	uint32_t HorizontalAccuracy; // [mm]
	uint32_t VerticalAccuracy; // [mm]
	int32_t VelN; // Speed North [mm/s]
	int32_t VelE; // Speed East [mm/s]
	int32_t VelD; // Speed Down [mm/s]
	uint32_t SpeedAcc; // Speed accuracy [mm/s]
};

class UBloxGPS
{
	public:
		UBloxGPS();
		virtual ~UBloxGPS();
		GPSData getData();

		void newRXPacket(uint8_t* data, int dataLen);

		struct Stats
		{
			int ACKCount;
			int NAKCount;
			int MsgTotal;
			int MsgParsed;
		};
		Stats getStats();		

	private:		
		struct CHKSUM
		{
			uint8_t CK_A;
			uint8_t CK_B;
		};
		CHKSUM calculateCheckSum(uint8_t* packet, int LENGTH);
		void parseMessage();


		// Parser State
		enum ERXPhase { SYNC1, SYNC2, CLASS, ID, LENLSB, LENMSB, PAYLOAD, CHK_A, CHK_B };
		ERXPhase m_RXPhase;

		// RX Data
		uint8_t m_CLASS;
		uint8_t m_ID;
		uint16_t m_Length;
		uint8_t m_Payload[1000];
		int32_t m_PayloadIndex;
		uint8_t m_ChkA;
		uint8_t m_ChkB;

		// data
		GPSData m_lastReceivedData;

		// stats
		Stats m_Stats{};
};
