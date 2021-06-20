#include "UBloxGPS.h"
#include <string.h>

UBloxGPS::UBloxGPS()
{
	m_RXPhase = UBloxGPS::SYNC1;

	m_Stats = {0}; // clear stats
	m_lastReceivedData = {0}; // clear GPS Data
}

UBloxGPS::~UBloxGPS()
{

}

GPSData UBloxGPS::getData()
{
	return m_lastReceivedData;
}

UBloxGPS::Stats UBloxGPS::getStats()
{
	return m_Stats;
}

void UBloxGPS::newRXPacket(uint8_t* data, int dataLen)
{
	// RX Parser
	for (int i = 0; i != dataLen; i++)
	{
		uint8_t b = data[i];
		switch (m_RXPhase)
		{
		case UBloxGPS::SYNC1:
			if (b == 0xB5)
			{
				m_Stats.MsgTotal++;
				m_RXPhase = UBloxGPS::SYNC2; // wait for start
			}
			break;

		case UBloxGPS::SYNC2:
			if (b == 0x62)
			{
				// reset chksum
				m_ChkA = 0;
				m_ChkB = 0;
				m_RXPhase = UBloxGPS::CLASS;
			}
			else m_RXPhase = UBloxGPS::SYNC1; // reset
			break;

		case UBloxGPS::CLASS:
			m_CLASS = b;
			m_ChkA = m_ChkA + b;
			m_ChkB = m_ChkB + m_ChkA;
			m_RXPhase = UBloxGPS::ID;
			break;

		case UBloxGPS::ID:
			m_ID = b;
			m_ChkA = m_ChkA + b;
			m_ChkB = m_ChkB + m_ChkA;
			m_RXPhase = UBloxGPS::LENLSB;
			break;

		case UBloxGPS::LENLSB:
			m_Length = b;
			m_ChkA = m_ChkA + b;
			m_ChkB = m_ChkB + m_ChkA;
			m_RXPhase = UBloxGPS::LENMSB;
			break;

		case UBloxGPS::LENMSB:
			m_Length = m_Length + (b * 256);
			m_PayloadIndex = 0;
			m_ChkA = m_ChkA + b;
			m_ChkB = m_ChkB + m_ChkA;
			if (m_Length == 0) m_RXPhase = UBloxGPS::CHK_A; // skip payload
			else m_RXPhase = UBloxGPS::PAYLOAD;
			break;

		case UBloxGPS::PAYLOAD:
			m_Payload[m_PayloadIndex] = b;
			m_PayloadIndex++;
			m_ChkA = m_ChkA + b;
			m_ChkB = m_ChkB + m_ChkA;
			if (m_PayloadIndex >= m_Length) m_RXPhase = UBloxGPS::CHK_A;
			break;

		case UBloxGPS::CHK_A:
			if (m_ChkA == b) m_RXPhase = UBloxGPS::CHK_B;
			else m_RXPhase = UBloxGPS::SYNC1; // checksum failed->reset
			break;

		case UBloxGPS::CHK_B:
			if (m_ChkB == b)
			{
				// OK
				parseMessage();
				m_Stats.MsgParsed++;
			}
			else
			{
				// checksum error!
			}
			m_RXPhase = UBloxGPS::SYNC1; // reset
			break;
		}
	}
}
 
// packet is whole packet (starts with SYNC1), chk "data" is 4 bytes longer
UBloxGPS::CHKSUM UBloxGPS::calculateCheckSum(uint8_t* packet, int LENGTH)
{
	uint8_t ck_A = 0;
	uint8_t ck_B = 0;

	for (int i = 2; i != LENGTH + 4 + 2; i++)
	{
		ck_A = ck_A + packet[i];
		ck_B = ck_B + ck_A;
	}

	CHKSUM chk;
	chk.CK_A = ck_A;
	chk.CK_B = ck_B;

	return chk;
}

// RX Messages
void UBloxGPS::parseMessage()
{
	uint8_t *pyld = m_Payload;

	switch (m_CLASS)
	{

	case 0x05: // ACK
		if (m_ID == 0x01)
		{
			// ACK-ACK
			m_Stats.ACKCount++;
		}
		if (m_ID == 0x00)
		{
			// ACK-NAK
			m_Stats.NAKCount++;
		}
		break;

	case 0x01: // NAV
		if (m_ID == 0x07)
		{
			GPSData newData{};

			//NAV-PVT
			memcpy(&newData.GPSTime, &pyld[0], sizeof(newData.GPSTime)); // GPS time of week [ms]
			newData.FixType = pyld[20]; // GNSSfix Type
			newData.FixFlags = pyld[21]; // FIX Flags
			newData.NumSV =  pyld[23]; // number of active satellites
			memcpy(&newData.Longitude, &pyld[24], sizeof(newData.Longitude)); // 1e-7 [deg]
			memcpy(&newData.Latitude, &pyld[28], sizeof(newData.Latitude)); // 1e-7 [deg]
			memcpy(&newData.HeightMSL, &pyld[36], sizeof(newData.HeightMSL)); // MSL [mm]
			memcpy(&newData.HorizontalAccuracy, &pyld[40], sizeof(newData.HorizontalAccuracy)); // [mm]
			memcpy(&newData.VerticalAccuracy, &pyld[44], sizeof(newData.VerticalAccuracy)); // [mm]
			memcpy(&newData.VelN, &pyld[48], sizeof(newData.VelN)); // Speed North [mm/s]
			memcpy(&newData.VelE, &pyld[52], sizeof(newData.VelE)); // Speed East [mm/s]
			memcpy(&newData.VelD, &pyld[56], sizeof(newData.VelD)); // Speed Down [mm/s]
			memcpy(&newData.SpeedAcc, &pyld[68], sizeof(newData.SpeedAcc));

			// copy to local
			m_lastReceivedData = newData;
		}
		break;
	}
}
