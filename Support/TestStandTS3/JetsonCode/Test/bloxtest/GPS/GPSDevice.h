#pragma once
#include <memory>
#include "SerialPort.h"
#include "UBloxGPS.h"


class GPSDevice
{
	public:
        GPSDevice();
        virtual ~GPSDevice();
        void run();

        GPSData getData();

    private:
        static constexpr char COMPORT[] = "ttyTHS1";
        std::unique_ptr<SerialPort> m_Serial;
        std::unique_ptr<UBloxGPS> m_GPS;

        // buffer
        static constexpr uint32_t BUFFER_SIZE = 4096;
        std::unique_ptr<uint8_t[]> m_buffer;
};
