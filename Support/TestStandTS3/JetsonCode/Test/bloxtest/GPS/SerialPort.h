#pragma once
#include <string>

class SerialPort
{
	public:
		SerialPort(std::string portName);
		virtual ~SerialPort();

		int32_t readData(uint8_t* buffer, uint32_t bufSize);
		bool writeData(uint8_t* buffer, uint32_t writeSize);
		bool isOpen();

	private:
		int m_serial = 0;

		bool m_serialPortOpen = false;
};

