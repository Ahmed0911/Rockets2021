#include "SerialPort.h"

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <iostream>

SerialPort::SerialPort(std::string portName)
{
	// Open serial port (e.g. "COM23" on Win, "ttyS0" on Linux )
	// Non-blocking Setup
	std::string fullPath = "/dev/" + portName;
	m_serial = open(fullPath.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (m_serial != -1)
	{
		if (fcntl(m_serial, F_SETFL, O_NONBLOCK) != -1)
		{
			// Retreive current options
			termios options{};
			if (tcgetattr(m_serial, &options) != -1)
			{
				// Define com port options
				options.c_cflag |= (CLOCAL | CREAD);		// Enable the receiver and set local mode...
				options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);	// No parity, 1 stop bit, mask character size bits
				options.c_cflag |= CS8;						// Select 8 data bits
				options.c_cflag &= ~CRTSCTS;				// Disable Hardware flow control

				// Disable software flow control
				options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);

				// Raw input
				options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG /*| IEXTEN | ECHONL*/);
				options.c_oflag &= ~OPOST;

				// Set timeout to 0
				options.c_cc[VMIN] = 0;
				options.c_cc[VTIME] = 1;

				// Set both input and output baud
				if ((cfsetispeed(&options, B115200) != -1) && (cfsetospeed(&options, B115200) != -1))
				{
					// Define options
					if (tcsetattr(m_serial, TCSANOW, &options) != -1)
					{
						// Flush port		
						if (tcflush(m_serial, TCIOFLUSH) == 0)
						{
							// all done
							std::cout << "Serial Port Open..." << std::endl;
							m_serialPortOpen = true;
						}
					}
				}
			}
		}
	}

	if( m_serialPortOpen == false)
	{
		std::cout << "Serial Port FAILED..." << std::endl;
	}
}

SerialPort::~SerialPort()
{
	if( m_serialPortOpen )
	{
		close(m_serial);
		m_serialPortOpen = false;
	}
}

int32_t SerialPort::readData(uint8_t* buffer, uint32_t bufSize)
{
	int bytesRead = 0;
	bytesRead = read(m_serial, buffer, bufSize);
	if (bytesRead > 0)
	{
		return bytesRead;
	}

	return -1; // error
}

bool SerialPort::writeData(uint8_t* buffer, uint32_t writeSize)
{
	int bytesWritten = 0;
	bytesWritten = write(m_serial, buffer, writeSize);
	if (bytesWritten > 0)
	{
		return (bytesWritten == writeSize);
	}

	return false; // write error
}

bool SerialPort::isOpen()
{
	return m_serialPortOpen;
}
