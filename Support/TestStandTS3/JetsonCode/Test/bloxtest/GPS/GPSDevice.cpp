#include "GPSDevice.h"
#include <iostream>

GPSDevice::GPSDevice()
{
    // open serail port with GPS device
    m_Serial = std::make_unique<SerialPort>(COMPORT);
    m_GPS = std::make_unique<UBloxGPS>();

    // create serial data buffer
    m_buffer = std::make_unique<uint8_t[]>(BUFFER_SIZE);
}
GPSDevice::~GPSDevice()
{

}

void GPSDevice::run()
{
    // get data from serial port
    int32_t rd =  m_Serial->readData(m_buffer.get(), BUFFER_SIZE );

    if( rd > 0 )
    {
        m_GPS->newRXPacket(m_buffer.get(), rd);
    }

    // dump stats
    //UBloxGPS::Stats stats = m_GPS->getStats();
    //std::cout << "GPS Total: " << stats.MsgTotal << std::endl;
    //std::cout << "GPS Parsed: " << stats.MsgParsed << std::endl;    
    //std::cout << "GPS ACKCount: " << stats.ACKCount << std::endl;
    //std::cout << "GPS NAKCount: " << stats.NAKCount << std::endl;
}

GPSData GPSDevice::getData()
{
    return m_GPS->getData();
}
