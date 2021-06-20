#include <iostream>
#include "GPS/GPSDevice.h"
#include <thread>

int main()
{
    std::cout << "test App" << std::endl;

    GPSDevice gpsDevice;

    // LOOP
    while(1)
    {        
        gpsDevice.run();
        GPSData data = gpsDevice.getData();

        std::cout << data.GPSTime << " sat#: " << (uint32_t)data.NumSV << " Lat: " << data.Latitude << " Long: " << data.Longitude << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    };

    return 0;
}