#include "rpi_driver.h"
#include <iostream>
#include <memory>
#include <pigpiod_if2.h>
#include <stdexcept>
#include <sstream>
#include <chrono>
#include <thread>

// compilation command: 
// g++ -Wall -pthread -o test2 test2.cpp rpi_driver.h rpi_driver.cpp driver.h driver.cpp -lpigpiod_if2 -lrt

void my_callback(driver::data data);

int main(int argc, char* argv[])
{
    std::shared_ptr<driver> pImuDriver = std::make_shared<rpi_driver>();

    std::function<void (driver::data)> f = std::bind(my_callback, std::placeholders::_1);

    pImuDriver->set_data_callback(f);

    pImuDriver->initialize(1, 0x68, 4);

    while(true)
    {
        pImuDriver->read_data();
        std::this_thread::sleep_for(std::chrono::nanoseconds(200));
    }

    // Bus_id, I2C address, interrupt pin.
    // I2c address is 1101000 when pin AD0 is pulled low and 1101001 when pin AD0 is pulled high.
    //pImuDriver->initialize_i2c(1, 0x68, 4);

    return 0;
}

void my_callback(driver::data data)
{
    std::cout << "x: " << data.accel_x << " y: " << data.accel_y << " z: " << data.accel_z << "\r\n";
}