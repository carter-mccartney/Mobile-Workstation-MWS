#include "rpi_driver.h"
#include <iostream>
#include <memory>
#include <pigpiod_if2.h>
#include <stdexcept>
#include <sstream>
#include <chrono>
#include <thread>

// compilation command: 
// g++ -Wall -pthread -o test test.cpp rpi_driver.h rpi_driver.cpp driver.h driver.cpp -lpigpiod_if2 -lrt

void my_callback(driver::data data);

unsigned char read_register(unsigned int i2c_handle, unsigned int address, int m_pigpio_handle);



int main(int argc, char* argv[])
{
    std::shared_ptr<rpi_driver> pImuDriver;

    std::function<void (driver::data)> f = std::bind(my_callback, std::placeholders::_1);

    //pImuDriver->set_data_callback(f);

    //pImuDriver->initialize(1, 0x68, 4);

    //pImuDriver->read_data();

    // Bus_id, I2C address, interrupt pin.
    // I2c address is 1101000 when pin AD0 is pulled low and 1101001 when pin AD0 is pulled high.
    //pImuDriver->initialize_i2c(1, 0x68, 4);

    int result = pigpio_start(nullptr, nullptr);
    if(result < 0)
    {
        throw std::runtime_error("initialize_i2c: Failed to connect to pigpio daemon.");
    }

    int m_pigpio_handle = result;

    int res = i2c_open(m_pigpio_handle, 1, 0x68, 0);
    if(res < 0)
    {
        switch(res)
        {
            case PI_BAD_I2C_BUS:
            {
                std::stringstream message;
                message << "initialize_i2c: Specified invalid I2C bus: " << 1;
                throw std::runtime_error(message.str());
            }
            case PI_BAD_I2C_ADDR:
            {
                std::stringstream message;
                message << "initialize_i2c: Specified invalid I2C address: 0x" << std::hex << 0x68;
                throw std::runtime_error(message.str());
            }
            case PI_BAD_FLAGS:
            {
                throw std::runtime_error("initialize_i2c: Specified invalid I2C flags.");
            }
            case PI_NO_HANDLE:
            {
                throw std::runtime_error("initialize_i2c: No handle.");
            }
            case PI_I2C_OPEN_FAILED:
            {
                throw std::runtime_error("initialize_i2c: Failed to open I2C.");
            }
            default:
            {
                std::stringstream message;
                message << "initialize_i2c: Unknown error: " << result;
                throw std::runtime_error(message.str());
            }
        }
    }

    result = set_mode(m_pigpio_handle, 4, PI_INPUT);
    if(result < 0)
    {
        switch(result)
        {
            case PI_BAD_GPIO:
            {
                std::stringstream message;
                message << "initialize: Invalid interrupt GPIO pin: " << 4;
                throw std::runtime_error(message.str());
            }
            case PI_BAD_MODE:
            {
                throw std::runtime_error("initialize: Invalid interrupt pin mode specified.");
            }
            case PI_NOT_PERMITTED:
            {
                throw std::runtime_error("initialize: Set mode for interrupt pin not permitted.");
            }
            default:
            {
                std::stringstream message;
                message << "initialize/set_mode: Unknown error: " << result;
                throw std::runtime_error(message.str());
            }
        }
    }
    unsigned short int acc_x;
    unsigned short int acc_y;
    unsigned short int acc_z;
    
    while(true)
    {
        acc_x = (((unsigned short int)read_register(res, 59, m_pigpio_handle)) << 8) + ((unsigned short int)read_register(res, 60, m_pigpio_handle));
        acc_y = (((unsigned short int)read_register(res, 61, m_pigpio_handle)) << 8) + ((unsigned short int)read_register(res, 62, m_pigpio_handle));
        acc_z = (((unsigned short int)read_register(res, 63, m_pigpio_handle)) << 8) + ((unsigned short int)read_register(res, 64, m_pigpio_handle));
        std::cout << "x: " << acc_x << " y: " << acc_y << " z: " << acc_z << "\r\n";
        std::this_thread::sleep_for(std::chrono::nanoseconds(100));
    }

    return 0;
}



void my_callback(driver::data data)
{
    std::cout << "x: " << data.accel_x << " y: " << data.accel_y << " z: " << data.accel_z << "\r\n";
}


unsigned char read_register(unsigned int i2c_handle, unsigned int address, int m_pigpio_handle)
{
    int result = i2c_read_byte_data(m_pigpio_handle, i2c_handle, address);
    if(result >= 0)
    {
        return static_cast<unsigned char>(result);
    }
    else
    {
        switch(result)
        {
            case PI_BAD_HANDLE:
            {
                std::stringstream message;
                message << "read_register: Specified invalid I2C handle: " << i2c_handle;
                throw std::runtime_error(message.str());
            }
            case PI_BAD_PARAM:
            {
                std::stringstream message;
                message << "read_register: Specified invalid register address: 0x" << std::hex << address;
                throw std::runtime_error(message.str());
            }
            case PI_I2C_READ_FAILED:
            {
                throw std::runtime_error("read_register: I2C read failed.");
            }
            default:
            {
                std::stringstream message;
                message << "read_register: Unknown error: " << result;
                throw std::runtime_error(message.str());
            }
        }
    }
}