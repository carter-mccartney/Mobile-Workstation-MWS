/* Compile using the command below:
 *      g++ -Wall -lserial -lpthread -o test test.cpp /usr/lib/aarch64-linux-gnu/libserial.so.1 
 * For some reason, the linker for g++ could not find the .so (shared object) files for 
 * libserial so the command above explicitly tells the linker to use the .so file at the specified location.
 * Even using the  -L/usr/lib/aarch64-linux-gnu  command to tell the linker to look at the specified directory
 * to find the .so or .a file it might need to link to did not work. It's a mystery.
 */

#include "arduino_comms_libserial.hpp"
#include <cstring>
#include <cstdlib>
#include <sstream>
#include <iostream>
#include <libserial/SerialPort.h>

/* This program is for testing the functionalities of the 'arduino_comms.hpp' header file 
 * for connecting to an arduino microcontroller via the serial port.
 */
int main()
{


    LibSerial::SerialPort serial_conn_;
    using namespace std;

    serial_conn_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_conn_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_DEFAULT);
    serial_conn_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_DEFAULT);
    serial_conn_.SetParity(LibSerial::Parity::PARITY_DEFAULT);
    serial_conn_.SetStopBits(LibSerial::StopBits::STOP_BITS_DEFAULT);
    
    try{
    serial_conn_.Open("/dev/ttyUSB0");
    } catch (exception& e) {
        cout << e.what() << endl;
    }

//  "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_5593034353635170C0D0-if00"
    ArduinoComms comms_;

    try{
        if(comms_.connect(115200, "/dev/ttyUSB0") != 0)
        {
            std::cout << "ERROR: could not connect to the arduino" << std::endl;
            return -1;
        }
        else
        {
            std::cout << "Connected to the arduino" << std::endl;
        }
    }
    catch (exception& e) 
    {
        cout << e.what() << endl;
    }

    if(comms_.connected())
    {
        std::cout << "Arduino is still connected." << std::endl;
    }
    else
    {
        std::cout << "ERROR: arduino did not stay connected." << std::endl;
        return -1;
    }

    float leftVelocity = 10;
    float rightVelocity = 15;

    /* Set the wheel velocities. */
    comms_.set_motor_values(leftVelocity, rightVelocity);

    std::string message = "";

    /* Display the arduino's response. */
    comms_.receive_msg(message);
    std::cout << message << std::endl;

    comms_.receive_msg(message);
    std::cout << message << std::endl;

    /* Disconnect from the arduino. */
    comms_.disconnect();

}