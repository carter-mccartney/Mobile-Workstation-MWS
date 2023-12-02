/* Compile using the command below:
 *      g++ -Wall -lserial -lpthread -o test test.cpp /usr/lib/aarch64-linux-gnu/libserial.so.1 
 * For some reason, the linker for g++ could not find the .so (shared object) files for 
 * libserial so the command above explicitly tells the linker to use the .so file at the specified location.
 * Even using the  -L/usr/lib/aarch64-linux-gnu  command to tell the linker to look at the specified directory
 * to find the .so or .a file it might need to link to did not work. It's a mystery.
 */

#include "arduino_comms.hpp"
#include <cstring>
#include <cstdlib>
#include <sstream>

/* This program is for testing the functionalities of the 'arduino_comms.hpp' header file 
 * for connecting to an arduino microcontroller via the serial port.
 */
int main()
{
    ArduinoComms comms_;

    std::cout << "Will attempt to connect to the arduino" << std::endl;

    if(comms_.connect("/dev/ttyUSB0") != 0)
    {
        std::cout << "ERROR: could not connect to the arduino" << std::endl;
        return -1;
    }
    else
    {
        std::cout << "Connected to the arduino" << std::endl;
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

    /* Print Serial port name. */
    std::string pName = comms_.getSerialPort();
    std::cout << pName << std::endl;

    float leftVelocity = 0;
    float rightVelocity = 0;


    /* Type into console the character 'v' and press enter to send a 'set velocity' command. */
    char run_motor_command = 'n';
    while(true)
    {
        run_motor_command = 'n';
        
        std::cin >> leftVelocity;
    	std::cin >> rightVelocity;
        
        std::cin >> run_motor_command;
        if(run_motor_command == 'v') comms_.set_motor_values(leftVelocity, rightVelocity);
    }

    /* Disconnect from the arduino. */
    comms_.disconnect();

}
