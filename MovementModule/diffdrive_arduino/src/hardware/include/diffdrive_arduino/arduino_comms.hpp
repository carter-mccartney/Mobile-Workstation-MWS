/* Documentation for the libserial library: https://libserial.readthedocs.io/en/latest/tutorial.html */
/* Source code for the libserial library is here: https://github.com/crayzeewulf/libserial */

#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP


#define BUFFER_SIZE  1024

#include <cstring>
#include <cstdlib>
#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
#include <unistd.h>
#include <experimental/filesystem>
#include <string>
#include <vector>
#include <stdlib.h>

namespace fs = std::experimental::filesystem;

class ArduinoComms
{

private:
    LibSerial::SerialPort* port = nullptr;

    /* Number of milliseconds to wait to return if a line termination character is not read
     * from a read operation on a serial port. */
    const size_t MILLISECOND_TIMEOUT = 0;

    bool findSerialPort()
    {
        bool isPortFound = false;
        std::string seriaPortName = "";

        // Get all ports available to find the correct one.
        std::string symbolicPath = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_5593034353635170C0D0-if00";
        char* path = realpath(symbolicPath.c_str(), NULL);

        // Open the port.
        bool isPortOpen = false;
        try
        {
            if(this->port == nullptr)
            {
                this->port = new LibSerial::SerialPort();
            }
            this->port->Open(std::string(path));
            free(path);
            isPortOpen = true;
            this->port->FlushIOBuffers();

            usleep(1000000); // Enough time for setup for port to open. ***IMPORTANT***

            // Set port parameters.
            this->port->SetBaudRate(LibSerial::BaudRate::BAUD_9600);
            this->port->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_DEFAULT);
            this->port->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_DEFAULT);
            this->port->SetParity(LibSerial::Parity::PARITY_DEFAULT);
            this->port->SetStopBits(LibSerial::StopBits::STOP_BITS_DEFAULT);

            // Write the ID command.
            this->port->FlushIOBuffers();
            this->port->Write("ID\n");

            // Wait a reasonable time for a response.
            usleep(1000000); // 1 second for now.

            std::string line = "";

            // If no response, do nothing.
            if (this->port->IsDataAvailable())
            {
                // Check the response given.
                this->port->ReadLine(line, '\n', MILLISECOND_TIMEOUT);

                // If specific response found, this is the correct port.
                if (line == "This is Arduino\n")
                {
                    isPortFound = true;
                }
                else
                {
                    this->port->Close();
                }
            }
        }
        catch(const LibSerial::AlreadyOpen& ex){}
        catch(const LibSerial::OpenFailed& ex){}

        if(!isPortFound)
        {
            delete this->port;
            this->port = nullptr;
        }
        return isPortFound;
    }

public:
    ArduinoComms() = default;
    ~ArduinoComms()
    {
        if(port != nullptr)
        {
            delete this->port;
        }
    }

    /* Attempts to connect to the arduino serial port by iterating through each port and
     * issueing a command to listen to a response from the arduino that is equivilent to
     * "This is Arduino\n". Returns a zero upon success, and a negative one otherwise.
     */
    bool connect()
    {
        // If the port has not been found, then...
        if (!this->findSerialPort())
        {
            // TODO: Light indicator that tells the user of the MWS that the Raspberry Pi is not connected to the Arduino.

            /* Failed to find the arduino serial port. */
            return false; // CHANGE: from true to false. surely this should be false.
        }

        /* Successfully connected to the arduino serial port. */
        return true;
    }

    /* Disconnects the serial port. */
    void disconnect()
    {
        this->port->Close();
    }

    /* Returns true if the serial port is connected, false otherwise. */
    bool connected() const
    {
        return this->port != nullptr && this->port->IsOpen();
    }

    /* Receives a message string through the serial port. */
    int receive_msg(std::string& response)
    {
        this->port->ReadLine(response, '\n', 1000);

        std::cout << "Response: " << response << std::endl;
        return 0;
    }

    /* Sends a message string through the serial port. */
    void send_msg(const std::string &msg_to_send)
    {
        if(this->connected())
        {
            this->port->FlushIOBuffers();
            this->port->Write(msg_to_send);
        }
    }

    /* Send float values to set the speed of both the right and left motor.
     * left_motor_rpm is the speed of the left motor in meters per second.
     * right_motor_rpm is the speed of the right motor in meters per second.
     */
    void set_motor_values(float left_motor_rpm, float right_motor_rpm)
    {
        std::stringstream ss;
        ss << "V" << left_motor_rpm << "," << right_motor_rpm << "\n";
        this->send_msg(ss.str());
    }

    /*
    *  This command will receive input as a single character for directions
    *  that we want to drive the MWS:
    *       F - forward
    *       B - backward
    *       L - left
    *       R - right
    *  It will drive the MWS in that direction by calling the "set_motor_values"
    *  function with values based on the desired direction of travel.
    */
    void update_mws_presentation_drive_control(char direction)
    {
        switch
        {
            case ('F'): set_motor_values(0.25, 0.25); break;        // Forward:     move both motors 0.25 m/s
            case ('B'): set_motor_values(0.25, 0.25); break;        // Backward:    move both motors 0.25 m/s
            case ('L'): set_motor_values(-0.25, 0.25); break;       // Left:        move both motors 0.25 m/s
            case ('R'): set_motor_values(0.25, -0.25); break;       // Right:       move both motors 0.25 m/s
        }
    }

    /* Reads in tachometer values sent from the arduino.
    * Places values in the respective arguments.
    * Returns zero on success, and negative one otherwise. 
    */
    int read_motor_tachometers(long* left_motor_tachometer, long* right_motor_tachometer)
    {
        // Retrieve the response.
        std::string response;

        /* Receive tachometer values from the Arduino. 
        * The message should be in the format LLL,RRR\n  where the LLL represents the 
        * long integer of the left wheel tachometer and the RRR represents the 
        * long integer of the right wheel tachometer with both values separated
        * by a comma character and the message is appended with a newline character '\n'
        * as shown above.
        */
        if (receive_msg(response) != 0)
        {
            /* Message was not recieved. */
            return -1;
        }
        else
        {
            size_t splitPoint = response.find_first_of(',');
            std::string leftString = response.substr(0, splitPoint);
            std::string rightString = response.substr(splitPoint + 1, response.size() - splitPoint - 1);
            *left_motor_tachometer = stol(leftString);
            *right_motor_tachometer = stol(rightString);
        }

        /* Successful conversion. */
        return 0;
    }

    

    /* Reads in velocity values sent from the arduino.
    * Places values in the respective arguments.
    * Returns zero on success, and negative one otherwise. 
    */
    int read_motor_velocities(double* left_motor_velocity, double* right_motor_velocity)
    {
        // Retrieve the response.
        std::string response;

        /* Receive current velocity from the Arduino. 
        * The response should be in the format L.L,R.R\n  where the L.L represents the 
        * floating point number of the left wheel velocity and the R.R represents the 
        * floating point number of the right wheel velocity with both values separated
        * by a comma character and the message is appended with a newline character '\n'
        * as shown above.
        */
        if (receive_msg(response) != 0)
        {
            /* Message was not recieved. */
            return -1;
        }
        else
        {
            size_t splitPoint = response.find_first_of(',');
            std::string leftString = response.substr(0, splitPoint);
            std::string rightString = response.substr(splitPoint + 1, response.size() - splitPoint - 1);
            *left_motor_velocity = stof(leftString);
            *right_motor_velocity = stof(rightString);
        }

        /* Successful conversion. */
        return 0;
    }
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP