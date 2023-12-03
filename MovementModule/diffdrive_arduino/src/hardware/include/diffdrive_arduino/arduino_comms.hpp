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
            return true;
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
    int receive_msg(unsigned char response[BUFFER_SIZE])
    {
        int n = 0;
        int index = 0;
        unsigned char buffer = '\0';
        unsigned char TERMINATING_CHARACTER = '\n';
        errno = 0;

        /* Set all bytes of the response array to null terminating characters. */
        memset(response, '\0', BUFFER_SIZE);

        int fd = this->port->GetFileDescriptor();
        do 
        {
        /* Iterate through the response until either we find a terminating character, there's no more characters to read, or the buffer is full. */
            n = read(fd, &buffer, 1); // Read from the 'tty' file. Documented in: https://man7.org/linux/man-pages/man2/read.2.html
            if (n < 0)
            {
            std::cout << "Error reading: " << strerror(errno) << std::endl;
            return -1;
            }
            else if (n == 0)
            {
            break;
            }
            else
            {
            response[index] = buffer;
            index++;
            }
        } while(buffer != TERMINATING_CHARACTER && index < BUFFER_SIZE);

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

    /* Reads in tachometer values sent from the arduino.
    * Places values in the respective arguments.
    * Returns zero on success, and negative one otherwise. 
    */
    int read_motor_tachometers(long* left_motor_tachometer, long* right_motor_tachometer)
    {
        // Retrieve the response.
        unsigned char response[BUFFER_SIZE] = {0};

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
        //std::cout << response << std::endl;

        int BASE_10 = 10;
        char *endPtr;

        /* Reset errono to zero. */
        errno = 0;

        /* Read in the left motor velocity value from the response message. */
        *left_motor_tachometer = strtol((char*)response, &endPtr, BASE_10);

        if (errno != 0 || (*endPtr) != ',' || endPtr != (char*)response)
        {
            /* An error occured when converting, the digits before 
                * the comma character was not reached, or no conversion was done.
                */
            return -1;
        }

        /* Move to the character that is after the comma character. */
        endPtr++;

        /* Read in the right motor velocity value from the response message. */
        *right_motor_tachometer = strtol((char*)response, &endPtr, BASE_10);

        if (errno != 0 || (*endPtr) != '\n' || endPtr != (char*)response)
        {
            /* An error occured when converting, the digits before 
                * the comma character was not reached, or no conversion was done.
                */
            return -1;
        }

        //std::cout << "Tachometer: " << *left_motor_tachometer << "," << *right_motor_tachometer << std::endl;

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
        unsigned char response[BUFFER_SIZE] = {0};

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

        //std::cout << response << std::endl;

        char *endPtr;

        /* Reset errono to zero. */
        errno = 0;

        /* Read in the left motor velocity value from the response message. */
        *left_motor_velocity = strtod((char*)response, &endPtr);

        if (errno != 0 || (*endPtr) != ',' || endPtr != (char*)response)
        {
            /* An error occured when converting, the digits before 
                * the comma character was not reached, or no conversion was done.
                */
            return -1;
        }

        /* Move to the character that is after the comma character. */
        endPtr++;

        /* Read in the right motor velocity value from the response message. */
        *right_motor_velocity = strtod((char*)response, &endPtr);

        if (errno != 0 || (*endPtr) != '\n' || endPtr != (char*)response)
        {
            /* An error occured when converting, the digits before 
                * the comma character was not reached, or no conversion was done.
                */
            return -1;
        }
        //std::cout << "Velocity: " << *left_motor_velocity << "," << *right_motor_velocity << std::endl;

        /* Successful conversion. */
        return 0;
    }
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP