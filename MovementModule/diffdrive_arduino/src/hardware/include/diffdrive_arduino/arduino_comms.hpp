/* Documentation for the libserial library: https://libserial.readthedocs.io/en/latest/tutorial.html */
/* Source code for the libserial library is here: https://github.com/crayzeewulf/libserial */

#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

#include <cstring>
#include <cstdlib>
#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
#include <unistd.h>
#include <experimental/filesystem>
#include <string>
#include <vector>

namespace fs = std::experimental::filesystem;

std::vector<std::string> get_available_ports()
{
    std::vector<std::string> port_names;

    fs::path p("/dev/serial/by-id");
    try
    {

        if (exists(p))
        {
            for (auto de : fs::directory_iterator(p))
            {
                if (is_symlink(de.symlink_status()))
                {
                    fs::path symlink_points_at = read_symlink(de);
                    fs::path canonical_path = fs::canonical(p / symlink_points_at);
                    // cout << canonical_path.generic_string() << std::endl;
                    port_names.push_back(canonical_path.generic_string());
                }
            }
        }
    }
    catch (const fs::filesystem_error &ex)
    {
        std::cout << ex.what() << '\n';
        throw ex;
    }

    return port_names;
}

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
        std::vector<std::string> listOfPorts = get_available_ports();
        for (long unsigned int i = 0; i < listOfPorts.size(); i++)
        {
            // Open the port.
            bool isPortOpen = false;
            try
            {
                if(this->port == nullptr)
                {
                    this->port = new LibSerial::SerialPort();
                }
                this->port->Open(listOfPorts[i]);
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
                        break;
                    }
                }
            }
            catch(const LibSerial::AlreadyOpen& ex){}

            if(isPortOpen)
            {
                // Close port if not correct.
                this->port->Close();
            }
        }

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
            return false;
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
    void receive_msg(std::string &msg_to_receive)
    {
        if(this->connected())
        {
            this->port->ReadLine(msg_to_receive, '\n', MILLISECOND_TIMEOUT);
        }
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
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP