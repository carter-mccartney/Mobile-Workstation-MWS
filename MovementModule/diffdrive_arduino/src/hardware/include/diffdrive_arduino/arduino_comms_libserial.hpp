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

//namespace fs = std::experimental::filesystem;

/* Converts integer baud rate to libserial-defined baud rate. */
LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

private:
  std::string portName = "";
  LibSerial::BaudRate buadRate;
  LibSerial::SerialPort serial_conn_;

  /* Number of milliseconds to wait to return if a line termination character is not read
   * from a read operation on a serial port. */
  const size_t MILLISECOND_TIMEOUT = 0;

public:

  ArduinoComms() = default;

  /* Attempts to connect to the arduino serial port by iterating through each port and
   * issueing a command to listen to a response from the arduino that is equivilent to 
   * "This is Arduino\n". Returns a zero upon success, and a negative one otherwise.
   */
  int connect(int32_t baud_rate)
  {  
    buadRate = convert_baud_rate(baud_rate);

    std::string serial_device = findSerialPort();

     // If the port has not been found, then... 
    if (portName == "")
    {
      // TODO: Light indicator that tells the user of the MWS that the Raspberry Pi is not connected to the Arduino.

      /* Failed to find the arduino serial port. */
      return -1;
    }

    /* Successfully connected to the arduino serial port. */
    return 0;
  }

  /* Attempts a serial connection with the Arduino by opening the serial port unsing both the
   * port name and the baud rate provided in the argument.
   * Issues a command to listen to a response from the arduino that is equivilent to 
   * "This is Arduino\n". Returns a zero upon success, and a negative one otherwise.
   */
  int connect(int32_t baud_rate, std::string port_name)
  {
    buadRate = convert_baud_rate(baud_rate);

    // Open the port.
    serial_conn_.Open(port_name);
    serial_conn_.FlushIOBuffers();

    usleep(1000000); // Enough time for setup for port to open. ***IMPORTANT***

    // Set port parameters.
    serial_conn_.SetBaudRate(buadRate);
    serial_conn_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_DEFAULT);
    serial_conn_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_DEFAULT);
    serial_conn_.SetParity(LibSerial::Parity::PARITY_DEFAULT);
    serial_conn_.SetStopBits(LibSerial::StopBits::STOP_BITS_DEFAULT);

    //Write the ID command.
    serial_conn_.FlushIOBuffers();
    serial_conn_.Write("ID\n");

    // Wait a reasonable time for a response.
    usleep(1000000); // 1 second for now.

    // If no response, do nothing.
    if(serial_conn_.IsDataAvailable())
    {
        // Check the response given.
        std::string line = "";
        serial_conn_.ReadLine(line, '\n', MILLISECOND_TIMEOUT);

        // If specific response found, this is the correct port.
        if(line == "This is Arduino\n")
        {
          /* Successfully connected to the arduino serial port. */
          portName = port_name;
          return 0;
        }
        else
        {
          /* Serial port does not respond correctly to command issued. */
          return -1;
        }
    }
    else
    {
      /* No data coming from this serial port. */
      return -1;
    }
  }

  /* Disconnects the serial port. */
  void disconnect()
  {
    portName = "";
    serial_conn_.Close();
  }

  /* Returns true if the serial port is connected, false otherwise. */
  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  /* Receives a message string through the serial port. */
  void receive_msg(std::string &msg_to_receive)
  {
    serial_conn_.ReadLine(msg_to_receive, '\n', MILLISECOND_TIMEOUT);
  }

  /* Sends a message string through the serial port. */
  void send_msg(const std::string &msg_to_send)
  {
    serial_conn_.FlushIOBuffers();
    serial_conn_.Write(msg_to_send);
  }

  /* Send float values to set the speed of both the right and left motor.
   * left_motor_rpm is the speed of the left motor in meters per second.
   * right_motor_rpm is the speed of the right motor in meters per second.
   */
  void set_motor_values(float left_motor_rpm, float right_motor_rpm)
  {
    std::stringstream ss;
    ss << "V" << left_motor_rpm << "," << right_motor_rpm << "\n";
    send_msg(ss.str());
  }

private:
/*
    std::vector<std::string> get_available_ports() 
    {
      std::vector<std::string> port_names;

      fs::path p("/dev/serial/by-id");
      try {
      
        if (exists(p)) {
          for (auto de : fs::directory_iterator(p)) {
            if (is_symlink(de.symlink_status())) {
              fs::path symlink_points_at = read_symlink(de);
              fs::path canonical_path = fs::canonical(p / symlink_points_at);
              //cout << canonical_path.generic_string() << std::endl;
              port_names.push_back(canonical_path.generic_string());
            }
          }
        }
      } catch (const fs::filesystem_error &ex) {
        cout << ex.what() << '\n';
        throw ex;
      }
    
      return port_names;
    }
*/
  std::string findSerialPort()
  {
      std::string seriaPortName = ""; /*

      // Get all ports available to find the correct one.
      std::vector<string> listOfPorts = get_available_ports();
      for (long unsigned int i = 0; i < listOfPorts.size(); i++)
      {
        // Open the port.
        serial_conn_.Open(listOfPorts[i]);
        serial_conn_.FlushIOBuffers();
  
        usleep(1000000); // Enough time for setup for port to open. ***IMPORTANT***

        // Set port parameters.
        serial_conn_.SetBaudRate(buadRate);
        serial_conn_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_DEFAULT);
        serial_conn_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_DEFAULT);
        serial_conn_.SetParity(LibSerial::Parity::PARITY_DEFAULT);
        serial_conn_.SetStopBits(LibSerial::StopBits::STOP_BITS_DEFAULT);

        //Write the ID command.
        serial_conn_.FlushIOBuffers();
        serial_conn_.Write("ID\n");

        // Wait a reasonable time for a response.
        usleep(1000000); // 1 second for now.

        std::string line = "";

        // If no response, do nothing.
        if(serial_conn_.IsDataAvailable())
        {
            // Check the response given.
            serial_conn_.ReadLine(line, '\n', MILLISECOND_TIMEOUT);

            // If specific response found, this is the correct port.
            if(line == "This is Arduino\n")
            {
                seriaPortName = listOfPorts[i];
                break;
            }
        }

        // Close port if not correct.
        serial_conn_.Close();
      } */

    return seriaPortName;
  } 

  /* returns the current port name, whether connected or not. */
  std::string getSerialPort()
  {
    return portName;
  }
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP