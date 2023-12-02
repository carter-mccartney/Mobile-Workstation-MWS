/* These are the links to code and examples for using POSIX functions for opening a serial port. */
// https://stackoverflow.com/questions/18108932/reading-and-writing-to-serial-port-in-c-on-linux
// https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c

#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

#include <cstring>
#include <cstdlib>
#include <sstream>
#include <iostream>

#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions

#include <libserial/SerialPort.h>
#include <experimental/filesystem>
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
  std::string portName = "";
  int fd = 0;
  #define BUFFER_SIZE  1024

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
        this->port->SetBaudRate(LibSerial::BaudRate::BAUD_38400);
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
            this->port->GetFileDescriptor();
            this->port->Close();
            break;
          }
        }
      }
      catch(const LibSerial::AlreadyOpen& ex){}
      catch(const LibSerial::OpenFailed& ex){}

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

  /* Attempts a serial connection with the Arduino by opening the serial port unsing both the
   * port name provided in the argument.
   * Issues a command to listen to a response from the arduino that is equivilent to 
   * "This is Arduino\n". Returns a zero upon success, and a negative one otherwise.
   */
  int connect(const char port_name[])
  {
    std::cout << "In the connect function." << std::endl;

    // If the port has not been found, then...
    if (!this->findSerialPort())
    {
      // TODO: Light indicator that tells the user of the MWS that the Raspberry Pi is not connected to the Arduino.

      /* Failed to find the arduino serial port. */
      std::cout << "Failed to find serial port." << std::endl;
      return -1;
    }
    else
    {
      std::cout << "Found serial port." << std::endl;
    }

    /* Open the port with file flags 'O_RDWR' which grant read and write permissions 
     * and 'O_NOCTTY' if the path-name refers to a 'terminal device-see' aka, a 'tty'.
     * Returns a non-negative file descriptor upon success and negative one upon failure. 
     */
    errno = 0; // Documented in: https://www.man7.org/linux/man-pages/man3/errno.3.html
    fd = open(port_name, O_RDWR | O_NOCTTY | O_SYNC); // Documented in: https://www.man7.org/linux/man-pages/man2/open.2.html
    if (fd < 0)
    {
      std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
      return -1;
    }
    else
    {
      std::cout << "Opened the serial port." << std::endl;
    }

    struct termios tty; // Documented in: https://man7.org/linux/man-pages/man3/termios.3.html along with descriptions of flags.
    errno = 0;

    /* Error Handling */
    if (tcgetattr (fd, &tty) != 0) 
    {
      std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
      return -1;
    }

    /* Set Baud Rate to 38400 */
    errno = 0;
    if (cfsetospeed (&tty, B38400) != 0)
    {
      std::cout << "Error " << errno << " from cfsetospeed: " << strerror(errno) << std::endl;
      return -1;
    }

    errno = 0;
    if (cfsetispeed (&tty, B38400) != 0)
    {
      std::cout << "Error " << errno << " from cfsetispeed: " << strerror(errno) << std::endl;
      return -1;
    }

    /* Setting serial port parameters. */
    tty.c_cflag &= ~CSIZE; tty.c_cflag |= CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 100;            // 30 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~PARENB; // no parity bit
    tty.c_cflag &= ~CSTOPB; // Only need one stop bit.
    tty.c_cflag &= ~CRTSCTS; // No hardware flow control.

    /* Make raw */
    //cfmakeraw(&tty);

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* Flush Port, then applies attributes */
    tcflush(fd, TCIFLUSH);  // Documented in: https://www.man7.org/linux/man-pages/man3/tcflush.3p.html
    if (tcsetattr(fd, TCSANOW, &tty) != 0) 
    {
      std::cout << "Error " << errno << " from tcsetattr" << std::endl;
    }

    //Write the ID command.
    tcflush(fd, TCIFLUSH);
    unsigned char getID[] = "\nID\n";
    if (send_msg(getID, 5) == 0)
    {
      std::cout << "Sent ID command." << std::endl;
    }
    else
    {
      std::cout << "Failed to send ID command." << std::endl;
      return -1;
    }

    // Wait a reasonable time for a response.
    usleep(1000000); // 1 second for now.

    // Retrieve the response.
    unsigned char response[BUFFER_SIZE] = {0};

    if (receive_msg(response) == 0)
    {
      std::cout << "Received a message." << std::endl;
    }
    else
    {
      std::cout << "Failed to receive a message." << std::endl;
      return -1;
    }

    /* This is the expected response to the command "ID\n". */
    unsigned char correctResponse[] = "This is Arduino\n";

    /* Compares the first 16 bytes of the two strings. */
    if(strncmp((const char*)response, (const char*)correctResponse, 16) == 0) // Documented in: https://man7.org/linux/man-pages/man3/strcmp.3.html
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

  /* Disconnects the serial port. */
  void disconnect()
  {
    portName = "";
    close(fd); // Documented in: https://www.man7.org/linux/man-pages/man2/close.2.html
  }

  /* Returns true if the serial port is connected, false otherwise. */
  bool connected() const
  {
    return true; //FIX ME!!!
  }

  /* Receives a message char array through the serial port. 
   * Returns zero upon success and negative one otherwise.
   */
  int receive_msg(unsigned char response[BUFFER_SIZE])
  {
    int n = 0;
    int index = 0;
    unsigned char buffer = '\0';
    unsigned char TERMINATING_CHARACTER = '\n';
    errno = 0;

    /* Set all bytes of the response array to null terminating characters. */
    memset(response, '\0', BUFFER_SIZE);

    do {
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

  /* Sends a message char array through the serial port. */
  int send_msg(const unsigned char cmd[], ssize_t messageLength)
  {
    tcflush(fd, TCOFLUSH);
    if (write(fd, cmd, messageLength - 1) != messageLength - 1) // Documented in: https://man7.org/linux/man-pages/man2/write.2.html
    {
      return -1;
    }
    else
    {
      return 0;
    }
  }

  /* Send float values represented in ascii characters to set the speed of both the right and left motor.
   * left_motor_rpm is the speed of the left motor in meters per second.
   * right_motor_rpm is the speed of the right motor in meters per second.
   * Returns zero upon success and negative one otherwise.
   */
  int set_motor_values(float left_motor_rpm, float right_motor_rpm)
  {
    int buffer_size = 20;

    /* Goal is to create an unsigned character array with the format "\nVLLLLL.LLLLL,RRRRR,RRRRR\n" 
     * where 'L' represents the floating point digits of the left motor velocity
     * and the 'R' represents the floating point digits of the right motor velocity.
     */
    unsigned char cmd[50] = {'\n'};

    /* Keeps track of which index in the buffer we are writing to. */
    int index = 0;

    cmd[index] = '\n'; index++;
    cmd[index] = 'V'; index++;

    int attemptedWriteSize = snprintf((char*)(cmd + index), buffer_size, "%.10f", left_motor_rpm); // Documented in: https://linux.die.net/man/3/snprintf

    /* Error had occured in the float to character array conversion process or
     * result was truncated. Buffer could not hold large enough value. 
     */
    if (attemptedWriteSize < 0  || attemptedWriteSize >= buffer_size) 
    {
      return -1;
    }
    index += attemptedWriteSize;

    cmd[index] = ','; index++;

    attemptedWriteSize = snprintf((char*)(cmd + index), buffer_size, "%.10f", right_motor_rpm);

    /* Error had occured in the float to character array conversion process or
     * result was truncated. Buffer could not hold large enough value. 
     */
    if (attemptedWriteSize < 0  || attemptedWriteSize >= buffer_size) 
    {
      return -1;
    }
    index += attemptedWriteSize;

    cmd[index] = '\n'; index++;

    std::cout << "sending: " << cmd << std::endl;

    if (send_msg(cmd, index) != 0)
    {
      /* An error occured while writing the message to serial port. */
      return -1;
    }

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

  /* returns the current port name, whether connected or not. */
  std::string getSerialPort()
  {
    return portName;
  }
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP