#include <string>
#include <cstring>
#include <unistd.h>
#include <unistd.h>
#include <libserial/SerialPort.h>
using namespace LibSerial;

// Represents an ESP32 microcontroller used for measuring RSSI.
typedef struct
{
    // The position of the ESP32 in the x-axis.
    float x_coord;

    // The position of the ESP32 in the y-axis.
    float y_coord;

    // The name of the ESP32.
    std::string Name;

    // The number corresponding to the ESP32.
    int number;

    // The serial port associated with it.
    LibSerial::SerialPort* port;
} Esp;