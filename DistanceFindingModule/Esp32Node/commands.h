#include <unistd.h>
#include <libserial/SerialPort.h>
#include <vector>
#include <string>
#include <cstring>
#include <iostream>
#include <experimental/filesystem>
#include <fstream>
#include "Esp.h"
#include "Mapping.h"
using namespace std;
using namespace LibSerial;
namespace fs = std::experimental::filesystem;

// The set of ESP32s.
Esp esps[4];

namespace Esp32Commands
{
    std::vector<std::string> get_available_ports()
    {
        std::vector<std::string> port_names;

        fs::path p("/dev/serial/by-id");
        try
        {

            if(exists(p))
            {
                for(auto de : fs::directory_iterator(p))
                {
                    if(is_symlink(de.symlink_status()))
                    {
                        fs::path symlink_points_at = read_symlink(de);
                        fs::path canonical_path = fs::canonical(p / symlink_points_at);
                        // cout << canonical_path.generic_string() << std::endl;
                        port_names.push_back(canonical_path.generic_string());
                    }
                }
            }
        }
        catch(const fs::filesystem_error& ex)
        {
            std::cout << ex.what() << '\n';
            throw ex;
        }

        return port_names;
    }

    /*
    * Finds the serial port for the Arduino contorller for the motor and stores it in the structure.
    *
    */
    bool findSerialPort(Esp& esp)
    {
        bool isPortFound = false;
        std::string seriaPortName = "";

        // Get all ports available to find the correct one.
        vector<string> listOfPorts = get_available_ports();
        for(long unsigned int i = 0; i < listOfPorts.size(); i++)
        {
            bool isPortOpen = false;
            try
            {
                if(esp.port == nullptr)
                {
                    esp.port = new LibSerial::SerialPort();
                }

                // Open the port.
                esp.port->Open(listOfPorts[i]);
                isPortOpen = true;
                esp.port->FlushIOBuffers();

                usleep(1000000); // Enough time for setup for port to open. ***IMPORTANT***

                // Set port parameters.
                esp.port->SetBaudRate(LibSerial::BaudRate::BAUD_9600);
                esp.port->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_DEFAULT);
                esp.port->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_DEFAULT);
                esp.port->SetParity(LibSerial::Parity::PARITY_DEFAULT);
                esp.port->SetStopBits(LibSerial::StopBits::STOP_BITS_DEFAULT);

                //Write the ID command.
                esp.port->FlushIOBuffers();
                esp.port->Write("ID\n");

                // Wait a reasonable time for a response.
                usleep(1000000); // 1 second for now.

                // If no response, do nothing.
                if(esp.port->IsDataAvailable())
                {
                    // Check the response given.
                    string line = listOfPorts[i];

                    // ESP32 has a bug in serial communications, so read to the end of the line to get the data we actually want.
                    esp.port->ReadLine(line, '\n');
                    esp.port->ReadLine(line, '\n');

                    // If specific response found, this is the correct port.
                    if(line == (esp.Name + "\n"))
                    {
                        isPortFound = true;
                        break;
                    }
                }
            }
            catch(const LibSerial::AlreadyOpen& ex) {}
            catch(const LibSerial::OpenFailed& ex) {}


            if(isPortOpen)
            {
                esp.port->FlushIOBuffers();
                esp.port->Close();
            }
        }

        if(!isPortFound)
        {
            delete esp.port;
            esp.port = nullptr;
        }
        return isPortFound;
    }

    // Loads the location of the ESP.
    void locationFinding(Esp& esp)
    {
        string fileName = "~/MwsConfig/ESP" + esp.Name + ".txt";
        std::ifstream file;
        file.open(fileName, std::ifstream::in);

        if(file.is_open())
        {
            char length[16];
            for(int i = 0; i < 16; i++)
            {
                length[i] = '\0';
            }
            file.getline(length, 15);
            double num = std::atof(length);

            esp.x_coord = num;
            for(int i = 0; i < 16; i++)
            {
                length[i] = '\0';
            }
            file.getline(length, 15);
            num = std::atof(length);
            esp.y_coord = num;
        }
    }

    bool init() 
    {
        //for the location finding show the trilateration working with the ESPArray points in 
        //a solid state enviroment
        locationFinding(esps[0]);
        esps[0].number = 1;
        esps[0].Name = "One";
        locationFinding(esps[1]);
        esps[1].number = 2;
        esps[1].Name = "Two";
        locationFinding(esps[2]);
        esps[2].number = 3;
        esps[2].Name = "Three";
        locationFinding(esps[3]);
        esps[3].number = 4;
        esps[3].Name = "Four";

        //check to see if all names can be found 
        return findSerialPort(esps[0]) &&
               findSerialPort(esps[1]) &&
               findSerialPort(esps[2]) &&
               findSerialPort(esps[3]);
    }

    void setName(std::string name) 
    {
        // Send the name to each.
        esps[0].port->FlushIOBuffers();
        esps[0].port->Write("NAME" + name.substr(0, 8) + "\n");
        esps[1].port->FlushIOBuffers();
        esps[1].port->Write("NAME" + name.substr(0, 8) + "\n");
        esps[2].port->FlushIOBuffers();
        esps[2].port->Write("NAME" + name.substr(0, 8) + "\n");
        esps[3].port->FlushIOBuffers();
        esps[3].port->Write("NAME" + name.substr(0, 8) + "\n");
    }

    Mapping::CoordinatePair findGoal() 
    {
        // Send the distance command to each.
        esps[0].port->FlushIOBuffers();
        esps[0].port->Write("DISTANCE\n");
        esps[1].port->FlushIOBuffers();
        esps[1].port->Write("DISTANCE\n");
        esps[2].port->FlushIOBuffers();
        esps[2].port->Write("DISTANCE\n");
        esps[3].port->FlushIOBuffers();
        esps[3].port->Write("DISTANCE\n");

        // Read the result from each.
        while(!esps[0].port->IsDataAvailable()) usleep(100000);
        string distanceString;
        esps[0].port->ReadLine(distanceString, '\n');
        esps[0].port->ReadLine(distanceString, '\n'); // Read twice due to ESP32 bug.
        double distance1 = std::stod(distanceString);
        while(!esps[1].port->IsDataAvailable()) usleep(100000);
        esps[1].port->ReadLine(distanceString, '\n');
        esps[1].port->ReadLine(distanceString, '\n'); // Read twice due to ESP32 bug.
        double distance2 = std::stod(distanceString);
        while(!esps[2].port->IsDataAvailable()) usleep(100000);
        esps[2].port->ReadLine(distanceString, '\n');
        esps[2].port->ReadLine(distanceString, '\n'); // Read twice due to ESP32 bug.
        double distance3 = std::stod(distanceString);
        while(!esps[3].port->IsDataAvailable()) usleep(100000);
        esps[3].port->ReadLine(distanceString, '\n');
        esps[3].port->ReadLine(distanceString, '\n'); // Read twice due to ESP32 bug.
        double distance4 = std::stod(distanceString);

        // Find the smallest distance and use the two adjacent ESP32s.
        Esp esp1;
        Esp esp2;
        Esp esp3;
        double d_1;
        double d_2;
        double d_3;
        if(distance1 <= distance2 &&
        distance1 <= distance3 &&
        distance1 <= distance4) 
        {
            // Find intersection with two adjacent ESP32s to ESP 1.
            esp1 = esps[0];
            d_1 = distance1;

            // First one shares x-coordinate.
            if(esps[1].x_coord == esp1.x_coord) 
            {
                esp2 = esps[1];
                d_2 = distance2;
            }
            else if(esps[2].x_coord == esp1.x_coord) 
            {
                esp2 = esps[2];
                d_2 = distance3;
            }
            else 
            {
                esp2 = esps[3];
                d_2 = distance4;
            }

            // Other one shares y-coordinate.
            if(esps[1].y_coord == esp1.y_coord)
            {
                esp3 = esps[1];
                d_3 = distance2;
            }
            else if(esps[2].y_coord == esp1.y_coord)
            {
                esp3 = esps[2];
                d_3 = distance3;
            }
            else
            {
                esp3 = esps[3];
                d_3 = distance4;
            }
        }
        else if(distance2 <= distance1 &&
                distance2 <= distance3 &&
                distance2 <= distance4) 
        {
            // Find intersection with two adjacent ESP32s to ESP 2.
            esp1 = esps[1];
            d_1 = distance2;

            // First one shares x-coordinate.
            if(esps[0].x_coord == esp1.x_coord)
            {
                esp2 = esps[0];
                d_2 = distance1;
            }
            else if(esps[2].x_coord == esp1.x_coord)
            {
                esp2 = esps[2];
                d_2 = distance3;
            }
            else
            {
                esp2 = esps[3];
                d_2 = distance4;
            }

            // Other one shares y-coordinate.
            if(esps[0].y_coord == esp1.y_coord)
            {
                esp3 = esps[0];
                d_3 = distance1;
            }
            else if(esps[2].y_coord == esp1.y_coord)
            {
                esp3 = esps[2];
                d_3 = distance3;
            }
            else
            {
                esp3 = esps[3];
                d_3 = distance4;
            }
        }
        else if(distance3 <= distance1 &&
                distance3 <= distance2 &&
                distance3 <= distance4)
        {
            // Find intersection with two adjacent ESP32s to ESP 3.
            esp1 = esps[2];
            d_1 = distance3;

            // First one shares x-coordinate.
            if(esps[1].x_coord == esp1.x_coord)
            {
                esp2 = esps[1];
                d_2 = distance2;
            }
            else if(esps[0].x_coord == esp1.x_coord)
            {
                esp2 = esps[0];
                d_2 = distance1;
            }
            else
            {
                esp2 = esps[3];
                d_2 = distance4;
            }

            // Other one shares y-coordinate.
            if(esps[1].y_coord == esp1.y_coord)
            {
                esp3 = esps[1];
                d_3 = distance2;
            }
            else if(esps[0].y_coord == esp1.y_coord)
            {
                esp3 = esps[0];
                d_3 = distance1;
            }
            else
            {
                esp3 = esps[3];
                d_3 = distance4;
            }
        }
        else 
        {
            // Find intersection with two adjacent ESP32s to ESP 4.
            esp1 = esps[3];
            d_1 = distance4;

            // First one shares x-coordinate.
            if(esps[1].x_coord == esp1.x_coord)
            {
                esp2 = esps[1];
                d_2 = distance2;
            }
            else if(esps[2].x_coord == esp1.x_coord)
            {
                esp2 = esps[2];
                d_2 = distance3;
            }
            else
            {
                esp2 = esps[0];
                d_2 = distance1;
            }

            // Other one shares y-coordinate.
            if(esps[1].y_coord == esp1.y_coord)
            {
                esp3 = esps[1];
                d_3 = distance2;
            }
            else if(esps[2].y_coord == esp1.y_coord)
            {
                esp3 = esps[2];
                d_3 = distance3;
            }
            else
            {
                esp3 = esps[0];
                d_3 = distance1;
            }
        }

        // Find the coordinate pair of the point.
        return Mapping::findIntersection(esp1.x_coord, esp1.y_coord, 
                                        esp2.x_coord, esp2.y_coord, 
                                        esp3.x_coord, esp3.y_coord, 
                                        d_1, d_2, d_3);
    }

    int calibrate(Esp& esp) 
    {
        esp.port->FlushIOBuffers();
        esp.port->Write("CALIBRATE\n");
        while(!esp.port->IsDataAvailable()) usleep(1000);

        string calibrationOutput;
        esp.port->ReadLine(calibrationOutput, '\n');
        esp.port->ReadLine(calibrationOutput, '\n'); // Read twice due to ESP#2 bug.

        return std::stoi(calibrationOutput);
    }

    void loadCalibration(Esp& esp, int rssi) 
    {
        esp.port->FlushIOBuffers();
        esp.port->Write("SETCAL" + std::to_string(rssi) + "\n");
    }
}

