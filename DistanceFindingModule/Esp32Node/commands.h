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
                    std::string fileName = de.path().filename().string();
                    std::cout << fileName << std::endl;

                    if(fileName != "usb-Arduino__www.arduino.cc__0042_5593034353635170C0D0-if00" &&
                       fileName != "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0")
                    {
                        if(is_symlink(de.symlink_status()))
                        {
                            fs::path symlink_points_at = read_symlink(de);
                            fs::path canonical_path = fs::canonical(p / symlink_points_at);
                            port_names.push_back(canonical_path.generic_string());
                        }
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
    	printf("Attempting to find %s\n", esp.Name.c_str());
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
    		printf("Attempting to open %s\n", listOfPorts[i].c_str());
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
    		printf("Sending ID\n");
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
		    printf("Returned %s\n", line.c_str());

                    // If specific response found, this is the correct port.
                    if(line == (esp.Name + "\n"))
                    {
                        isPortFound = true;
                        break;
                    }
                }
            }
            catch (const LibSerial::AlreadyOpen& ex) { printf("Path already opened \n"); }
            catch (const LibSerial::OpenFailed& ex) { printf("Serial Port failed on open\n"); }


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

    bool init() 
    {
        //for the location finding show the trilateration working with the ESPArray points in 
        //a solid state enviroment
        esps[0].number = 1;
        esps[0].Name = "One";
        esps[0].x_coord = -0.29;
        esps[0].y_coord = -0.30;
        esps[1].number = 2;
        esps[1].Name = "Two";
        esps[1].x_coord = 0.29;
        esps[1].y_coord = -0.30;
        esps[2].number = 3;
        esps[2].Name = "Three";
        esps[2].x_coord = -0.29;
        esps[2].y_coord = 0.30;
        esps[3].number = 4;
        esps[3].Name = "Four";
        esps[3].x_coord = 0.29;
        esps[3].y_coord = 0.30;

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

        //TO DO: Change the code to work without a potential lock up



        // Read the result from each.
        usleep(12000000);//sleep for the appropriate time 
        string distanceString;
        esps[0].port->ReadLine(distanceString, '\n'); //add in a clear and check for each 
        while(!esps[0].port->IsDataAvailable()) usleep(10000);

        esps[0].port->ReadLine(distanceString, '\n'); // Read twice due to ESP32 bug.
        double distance1 = std::stod(distanceString);
        esps[1].port->ReadLine(distanceString, '\n'); //add in a clear and check for each 
        while(!esps[1].port->IsDataAvailable()) usleep(10000);
        
        esps[1].port->ReadLine(distanceString, '\n'); // Read twice due to ESP32 bug.
        double distance2 = std::stod(distanceString);
        esps[2].port->ReadLine(distanceString, '\n'); //add in a clear and check for each
        while(!esps[2].port->IsDataAvailable()) usleep(10000);
       
        esps[2].port->ReadLine(distanceString, '\n'); // Read twice due to ESP32 bug.
        double distance3 = std::stod(distanceString);
        esps[3].port->ReadLine(distanceString, '\n'); //add in a clear and check for each
        while(!esps[3].port->IsDataAvailable()) usleep(10000);
       
        esps[3].port->ReadLine(distanceString, '\n'); // Read twice due to ESP32 bug.
        double distance4 = std::stod(distanceString);

        // Find the smallest distance and use the two adjacent ESP32s.
        /*
            This displays the ESP locations from a Top-Down, Eagle-eye perspective.

                        Back of MWS
                3                           4

                        Center


                1                           2
                        Front

        */
        Esp esp1;
        Esp esp2;
        Esp esp3;
        double d_1;
        double d_2;
        double d_3;
        if(distance1 <= distance2 &&
        distance1 <= distance3 &&
        distance1 <= distance4) 
        { // if the shortest distance is ESP1, then ESPS 2 and 3 are adjacent, so use those.
            // Find intersection with two adjacent ESP32s to ESP 1.
            esp1 = esps[0];
            d_1 = distance1;
            esp2 = esps[1];
            d_2 = distance2;
            esp3 = esps[2];
            d_3 = distance3;
        }
        else if(distance2 <= distance1 &&
                distance2 <= distance3 &&
                distance2 <= distance4) 
        { // if the shortest distance is ESP2, then ESPS 1 and 4 are adjacent, so use those.
            // Find intersection with two adjacent ESP32s to ESP 2.
            esp1 = esps[1];
            d_1 = distance2;
            esp2 = esps[0];
            d_2 = distance1;
            esp3 = esps[3];
            d_3 = distance4;

        }
        else if(distance3 <= distance1 &&
                distance3 <= distance2 &&
                distance3 <= distance4)
        { // if the shortest distance is ESP3, then ESPS 1 and 4 are adjacent, so use those.
            // Find intersection with two adjacent ESP32s to ESP 3.
            esp1 = esps[2];
            d_1 = distance3;
            esp2 = esps[0];
            d_2 = distance1;
            esp3 = esps[3];
            d_3 = distance4;
	}
        else // distance4 is the shortest
        { // if the shortest distance is ESP4, then ESPS 2 and 3 are adjacent, so use those.
            // Find intersection with two adjacent ESP32s to ESP 4.
            esp1 = esps[3];
            d_1 = distance4;
            esp2 = esps[1];
            d_2 = distance2;
            esp3 = esps[2];
            d_3 = distance3;

        }

        // Find the coordinate pair of the point.
        printf("Finding intersection: %s %3.2f %s %3.2f %s %3.2f\n", esp1.Name.c_str(), d_1, esp2.Name.c_str(), d_2, esp3.Name.c_str(), d_3);
        return Mapping::findIntersection(esp1.x_coord, esp1.y_coord, 
                                        esp2.x_coord, esp2.y_coord, 
                                        esp3.x_coord, esp3.y_coord, 
                                        d_1, d_2, d_3);
    }

    int calibrate(Esp& esp) 
    {
        esp.port->FlushIOBuffers();
        esp.port->Write("CALIBRATE\n");
        string calibrationOutput;
        esp.port->ReadLine(calibrationOutput, '\n');
        usleep(40000000);//sleep for longer.
        while(!esp.port->IsDataAvailable()) usleep(1000);

      
        esp.port->ReadLine(calibrationOutput, '\n'); // Read twice due to ESP#2 bug.

        return std::stoi(calibrationOutput);
    }

    void loadCalibration(Esp& esp, int rssi) 
    {
        esp.port->FlushIOBuffers();
        esp.port->Write("SETCAL" + std::to_string(rssi) + "\n");
    }
}

