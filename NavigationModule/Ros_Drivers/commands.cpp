#include <unistd.h>
#include <libserial/SerialPort.h>
#include <vector>
#include <string>
#include <cstring>
#include <iostream>
#include <experimental/filesystem>
#include <algorithm>
#include <math.h>
#include<cmath>
#include <fstream>
#include "ESP.h"
#include <libgen.h>
#include <linux/limits.h>
#include "Mapping.h"
using namespace std;
using namespace LibSerial;

namespace fs = std::experimental::filesystem;

std::vector<std::string> get_available_ports() {
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



// STORES IT GLOBALLY??? DOES IT??
/*
 * Finds the serial port for the Arduino contorller for the motor and stores it globally.
 * 
 */
string findSerialPort(string espName)
{
    SerialPort port;
    //bool isFound = false;
    string name = "";

    // Get all ports available to find the correct one.
    vector<string> listOfPorts = get_available_ports();
    for (int i = 0; i < listOfPorts.size(); i++)
    {
        // Open the port.
        port.Open(listOfPorts[i]);
        cout<<listOfPorts[i];
        cout<<"\n";
        port.FlushIOBuffers();
  
        usleep(1000000); // Enough time for setup for port to open. ***IMPORTANT***

        // Set port parameters.
        port.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
        port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_DEFAULT);
        port.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_DEFAULT);
        port.SetParity(LibSerial::Parity::PARITY_DEFAULT);
        port.SetStopBits(LibSerial::StopBits::STOP_BITS_DEFAULT);

        //Write the ID command.
        port.FlushIOBuffers();
        port.Write("ID\n");

        // Wait a reasonable time for a response.
        usleep(1000000); // 1 second for now.

        // If no response, do nothing.
        if(port.IsDataAvailable())
        {
            // Check the response given.
            string line = listOfPorts[i];
            port.ReadLine(line, '\n');
            // ARENT YOU FLUSHING NECESSARY INFO HERE???? THE SPECIFIC RESPONSE BELOW MUST BE RELATED TO THE SECOND LINE OF INPUT THEN???
            std::cout << line;
            port.ReadLine(line, '\n');

            std::cout << line;
            std::cout << "\n";

            // If specific response found, this is the correct port.
            if(line == (espName+"\n"))
            {
                printf("Found port\n");
                //isFound = true;
                //port.FlushIOBuffers();
                //port.Write("MOVE\n");
                //usleep(1000000);
                name = listOfPorts[i];
                //break;
                
            }
        }
        else
        {
          printf("No response back.");
        }
        port.FlushIOBuffers();
        // Close port if not correct.
        port.Close();
    }

    return name;
}


struct{
  double found_x_1;
  double found_y_1;
}FoundValue;


//the SerialPort is breaking it on the sort make it seperate and link with the number.
//can sort by number and then distance each time. 



  
  
bool comparESPDistance(ESP i, ESP j) { return (i.distance < j.distance); }

bool compareESPNumber(ESP i, ESP j){return(i.number < j.number);}


// void getCoordinates(double x_1, double y_1, double x_2, double y_2, double x_3, double y_3, double d_1, double d_2, double d_3)
// {
// 	//change from doubles to int to be use take the last 3 decimals
// //makes the first x_1 and y_1 the shortest values and subtract those from the distances. 
// 	int X_1 = x_1 * 1000;
// 	int Y_1 = y_1 * 1000;
// 	int X_2 = x_2 * 1000;
// 	int Y_2 = y_2 * 1000;
// 	int X_3 = x_3 * 1000;
// 	int Y_3 = y_3 * 1000;
// 	//change system so that we can find the distance easier
// 	X_3 -= X_1;
// 	Y_3 -= Y_1;
// 	X_2 -= X_1;
// 	Y_2 -= Y_1;
// 	X_1 -= X_1;
// 	Y_1 -= Y_1;

// 	//needs confirmation that location of X_2  and Y_2 is never zero if so swap 
// 	//Formula is now
// 	// x^2 +y^2 = d_1^2
// 	// (x-X_2)^2 + y^2 = d_2^2
// 	//
// 	//found by deriving 
// 	if (X_2 == 0)
// 	{
// 		//this means swap X_2 and X_3
// 		int place = X_2;
// 		X_2 = X_3;
// 		X_3 = place;
// 		place = Y_2;
// 		Y_2 = Y_3;
// 		Y_3 = place;
// 		std::cout << "Swapped";
// 	}

// 	double foundX = (pow(d_2, 2) - pow(d_1, 2) - pow((double)X_2 / 1000, 2)) / (-2 * ((double)X_2) / 1000);

// 	//use teh 3rd point to find y
// 	double foundY = sqrt(pow(d_2, 2) - pow(foundX - (double)X_2 / 1000, 2));
// 	FoundValue.found_x_1 = foundX;
// 	//oundY = pow(d_2, 2) - pow(foundX + (double)X_2/1000, 2);
// 	//found1 is +- in the kit 
// 	//test it out using point 3
// 	double distance1 = sqrt(pow(foundX - (double)X_3 / 1000, 2) + (pow(foundY - (double)Y_3 / 1000, 2)));
// 	double offFactor1 = abs(((distance1 - d_3) / (d_3)) * 100);
// 	double distance2 = sqrt(pow(foundX - (double)X_3 / 1000, 2) + (pow(-foundY - (double)Y_3 / 1000, 2)));
// 	double offFactor2 = abs((((distance2 - d_3) / (d_3)) * 100));
// 	if (offFactor1 <= (offFactor2))
// 	{
// 		FoundValue.found_y_1 = foundY;
		
// 	}
// 	else
// 	{
// 		FoundValue.found_y_1 = -foundY;
// 	}

// }


class ESPName
  {
    public:

    int number;
    LibSerial::SerialPort portName;
  };


ESP locationFinding(string name)
{
  string fileName = "/ESP_Locations/ESP" + name+".txt";
  std::ifstream file;
  ESP newESP;
  char result[100];
  for( int i = 0; i <100 ;i++)
  {
    result[i] = '\0';
  }
  ssize_t pathConfirm = readlink("/proc/self/exe",result,100); 
  if(pathConfirm!= -1)
  {
    printf("Found path is %s\n",result);
    printf("It should work read and confirm\n");
    char *finalPath = dirname(result);
   printf("Found Path is %s\n", finalPath);
  
  }
  string convert(result);
  //use the base char array to string converter
  string fileLocation = convert + fileName;
  std::cout << "Found string = " << fileLocation << "\n";
  newESP.Name = name;
  std::cout << fileLocation + "\n";
  printf("Attempting to open a file\n");
  file.open(fileLocation,std::ifstream::in);

  if(file.is_open())
  {
    char length[16]; //SHOULDNT THIS BE 16? ESPECIALY IF YOU ARE READING AT MOST 15 CHARS?
    for(int i = 0; i < 16; i ++)
    {
      length[i] = '0';
    }
  file.getline(length,15);
  double num = std::atof(length);
  printf("The read in value is: %f",num);
  
  newESP.x_coord =num;
  for(int i = 0; i < 16; i ++)
    {
      length[i] = '0';
    }
  file.getline(length,15);
  num = std::atof(length);
  newESP.y_coord = num;
  printf("The read in value is: %f",num);

  }
  else
  {
    printf("File not found abort");
    
  }
  return newESP;
} 

int main()
{

  Mapping map;
  ESP espArray[4];
  ESPName espLocations[4];
  //for the location finding show the trilateration working with the ESPArray points in 
  //a solid state enviroment
  espArray[0] = locationFinding("One");
  espArray[0].number = 0;
  espArray[1] = locationFinding("Two");
  espArray[1].number = 1;
  espArray[2] = locationFinding("Three");
  espArray[2].number = 2;
  espArray[3] = locationFinding("Four");
  espArray[3].number = 3;
  
  //return -1;
  //SerialPort espPort
  //check to see if all names can be found 
    for(int i = 1; i<5;i++ )
    {
      string ardPortName = "";
      switch(i)
      {
        case 1:
        printf("StartOne\n");
        ardPortName = findSerialPort("One");
        //espArray[i].Name= ardPortName;
        break;
        case 2:
        printf("StartTwo\n");
        ardPortName = findSerialPort("Two");
        break;
        case 3:
        printf("StartThree\n");
        ardPortName = findSerialPort("Three");
        break;
        case 4:
        printf("StartFour\n");
        ardPortName = findSerialPort("Four");
        break;

        
      }
      if (ardPortName == "")
        {
          std::cout << "Failed to find ESP " + std::to_string(i) + "\n"; // For right now, debugging for the port.
          return -1;
        } // DO WE WANT MAIN TO JUST DIE IF WE CANT FIND ONE ESP. THAT MAKES SENSE TO ME, BUT DOES THIS ALLOW ENOUH=GH TIME TO FIND THEM PROPERLY?
      espArray[i-1].Name= ardPortName;
      // WHY DONT YOU JUST START THE LOOP AT 0? THEN YOU DONT NEED TO DO THE SEBTRACTION FOR THESE NAMES..
    }
    
    
    // If the port has not been found, then... 
    printf("All have been found procede\n");


    // Do whatever else you want.

    for(int i = 0; i < 4; i++)
    {
      espLocations[i].portName.Open(espArray[i].Name);

      espLocations[i].portName.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
      espLocations[i].portName.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_DEFAULT);
      espLocations[i].portName.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_DEFAULT);
      espLocations[i].portName.SetParity(LibSerial::Parity::PARITY_DEFAULT);
      espLocations[i].portName.SetStopBits(LibSerial::StopBits::STOP_BITS_DEFAULT);
      espLocations[i].number = i;// IS THIS NECESARY? SEE LINE 275-285
    }
    char cmd[10];//WHY DECLARE SIZE 10 WHEN YOU ONLYUSE 7
    for (int a = 0; a < 7; a++) cmd[a] = 0;
    

      usleep(1000000);

      // Set port parameters.
      
    
    while (true)
    {
       cin.read(cmd,7);

       int b = 0;

      if (strstr(cmd, "QUIT") != NULL) // WOAH HAVENT SEEEN TIS BEFORE. IS THIS LIKE STRCMP?
      {
          break;
      }

      if (strstr(cmd,"PIN") != NULL)
      {
       printf("Starting PinPointFinding \n");


        int size = sizeof(espArray)/sizeof(espArray[0]);
        //needs to fix sorting first
         std::sort(espArray,espArray+size, compareESPNumber);
        //std::sort(espArray,espArray + 4, comparESPDistance);
          espLocations[0].portName.FlushIOBuffers();
          espLocations[0].portName.Write("DISTANCE\n");

          espLocations[1].portName.FlushIOBuffers();
          espLocations[1].portName.Write("DISTANCE\n");
          
          espLocations[2].portName.FlushIOBuffers();
          espLocations[2].portName.Write("DISTANCE\n");
          
          espLocations[3].portName.FlushIOBuffers();
          espLocations[3].portName.Write("DISTANCE\n");
          
          printf("Sleep for 8 seconds. \n"); 
          //fixed
          usleep(10000000);
          for(int i = 0; i < 4; i++)
          {
            if(espLocations[i].portName.IsDataAvailable())
            {
              //do data now
              string foundDistance;
              espLocations[i].portName.ReadLine(foundDistance,'\n');
              //have to clear the esps each time 
              espLocations[i].portName.ReadLine(foundDistance,'\n');

              //10 ^ ((-69 -(<RSSI_VALUE>))/(10 * 2))
              printf("Found distance for ESP %d,",i);
              //std::cout<< foundDistance + "\n";
              std::cout << std::stod(foundDistance);
              std::cout << "\n Next \n";
              std::cout << ((-espArray[i].RSSI - std::stod(foundDistance))/(40));
              std::cout << "\n";
              std::cout << "All together now\n";        

              espArray[i].distance = std::pow(10,((-69 - std::stod(foundDistance))/(40))) *100;
              std::cout << espArray[i].distance;

            }
            else
            {
              //break -1;
              //do proper error handling later
              printf("The port number %d, did ot work",i);
              return -1;
            }
          }
          //do pinpointing
            //espPort1.Write("MOVE\n");
            std::sort(espArray, espArray + size,comparESPDistance);
            //IS THIS EVEN SETTING ANYTHING? BECAUSE THE FUNCTION RETURNS VOID, AND THE VAR IS NOT PASSED AND IT ISNT GLOBAL
          map.getCoordinates(espArray[0].x_coord, espArray[0].y_coord, espArray[1].x_coord, espArray[1].y_coord,
           espArray[2].x_coord, espArray[2].y_coord, espArray[0].distance, espArray[1].distance, espArray[2].distance);

        cout << "X Coord : " + std::to_string(map.found_x_1) + "\n";

        cout << "Y Coord : " + std::to_string(map.found_y_1) + "\n";
      }
      if(strstr(cmd,"CALIBRATE")!= NULL)
      {
        int size = sizeof(espArray)/sizeof(espArray[0]);
        //needs to fix sorting first
         std::sort(espArray,espArray+size, compareESPNumber);
        //ask user for the correct pin
        printf("Please input the desired ESP to calibrate as a number\n");
        string calibrationInput;
        char c  = getchar();
        if( isdigit(c) != true)
        {
          printf("invalid INput please try agin");
        }
        else if( 1 <= std::stoi(calibrationInput) <=4 )
        {
          int selected = std::stoi(calibrationInput);
          printf("Please set or hold the communication device 1 meter away from the desired ESP. Hit enter when ready");
          std::cin;//just to stop the code and wait for input
          // THIS NEEDS TO BE -1 AGAIN SINCE YOU'RE SWITCHING THIS UP AND DOING 1-4 INSTEAD OF 0-3
          espLocations[selected-1].portName.FlushIOBuffers();
          espLocations[selected-1].portName.Write("CALIBRATE\n");
          usleep(32000000);// WHY 32 SECONDS?
          if(espLocations[selected-1].portName.IsDataAvailable())
          {
            // DO YOU ALWAYS NEED TO CLEAR SOMETHING HERE??
            espLocations[selected-1].portName.ReadLine(calibrationInput,'\n');

            espLocations[selected-1].portName.ReadLine(calibrationInput,'\n');
            printf("THe found distance for the RSSI is %s\n,",calibrationInput);

            espArray[selected-1].RSSI = std::stoi(calibrationInput);

            //have the found calibrated esps RRSI value save into the specifc ESP

          }
        }
        else
        {
          printf("Invalid ESP's selected, only from 1-4 is allowed");
        }
        
      }

      
// WHY SLEEP .1 SECONDS?
       usleep(100000);
// WHAT ARE THESE FOR?
       int h = 0;

       int c = 0;

       
    }

    for(int i = 0; i < 4; i ++)
    {
      espLocations[i].portName.Close();
    }
   // espPort1.Close();

}


