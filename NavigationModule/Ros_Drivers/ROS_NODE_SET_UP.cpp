#include <rclcpp/rclcpp.hpp>
//#include "ESP32_comms.hpp"
//#include "ros2/tf2/Quaternion.h"
//HEADERS
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64.hpp>
#include <example_interfaces/msg/string.hpp>
#include <example_interfaces/msg/u_int8.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2_msgs/msg/tf_message.hpp>
#include "rclcpp/time.hpp"
#include "ESP.h"
#include<pthread.h>
#include <libserial/SerialPort.h>
#include <vector>
#include <string>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <math.h>
#include<cmath>
#include <fstream>
#include "ESP.h"
#include <libgen.h>
#include <linux/limits.h>
#include "Mapping.h"
#include <experimental/filesystem>
//#include <tf2_ros/transform_listener.h>
//#include<tf2/LinearMath/Quaternion.hpp>
//#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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
    //printf("Found path is %s\n",result);
    //printf("It should work read and confirm\n");
    char *finalPath = dirname(result);
   //printf("Found Path is %s\n", finalPath);
  
  }
  string convert(result);
  //use the base char array to string converter
  string fileLocation = convert + fileName;
  //std::cout << "Found string = " << fileLocation << "\n";
  newESP.Name = name;
  //std::cout << fileLocation + "\n";
  ///printf("Attempting to open a file\n");
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
  //printf("The read in value is: %f",num);
  
  newESP.x_coord =num;
  for(int i = 0; i < 16; i ++)
    {
      length[i] = '0';
    }
  file.getline(length,15);
  num = std::atof(length);
  newESP.y_coord = num;
  //printf("The read in value is: %f",num);

  }
  else
  {
    //printf("File not found abort");
    
  }
  return newESP;
} 

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
        //cout<<listOfPorts[i];
        //cout<<"\n";
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
           // std::cout << line;
            port.ReadLine(line, '\n');

            //std::cout << line;
            //std::cout << "\n";

            // If specific response found, this is the correct port.
            if(line == (espName+"\n"))
            {
                //printf("Found port\n");
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
          //printf("No response back.");
        }
        port.FlushIOBuffers();
        // Close port if not correct.
        port.Close();
    }

    return name;
}

bool comparESPDistance(ESP i, ESP j) { return (i.distance < j.distance); }

bool compareESPNumber(ESP i, ESP j){return(i.number < j.number);}

class ESPName
  {
    public:

    int number;
    LibSerial::SerialPort portName;
  };
//made like this to allow port sorting with no issue 
bool threaed=false;
ESP espArray[4];
ESPName espLocations[4];

 int size = sizeof(espArray)/sizeof(espArray[0]);
        //needs to fix sorting first
      
//node class Starts here

 void *parameterPassing(void *param)
    {
        ESPNode.changeMapping(true);
        
        while(ESPNode.getMapping())
        {
            ESPNode.getDistancesCurrently();
        }
        //run the exit process
        pthread_exit(0);
        //now run
        //do the calling of the function here should be good. 
        //call the esp driver code here. 
    }





class ESP32Driver : public rclcpp::Node
{
public:
    ESP32Driver() : Node("esp32_driver") {
         // timer_ = this->create_wall_timer(
            //std::chrono::milliseconds(200),
            //std::bind(&ESP32Driver::timerCallback, this));
            //DOALL OF THE SUBSCRIPTIONS AS EVENTS, THEY WILL NOT TAKE OVER FROM EACH OTHER ONLY ONE THAN THE OTHER., 
            //use the same names for the subscriptions that it has.
            
            using std::placeholders::_1;
          this->publisherLocation = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_update",10);
          this->signatureSubscriber = this->create_subscription<example_interfaces::msg::String>("followee_signature", 1, std::bind(&ESP32Driver::followee_callback,this,_1));
          //only want active and inactive values 
          this->isRunningSubScriber = this->create_subscription<example_interfaces::msg::Bool>("follower_mode_is_running", 10,std::bind(&ESP32Driver::follower_Thread_Run,this,_1));
          this->imuValues = this->create_subscription<example_interfaces::msg::Float64>("yaw",10,std::bind(&ESP32Driver::yawChanger,this,_1));
        //CHANGED SUBSCRIBERS
        //create callbaack Nodes to read subscriber data 
          //pubMsg = "";
          //subMsg = "";   
    }

    // Startup code.....
    
    void startUp()
    {
        RCLCPP_INFO(rclcpp::get_logger("ESP32Drive"), "Starting up ESP32 driver node...please wait...");
        // if (!comms_.isconnected())
        // {
        //     comms_.disconnect();
        // }

        // espArray[0] = locationFinding("One");
        // espArray[0].number = 0;
        // comms_.espArray[1] = locationFinding("Two");
        // comms_.espArray[1].number = 1;
        // comms_.espArray[2] = locationFinding("Three");
        // comms_.espArray[2].number = 2;
        // comms_.espArray[3] = locationFinding("Four");
        // comms_.espArray[3].number = 3;

        // comms_.connect(); //***Create description file***//

        RCLCPP_INFO(rclcpp::get_logger("ESP32Drive"), "Success! ESP32 driver node started up!");

    }
    void changeMapping(bool val)
    {
      this->currentlyMapping =val;
    }
    //TO DO, figure out wwhat is going on
    std::string getCurrentlyConnectedName()
    {
        return this->connectedName;
    }

    //base getter
    bool getMapping()
    {
      return this->currentlyMapping;
    }
    void setConnectedName(std::string deviceName)
    {
        this->connectedName = deviceName;
    }
    //ADDING IN THE APPROPRIATE GETTERS AND SETTERS FOR NODE SETUP


    //NEEDS TO READ FROM THE SUBSCRIPTIONS AND PARSE OUT FROM THEM, 

    //READ EACH TO SEE IF THE IT IS CONNECTED IF NAME CHANGED AND IF CHANGED CHANGE THE NAME THEN READ,

 ///
   /**
    * \param[in]  time    The time at the start of this control loop iteration
    * \param[in]  period  The measured period of the last control loop iteration
    */
    // void publish(const rclcpp::Time &time, const rclcpp::Duration &period)
    // {
    //   // x, y, and angle from map object to publisher

    //   pubMsg.pose.position.x = x_relative_pos_; // toString(this->x_relative_pos_) & " " & toString(this->y_relative_pos_) & " " & toString(this->relative_angle_);
    //   pubMsg.pose.position.y = y_relative_pos_;
    //   pubMsg.pose.orientation = createQuaternionMsgFromYaw(relative_angle_);

    //   publisher_.publish(pubMsg); // goal_update
    // };

///
   /**
    * \param[in]  time    The time at the start of this control loop iteration
    * \param[in]  period  The measured period of the last control loop iteration
    */
    
   string currentConnectedName()
   {
    return this->connectedName;
   }

/// Read values to state interfaces.
  /**
   * Read current values from hardware to state interfaces.
   * **The method called in the (real-time) control loop.**
   *
   * \param[in]  time    The time at the start of this control loop iteration
   * \param[in]  period  The measured period of the last control loop iteration
   */
  // bool read(const rclcpp::Time & time, const rclcpp::Duration & period)
  // {
        
  //       if (!comms_.connected)
  //       {
  //          RCLCPP_INFO(rclcpp::get_logger("ESP32Drive"), "Success! ESP32 driver node started up!");
  //          return false;
  //       }

  //       // Get relative distances and angle-from-center from the map object:
  //       comms_.read_ESP_values();

  //       usleep(1000000);

  //       x_relative_pos_ = comms_.map.found_x_1;
  //       y_relative_pos_ = comms_.map.found_y_1;
  //       relative_angle_ = comms_.map.getAngleFromCenter();

  //       return true;
  // };

  void getDistancesCurrently()
  {
    this->getDistanceRepeating();
  }


  /////
  /////LOOOK

private:

    
    Mapping map;
    std::string connectedName;
    //add in a thread varaible here, might need to be a global
    pthread_t trilaterationFinding;

    pthread_attr_t threadAttributes;

    rclcpp::TimerBase::SharedPtr timer_;
    bool currentlyMapping = false;
    //ADDED IN BELOW
   //  endGoal;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisherLocation;
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr signatureSubscriber;
    rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr isRunningSubScriber;
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr rangeSubscriber;
     rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr imuValues;//test for now
    rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr finishedCalibrate;
    //STOP HERE 


    // ESP specific info for NAV2
    //ESP32Comms comms_;
   

    // Other members that contains information subscribed from
    // SLAM node(s).
    //Will need the orientation from the IMU node as such will attempt how it should look. 
    
    double roll, pitch, yaw;


    geometry_msgs::msg::PoseStamped endGoal;
    //std::msgs::msg::String subMsg;

    void yawChanger(const example_interfaces::msg::Float64 &msg)
    {
      double yaw = msg.data;
      //just example stuff to see for the IMU needs placement
    }

    void followee_callback(const example_interfaces::msg::String &msg)
    {
        string name(msg.data.c_str());//build a string from the base lib c_string and string constructors
        if(name == this->currentConnectedName())
        {
            //do nothing since already the same name
        }
        else
        {
            if(currentlyMapping)
            {
                //output error 
                return;
            }
            else
            {
            
                espLocations[0].portName.FlushIOBuffers();
                espLocations[0].portName.Write("NAME" + name + "\n");

                espLocations[1].portName.FlushIOBuffers();
                espLocations[1].portName.Write("NAME" + name + "\n");
          
                espLocations[2].portName.FlushIOBuffers();
                espLocations[2].portName.Write("NAME" + name + "\n");
          
                espLocations[3].portName.FlushIOBuffers();
                espLocations[3].portName.Write("NAME" + name + "\n");

            }
            //change the name by calling the function for the ESP Name change;
           
        }
        //check the signature if it is the same do nothing else procede,.... change a command so that the esps are running as well. 

    }

    void follower_Thread_Run(const example_interfaces::msg::Bool msg)
    {
        bool connectedValue = &msg.data;
        //confirm later
        if(connectedValue && currentlyMapping)
        {
            //if they are the same do nothing;
        }
        else if( connectedValue)
        {
            //if we need to connected runa  thread here
            //run a pthread heree
            pthread_attr_init(&threadAttributes);
            pthread_create(&trilaterationFinding,&threadAttributes,parameterPassing,NULL);
        }
        else
        {
            currentlyMapping = false;
            while(0 != pthread_tryjoin_np(trilaterationFinding,NULL))
            {
                //sleep for 5 miliseconds increase in time if needed. 
                usleep(500);
            }
            //kill the running thread
        }
    }

    

    
    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), "***Running ESP32 Driver***");
    }
    
    void getDistanceRepeating()
    {
           std::sort(espArray,espArray+size, compareESPNumber);

           espLocations[0].portName.FlushIOBuffers();
          espLocations[0].portName.Write("DISTANCE\n");

          espLocations[1].portName.FlushIOBuffers();
          espLocations[1].portName.Write("DISTANCE\n");
          
          espLocations[2].portName.FlushIOBuffers();
          espLocations[2].portName.Write("DISTANCE\n");
          
          espLocations[3].portName.FlushIOBuffers();
          espLocations[3].portName.Write("DISTANCE\n");
          
        
          //fixed
          usleep(12000000);   
        //use a gloabl and define
        //get the distances and updat eit here 
        //change values in here
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
              //printf("Found distance for ESP %d,",i);
              //std::cout<< foundDistance + "\n";
              //std::cout << std::stod(foundDistance);
              //std::cout << "\n Next \n";
              //std::cout << ((-espArray[i].RSSI - std::stod(foundDistance))/(40));
              //std::cout << "\n";
              //std::cout << "All together now\n";        

              espArray[i].distance = std::pow(10,((-69 - std::stod(foundDistance))/(40))) *100;
              //std::cout << espArray[i].distance;

            }
            else
            {
              
              //TO DO: Write Error message to the App
            }
          }
          std::sort(espArray, espArray + size,comparESPDistance);

           this->map.getCoordinates(espArray[0].x_coord, espArray[0].y_coord, espArray[1].x_coord, espArray[1].y_coord,
           espArray[2].x_coord, espArray[2].y_coord, espArray[0].distance, espArray[1].distance, espArray[2].distance);
           this->map.getAngleFromCenter();

          geometry_msgs::msg::PoseStamped message;
          //tf2_msgs::msg::TFMessage message;
          message.header.stamp = this->get_clock()->now();          //get name here
          message.header.frame_id = "map";
        endGoal.pose.position.x = this->map.found_x_1;
        endGoal.pose.position.y = this->map.found_y_1;
        endGoal.pose.position.z = 0;
          //change the postions and get the values and adjust to amtch 
        tf2::Quaternion base;
        base.setRPY(0,0,0);
        base.normalize();
        base.setX(this->map.found_x_1);
        base.setY(this->map.found_y_1);
        base.setZ(0);
        base.setRPY(roll,pitch,yaw);
        base.normalize();//should be it??

        //quat.
        
      
        endGoal.pose.orientation.x =base.getX();


        endGoal.pose.orientation.y =base.getY();

        endGoal.pose.orientation.z =base.getZ();

        endGoal.pose.orientation.w =base.getW();

       
        this->publisherLocation->publish(this->endGoal);
    }

} ESPNode;

//main node code here.
int main(int argc, char **argv)
{
    //Find the Global Variable Locations
    espArray[0] = locationFinding("One");
    espArray[0].number = 0;
    espArray[1] = locationFinding("Two");
    espArray[1].number = 1;
    espArray[2] = locationFinding("Three");
    espArray[2].number = 2;
    espArray[3] = locationFinding("Four");
    espArray[3].number = 3;

    for(int i = 1; i<5;i++ )
    {
        //find the serial port to Match
      string ardPortName = "";
      switch(i)
      {
        case 1:
        //printf("StartOne\n");
        ardPortName = findSerialPort("One");
        //espArray[i].Name= ardPortName;
        break;
        case 2:
        //printf("StartTwo\n");
        ardPortName = findSerialPort("Two");
        break;
        case 3:
        //printf("StartThree\n");
        ardPortName = findSerialPort("Three");
        break;
        case 4:
        //printf("StartFour\n");
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

    for(int i = 0; i < 4; i++)
    {
      espLocations[i].portName.Open(espArray[i].Name);
        //Set Up permission for the SSerial Ports
      espLocations[i].portName.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
      espLocations[i].portName.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_DEFAULT);
      espLocations[i].portName.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_DEFAULT);
      espLocations[i].portName.SetParity(LibSerial::Parity::PARITY_DEFAULT);
      espLocations[i].portName.SetStopBits(LibSerial::StopBits::STOP_BITS_DEFAULT);
      espLocations[i].number = i;
    }

    rclcpp::init(argc, argv);
    //might need to be made into a global
    //spin seems to run that code over and over and over again confirm if not 
    auto node = std::make_shared<rclcpp::Node>("esp32_driver");

    rclcpp::spin(node);


    // Start thread for ESP32 Driver.
    




    rclcpp::shutdown();
    return 0;
}