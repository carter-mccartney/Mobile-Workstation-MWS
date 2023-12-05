// Contains ROS node source code.
#include <rclcpp/rclcpp.hpp>

// Used for topics.
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/int8.hpp>
#include <example_interfaces/msg/u_int8.hpp>

// Used for gettign timestamps.
#include <rclcpp/time.hpp>

// Used for converting from robot coordinates to world coordinates.
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

// Contains the commands to send to the ESP32s.
#include "commands.h"

using namespace std;

// Whether the goal is being updated.
static bool isRunning = false;

class Esp32Driver : public rclcpp::Node
{
public:
    Esp32Driver() : Node("esp32_driver")
    {
        using std::placeholders::_1;

        // Make publishers and subscribers.
        this->publisherLocation = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_update", 10);
        this->signatureSubscriber = this->create_subscription<example_interfaces::msg::String>("followee_signature", 1, std::bind(&Esp32Driver::singatureChanged, this, _1));
        this->isRunningSubscriber = this->create_subscription<example_interfaces::msg::Bool>("follower_mode_is_running", 10, std::bind(&Esp32Driver::isRunningChanged, this, _1));
        this->messagePublisher = this->create_publisher<example_interfaces::msg::String>("user_message", 10);
		this->oneSubscriber = this->create_subscription<example_interfaces::msg::Int8>("calibration_value_one", 10, std::bind(&Esp32Driver::onOneUpdated, this, _1));
		this->twoSubscriber = this->create_subscription<example_interfaces::msg::Int8>("calibration_value_two", 10, std::bind(&Esp32Driver::onTwoUpdated, this, _1));
		this->threeSubscriber = this->create_subscription<example_interfaces::msg::Int8>("calibration_value_three", 10, std::bind(&Esp32Driver::onThreeUpdated, this, _1));
		this->fourSubscriber = this->create_subscription<example_interfaces::msg::Int8>("calibration_value_four", 10, std::bind(&Esp32Driver::onFourUpdated, this, _1));
		this->newOnePublisher = this->create_publisher<example_interfaces::msg::Int8>("new_calibration_value_one", 10);
		this->newTwoPublisher = this->create_publisher<example_interfaces::msg::Int8>("new_calibration_value_two", 10);
		this->newThreePublisher = this->create_publisher<example_interfaces::msg::Int8>("new_calibration_value_three", 10);
		this->newFourPublisher = this->create_publisher<example_interfaces::msg::Int8>("new_calibration_value_four", 10);
		this->isCalibratingSubscriber = this->create_subscription<example_interfaces::msg::Bool>("is_calibrating", 10, std::bind(&Esp32Driver::onCalibrationStarted, this, _1));
		this->isCalibratingPublisher = this->create_publisher<example_interfaces::msg::Bool>("is_calibrating_return", 10);
		this->targetSubscriber = this->create_subscription<example_interfaces::msg::UInt8>("calibration_target", 10, std::bind(&Esp32Driver::targetChanged, this, _1));
        this->counterValue = 0;

        // Create transform buffer and listener.
        this->tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        this->tfListener = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer);
    }

    // Sets the current goal to publish.
    void setGoal(geometry_msgs::msg::PoseStamped goal)
    {
        this->goal.header.frame_id = goal.header.frame_id;
        this->goal.pose.position.x = goal.pose.position.x;
        this->goal.pose.position.y = goal.pose.position.y;
        this->goal.pose.position.z = goal.pose.position.z;
        this->goal.pose.orientation.w = goal.pose.orientation.w;
        this->goal.pose.orientation.x = goal.pose.orientation.x;
        this->goal.pose.orientation.y = goal.pose.orientation.y;
        this->goal.pose.orientation.z = goal.pose.orientation.z;
    }

    // Publishes the goal provided.
    void publishGoal() 
    {
        this->goal.header.stamp = this->get_clock()->now();
        this->publisherLocation->publish(this->goal);
    }

    // Publishes the given message for the user.
    void publishMessage(std::string message) 
    {
        RCLCPP_INFO(this->get_logger(), message.c_str());
        example_interfaces::msg::String topicMessage;
        topicMessage.data = message;
        this->messagePublisher->publish(topicMessage);
    }

    // Transforms the pose provided using the tf2_ros::Buffer transform.
    geometry_msgs::msg::PoseStamped transformPose(geometry_msgs::msg::PoseStamped& pose, geometry_msgs::msg::PoseStamped& newPose,
                                                  const std::string frame_id, tf2::Duration timeout)
    {
        return this->tfBuffer->transform<geometry_msgs::msg::PoseStamped>(pose, newPose, frame_id, timeout);
    }
    //this below is used to stop multiple bad data points from the ESP32's
    void increaseCounter()
    {
        this->counterValue++;
    }
    int getCounterValue()
    {
        return this->counterValue;
    }
    void resetCounter()
    {
        this->counterValue = 0;
    }
private:
    // The publisher of the user's location.
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisherLocation;

    // The subscription to the user's signature.
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr signatureSubscriber;

    // The subscription to whether the location tracking is running.
    rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr isRunningSubscriber;

    // The publisher of the error message for the user.
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr messagePublisher;

	// The subscriber for the calibration of one.
	rclcpp::Subscription<example_interfaces::msg::Int8>::SharedPtr oneSubscriber;

	// The publisher for the calibration of one.
	rclcpp::Publisher<example_interfaces::msg::Int8>::SharedPtr newOnePublisher;

	// The subscriber for the calibration of two.
	rclcpp::Subscription<example_interfaces::msg::Int8>::SharedPtr twoSubscriber;

	// The publisher for the calibration of two.
	rclcpp::Publisher<example_interfaces::msg::Int8>::SharedPtr newTwoPublisher;

	// The subscriber for the calibration of three.
	rclcpp::Subscription<example_interfaces::msg::Int8>::SharedPtr threeSubscriber;

	// The publisher for the calibration of three.
	rclcpp::Publisher<example_interfaces::msg::Int8>::SharedPtr newThreePublisher;

	// The subscriber for the calibration of four.
	rclcpp::Subscription<example_interfaces::msg::Int8>::SharedPtr fourSubscriber;

	// The publisher for the calibration of four.
	rclcpp::Publisher<example_interfaces::msg::Int8>::SharedPtr newFourPublisher;

	// The subscriber for whether calibration is on-going.
	rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr isCalibratingSubscriber;

	// The publisher for the whether calibration is on-going.
	rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr isCalibratingPublisher;

	// The subscriber for the calibration target.
	rclcpp::Subscription<example_interfaces::msg::UInt8>::SharedPtr targetSubscriber;

    // The buffer for holding the transformations.
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;

    // The listener for transforms.
    std::shared_ptr<tf2_ros::TransformListener> tfListener;

    // The current goal to publish.
    geometry_msgs::msg::PoseStamped goal;

    // The current target of calibration.
    uint8_t calibrationTarget;
    
    int counterValue;

    // Updates the signature of the user.
    void singatureChanged(const example_interfaces::msg::String& msg)
    {
        string name(msg.data.c_str());//build a string from the base lib c_string and string constructors
        Esp32Commands::setName(name);
    }

    // Updates the running state and launches a thread to update position.
    void isRunningChanged(const example_interfaces::msg::Bool& msg);

    // Updates the set calibration value of one.
    void onOneUpdated(const example_interfaces::msg::Int8& message)
    {
        RCLCPP_INFO(this->get_logger(), ("One Updated to " + std::to_string(message.data)).c_str());
        Esp32Commands::loadCalibration(esps[0], message.data);
    }

    // Updates the set calibration value of two.
    void onTwoUpdated(const example_interfaces::msg::Int8& message)
    {
        RCLCPP_INFO(this->get_logger(), ("Two Updated to " + std::to_string(message.data)).c_str());
        Esp32Commands::loadCalibration(esps[1], message.data);
    }

    // Updates the set calibration value of three.
    void onThreeUpdated(const example_interfaces::msg::Int8& message)
    {
        RCLCPP_INFO(this->get_logger(), ("Three Updated to " + std::to_string(message.data)).c_str());
        Esp32Commands::loadCalibration(esps[2], message.data);
    }

    // Updates the set calibration value of four.
    void onFourUpdated(const example_interfaces::msg::Int8& message)
    {
        RCLCPP_INFO(this->get_logger(), ("Four Updated to " + std::to_string(message.data)).c_str());
        Esp32Commands::loadCalibration(esps[3], message.data);
    }

    // Updates the current target.
    void targetChanged(const example_interfaces::msg::UInt8& message)
    {
        RCLCPP_INFO(this->get_logger(), ("Target is " + std::to_string(message.data)).c_str());
        this->calibrationTarget = message.data;
    }

    // Performs the calibration of the target.
    void onCalibrationStarted(const example_interfaces::msg::Bool& message)
    {
        RCLCPP_INFO(this->get_logger(), ("Calibration Started. Calibrating " + std::to_string(this->calibrationTarget)).c_str());
        int result = Esp32Commands::calibrate(esps[this->calibrationTarget - 1]);
        if(this->calibrationTarget == 1)
        {
            example_interfaces::msg::Int8 valueMessage;
            valueMessage.data = result;
            this->newOnePublisher->publish(valueMessage);
        }
        else if(this->calibrationTarget == 2)
        {
            example_interfaces::msg::Int8 valueMessage;
            valueMessage.data = result;
            this->newTwoPublisher->publish(valueMessage);
        }
        else if(this->calibrationTarget == 3)
        {
            example_interfaces::msg::Int8 valueMessage;
            valueMessage.data = result;
            this->newThreePublisher->publish(valueMessage);
        }
        else
        {
            example_interfaces::msg::Int8 valueMessage;
            valueMessage.data = result;
            this->newFourPublisher->publish(valueMessage);
        }
        example_interfaces::msg::Bool output;
        output.data = false;
        this->isCalibratingPublisher->publish(output);
    }
};

// The global reference to the node.
static std::shared_ptr<Esp32Driver> node;

// Gets the current readings of position and publishes them until stopped.
void getPosition()
{
    // Continue while not directed to stop.
    while(isRunning)
    {
        // Get the goal.
        std::cout << "Finding point" << std::endl;
        Mapping::CoordinatePair goal = Esp32Commands::findGoal();
        std::cout << "Found x value " << goal.x << "\n";
        std::cout << "Found y value " << goal.y <<"\n";
        // Adjust to the rotation of the coordinate system.
        //needs to use old values if it returns 0
        if (( - 0.05 <= goal.x <= 0.050 && (-0.05 <= goal.y && node->getCounterValue() < 10)
        {
            node->increaseCounter();

            node->publishGoal();
        }
        else
        {
            node->resetCounter();
			geometry_msgs::msg::PoseStamped pose;
			pose.header.frame_id = "base_link";
			pose.pose.position.x = goal.x;
			pose.pose.position.y = goal.y;
			pose.pose.position.z = 0;

			// Point to user.
			tf2::Quaternion q;
			double angle = atan2(goal.y, goal.x);
			if (angle < 0)
			{
				angle += 2 * Mapping::PI;
			}
			q.setRPY(0, 0, angle);
			q.normalize();
			pose.pose.orientation = tf2::toMsg(q);

			// Compute the transform.
			geometry_msgs::msg::PoseStamped newPose;
			newPose = node->transformPose(pose, newPose, "odom", tf2::durationFromSec(1000));

			// Save the new goal.
			node->setGoal(newPose);
        }

    }
}

void sendGoal()
{
    while(isRunning)
    {
        node->publishGoal();
        usleep(1000);
    }
}

int main(int argc, char* argv[])
{
    if(!Esp32Commands::init())
    {
        // There is an error. TODO: Make some notification method.
        printf("ESP32's not connected or configured.");
        return -1;
    }
    else
    {
        rclcpp::init(argc, argv);
        node = std::make_shared<Esp32Driver>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }
}

void Esp32Driver::isRunningChanged(const example_interfaces::msg::Bool& msg)
{
    bool connectedValue = msg.data;
    if(!isRunning && connectedValue)
    {
        // Launch a thread to get the position readings.
        isRunning = true;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "base_link";
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
        pose.pose.orientation.w = 1;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        this->goal = node->transformPose(pose, this->goal, "odom", tf2::durationFromSec(0));
        std::thread readingThread(getPosition);
        readingThread.detach();
        std::thread publishingThread(sendGoal);
        publishingThread.detach();
    }
    else if(isRunning && !connectedValue)
    {
        isRunning = false;
    }
}
