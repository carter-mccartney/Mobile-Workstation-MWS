// Contains ROS node source code.
#include <rclcpp/rclcpp.hpp>

// Used for topics.
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

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

private:
    // The publisher of the user's location.
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisherLocation;

    // The subscription to the user's signature.
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr signatureSubscriber;

    // The subscription to whether the location tracking is running.
    rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr isRunningSubscriber;

    // The publisher of the error message for the user.
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr messagePublisher;

    // The buffer for holding the transformations.
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;

    // The listener for transforms.
    std::shared_ptr<tf2_ros::TransformListener> tfListener;

    // The current goal to publish.
    geometry_msgs::msg::PoseStamped goal;

    // Updates the signature of the user.
    void singatureChanged(const example_interfaces::msg::String& msg)
    {
        string name(msg.data.c_str());//build a string from the base lib c_string and string constructors
        Esp32Commands::setName(name);
    }

    // Updates the running state and launches a thread to update position.
    void isRunningChanged(const example_interfaces::msg::Bool& msg);
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
        Mapping::CoordinatePair goal = Esp32Commands::findGoal();

        // Adjust to the rotation of the coordinate system.
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "base_link";
        pose.pose.position.x = goal.x;
        pose.pose.position.y = goal.y;
        pose.pose.position.z = 0;

        // Point to user.
        tf2::Quaternion q;
        double angle = atan2(goal.y, goal.x);
        if(angle < 0) 
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