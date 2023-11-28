// Adapted from https://docs.ros.org/en/iron/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html and 
// https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

// Navigate message structure.
#include <nav2_msgs/action/navigate_to_pose.hpp>

// Current position message structure.
#include <tf2_msgs/msg/tf_message.hpp>

// General position message structure.
#include <geometry_msgs/msg/pose_stamped.hpp>

// User message structure.
#include <example_interfaces/msg/string.hpp>

// Is running topis structure.
#include <example_interfaces/msg/bool.hpp>

// Basic ROS functionality.
#include <rclcpp/rclcpp.hpp>

// ROS actions.
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// Used for converting from robot coordinates to world coordinates.
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

// Contians metrics for current time.
#include <chrono>

// The last time recorded.
static std::chrono::time_point<std::chrono::system_clock> lastTime;

// Helper for conversion of time to UTC.
time_t toUTC(std::tm& timeinfo)
{
    time_t tt = timegm(&timeinfo);
    return tt;
}

// Helper for creating a date time.
std::chrono::system_clock::time_point createDateTime(int year, int month, int day, int hour, int minute, int second) // these are UTC values
{
    tm timeInfo = tm();
    timeInfo.tm_year = year - 1900;
    timeInfo.tm_mon = month - 1;
    timeInfo.tm_mday = day;
    timeInfo.tm_hour = hour;
    timeInfo.tm_min = minute;
    timeInfo.tm_sec = second;
    tm time = timeInfo;
    time_t tt = toUTC(time);
    return std::chrono::system_clock::from_time_t(tt);
}

namespace MwsNavigation
{
    class MwsNavigationNode : public rclcpp::Node
    {
    public:
        // Create a new navigation node.
        MwsNavigationNode() : Node("mws_navigation_node")
        {
            // Create the subscription and publisher for the current position.
            using namespace std::placeholders;

            // Create the subscription for the is running topic.
            this->isRunningSubscriber = this->create_subscription<example_interfaces::msg::Bool>("follower_mode_is_running", 10, std::bind(&MwsNavigationNode::onIsRunningUpdated, this, _1));

            // Create publisher for user messages.
            this->messagePublisher = this->create_publisher<example_interfaces::msg::String>("user_message", 10);
        }

        //Checks whether navigation is currently running.
        bool isNavigationRunning()
        {
            return this->isRunning;
        }

        // Publishes the message to the user.
        void publishMessage(std::string message)
        {
            RCLCPP_INFO(this->get_logger(), message.c_str());
            example_interfaces::msg::String topicMessage;
            topicMessage.data = message; 
            this->messagePublisher->publish(topicMessage);
        }
        
    private:
        // The publisher of the user's location.
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisherLocation;

        // The publisher for the message.
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr messagePublisher;

        // The last value of is running recorded.
        bool isRunning;

        // The subscription for whether it is running.
        rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr isRunningSubscriber;

        // Changes the state of the navigation.
        void onIsRunningUpdated(const example_interfaces::msg::Bool& isRunning);
    };

    class MwsNavigationClient : public rclcpp::Node
    {
    public:
        // Shorthand for long namespace and type.
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandler = rclcpp_action::ClientGoalHandle<NavigateToPose>;

        // Create a new Client node.
        explicit MwsNavigationClient(const rclcpp::NodeOptions& options) : Node("mws_navigation_controller", options)
        {
            // Create the client for the navigation.
            this->client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

            // Create transform buffer and listener.
            this->tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            this->tfListener = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer);
        }

        // Starts the navigation action with the default behavior tree.
        void startNavigating();

        // Cancels the navigation service.
        void stopNavigating()
        {
            RCLCPP_INFO(this->get_logger(), "Stopping Navigation");
            
            // Wait for sending to be done.
            while(this->isSendingRequest) usleep(100);

            // Check if request was successful.
            if(this->goalHandler.get() != nullptr)
            {
                using namespace std::placeholders;

                // Cancel the goal.
                this->client->async_cancel_goal(this->goalHandler.get(), nullptr);
                RCLCPP_INFO(this->get_logger(), "Navigation Stopped");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Navigation Not Stopped");
            }
        }
    
    private:
        // Whether a request is being sent.
        bool isSendingRequest;

        // The handle for the goal responses.
        std::shared_future<std::shared_ptr<GoalHandler>> goalHandler;

        // The action client object.
        rclcpp_action::Client<NavigateToPose>::SharedPtr client;

        // The buffer for holding the transformations.
        std::shared_ptr<tf2_ros::Buffer> tfBuffer;

        // The listener for transforms.
        std::shared_ptr<tf2_ros::TransformListener> tfListener;

        // The callback for when a goal was initiated.
        void goalResponseCallback(const GoalHandler::SharedPtr& goal_handle)
        {
            this->isSendingRequest = false;
            if(!goal_handle) 
            {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            } 
            else 
            {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        }

        // Processes feedback given.
        void feedBackCallback(const GoalHandler::SharedPtr& goal_handle) 
        { 
        }

        // The callback for when a goal is reached.
        void resultCallback(const GoalHandler::WrappedResult& result);
    };

    // The node for publishing and subscribing to topics.
    std::shared_ptr<MwsNavigationNode> globalNode;

    // The client for interacting with nav2.
    std::shared_ptr<MwsNavigationClient> globalClient;

    void MwsNavigationNode::onIsRunningUpdated(const example_interfaces::msg::Bool& isRunning)
    {
        // Do not set if already set.
        if(this->isRunning != isRunning.data)
        {
            RCLCPP_INFO(this->get_logger(), "Mode changed");

            // Record the last value.
            this->isRunning = isRunning.data;

            // Start or stop navigating.
            if(this->isRunning)
            {
                RCLCPP_INFO(this->get_logger(), "Starting Navigation");
                globalClient->startNavigating();
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Stopping Navigation");
                globalClient->stopNavigating();
            }
        }
    }

    void MwsNavigationClient::startNavigating()
    {
        using namespace std::placeholders;

        if (!this->client->wait_for_action_server()) 
        {
            // Could not connect.
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        }
        else
        {
            // Build action message using the adapted follower tree with recovery and the current postion.
            auto goal_msg = NavigateToPose::Goal();
            goal_msg.pose.header.frame_id = "odom";
            goal_msg.behavior_tree = "/home/developer/BehaviorTrees/navigate_to_pose_w_replanning_and_recovery.xml";
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.orientation.w = 1;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 0;
            pose.header.frame_id = "base_link";
            geometry_msgs::msg::PoseStamped newPose;
            newPose = this->tfBuffer->transform<geometry_msgs::msg::PoseStamped>(pose, newPose, "odom", tf2::durationFromSec(0));
            goal_msg.pose.pose.orientation.w = newPose.pose.orientation.w; 
            goal_msg.pose.pose.orientation.x = newPose.pose.orientation.x; 
            goal_msg.pose.pose.orientation.y = newPose.pose.orientation.y; 
            goal_msg.pose.pose.orientation.z = newPose.pose.orientation.z; 
            goal_msg.pose.pose.position.x = newPose.pose.position.x;
            goal_msg.pose.pose.position.y = newPose.pose.position.y;
            goal_msg.pose.pose.position.z = newPose.pose.position.z;

            RCLCPP_INFO(this->get_logger(), ("Sending goal: (" + std::to_string(newPose.pose.position.x) + ", " 
                                                               + std::to_string(newPose.pose.position.y) + ", " 
                                                               + std::to_string(newPose.pose.position.z) + ")").c_str());

            // Send the goal.
            auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&MwsNavigationClient::goalResponseCallback, this, _1);
            send_goal_options.feedback_callback = std::bind(&MwsNavigationClient::feedBackCallback, this, _1);
            send_goal_options.result_callback = std::bind(&MwsNavigationClient::resultCallback, this, _1);
            goal_msg.pose.header.stamp = this->get_clock()->now();
            this->goalHandler = this->client->async_send_goal(goal_msg, send_goal_options);
            this->isSendingRequest = true;
            RCLCPP_INFO(this->get_logger(), "Goal Sent");
        }
    }

    void MwsNavigationClient::resultCallback(const GoalHandler::WrappedResult& result)
    {
        std::chrono::time_point<std::chrono::system_clock> currentTime = std::chrono::system_clock::now();
        RCLCPP_INFO(this->get_logger(), "Navigation Stopped");
        switch(result.code) 
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                return;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");

                // Retry navigation.
                if(globalNode->isNavigationRunning())
                {
                    this->startNavigating();
                }
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                
                // Notify user if they haven't recently been notified.
                if(std::chrono::duration_cast<std::chrono::minutes>(currentTime - lastTime).count() > 30)
                {
                    lastTime = currentTime;
                    globalNode->publishMessage("There was a navigation error.");
                }
                return;
        }
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(MwsNavigation::MwsNavigationClient)

using namespace MwsNavigation;

void serviceMain()
{
    rclcpp::spin(globalClient);
}

int main(int argc, char* argv[])
{
    // Set start time.
    lastTime = createDateTime(1970, 1, 1, 0, 0, 0);

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    globalNode = std::make_shared<MwsNavigationNode>();
    globalClient = std::make_shared<MwsNavigationClient>(options);

    std::thread serviceThread(serviceMain);

    rclcpp::spin(globalNode);
    rclcpp::shutdown();
    serviceThread.join();
    return 0;
}