
#include "mws_robot/fake_goal_checker.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace mws_robot
{
    FakeGoalChecker::FakeGoalChecker()
    {
    }

    void FakeGoalChecker::initialize(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        const std::string &plugin_name,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
    {
        plugin_name_ = plugin_name;
        auto node = parent.lock();

        // Add callback for dynamic parameters
        dyn_params_handler_ = node->add_on_set_parameters_callback(
            std::bind(&FakeGoalChecker::dynamicParametersCallback, this, _1));
    }

    void FakeGoalChecker::reset()
    {
    }

    bool FakeGoalChecker::isGoalReached(
        const geometry_msgs::msg::Pose &query_pose, const geometry_msgs::msg::Pose &goal_pose,
        const geometry_msgs::msg::Twist &)
    {
        return false;
    }

    bool FakeGoalChecker::getTolerances(
        geometry_msgs::msg::Pose &pose_tolerance,
        geometry_msgs::msg::Twist &vel_tolerance)
    {
        return true;
    }

    rcl_interfaces::msg::SetParametersResult
    FakeGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mws_robot::FakeGoalChecker, nav2_core::GoalChecker)