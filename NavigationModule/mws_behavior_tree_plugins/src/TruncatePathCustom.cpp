#include <string>
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

#include "../include/mws_behavior_tree_plugins/TruncatePathCustom.hpp"

namespace mws_robot
{
    TruncatePathCustom::TruncatePathCustom(const std::string& name, const BT::NodeConfiguration& conf) : BT::ActionNodeBase(name, conf)
    {
    }

    inline BT::NodeStatus TruncatePathCustom::tick()
    {
        this->setStatus(BT::NodeStatus::RUNNING);

        nav_msgs::msg::Path input_path;
        double distance;

        this->getInput("input_path", input_path);
        this->getInput("distance", distance);

        if (input_path.poses.empty()) 
        {
            this->setOutput("output_path", input_path);
            return BT::NodeStatus::SUCCESS;
        }

        geometry_msgs::msg::PoseStamped final_pose = input_path.poses.back();

        double distance_to_goal = nav2_util::geometry_utils::euclidean_distance(input_path.poses.back(), final_pose);

        while (distance_to_goal < distance && input_path.poses.size() > 2) 
        {
            input_path.poses.pop_back();
            distance_to_goal = nav2_util::geometry_utils::euclidean_distance(input_path.poses.back(), final_pose);
        }

        double dx = final_pose.pose.position.x - input_path.poses.back().pose.position.x;
        double dy = final_pose.pose.position.y - input_path.poses.back().pose.position.y;

        double final_angle = atan2(dy, dx);

        if(std::isnan(final_angle) || 
           std::isinf(final_angle)) 
        {
            RCLCPP_WARN(this->config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
            "Final angle is not valid while truncating path. Setting to 0.0");
            final_angle = 0.0;
        }

        input_path.poses.back().pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(final_angle);

        this->setOutput("output_path", input_path);

        return BT::NodeStatus::SUCCESS;
    }

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mws_robot::TruncatePathCustom>("TruncatePathCustom");
}