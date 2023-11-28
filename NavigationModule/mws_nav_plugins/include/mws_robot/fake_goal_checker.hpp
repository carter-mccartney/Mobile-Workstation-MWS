

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace mws_robot
{
    class FakeGoalChecker : public nav2_core::GoalChecker
    {
    public:
        FakeGoalChecker();
        // Standard GoalChecker Interface
        void initialize(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            const std::string &plugin_name,
            const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
        void reset() override;
        bool isGoalReached(
            const geometry_msgs::msg::Pose &query_pose, const geometry_msgs::msg::Pose &goal_pose,
            const geometry_msgs::msg::Twist &velocity) override;
        bool getTolerances(
            geometry_msgs::msg::Pose &pose_tolerance,
            geometry_msgs::msg::Twist &vel_tolerance) override;

    protected:
        // Dynamic parameters handler
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
        std::string plugin_name_;

        /**
         * @brief Callback executed when a paramter change is detected
         * @param parameters list of changed parameters
         */
        rcl_interfaces::msg::SetParametersResult
        dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
    };
}