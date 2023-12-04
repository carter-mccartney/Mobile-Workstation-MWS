
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/progress_checker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace mws_robot
{
    class FakeProgressChecker : public nav2_core::ProgressChecker
    {
    public:
        void initialize(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
            const std::string & plugin_name) override;
        bool check(geometry_msgs::msg::PoseStamped & current_pose) override;
        void reset() override;

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