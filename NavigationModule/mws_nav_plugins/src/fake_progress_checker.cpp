
#include "mws_robot/fake_progress_checker.hpp"
#include "pluginlib/class_list_macros.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace mws_robot
{
    void FakeProgressChecker::initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name)
    {
        plugin_name_ = plugin_name;
        auto node = parent.lock();

        // Add callback for dynamic parameters
        dyn_params_handler_ = node->add_on_set_parameters_callback(
            std::bind(&FakeProgressChecker::dynamicParametersCallback, this, _1));
    }

    bool FakeProgressChecker::check(geometry_msgs::msg::PoseStamped & current_pose)
    {
        return true;
    }

    void FakeProgressChecker::reset()
    {
    }

    rcl_interfaces::msg::SetParametersResult
    FakeProgressChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

}

PLUGINLIB_EXPORT_CLASS(mws_robot::FakeProgressChecker, nav2_core::ProgressChecker)