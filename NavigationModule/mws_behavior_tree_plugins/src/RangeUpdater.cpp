#include <string>
#include <memory>

#include <example_interfaces/msg/float64.hpp>
#include <behaviortree_cpp_v3/decorator_node.h>

#include "../include/mws_behavior_tree_plugins/RangeUpdater.hpp"

#include <rclcpp/rclcpp.hpp>

namespace mws_robot
{

    using std::placeholders::_1;

    RangeUpdater::RangeUpdater(const std::string& name, const BT::NodeConfiguration& conf) : BT::DecoratorNode(name, conf)
    {
        this->lastRange = -1.0;
        this->node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        this->callbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        this->callbackGroupExecutor.add_callback_group(this->callbackGroup, node->get_node_base_interface());

        std::string rangeUpdaterTopic;
        this->node->get_parameter_or<std::string>("range_updater_topic", rangeUpdaterTopic, "range_update");

        rclcpp::SubscriptionOptions subscriptionOptions;
        subscriptionOptions.callback_group = this->callbackGroup;
        this->rangeSubscriber = this->node->create_subscription<example_interfaces::msg::Float64>(rangeUpdaterTopic, 
                                                                                                  rclcpp::SystemDefaultsQoS(), 
                                                                                                  std::bind(&RangeUpdater::onRangeChanged, this, _1), 
                                                                                                  subscriptionOptions);
    }

    inline BT::NodeStatus RangeUpdater::tick()
    {
        double range;

        this->getInput("input_range", range);

        this->callbackGroupExecutor.spin_some();

        if(this->lastRange > 0.0) range = this->lastRange;

        this->setOutput("output_range", range);
        return this->child_node_->executeTick();
    }

    void RangeUpdater::onRangeChanged(const example_interfaces::msg::Float64::SharedPtr msg)
    {
        RCLCPP_INFO(this->node->get_logger(), ("Range set to " + std::to_string(msg->data)).c_str());
        this->lastRange = msg->data;
    }

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mws_robot::RangeUpdater>("RangeUpdater");
}