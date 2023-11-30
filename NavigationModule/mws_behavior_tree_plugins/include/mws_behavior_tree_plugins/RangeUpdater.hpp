#include <memory>
#include <string>

#include <example_interfaces/msg/float64.hpp>

#include <behaviortree_cpp_v3/decorator_node.h>

#include <rclcpp/rclcpp.hpp>

namespace mws_robot
{

    /**
     * @brief A BT::DecoratorNode that subscribes to a range topic and updates
     * the current range on the blackboard
     */
    class RangeUpdater : public BT::DecoratorNode
    {
    public:
        /**
         * @brief A constructor for mws_robot::RangeUpdater
         * @param xml_tag_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        RangeUpdater(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);

        /**
         * @brief Creates list of BT ports
         * @return BT::PortsList Containing node-specific ports
         */
        static BT::PortsList providedPorts()
        {
            return 
            {
                BT::InputPort<double>("input_range", "Original range in"),
                BT::OutputPort<double>("output_range", "Output range set by subscription"),
            };
        }

    private:
        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        /**
         * @brief Callback function for range update topic
         * @param msg Shared pointer to example_interfaces::msg::Float64 message
         */
        void onRangeChanged(const example_interfaces::msg::Float64::SharedPtr msg);

        double lastRange;

        rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr rangeSubscriber;

        rclcpp::Node::SharedPtr node;
        rclcpp::CallbackGroup::SharedPtr callbackGroup;
        rclcpp::executors::SingleThreadedExecutor callbackGroupExecutor;
    };

}