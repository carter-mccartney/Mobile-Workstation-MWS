#include <memory>
#include <string>

#include "nav_msgs/msg/path.hpp"

#include "behaviortree_cpp_v3/action_node.h"

namespace mws_robot
{

    /**
     * @brief A BT::ActionNodeBase to shorten path by some distance
     */
    class TruncatePathCustom : public BT::ActionNodeBase
    {
    public:
        /**
         * @brief A mws_robot::TruncatePathCustom constructor
         * @param xml_tag_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        TruncatePathCustom(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);

        /**
         * @brief Creates list of BT ports
         * @return BT::PortsList Containing basic ports along with node-specific ports
         */
        static BT::PortsList providedPorts()
        {
            return 
            {
                BT::InputPort<nav_msgs::msg::Path>("input_path", "Original Path"),
                BT::OutputPort<nav_msgs::msg::Path>("output_path", "Path truncated to a certain distance"),
                BT::InputPort<double>("distance", "distance"),
            };
        }

    private:
        /**
         * @brief The other (optional) override required by a BT action.
         */
        void halt() override {}

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;
    };
}