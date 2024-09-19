#include "autonav_shared/node.hpp"

namespace AutoNav
{
    Node::Node(const std::string & node_name) : rclcpp::Node(node_name)
    {
        // TODO: Setup all relevant publishers, subscribers, services, clients, etc
    }

    Node::~Node()
    {
        // TOOD: Cleanup stuff as required
    }
}