
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
     MinimalSubscriber();

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    void topicCallback(const std_msgs::msg::String::SharedPtr msg);
};