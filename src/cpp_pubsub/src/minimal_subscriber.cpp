#include "cpp_pubsub/minimal_subscriber.hpp"

MinimalSubscriber::MinimalSubscriber() : Node("minial_subscriber")
{
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "tutorial_topic",
        rclcpp::QoS(10).transient_local(),
        std::bind(&MinimalSubscriber::topicCallback, this, std::placeholders::_1));
}

void MinimalSubscriber::topicCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}
