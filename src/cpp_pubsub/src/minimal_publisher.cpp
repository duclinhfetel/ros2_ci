#include "cpp_pubsub/minimal_publisher.hpp"

MinimalPublisher::MinimalPublisher() : Node("minimal_publisher"),
                                       count_(0)
{
    

    publisher_ = this->create_publisher<std_msgs::msg::String>("tutorial_topic", 
    rclcpp::QoS(10).keep_last(1).transient_local());
    timer_ = this->create_wall_timer(
        500ms,
        std::bind(&MinimalPublisher::timerCallback, this));
}

void MinimalPublisher::timerCallback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_);

    if (count_ <= 10)
    {
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    count_++;
}