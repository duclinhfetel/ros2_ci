#include "test_custom_msg/test_new_msg.hpp"

NewPublisher::NewPublisher() : Node("test_custom_msg")
{
    publisher_ = this->create_publisher<autonavi_msgs::msg::Waypoint>(
        "new_msgs",
        rclcpp::QoS(10).keep_last(1).transient_local());

    timer_ = this->create_wall_timer(
        500ms,
        std::bind(&NewPublisher::timerCallback, this));
}

void NewPublisher::timerCallback()
{
    autonavi_msgs::msg::Waypoint way = autonavi_msgs::msg::Waypoint();
    way.header.stamp = rclcpp::Time(rclcpp::Clock().now());
    way.velocity = 5.6;
    way.pose.position.x = 0.4;
    way.pose.position.y = 2.4;
    way.pose.position.z = 1.4;
    way.pose.orientation.w = 1.0;

    publisher_->publish(way);
    RCLCPP_INFO(this->get_logger(), "Publish Done!");
}