#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "autonavi_msgs/msg/waypoint.hpp"

using namespace std::chrono_literals;

class NewPublisher : public rclcpp::Node
{
public:
    NewPublisher();

private:
    rclcpp::Publisher<autonavi_msgs::msg::Waypoint>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timerCallback();
};