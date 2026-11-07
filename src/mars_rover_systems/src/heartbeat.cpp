#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <string>
#include <functional>

class HeartbeatNode : public rclcpp::Node
{
public:
    HeartbeatNode(const std::string& rover_name, double timer_period = 0.2)
    : Node(rover_name)
    {
        // create a timer sending two parameters:
        // - the duration between two callbacks (timer_period seconds)
        // - the timer function (timer_callback)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(timer_period * 1000)),
            std::bind(&HeartbeatNode::timer_callback, this));
    }
        
private:
    void timer_callback()
    {
        auto ros_time_stamp = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "%s is alive...Time(nanoseconds=%ld, clock_type=ROS_TIME)", 
                    this->get_name(), 
                    ros_time_stamp.nanoseconds());
    }
    
    std::string rover_name_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    // initialize the ROS2 communication
    rclcpp::init(argc, argv);
    // declare the node constructor
    auto node = std::make_shared<HeartbeatNode>("mars_rover_1", 1.0);
    // keeps the node alive, waits for a request to kill the node (ctrl+c)
    rclcpp::spin(node);
    // shutdown the ROS2 communication
    rclcpp::shutdown();
    return 0;
}