#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <random>
#include <string>

class TemperatureMonitorNode : public rclcpp::Node {
public:
  TemperatureMonitorNode(double timer_period = 1.0)
      : Node("temperature_monitor"), temperature_threshold_(70.0), rd_(),
        gen_(rd_()), temp_dist_(20.0, 100.0) {
    RCLCPP_INFO(this->get_logger(),
                "Temperature Monitor Node has been started.");

    // Create a timer that triggers every second (1 Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(timer_period * 1000)),
        std::bind(&TemperatureMonitorNode::monitor_temperature_callback, this));
  }

private:
  void monitor_temperature_callback() {
    double current_temperature = get_temperature();
    RCLCPP_INFO(this->get_logger(), "Current temperature: %.2f°C",
                current_temperature);

    if (current_temperature > temperature_threshold_) {
      RCLCPP_WARN(this->get_logger(),
                  "Warning: High temperature detected! %.2f°C",
                  current_temperature);
    }
  }

  double get_temperature() {
    // Simulate getting a temperature reading (in reality, this would come from
    // a sensor)
    return temp_dist_(gen_);
  }

  double temperature_threshold_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<double> temp_dist_;
};

int main(int argc, char **argv) {
  // initialize the ROS2 communication
  rclcpp::init(argc, argv);
  // declare the node constructor
  auto node = std::make_shared<TemperatureMonitorNode>(1.0);
  // keeps the node alive, waits for a request to kill the node (ctrl+c)
  rclcpp::spin(node);
  // shutdown the ROS2 communication
  rclcpp::shutdown();
  return 0;
}