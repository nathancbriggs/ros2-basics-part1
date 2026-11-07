#include <chrono>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class LogDemo : public rclcpp::Node {
public:
  LogDemo() : Node("logger_example") {
    // Logger level configuration
    // Set logger level to DEBUG to see all messages
    auto ret = rcutils_logging_set_logger_level(this->get_logger().get_name(),
                                                RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set logger level");
    }

    // Create a timer with 200ms interval (0.2 seconds)
    timer_ = this->create_wall_timer(200ms,
                                     std::bind(&LogDemo::timer_callback, this));
  }

private:
  void timer_callback() {
    // prints a ROS2 log debugging
    RCLCPP_DEBUG(this->get_logger(),
                 "Laser sensor detected, ready for distance measurements.");
    // prints a ROS2 log info
    RCLCPP_INFO(this->get_logger(),
                "All systems operational, robot ready to proceed.");
    // prints a ROS2 log warning
    RCLCPP_WARN(this->get_logger(),
                "Motor temperature approaching limit, monitoring closely.");
    // prints a ROS2 log error
    RCLCPP_ERROR(this->get_logger(),
                 "Motor failure detected, unable to proceed with movement!");
    // prints a ROS2 log fatal
    RCLCPP_FATAL(
        this->get_logger(),
        "Critical error: Collision detected, emergency stop activated!");
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  // initialize the ROS communication
  rclcpp::init(argc, argv);
  // declare the node constructor
  auto log_demo = std::make_shared<LogDemo>();
  // pause the program execution, waits for a request to kill the node (ctrl+c)
  rclcpp::spin(log_demo);
  // shutdown the ROS communication
  rclcpp::shutdown();
  return 0;
}