#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

class RobotStatus : public rclcpp::Node {
public:
  RobotStatus() : Node("robot_status"), time_robot_on_(0.0) {}

  void robot_message(const std::string &text,
                     const std::string &robot_name = "Robot-1") {
    RCLCPP_INFO(this->get_logger(), "%s: %s", robot_name.c_str(), text.c_str());
  }

  void timer_counter(double time_passed) {
    time_robot_on_ += time_passed;
    RCLCPP_INFO(this->get_logger(), "Updated Time Robot On=%f", time_robot_on_);
  }

  void main_task() {
    double period = 1.0;

    robot_message("Robot Booting Up...");
    std::this_thread::sleep_for(std::chrono::duration<double>(period));
    timer_counter(period);

    robot_message("Robot Ready...");
    std::this_thread::sleep_for(std::chrono::duration<double>(period));
    timer_counter(period);

    robot_message("Robot ShuttingDown...");
  }

private:
  double time_robot_on_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto robot_status_node = std::make_shared<RobotStatus>();
  robot_status_node->main_task();

  rclcpp::shutdown();
  return 0;
}