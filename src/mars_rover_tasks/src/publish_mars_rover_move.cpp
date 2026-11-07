#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

class MoveRoverNode : public rclcpp::Node
{
public:
  MoveRoverNode() : Node("move_rover_node")
  {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    auto timer_period = std::chrono::milliseconds(500);

    timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&MoveRoverNode::timer_callback, this));
  }

  void stop_rover()
  {
    auto stop_msg = geometry_msgs::msg::Twist();

    // All Twist fields are zero by default
    publisher_->publish(stop_msg);

    RCLCPP_INFO(
        this->get_logger(),
        "Publishing stop message before shutdown");
  }

private:
  void timer_callback()
  {
    auto msg = geometry_msgs::msg::Twist();

    msg.linear.x = 0.5;
    msg.angular.z = 0.5;

    publisher_->publish(msg);

    RCLCPP_INFO(
        this->get_logger(),
        "Publishing: linear.x=%.2f, angular.z=%.2f",
        msg.linear.x,
        msg.angular.z);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MoveRoverNode>();

  auto context = node->get_node_options().context();

  context->add_pre_shutdown_callback(
      [node]() {
        node->stop_rover();
      });

  rclcpp::spin(node);

  return 0;
}