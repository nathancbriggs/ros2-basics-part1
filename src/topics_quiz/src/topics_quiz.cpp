#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

class TopicsQuizNode : public rclcpp::Node {
public:
  TopicsQuizNode() : rclcpp::Node("topics_quiz_node") {
    velocity_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    mission_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/nasa_mission", 10, [this](std_msgs::msg::String::ConstSharedPtr msg) {
          this->mission_callback(msg);
        });

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          this->odom_callback(msg);
        });

    laser_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/laser_scan", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
              this->laser_callback(msg);
            });

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), [this]() { this->control_loop(); });

    RCLCPP_INFO(this->get_logger(), "Topics quiz node started.");
  }

private:
  enum class NavigationMode { GoToGoal, FollowObstacle };

  void mission_callback(std_msgs::msg::String::ConstSharedPtr msg) {
    if (msg->data == "Go-Home") {
      target_x_ = 0.0;
      target_y_ = 0.0;
    } else if (msg->data == "Go-Pickup") {
      target_x_ = -2.342;
      target_y_ = -2.432;
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown mission: '%s'",
                  msg->data.c_str());

      return;
    }

    if (has_active_goal_ && msg->data == active_command_) {
      return;
    }

    active_command_ = msg->data;
    has_active_goal_ = true;
    navigation_mode_ = NavigationMode::GoToGoal;
    mission_start_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "Goal selected: x=%.3f, y=%.3f", target_x_,
                target_y_);
  }

  void odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    const auto &q = msg->pose.pose.orientation;

    const double heading_x = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

    const double heading_y = 2.0 * (q.w * q.z + q.x * q.y);

    current_yaw_ = std::atan2(heading_y, heading_x);
    last_odom_time_ = std::chrono::steady_clock::now();

    if (!have_odometry_) {
      have_odometry_ = true;

      RCLCPP_INFO(this->get_logger(),
                  "Odometry ready: x=%.3f, y=%.3f, yaw=%.3f rad", current_x_,
                  current_y_, current_yaw_);
    }
  }

  void laser_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    // Sector order: 0 = front, 1 = left, 2 = right.
    std::array<double, 3> nearest{msg->range_max, msg->range_max,
                                  msg->range_max};

    std::array<std::size_t, 3> total{};
    std::array<std::size_t, 3> usable{};

    const double pi = std::acos(-1.0);
    const double front_limit = pi / 6.0;
    const double side_limit = pi / 2.0;

    for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
      const double angle = msg->angle_min + i * msg->angle_increment;

      std::size_t sector;

      if (std::abs(angle) <= front_limit) {
        sector = 0;
      } else if (angle > front_limit && angle <= side_limit) {
        sector = 1;
      } else if (angle >= -side_limit && angle < -front_limit) {
        sector = 2;
      } else {
        continue;
      }

      ++total[sector];

      double range = msg->ranges[i];

      if (std::isnan(range)) {
        continue;
      }

      if (std::isinf(range)) {
        if (range > 0.0) {
          range = msg->range_max;
        } else {
          range = 0.0;
        }
      } else if (range < msg->range_min || range > msg->range_max) {
        continue;
      }

      nearest[sector] = std::min(nearest[sector], range);
      ++usable[sector];
    }

    have_laser_ = true;

    for (std::size_t sector = 0; sector < 3; ++sector) {
      if (total[sector] == 0 || usable[sector] != total[sector]) {
        have_laser_ = false;
      }
    }

    if (!have_laser_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Incomplete laser sectors. Usable/total: "
                           "front=%zu/%zu left=%zu/%zu right=%zu/%zu",
                           usable[0], total[0], usable[1], total[1], usable[2],
                           total[2]);

      return;
    }

    front_clearance_ = nearest[0];
    left_clearance_ = nearest[1];
    right_clearance_ = nearest[2];

    last_laser_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Laser [%s]: front=%.2f m | left=%.2f m | right=%.2f m",
        msg->header.frame_id.c_str(), front_clearance_, left_clearance_,
        right_clearance_);
  }

  double limit_turn_speed(double requested_speed) const {
    return std::max(-max_turn_speed_,
                    std::min(max_turn_speed_, requested_speed));
  }

  void control_loop() {
    geometry_msgs::msg::Twist command{};

    if (!have_odometry_ || !have_laser_ || !has_active_goal_) {
      velocity_publisher_->publish(command);
      return;
    }

    const auto now = std::chrono::steady_clock::now();

    const double odom_age =
        std::chrono::duration<double>(now - last_odom_time_).count();

    const double laser_age =
        std::chrono::duration<double>(now - last_laser_time_).count();

    if (odom_age > odom_timeout_ || laser_age > laser_timeout_) {
      velocity_publisher_->publish(command);

      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for fresh odometry and laser data.");

      return;
    }

    const double dx = target_x_ - current_x_;
    const double dy = target_y_ - current_y_;
    const double distance_to_goal = std::hypot(dx, dy);

    if (distance_to_goal < goal_tolerance_) {
      has_active_goal_ = false;
      navigation_mode_ = NavigationMode::GoToGoal;
      velocity_publisher_->publish(command);

      const double elapsed =
          std::chrono::duration<double>(now - mission_start_time_).count();

      RCLCPP_INFO(this->get_logger(),
                  "Goal reached: distance=%.3f m | elapsed=%.1f s",
                  distance_to_goal, elapsed);

      return;
    }

    const double goal_heading = std::atan2(dy, dx);
    const double raw_error = goal_heading - current_yaw_;

    const double heading_error =
        std::atan2(std::sin(raw_error), std::cos(raw_error));

    const bool sides_clear = left_clearance_ > minimum_side_clearance_ &&
                             right_clearance_ > minimum_side_clearance_;

    if (navigation_mode_ == NavigationMode::FollowObstacle) {
      const bool made_progress =
          distance_to_goal < obstacle_entry_distance_ - progress_margin_;

      const bool facing_goal =
          std::abs(heading_error) < leave_heading_tolerance_;

      const bool front_clear = front_clearance_ > clear_path_distance_;

      if (made_progress && facing_goal && front_clear && sides_clear) {
        navigation_mode_ = NavigationMode::GoToGoal;

        RCLCPP_INFO(this->get_logger(),
                    "Resuming goal tracking after making progress.");
      } else {
        const double side_clearance =
            wall_side_ > 0.0 ? left_clearance_ : right_clearance_;

        if (front_clearance_ < obstacle_distance_ ||
            side_clearance < minimum_side_clearance_) {
          command.angular.z = -wall_side_ * avoidance_turn_speed_;
        } else if (side_clearance > wall_lost_distance_) {
          command.linear.x = wall_forward_speed_;
          command.angular.z = wall_side_ * corner_turn_speed_;
        } else {
          const double side_error = side_clearance - desired_wall_distance_;

          command.linear.x = wall_forward_speed_;
          command.angular.z =
              limit_turn_speed(wall_side_ * wall_turn_gain_ * side_error);
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Following obstacle on %s: front=%.2f m | "
                             "side=%.2f m | goal distance=%.3f m",
                             wall_side_ > 0.0 ? "left" : "right",
                             front_clearance_, side_clearance,
                             distance_to_goal);

        velocity_publisher_->publish(command);
        return;
      }
    }

    if (std::abs(heading_error) > drive_heading_tolerance_) {
      command.angular.z = limit_turn_speed(turn_gain_ * heading_error);

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Turning toward goal: error=%.3f rad",
                           heading_error);
    } else if (front_clearance_ < obstacle_distance_ || !sides_clear) {
      navigation_mode_ = NavigationMode::FollowObstacle;
      obstacle_entry_distance_ = distance_to_goal;

      if (right_clearance_ >= left_clearance_) {
        wall_side_ = 1.0;
      } else {
        wall_side_ = -1.0;
      }

      command.angular.z = -wall_side_ * avoidance_turn_speed_;

      RCLCPP_INFO(this->get_logger(),
                  "Obstacle detected. Turning %s; keeping obstacle on %s.",
                  wall_side_ > 0.0 ? "right" : "left",
                  wall_side_ > 0.0 ? "left" : "right");
    } else {
      command.linear.x =
          std::min(max_forward_speed_, forward_gain_ * distance_to_goal);

      command.angular.z = limit_turn_speed(turn_gain_ * heading_error);

      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Driving toward goal: distance=%.3f m | speed=%.3f m/s",
          distance_to_goal, command.linear.x);
    }

    velocity_publisher_->publish(command);
  }

  double target_x_{0.0};
  double target_y_{0.0};
  bool has_active_goal_{false};
  std::string active_command_{};

  double current_x_{0.0};
  double current_y_{0.0};
  double current_yaw_{0.0};
  bool have_odometry_{false};

  double front_clearance_{0.0};
  double left_clearance_{0.0};
  double right_clearance_{0.0};
  bool have_laser_{false};

  NavigationMode navigation_mode_{NavigationMode::GoToGoal};
  double wall_side_{1.0};
  double obstacle_entry_distance_{0.0};

  std::chrono::steady_clock::time_point last_odom_time_{};
  std::chrono::steady_clock::time_point last_laser_time_{};
  std::chrono::steady_clock::time_point mission_start_time_{};

  const double goal_tolerance_{0.1};

  const double turn_gain_{1.2};
  const double max_turn_speed_{0.6};
  const double drive_heading_tolerance_{0.20};

  const double forward_gain_{0.6};
  const double max_forward_speed_{0.20};

  const double obstacle_distance_{0.80};
  const double clear_path_distance_{1.00};
  const double minimum_side_clearance_{0.40};

  const double desired_wall_distance_{0.70};
  const double wall_lost_distance_{1.20};
  const double wall_turn_gain_{1.0};
  const double wall_forward_speed_{0.15};
  const double avoidance_turn_speed_{0.50};
  const double corner_turn_speed_{0.20};

  const double progress_margin_{0.20};
  const double leave_heading_tolerance_{0.40};

  const double odom_timeout_{1.0};
  const double laser_timeout_{1.0};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_subscription_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TopicsQuizNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}