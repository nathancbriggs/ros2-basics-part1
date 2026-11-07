#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <limits>
#include <map>
#include <string>

class ObstacleDetectorNode : public rclcpp::Node {
public:
  ObstacleDetectorNode(const std::string &node_name = "obstacle_detector_node")
      : Node(node_name) {
    // create the subscriber object
    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/laser_scan", qos,
        std::bind(&ObstacleDetectorNode::laserscan_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "%s Ready...", this->get_name());
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;

  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Define the sectors with their index ranges
    std::map<std::string, std::pair<int, int>> sectors = {
        {"Right_Rear", {0, 33}},    {"Right", {34, 66}},
        {"Front_Right", {67, 100}}, {"Front_Left", {101, 133}},
        {"Left", {134, 166}},       {"Left_Rear", {167, 199}}};

    // Initialize the minimum distances for each sector
    std::map<std::string, float> min_distances;
    for (const auto &sector : sectors) {
      min_distances[sector.first] = std::numeric_limits<float>::infinity();
    }

    // Find the minimum distance in each sector
    for (const auto &sector : sectors) {
      int start_idx = sector.second.first;
      int end_idx = sector.second.second;

      // Ensure the index range is within bounds
      if (start_idx < static_cast<int>(msg->ranges.size()) &&
          end_idx < static_cast<int>(msg->ranges.size())) {
        auto start_it = msg->ranges.begin() + start_idx;
        auto end_it = msg->ranges.begin() + end_idx + 1;

        if (start_it < end_it) {
          min_distances[sector.first] = *std::min_element(start_it, end_it);
        }
      }
    }

    // Log the minimum distances
    for (const auto &distance : min_distances) {
      RCLCPP_INFO(this->get_logger(), "%s: %.2f meters", distance.first.c_str(),
                  distance.second);
    }

    // Define the threshold for obstacle detection
    float obstacle_threshold = 0.8f; // meters

    // Determine detected obstacles
    std::map<std::string, bool> detections;
    for (const auto &distance : min_distances) {
      detections[distance.first] = distance.second < obstacle_threshold;
    }

    // Determine suggested action based on detection and ordered conditions by
    // priority Priority 1: Front detection, Priority 2: Side detections,
    // Priority 3: Rear detections
    std::string action;

    // Priority 1
    if (detections["Front_Left"] && detections["Front_Right"]) {
      std::string arbitrary_direction = "Right";
      action = "Selected Turn Arbitrary Direction " + arbitrary_direction;
    } else if (detections["Front_Left"] && !detections["Front_Right"]) {
      action = "Turn Right to avoid obstacle on the front-left";
    } else if (detections["Front_Right"] && !detections["Front_Left"]) {
      action = "Turn Left to avoid obstacle on the front-right";
    }
    // Priority 2
    else if (detections["Left"]) {
      action =
          "Go Forwards turning slightly right to avoid obstacle on the left";
    } else if (detections["Right"]) {
      action =
          "Go Forwards turning slightly left to avoid obstacle on the right";
    }
    // Priority 3
    else if (detections["Right_Rear"]) {
      action = "Go Forwards, BUT DONT reverse Right";
    } else if (detections["Left_Rear"]) {
      action = "Go Forwards, BUT DONT reverse left";
    } else {
      action = "Go Forwards";
    }

    // Log the suggested action
    RCLCPP_INFO(this->get_logger(), "Suggested action: %s", action.c_str());
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}