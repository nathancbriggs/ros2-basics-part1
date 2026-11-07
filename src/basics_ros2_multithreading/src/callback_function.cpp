#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vector>

class RobotStatus : public rclcpp::Node {
public:
  RobotStatus()
      : Node("robot_status"), time_robot_on_(0.0), timer_period_(1.0),
        sleep_timer_counter_(0.0), main_task_period_(1.5),
        sleep_time_main_task_(0.0),
        robot_status_({"Robot Booting Up...", "Robot Ready...",
                       "Robot ShuttingDown..."}) {

    // Define the output directory
    output_dir_ =
        "/home/simulations/ros2_sims_ws/src/cpp_callback_visual_scripts/";

    // Open timing log files with full path
    timer_counter_log_.open(output_dir_ + "timer_counter_timings.csv");
    main_task_log_.open(output_dir_ + "main_task_timings.csv");

    // Write CSV headers
    timer_counter_log_ << "timestamp,duration_ms,callback_type\n";
    main_task_log_ << "timestamp,duration_ms,callback_type\n";

    // Create timers with wrapper functions for timing instrumentation
    timer_counter_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_period_),
        std::bind(&RobotStatus::timer_counter_wrapper, this));

    main_task_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(main_task_period_),
        std::bind(&RobotStatus::main_task_wrapper, this));

    RCLCPP_INFO(this->get_logger(), "Callback timing instrumentation enabled");
    RCLCPP_INFO(this->get_logger(), "CSV logs will be saved to: %s",
                output_dir_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "Files: timer_counter_timings.csv, main_task_timings.csv");
  }

  ~RobotStatus() {
    if (timer_counter_log_.is_open())
      timer_counter_log_.close();
    if (main_task_log_.is_open())
      main_task_log_.close();

    // Generate summary
    generate_timing_summary();
  }

private:
  // Wrapper functions that add timing instrumentation
  void timer_counter_wrapper() {
    auto start_time = std::chrono::high_resolution_clock::now();
    uint64_t timestamp = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            start_time.time_since_epoch())
            .count());

    // Call the actual callback
    timer_counter();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                        end_time - start_time)
                        .count() /
                    1000.0; // Convert to milliseconds

    // Log timing data
    timer_counter_log_ << timestamp << "," << duration << ",timer_counter\n";
    timer_counter_log_.flush();

    // Store for analysis
    timer_counter_timings_.push_back({timestamp, duration});
  }

  void main_task_wrapper() {
    auto start_time = std::chrono::high_resolution_clock::now();
    uint64_t timestamp = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            start_time.time_since_epoch())
            .count());

    // Call the actual callback
    main_task();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                        end_time - start_time)
                        .count() /
                    1000.0; // Convert to milliseconds

    // Log timing data
    main_task_log_ << timestamp << "," << duration << ",main_task\n";
    main_task_log_.flush();

    // Store for analysis
    main_task_timings_.push_back({timestamp, duration});
  }

  void robot_message(const std::string &text,
                     const std::string &robot_name = "Robot-1") {
    RCLCPP_INFO(this->get_logger(), "%s: %s", robot_name.c_str(), text.c_str());
  }

  // Your original callback functions (unchanged)
  void timer_counter() {
    time_robot_on_ += timer_period_;
    RCLCPP_INFO(this->get_logger(), "Updated Time Robot On=%f", time_robot_on_);
    std::this_thread::sleep_for(
        std::chrono::duration<double>(sleep_timer_counter_));
    RCLCPP_INFO(this->get_logger(), "Updated Time Robot On=%f", time_robot_on_);
  }

  void main_task() {
    std::string status;
    if (!robot_status_.empty()) {
      status = robot_status_.front();
      robot_status_.erase(robot_status_.begin());
      robot_message(status);
    } else {
      status = "ShuttingDown";
    }

    if (status.find("ShuttingDown") != std::string::npos) {
      RCLCPP_INFO(this->get_logger(), "Shutting down node...");
      rclcpp::shutdown();
    } else {
      RCLCPP_INFO(this->get_logger(), "Continue....");
      std::this_thread::sleep_for(
          std::chrono::duration<double>(sleep_time_main_task_));
    }
  }

  void generate_timing_summary() {
    RCLCPP_INFO(this->get_logger(), "=== CALLBACK TIMING SUMMARY ===");

    if (!timer_counter_timings_.empty()) {
      double avg_timer = 0, min_timer = timer_counter_timings_[0].duration,
             max_timer = 0;
      for (const auto &timing : timer_counter_timings_) {
        avg_timer += timing.duration;
        min_timer = std::min(min_timer, timing.duration);
        max_timer = std::max(max_timer, timing.duration);
      }
      avg_timer /= timer_counter_timings_.size();

      RCLCPP_INFO(
          this->get_logger(),
          "timer_counter: %zu calls, avg=%.2fms, min=%.2fms, max=%.2fms",
          timer_counter_timings_.size(), avg_timer, min_timer, max_timer);
    }

    if (!main_task_timings_.empty()) {
      double avg_main = 0, min_main = main_task_timings_[0].duration,
             max_main = 0;
      for (const auto &timing : main_task_timings_) {
        avg_main += timing.duration;
        min_main = std::min(min_main, timing.duration);
        max_main = std::max(max_main, timing.duration);
      }
      avg_main /= main_task_timings_.size();

      RCLCPP_INFO(this->get_logger(),
                  "main_task: %zu calls, avg=%.2fms, min=%.2fms, max=%.2fms",
                  main_task_timings_.size(), avg_main, min_main, max_main);
    }

    RCLCPP_INFO(this->get_logger(), "CSV files saved to: %s",
                output_dir_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "Files: timer_counter_timings.csv, main_task_timings.csv");
  }

private:
  double time_robot_on_;
  double timer_period_;
  double sleep_timer_counter_;
  double main_task_period_;
  double sleep_time_main_task_;
  std::vector<std::string> robot_status_;
  rclcpp::TimerBase::SharedPtr timer_counter_timer_;
  rclcpp::TimerBase::SharedPtr main_task_timer_;

  // Timing instrumentation variables
  std::string output_dir_;
  std::ofstream timer_counter_log_;
  std::ofstream main_task_log_;

  struct TimingData {
    uint64_t timestamp;
    double duration;
  };

  std::vector<TimingData> timer_counter_timings_;
  std::vector<TimingData> main_task_timings_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto robot_status_node = std::make_shared<RobotStatus>();
  rclcpp::spin(robot_status_node);
  rclcpp::shutdown();
  return 0;
}