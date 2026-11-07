#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>

using std::placeholders::_1;

class GreenDetector : public rclcpp::Node
{
public:
  GreenDetector()
  : Node("green_detector")
  {
    // Create a subscriber to the camera image topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/leo/camera/image_raw", 10, std::bind(&GreenDetector::image_callback, this, _1));
    
    // Create fullscreen window with trackbars for adjusting HSV values
    cv::namedWindow("Green Detection", cv::WINDOW_NORMAL);
    cv::setWindowProperty("Green Detection", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    cv::createTrackbar("Lower H", "Green Detection", nullptr, 179, nullptr);
    cv::createTrackbar("Lower S", "Green Detection", nullptr, 255, nullptr);
    cv::createTrackbar("Lower V", "Green Detection", nullptr, 255, nullptr);
    cv::createTrackbar("Upper H", "Green Detection", nullptr, 179, nullptr);
    cv::createTrackbar("Upper S", "Green Detection", nullptr, 255, nullptr);
    cv::createTrackbar("Upper V", "Green Detection", nullptr, 255, nullptr);
    
    // Set initial values
    cv::setTrackbarPos("Upper H", "Green Detection", 179);
    cv::setTrackbarPos("Upper S", "Green Detection", 255);
    cv::setTrackbarPos("Upper V", "Green Detection", 255);
  }

  ~GreenDetector()
  {
    cv::destroyAllWindows();
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      // Convert the ROS image message to an OpenCV image
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat cv_image = cv_ptr->image;

      // Get current positions of all trackbars
      lower_h_ = cv::getTrackbarPos("Lower H", "Green Detection");
      lower_s_ = cv::getTrackbarPos("Lower S", "Green Detection");
      lower_v_ = cv::getTrackbarPos("Lower V", "Green Detection");
      upper_h_ = cv::getTrackbarPos("Upper H", "Green Detection");
      upper_s_ = cv::getTrackbarPos("Upper S", "Green Detection");
      upper_v_ = cv::getTrackbarPos("Upper V", "Green Detection");

      // Set the lower and upper bounds for HSV
      cv::Scalar lower_green(lower_h_, lower_s_, lower_v_);
      cv::Scalar upper_green(upper_h_, upper_s_, upper_v_);

      // Convert the image to HSV color space
      cv::Mat hsv_image;
      cv::cvtColor(cv_image, hsv_image, cv::COLOR_BGR2HSV);

      // Create a mask to threshold the image
      cv::Mat mask;
      cv::inRange(hsv_image, lower_green, upper_green, mask);

      // Optionally visualize the green detection on the original image
      cv::Mat green_detected;
      cv::bitwise_and(cv_image, cv_image, green_detected, mask);

      // Display the green-detected image in fullscreen
      cv::imshow("Green Detection", green_detected);

      // Press 'q' to exit the loop
      if ((cv::waitKey(1) & 0xFF) == 'q') {
        rclcpp::shutdown();
      }
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  int lower_h_ = 0, lower_s_ = 0, lower_v_ = 0;
  int upper_h_ = 179, upper_s_ = 255, upper_v_ = 255;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto green_detector = std::make_shared<GreenDetector>();

  try {
    rclcpp::spin(green_detector);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(green_detector->get_logger(), "Exception: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}