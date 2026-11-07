#include "mars_rover_tasks/plant_detector.hpp"

#include <algorithm>
#include <iostream>

PlantDetector::PlantDetector(const std::string &model_path)
    : model_loaded_(true), device_(CPU) {
  // The model path is kept only for interface compatibility.
  // This implementation uses OpenCV instead of an AI model.
  (void)model_path;

  std::cout << "PlantDetector: Using OpenCV plant detector" << std::endl;
}

float PlantDetector::predict(const cv::Mat &image) {
  if (image.empty()) {
    std::cerr << "PlantDetector: Empty image provided" << std::endl;

    return 0.0f;
  }

  try {
    return detect_green_objects(image);
  } catch (const std::exception &e) {
    std::cerr << "PlantDetector: Detection error: " << e.what() << std::endl;

    return 0.0f;
  }
}

float PlantDetector::detect_green_objects(const cv::Mat &image) {
  // Convert the RGB image to HSV for easier color detection
  cv::Mat hsv_image;

  cv::cvtColor(image, hsv_image, cv::COLOR_RGB2HSV);

  // ---------------------------------------------------------
  // Detect green foliage
  // ---------------------------------------------------------

  cv::Mat green_mask;

  // Wider green range to account for the low-resolution
  // and bright robot camera image
  cv::inRange(hsv_image, cv::Scalar(35, 50, 30), cv::Scalar(95, 255, 220),
              green_mask);

  // ---------------------------------------------------------
  // Detect red / magenta flowers
  // ---------------------------------------------------------

  cv::Mat red_mask_1;
  cv::Mat red_mask_2;
  cv::Mat magenta_mask;
  cv::Mat flower_mask;

  // Red / orange-red range
  cv::inRange(hsv_image, cv::Scalar(0, 70, 40), cv::Scalar(20, 255, 255),
              red_mask_1);

  // Deep red range
  cv::inRange(hsv_image, cv::Scalar(160, 60, 40), cv::Scalar(179, 255, 255),
              red_mask_2);

  // Magenta / purple range
  cv::inRange(hsv_image, cv::Scalar(135, 50, 30), cv::Scalar(179, 255, 255),
              magenta_mask);

  flower_mask = red_mask_1 | red_mask_2 | magenta_mask;

  // ---------------------------------------------------------
  // Remove small isolated pixels
  // ---------------------------------------------------------

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

  cv::morphologyEx(green_mask, green_mask, cv::MORPH_OPEN, kernel);

  cv::morphologyEx(flower_mask, flower_mask, cv::MORPH_OPEN, kernel);

  // ---------------------------------------------------------
  // Calculate how much green and flower color is visible
  // ---------------------------------------------------------

  const int total_pixels = image.rows * image.cols;

  if (total_pixels == 0) {
    return 0.0f;
  }

  const float green_ratio = static_cast<float>(cv::countNonZero(green_mask)) /
                            static_cast<float>(total_pixels);

  const float flower_ratio = static_cast<float>(cv::countNonZero(flower_mask)) /
                             static_cast<float>(total_pixels);

  // Temporary debug output to help tune the detector
  // std::cout << "Green ratio: " << green_ratio
  //          << " | Flower ratio: " << flower_ratio << std::endl;

  // ---------------------------------------------------------
  // Detect the plant
  // ---------------------------------------------------------

  // Require BOTH green foliage and red/magenta flowers
  constexpr float MIN_GREEN_RATIO = 0.005f;  // 0.5%
  constexpr float MIN_FLOWER_RATIO = 0.002f; // 0.2%

  if (green_ratio < MIN_GREEN_RATIO || flower_ratio < MIN_FLOWER_RATIO) {
    return 0.0f;
  }

  // ---------------------------------------------------------
  // Calculate confidence
  // ---------------------------------------------------------

  const float green_score = std::min(1.0f, green_ratio / 0.04f);

  const float flower_score = std::min(1.0f, flower_ratio / 0.025f);

  // If both characteristic colors are present,
  // start from a base confidence of 0.5
  float confidence = 0.5f + 0.25f * green_score + 0.25f * flower_score;

  return std::min(1.0f, confidence);
}

float PlantDetector::analyze_texture(const cv::Mat &image) {
  // Kept for compatibility with the existing class interface.
  // Texture analysis is not required in this detector.

  if (image.empty()) {
    return 0.0f;
  }

  cv::Mat gray_image;

  cv::cvtColor(image, gray_image, cv::COLOR_RGB2GRAY);

  cv::Scalar mean;
  cv::Scalar stddev;

  cv::meanStdDev(gray_image, mean, stddev);

  return std::min(1.0f, static_cast<float>(stddev[0] / 50.0));
}