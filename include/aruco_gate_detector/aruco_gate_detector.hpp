#ifndef ARUCO_GATE_DETECTOR_HPP_
#define ARUCO_GATE_DETECTOR_HPP_

#include <cv_bridge/cv_bridge.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <memory>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "aerostack2_core/node.hpp"
#include "aerostack2_core/sensor.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

// #include <image_transport/image_transport.h>

class ArucoGateDetector : public aerostack2::Node
{
public:
  ArucoGateDetector();
  ~ArucoGateDetector(){};

private:
  // Sensor comm
  std::string serial_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_image_;
  // Sensor measurement
  std::shared_ptr<aerostack2::Sensor<nav_msgs::msg::Path>> gate_pose_;
  std::shared_ptr<aerostack2::Sensor<sensor_msgs::msg::Image>> gate_img_;
  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr img);

};

#endif  // ARUCO_GATE_DETECTOR_HPP_