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
  ~ArucoGateDetector();
  // void setupDetector();
  // void runDetector();
  // void setupTF();

private:
  // Sensor comm
  std::string serial_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_image_;
  // Sensor measurement
  std::shared_ptr<aerostack2::Sensor<nav_msgs::msg::Path>> gate_pose_;
  std::shared_ptr<aerostack2::Sensor<sensor_msgs::msg::Image>> gate_img_;
  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr img);

  // Sensor Tf
  // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfstatic_broadcaster_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::vector<geometry_msgs::msg::TransformStamped> tf2_fix_transforms_;

  // Camera offsets from base_link frame ENU
  // const float camera_offset_x_ = 0.0f;
  // const float camera_offset_y_ = 0.11f;
  // const float camera_offset_z_ = 0.01f;
  // const float camera_offset_roll_  = 0.0f/180.0f *M_PI;
  // const float camera_offset_pitch_ = 0.0f/180.0f *M_PI;
  // const float camera_offset_yaw_   = 0.0f/180.0f *M_PI;
};

#endif  // ARUCO_GATE_DETECTOR_HPP_