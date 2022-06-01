/*!*******************************************************************************************
 *  \file       aruco_gate_detector.hpp
 *  \brief      Aruco gate detector header file.
 *  \authors    David Perez Saura
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef ARUCO_GATE_DETECTOR_HPP_
#define ARUCO_GATE_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include "as2_core/node.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/names/topics.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <memory>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

class ArucoGateDetector : public as2::Node
{
public:
  /**
   * @brief Construct a new Aruco Gate Detector object
   */
  ArucoGateDetector();

  /**
   * @brief Destroy the Aruco Gate Detector object
   */
  ~ArucoGateDetector(){};

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_image_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gate_pose_pub_;
  std::shared_ptr<as2::sensors::Camera> gate_img_transport_;

  // std::shared_ptr<as2::sensors::Sensor<nav_msgs::msg::Path>> gate_pose_;
  // std::shared_ptr<as2::sensors::Sensor<sensor_msgs::msg::Image>> gate_img_;

  int n_aruco_ids_;
  float aruco_size_;
  float gate_size_;
  std::string camera_model_;
  std::string distorsion_model_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;

  void setCameraInfo(const cv::Mat &_camera_matrix, const cv::Mat &_dist_coeffs);
  void loadParameters();

public:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr img);
};

#endif // ARUCO_GATE_DETECTOR_HPP_