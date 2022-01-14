/*!*******************************************************************************************
 *  \file       aruco_gate_detector.cpp
 *  \brief      Aruco gate detector implementation file.
 *  \authors    David Perez Saura
 *  \copyright  Copyright (c) 2021 Universidad Politécnica de Madrid
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

#include "aruco_gate_detector.hpp"

//TODO: MOVE TO CONFIG

// REAL CAMERA PARAMETERS
double cm[3][3] = {{519.9198295939999, 0.0, 659.1468132131484}, {0.0, 502.46760776515197, 357.57506142772786}, {0.0, 0.0, 1.0}};
double dc[3][3] = {-0.021303744666207214, -0.006628539603283135, -0.007097678030316164, 0.002559386685475455};

#define CAMERA_TOPIC "camera1/image_raw"
#define ARUCO_SIZE 0.175 //meters
#define N_GATES 4
#define GATE_SIZE 1.4 //meters

 
// SIMULATION PARAMETERS
/* double cm[3][3] = { {507.87273461908296,0, 640.5},
                    {0, 507.87273461908296,360.5},
                    {0, 0,1.0}};
double dc[3][3]  = {0,0,0,0,0};

#define CAMERA_TOPIC "camera1/image_raw"

#define ARUCO_SIZE 0.3 //meters
#define N_GATES 6
#define GATE_SIZE 2.4 //1.6 //meters */


//----

static const cv::Mat camera_matrix(3, 3, CV_64F, &cm);
static const cv::Mat dist_coeffs(1, 4, CV_64F, &dc);


ArucoGateDetector::ArucoGateDetector()
    :as2::Node("aruco_gate_detector")
{
  std::string ns = this->get_namespace();

  gate_pose_  = std::make_shared<as2::sensors::Sensor<nav_msgs::msg::Path>>("gate_pose_topic", this);
  gate_img_   = std::make_shared<as2::sensors::Sensor<sensor_msgs::msg::Image>>("gate_img_topic", this);

  cam_image_  = this->create_subscription<sensor_msgs::msg::Image>(
    this->generate_global_name(CAMERA_TOPIC), 1, std::bind(&ArucoGateDetector::imageCallback, this, std::placeholders::_1));
  
  aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
};

void ArucoGateDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr img){

    RCLCPP_DEBUG(this->get_logger(),"Image received by callback");

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &ex) {
        RCLCPP_ERROR(this->get_logger(),"CV_bridge exception: %s\n", ex.what());
    }

    // init ArUco detection
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params = cv::aruco::DetectorParameters::create();

    // detect markers on the fisheye, it's no worth it to detect over the rectified image
    cv::aruco::detectMarkers(cv_ptr->image, aruco_dict_, marker_corners, marker_ids, detector_params, rejected_candidates);
    cv::Mat output_image = cv_ptr->image.clone();
    std::vector<cv::Vec3d> gate_positions(N_GATES), gate_rotations(N_GATES);
    
    if (marker_ids.size() > 0) {
        cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids); // Draw markers to ensure orientation
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(marker_corners, ARUCO_SIZE, camera_matrix, dist_coeffs, rvecs, tvecs);

        for (int i = 0; i < rvecs.size(); ++i){
            int id = marker_ids[i] - 1;
            std::cout << "Found marker ID: " << id << std::endl;

            if (id < N_GATES && id >= 0){
                auto rvec = rvecs[i];
                auto tvec = tvecs[i];
                cv::Vec3d t_gate;
                t_gate[0] = -GATE_SIZE / 2.0f;
                auto rout = rvec * 0;
                auto tout = tvec * 0;
                cv::composeRT(rvec * 0, t_gate, rvec, tvec, rout, tout);
                gate_positions[id] = tout;
                gate_rotations[id] = rout;
                cv::aruco::drawAxis(output_image, camera_matrix, dist_coeffs, rout, tout, 0.5);
            }
        }
    }
    
    //TODO : CONFIGURE THIS ON A LAUNCH FILE
    //use opencv functions to rectify output image using camera matrix and distortion coefficients
    //OPTION 1:
    cv::Mat rectified_image, undistort_camera_matrix;
    cv::Rect roi;
    undistort_camera_matrix = cv::getOptimalNewCameraMatrix(camera_matrix,dist_coeffs,output_image.size(),1,output_image.size(),&roi);
    cv::undistort(output_image,rectified_image,camera_matrix,dist_coeffs,undistort_camera_matrix);
    cv::Mat cropped_image = rectified_image(roi);
     
    //OPTION 2:
    // cv::Mat cropped_image;
    // cv::OutputArray map1 = cv::noArray();
    // cv::OutputArray map2 = cv::noArray();
    // cv::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat(), camera_matrix, output_image.size(),CV_32FC1, map1, map2);
    // cv::remap(output_image, cropped_image, map1, map2, cv::INTER_LINEAR);



    sensor_msgs::msg::Image output_image_msg = *(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cropped_image).toImageMsg().get());
    gate_img_->publishData(output_image_msg);

    // Publish path
    nav_msgs::msg::Path path;
    path.header.frame_id = "camera_link";
    for (int i = 0; i < N_GATES; i++)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "camera_link";
        pose.pose.position.x = gate_positions[i][0];
        pose.pose.position.y = gate_positions[i][1];
        pose.pose.position.z = gate_positions[i][2];
        tf2::Quaternion rot;
        rot.setRPY(gate_rotations[i][2],gate_rotations[i][0],gate_rotations[i][1]);
        rot=rot.normalize();
        pose.pose.orientation.x = (double)rot.x();
        pose.pose.orientation.y = (double)rot.y();
        pose.pose.orientation.z = (double)rot.z();
        pose.pose.orientation.w = (double)rot.w();

        path.poses.push_back(pose);
    }
    
    gate_pose_->publishData(path);
};
