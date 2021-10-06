#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "tf/transform_datatypes.h"

//TODO: MOVE TO CONFIG
double cm[3][3] = {
  {271.41185355986505, 0.0, 328.1468456833846},
  {0.0, 271.9319426344339, 244.31031054397735},
  {0.0, 0.0, 1.0}};
double dc[4] = {
  -0.0392415594794219, -0.014237193354866373, 0.012623563386809904, -0.004288976471675887};
static const cv::Mat cameraMatrix(3, 3, CV_64F, &cm);
static const cv::Mat distCoeffs(1, 4, CV_64F, &dc);
static const std::string OPENCV_WINDOW = "Image window";
#define INPUT_TOPIC "/usb_cam/image_raw/"
#define ARUCO_SIZE 0.175  //meters
#define N_GATES 2
#define GATE_SIZE 1.4  //meters

class ArucoGateDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  image_transport::Publisher output_image_publisher = it_.advertise("gate_detector/image", 1);
  ros::Publisher pub_detections = nh_.advertise<nav_msgs::Path>("gate_detector/detected_poses", 1);
  cv::Ptr<cv::aruco::Dictionary> dictionary;

public:
  ArucoGateDetector() : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe(INPUT_TOPIC, 1, &ArucoGateDetector::imageCb, this);
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    cv::namedWindow(OPENCV_WINDOW);
  }
  ~ArucoGateDetector() { cv::destroyWindow(OPENCV_WINDOW); }

  void imageCb(const sensor_msgs::ImageConstPtr & msg)
  {
    //GEt image
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception & e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //init aruco detection
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    //detect markers on the fisheye, its no worth it to detect over the rectified image
    cv::aruco::detectMarkers(
      cv_ptr->image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    cv::Mat outputImage = cv_ptr->image.clone();
    std::vector<cv::Vec3d> gate_positions(N_GATES), gate_rotations(N_GATES);
    if (markerIds.size() > 0) {  //at least one aruco is detected
      cv::aruco::drawDetectedMarkers(
        outputImage, markerCorners, markerIds);  //draw markers to ensure orientation
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(
        markerCorners, ARUCO_SIZE, cameraMatrix, distCoeffs, rvecs, tvecs);
      for (int i = 0; i < rvecs.size(); ++i) {
        int id = markerIds[i] - 1;
        std::cout << "FOUND MARKER ID: " << id << std::endl;

        if (id < N_GATES && id >= 0) {
          auto rvec = rvecs[i];
          auto tvec = tvecs[i];
          cv::Vec3d t_gate;
          t_gate[0] = -GATE_SIZE / 2;
          auto rout = rvec * 0;
          auto tout = tvec * 0;
          cv::composeRT(rvec * 0, t_gate, rvec, tvec, rout, tout);
          gate_positions[id] = tout;
          gate_rotations[id] = rout;
          cv::aruco::drawAxis(outputImage, cameraMatrix, distCoeffs, rout, tout, 0.5);
        }
      }
    }
    sensor_msgs::ImagePtr output_image_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputImage).toImageMsg();
    output_image_publisher.publish(output_image_msg);
    //Publish path
    nav_msgs::Path path;
    path.header.frame_id = "camera_link";
    for (int i = 0; i < N_GATES; i++) {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "camera_link";
      pose.pose.position.x = gate_positions[i][0];
      pose.pose.position.y = gate_positions[i][1];
      pose.pose.position.z = gate_positions[i][2];
      tf::Quaternion rot;
      rot.setRPY(gate_rotations[i][2], gate_rotations[i][0], gate_rotations[i][1]);
      rot = rot.normalize();
      pose.pose.orientation.x = (double)rot.x();
      pose.pose.orientation.y = (double)rot.y();
      pose.pose.orientation.z = (double)rot.z();
      pose.pose.orientation.w = (double)rot.w();

      path.poses.push_back(pose);
    }
    pub_detections.publish(path);
    // cv::imshow(OPENCV_WINDOW, outputImage);
    // cv::waitKey(3);
  }
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "image_converter");
  ArucoGateDetector agd;

  ros::Rate r(30);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}