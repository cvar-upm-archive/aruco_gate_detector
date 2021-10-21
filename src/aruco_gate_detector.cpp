#include "aruco_gate_detector.hpp"

//TODO: MOVE TO CONFIG

// double cm[3][3] = {{790.8989920514197, 0.0, 670.332791421756},{0.0, 789.6808338497912, 370.6481124492188}, {0.0, 0.0, 1.0}};
// double dc[3][3] = {-0.03448682771417174, -0.055932650937412745, 0.11969799783448262, -0.09163586323944228};


double cm[3][3] = {{3.92621786e+03,0.00000000e+00 , 6.65363971e+02},
 {0.00000000e+00, 4.60080223e+03,3.72093033e+02},
 {0.00000000e+00, 0.00000000e+00,1.00000000e+00}};


double dc[3][3]  = {-1.58518103e+01 , 3.51438196e+02 , 3.63865440e-03, -4.48952034e-02, -3.11620466e+03};
// double cm[3][3] = {{271.41185355986505, 0.0, 328.1468456833846}, 
//                 {0.0, 271.9319426344339, 244.31031054397735}, 
//                 {0.0, 0.0, 1.0}};
// double dc[4] = {-0.0392415594794219, -0.014237193354866373, 0.012623563386809904, -0.004288976471675887};
static const cv::Mat camera_matrix(3, 3, CV_64F, &cm);
static const cv::Mat dist_coeffs(1, 4, CV_64F, &dc);

#define ARUCO_SIZE 0.175 //meters
#define N_GATES 2
#define GATE_SIZE 1.4 //meters
// #define CAMERA_TOPIC "/drone0/camera1/image_raw"
#define CAMERA_TOPIC "image_raw"

ArucoGateDetector::ArucoGateDetector()
    :aerostack2::Node("aruco_gate_detector")
{
  gate_pose_  = std::make_shared<aerostack2::Sensor<nav_msgs::msg::Path>>("gate_pose_topic", this);
  gate_img_   = std::make_shared<aerostack2::Sensor<sensor_msgs::msg::Image>>("gate_img_topic", this);

  cam_image_  = this->create_subscription<sensor_msgs::msg::Image>(
    CAMERA_TOPIC, 1, std::bind(&ArucoGateDetector::imageCallback, this, std::placeholders::_1));
  
  aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
};

void ArucoGateDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr img){

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
                t_gate[0] = -GATE_SIZE / 2;
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
