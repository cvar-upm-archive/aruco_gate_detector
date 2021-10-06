// "Copyright [year] <Copyright Owner>"

#include "aruco_gate_detector.hpp"

int main(int argc, char * argv[])
{
  // find_device_with_streams
  std::cout << "Starting camera listener node... " << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto ptr = std::make_shared<ArucoGateDetector>();
  //   ptr->setupDetector();
  //   ptr->setupTf();
  //   rclcpp::Rate r(200);

  while (rclcpp::ok()) {
    rclcpp::spin(ptr);
    // r.sleep();
  }

  rclcpp::shutdown();
  return 0;
}