// "Copyright [year] <Copyright Owner>"

#include "aruco_gate_detector.hpp"

int main(int argc, char * argv[])
{
  // find_device_with_streams
  std::cout << "Starting camera listener node... " << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto ptr = std::make_shared<ArucoGateDetector>();

  while (rclcpp::ok()) {

    rclcpp::spin(ptr);
    
  }

  rclcpp::shutdown();
  return 0;
}