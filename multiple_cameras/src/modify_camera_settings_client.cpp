/*
Author:
    Lachlan Mares, lachlan.mares@gmail.com

License:
    GPL-3.0

Description:

*/

#include "rclcpp/rclcpp.hpp"
#include "multiple_cameras/srv/modify_camera_settings.hpp"                                      

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 6) { 
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: modify_camera_settings ID enable_publishing height width fps");      
    return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("modify_camera_settings_client");  
  rclcpp::Client<multiple_cameras::srv::ModifyCameraSettings>::SharedPtr client =                
    node->create_client<multiple_cameras::srv::ModifyCameraSettings>("multiple_camera/modify_camera_settings");          

  auto request = std::make_shared<multiple_cameras::srv::ModifyCameraSettings::Request>();       
  request->camera_id = atoll(argv[1]);
  request->enable_publishing = atoll(argv[2]) > 0 ? true : false;
  request->height = atoll(argv[3]);         
  request->width = atoll(argv[4]);   
  request->fps = atoll(argv[5]);                                                        

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Modify camera settings: %s", result.get()->success ? "Success" : "Fail");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service modify_camera_settings");    
  }

  rclcpp::shutdown();
  return 0;
}