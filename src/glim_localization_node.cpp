#include <iostream>
#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>

#include <glim_ros/glim_ros_localization.hpp>
#include <glim/util/config.hpp>

/**
 * @brief GLIM localization node
 * 
 * Launch with: ros2 run glim_ros localization
 * 
 * Configuration is loaded from glim/config/config_localization.json
 * This node creates and runs GlimROSLocalization which extends
 * the standard GlimROS with localization capabilities.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  // Create localization node
  auto glim_localization = std::make_shared<glim::GlimROSLocalization>(options);
  
  spdlog::info("Starting GLIM localization node");
  spdlog::info("Localization mode: Loading pre-built map and continuing SLAM from it");
  
  // Spin the node
  rclcpp::spin(glim_localization);
  
  // Shutdown
  rclcpp::shutdown();
  
  // Wait for processing to complete
  glim_localization->wait();
  
  // Shutdown settings are configured in config_localization.json
  // No automatic save on shutdown for localization mode by default
  
  return 0;
}