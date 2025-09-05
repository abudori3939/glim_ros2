#include <iostream>
#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>

#include <glim_ros/glim_ros_localization.hpp>
#include <glim/util/config.hpp>

/**
 * @brief Simple launcher node for GLIM localization
 * 
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
  
  // Optional: Save current state on shutdown
  std::string dump_path = "/tmp/dump";
  bool dump_on_shutdown = false;
  
  // Parameters should be declared before shutdown
  try {
    glim_localization->get_parameter("dump_path", dump_path);
    glim_localization->get_parameter("dump_on_shutdown", dump_on_shutdown);
  } catch (...) {
    // Use defaults if parameters not found
  }
  
  if (dump_on_shutdown) {
    spdlog::info("Saving current state to: {}", dump_path);
    glim_localization->save(dump_path);
  }
  
  return 0;
}