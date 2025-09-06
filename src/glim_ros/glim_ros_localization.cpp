#include <glim_ros/glim_ros_localization.hpp>

#include <filesystem>
#include <fstream>
#include <boost/format.hpp>
#include <spdlog/spdlog.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nlohmann/json.hpp>

#include <gtsam/geometry/Pose3.h>
#include <glim/util/config.hpp>
#include <glim/mapping/global_mapping.hpp>
#include <glim/mapping/async_global_mapping.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/odometry/async_odometry_estimation.hpp>

namespace glim {

GlimROSLocalization::GlimROSLocalization(const rclcpp::NodeOptions& options)
  : GlimROS(options),
    map_loaded_(false) {
  
  // Initialize current pose
  current_pose_ = Eigen::Isometry3d::Identity();
  
  // Load localization config (temporarily suppress warnings for missing default params)
  auto current_level = spdlog::get_level();
  spdlog::set_level(spdlog::level::err);  // Suppress warnings temporarily
  glim::Config config_localization(glim::GlobalConfig::get_config_path("config_localization"));
  spdlog::set_level(current_level);  // Restore original log level
  
  // Get parameters from config
  map_path_ = config_localization.param<std::string>("localization", "map_path", "/tmp/dump");
  use_rviz_initial_pose_ = config_localization.param<bool>("localization", "use_rviz_initial_pose", true);
  
  // Initial pose from config (use correct path format)
  initial_x_ = config_localization.param<double>("localization/initial_pose", "x", 0.0);
  initial_y_ = config_localization.param<double>("localization/initial_pose", "y", 0.0);
  initial_z_ = config_localization.param<double>("localization/initial_pose", "z", 0.0);
  initial_roll_ = config_localization.param<double>("localization/initial_pose", "roll", 0.0);
  initial_pitch_ = config_localization.param<double>("localization/initial_pose", "pitch", 0.0);
  initial_yaw_ = config_localization.param<double>("localization/initial_pose", "yaw", 0.0);
  
  // All configuration is primarily from config_localization.json
  // ROS parameters are minimal and for debug/override purposes only
  spdlog::info("Localization configuration loaded from config_localization.json");
  spdlog::info("Map path: {}", map_path_);
  spdlog::info("Initial pose: [{:.2f}, {:.2f}, {:.2f}] [{:.2f}, {:.2f}, {:.2f}]",
               initial_x_, initial_y_, initial_z_, initial_roll_, initial_pitch_, initial_yaw_);
  
  // Setup localization-specific interfaces
  setup_localization_interfaces();
  
  // Load map
  if (!map_path_.empty()) {
    // Note: We need to wait for the base class to initialize before loading the map
    // This will be done after the first timer callback
    spdlog::info("Map will be loaded from: {}", map_path_);
  }
  
  // Set initial pose
  set_initial_pose(initial_x_, initial_y_, initial_z_, 
                   initial_roll_, initial_pitch_, initial_yaw_);
  
  // Create timer for localization processing
  localization_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&GlimROSLocalization::process_localization, this));
  
  spdlog::info("GlimROSLocalization initialized");
}

GlimROSLocalization::~GlimROSLocalization() {
  spdlog::debug("GlimROSLocalization shutting down");
}

bool GlimROSLocalization::load_map(const std::string& map_path) {
  spdlog::info("Loading saved map from: {}", map_path);
  
  // Check if the dump directory exists
  if (!std::filesystem::exists(map_path)) {
    spdlog::error("Map path does not exist: {}", map_path);
    return false;
  }
  
  // Check for required files
  if (!std::filesystem::exists(map_path + "/graph.bin") || 
      !std::filesystem::exists(map_path + "/values.bin")) {
    spdlog::error("Required map files (graph.bin, values.bin) not found in: {}", map_path);
    return false;
  }
  
  // Create standalone GlobalMapping instance to load the map
  // Similar to OfflineViewer approach
  if (!loaded_global_mapping_) {
    glim::GlobalMappingParams params;
    // Disable optimization during loading for faster startup
    params.enable_optimization = false;
    params.isam2_relinearize_skip = 1;
    params.isam2_relinearize_thresh = 0.0;
    
    spdlog::info("Creating GlobalMapping instance for map loading");
    loaded_global_mapping_.reset(new glim::GlobalMapping(params));
  }
  
  // Load the map using GlobalMapping::load()
  spdlog::info("Loading map data from: {}", map_path);
  if (!loaded_global_mapping_->load(map_path)) {
    spdlog::error("Failed to load map from: {}", map_path);
    map_loaded_ = false;
    return false;
  }
  
  spdlog::info("Map loaded successfully!");
  
  // Transfer loaded map to the main GLIM pipeline
  // Now we can access global_mapping as it's protected
  if (global_mapping) {
    // Get the internal GlobalMapping instance
    auto async_global = global_mapping.get();
    if (async_global) {
      auto global_mapping_impl = async_global->get_global_mapping();
      auto main_global_mapping = std::dynamic_pointer_cast<GlobalMapping>(global_mapping_impl);
      
      if (main_global_mapping) {
        // Load the map into the main global mapping
        spdlog::info("Transferring loaded map to main GLIM pipeline");
        if (main_global_mapping->load(map_path)) {
          spdlog::info("Map successfully integrated into main GLIM pipeline");
        } else {
          spdlog::error("Failed to load map into main GLIM pipeline");
        }
      }
    }
  } else {
    spdlog::warn("Main global_mapping not initialized yet, map loaded in standalone mode");
  }
  
  map_loaded_ = true;
  return map_loaded_;
}

void GlimROSLocalization::set_initial_pose(double x, double y, double z,
                                           double roll, double pitch, double yaw) {
  // Create pose
  gtsam::Pose3 initial_pose(
    gtsam::Rot3::RzRyRx(roll, pitch, yaw),
    gtsam::Point3(x, y, z)
  );
  
  // Convert to Eigen
  current_pose_ = Eigen::Isometry3d::Identity();
  current_pose_.linear() = initial_pose.rotation().matrix();
  current_pose_.translation() = initial_pose.translation();
  
  spdlog::info("Initial pose set to: [{:.2f}, {:.2f}, {:.2f}] [{:.2f}, {:.2f}, {:.2f}]",
               x, y, z, roll, pitch, yaw);
  if (map_loaded_) {
    spdlog::info("New trajectory will continue from this position in the loaded map coordinate");
  }
}

void GlimROSLocalization::setup_localization_interfaces() {
  using std::placeholders::_1;
  
  // Initial pose subscriber (from RViz)
  if (use_rviz_initial_pose_) {
    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      std::bind(&GlimROSLocalization::initial_pose_callback, this, _1));
    spdlog::info("Subscribed to /initialpose for RViz2 initial pose");
  }
  
  // Publishers with namespace
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/glim/pose", 10);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/glim/odom", 10);
  
  // TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  
  spdlog::info("Localization ROS interfaces setup complete");
  spdlog::info("Publishing pose to /glim/pose and /glim/odom");
}

void GlimROSLocalization::initial_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
  spdlog::info("Received initial pose from RViz2");
  
  // Extract pose from message
  const auto& pose = msg->pose.pose;
  
  // Convert quaternion to Euler angles
  tf2::Quaternion q(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
  
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  
  // Set new initial pose
  set_initial_pose(pose.position.x, pose.position.y, pose.position.z,
                   roll, pitch, yaw);
}

void GlimROSLocalization::process_localization() {
  // Try to load map if not loaded yet
  static bool first_load_attempt = false;
  if (!map_loaded_ && !first_load_attempt && !map_path_.empty()) {
    first_load_attempt = true;
    // Load the map immediately
    if (load_map(map_path_)) {
      spdlog::info("Map loading completed in process_localization");
      spdlog::info("Map is ready for localization");
    }
  }
  
  // Get actual pose estimation from odometry
  if (odometry_estimation && map_loaded_) {
    // Get the latest estimation results
    std::vector<EstimationFrame::ConstPtr> estimation_frames;
    std::vector<EstimationFrame::ConstPtr> marginalized_frames;
    odometry_estimation->get_results(estimation_frames, marginalized_frames);
    
    if (!estimation_frames.empty()) {
      // Use the latest frame for current pose
      const auto& latest_frame = estimation_frames.back();
      current_pose_ = latest_frame->T_world_sensor();
      
      // Debug output
      static int count = 0;
      if (++count % 100 == 0) {  // Print every 100 frames
        spdlog::debug("Current pose: [{:.2f}, {:.2f}, {:.2f}]",
                     current_pose_.translation().x(),
                     current_pose_.translation().y(),
                     current_pose_.translation().z());
      }
    }
  }
  
  // Publish the current pose (either from odometry or initial pose)
  publish_current_pose();
}

void GlimROSLocalization::publish_current_pose() {
  // Publish as PoseStamped
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = this->now();
  pose_msg.header.frame_id = "map";
  
  pose_msg.pose.position.x = current_pose_.translation().x();
  pose_msg.pose.position.y = current_pose_.translation().y();
  pose_msg.pose.position.z = current_pose_.translation().z();
  
  Eigen::Quaterniond q(current_pose_.rotation());
  pose_msg.pose.orientation.w = q.w();
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  
  pose_pub_->publish(pose_msg);
  
  // Publish as Odometry
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header = pose_msg.header;
  odom_msg.child_frame_id = "base_link";
  odom_msg.pose.pose = pose_msg.pose;
  
  odom_pub_->publish(odom_msg);
  
  // Publish TF
  geometry_msgs::msg::TransformStamped transform;
  transform.header = pose_msg.header;
  transform.child_frame_id = "base_link";
  
  transform.transform.translation.x = pose_msg.pose.position.x;
  transform.transform.translation.y = pose_msg.pose.position.y;
  transform.transform.translation.z = pose_msg.pose.position.z;
  transform.transform.rotation = pose_msg.pose.orientation;
  
  tf_broadcaster_->sendTransform(transform);
}

}  // namespace glim