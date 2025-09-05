#ifndef GLIM_ROS_LOCALIZATION_HPP
#define GLIM_ROS_LOCALIZATION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glim_ros/glim_ros.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace glim {

class GlobalMapping;

/**
 * @brief Localization extension of GlimROS
 * 
 * This class extends GlimROS to add pure localization functionality:
 * - Loading pre-built maps
 * - Setting initial pose
 * - Publishing localization results
 */
class GlimROSLocalization : public GlimROS {
public:
  explicit GlimROSLocalization(const rclcpp::NodeOptions& options);
  virtual ~GlimROSLocalization();

  /**
   * @brief Load saved map and initialize localization
   */
  bool load_map(const std::string& map_path);

  /**
   * @brief Set initial pose for localization
   */
  void set_initial_pose(double x, double y, double z, 
                        double roll, double pitch, double yaw);

  /**
   * @brief Process localization updates in timer
   */
  void process_localization();

private:
  /**
   * @brief Setup additional ROS interfaces for localization
   */
  void setup_localization_interfaces();

  /**
   * @brief Callback for initial pose from RViz
   */
  void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);

  /**
   * @brief Publish current pose estimate
   */
  void publish_current_pose();

private:
  // Localization-specific parameters
  std::string map_path_;
  bool use_rviz_initial_pose_;
  bool map_loaded_;
  
  // Initial pose
  double initial_x_, initial_y_, initial_z_;
  double initial_roll_, initial_pitch_, initial_yaw_;
  
  // Current pose estimate
  Eigen::Isometry3d current_pose_;
  
  // Standalone GlobalMapping for loading maps
  std::shared_ptr<GlobalMapping> loaded_global_mapping_;
  
  // Additional ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr localization_timer_;
};

}  // namespace glim

#endif  // GLIM_ROS_LOCALIZATION_HPP