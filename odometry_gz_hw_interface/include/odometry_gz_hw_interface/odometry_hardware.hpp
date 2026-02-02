#pragma once

#include <gz_ros2_control/gz_system_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <map>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace odometry_gz_hw_interface {

class OdometryHardware : public gz_ros2_control::GazeboSimSystemInterface {
 public:
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  bool initSim(rclcpp::Node::SharedPtr& model_nh,
               std::map<std::string, std::size_t>& joint_name_to_index,
               const hardware_interface::HardwareInfo& hardware_info,
               gz::sim::v8::EntityComponentManager& ecm,
               unsigned int model_instance) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  std::string topic_name_ = "/mobile_base_controller/odom";

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Buffer odom
  nav_msgs::msg::Odometry last_odom_;
  bool has_msg_ = false;
  std::mutex mutex_;

  // Position
  double pos_x_ = 0.0;
  double pos_y_ = 0.0;
  double pos_z_ = 0.0;

  // Orientation (quaternion)
  double ori_x_ = 0.0;
  double ori_y_ = 0.0;
  double ori_z_ = 0.0;
  double ori_w_ = 1.0;

  // linear velocities
  double lin_vel_x_ = 0.0;
  double lin_vel_y_ = 0.0;
  double lin_vel_z_ = 0.0;

  // angular velocities
  double ang_vel_x_ = 0.0;
  double ang_vel_y_ = 0.0;
  double ang_vel_z_ = 0.0;
};

}  // namespace odometry_gz_hw_interface
