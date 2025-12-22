#ifndef ODOMETRY_HARDWARE_INTERFACE_HPP
#define ODOMETRY_HARDWARE_INTERFACE_HPP

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include <string>

namespace odometry_hardware_interface
{

class OdometryHardware : public hardware_interface::SensorInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  double pos_x_ = 0.0; 
  double pos_y_ = 0.0; 
  double pos_z_ = 0.0;
  double ori_x_ = 0.0; 
  double ori_y_ = 0.0; 
  double ori_z_ = 0.0; 
  double ori_w_ = 1.0;

  double lin_vel_x_ = 0.0; 
  double lin_vel_y_ = 0.0; 
  double lin_vel_z_ = 0.0;
  double ang_vel_x_ = 0.0; 
  double ang_vel_y_ = 0.0; 
  double ang_vel_z_ = 0.0;
};

} // namespace

#endif
