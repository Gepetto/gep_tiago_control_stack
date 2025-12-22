#include "odometry_gz_hw_interface/odometry_hardware.hpp"

#include <functional>  // std::bind
#include <pluginlib/class_list_macros.hpp>

namespace odometry_gz_hw_interface
{

hardware_interface::CallbackReturn OdometryHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  auto ret = hardware_interface::SystemInterface::on_init(info);
  if (ret != hardware_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  auto it = info.hardware_parameters.find("topic_name");
  if (it != info.hardware_parameters.end()) {
    topic_name_ = it->second;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("OdometryHardware"),
    "OdometryHardware on_init(), topic_name = '%s'", topic_name_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool OdometryHardware::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, std::size_t> & joint_name_to_index,
  const hardware_interface::HardwareInfo & hardware_info,
  gz::sim::v8::EntityComponentManager & ecm,
  unsigned int model_instance)
{
  (void)joint_name_to_index;
  (void)hardware_info;
  (void)ecm;
  (void)model_instance;

  nh_ = model_nh;

  RCLCPP_INFO(
    nh_->get_logger(),
    "OdometryHardware initSim(), subscribing to '%s'", topic_name_.c_str());

  auto qos = rclcpp::SystemDefaultsQoS();
  odom_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(
    topic_name_,
    qos,
    std::bind(&OdometryHardware::odomCallback, this, std::placeholders::_1));

  return true;
}

std::vector<hardware_interface::StateInterface> OdometryHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  const auto & sensor_name = info_.sensors[0].name;

  // Position
  state_interfaces.emplace_back(sensor_name, "position.x", &pos_x_);
  state_interfaces.emplace_back(sensor_name, "position.y", &pos_y_);
  state_interfaces.emplace_back(sensor_name, "position.z", &pos_z_);

  // Orientation
  state_interfaces.emplace_back(sensor_name, "orientation.x", &ori_x_);
  state_interfaces.emplace_back(sensor_name, "orientation.y", &ori_y_);
  state_interfaces.emplace_back(sensor_name, "orientation.z", &ori_z_);
  state_interfaces.emplace_back(sensor_name, "orientation.w", &ori_w_);

  // Linear velocities
  state_interfaces.emplace_back(sensor_name, "linear_velocity.x", &lin_vel_x_);
  state_interfaces.emplace_back(sensor_name, "linear_velocity.y", &lin_vel_y_);
  state_interfaces.emplace_back(sensor_name, "linear_velocity.z", &lin_vel_z_);

  // Angular velocities
  state_interfaces.emplace_back(sensor_name, "angular_velocity.x", &ang_vel_x_);
  state_interfaces.emplace_back(sensor_name, "angular_velocity.y", &ang_vel_y_);
  state_interfaces.emplace_back(sensor_name, "angular_velocity.z", &ang_vel_z_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> OdometryHardware::export_command_interfaces()
{
  return {};
}

hardware_interface::return_type OdometryHardware::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  (void)time;
  (void)period;

  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_msg_) {
    return hardware_interface::return_type::OK;
  }

  // Position
  const auto & p = last_odom_.pose.pose.position;
  pos_x_ = p.x;
  pos_y_ = p.y;
  pos_z_ = p.z;

  // Orientation (quaternion)
  const auto & q = last_odom_.pose.pose.orientation;
  ori_x_ = q.x;
  ori_y_ = q.y;
  ori_z_ = q.z;
  ori_w_ = q.w;

  // Linear velocity
  const auto & v = last_odom_.twist.twist.linear;
  lin_vel_x_ = v.x;
  lin_vel_y_ = v.y;
  lin_vel_z_ = v.z;

  // Angular velocity
  const auto & w = last_odom_.twist.twist.angular;
  ang_vel_x_ = w.x;
  ang_vel_y_ = w.y;
  ang_vel_z_ = w.z;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OdometryHardware::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  (void)time;
  (void)period;

  return hardware_interface::return_type::OK;
}

void OdometryHardware::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_odom_ = *msg;
  has_msg_ = true;
}

}  // namespace odometry_gz_hw_interface

// pluginlib : base = GazeboSimSystemInterface
PLUGINLIB_EXPORT_CLASS(
  odometry_gz_hw_interface::OdometryHardware,
  gz_ros2_control::GazeboSimSystemInterface
)
