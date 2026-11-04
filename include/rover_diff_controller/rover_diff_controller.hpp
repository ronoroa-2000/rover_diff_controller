#ifndef ROVER_DIFF_CONTROLLER__ROVER_DIFF_CONTROLLER_HPP_
#define ROVER_DIFF_CONTROLLER__ROVER_DIFF_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace rover_diff_controller
{

class RoverDiffController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Command
  {
    double linear = 0.0;
    double angular = 0.0;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  struct Limits
  {
    double min_vel = 0.0;
    double max_vel = 0.0;
    double max_acc = 0.0;  // <= 0 disables acceleration limiting
  };

  struct Side
  {
    std::vector<std::string> names;
    double radius_multiplier = 1.0;
  };

  // Slip / lifted-wheel detection from per-wheel torque (effort) and velocity.
  struct SlipParams
  {
    bool enabled = false;
    double min_wheel_speed = 0.5;   // [rad/s] side median speed below which detection is idle
    double min_torque_ref = 0.3;    // [Nm] side median |torque| below which detection is idle
    double torque_ratio = 0.25;     // wheel is "unloaded" below ratio * side median |torque|
    double velocity_excess = 1.5;   // [rad/s] wheel faster than side median by this => spinning
    double confirm_time = 0.1;      // [s] condition must hold this long to flag
    double release_time = 0.3;      // [s] condition must be clear this long to unflag
    double command_scale = 0.5;     // command multiplier applied to a flagged wheel
  };

  struct WheelSlip
  {
    double candidate_time = 0.0;
    double clear_time = 0.0;
    bool slipping = false;
  };

  void update_slip(size_t begin, size_t end, double dt);
  static double median(std::vector<double> & v);

  static double limit(double target, double previous, const Limits & l, double dt);
  void set_wheels_zero();

  // Parameters
  Side left_, right_;
  double wheel_radius_ = 0.0;
  double wheel_separation_ = 0.0;
  double cmd_vel_timeout_ = 0.25;
  double publish_rate_ = 50.0;
  bool use_stamped_vel_ = true;
  bool enable_odom_tf_ = false;
  std::string odom_frame_id_, base_frame_id_;
  Limits linear_limits_, angular_limits_;
  std::array<double, 36> pose_cov_{}, twist_cov_{};

  SlipParams slip_;
  std::vector<WheelSlip> wheel_slip_;     // left wheels first, then right
  std::vector<double> wheel_vel_, wheel_torque_, scratch_;

  // Derived
  double r_left_ = 0.0, r_right_ = 0.0, sep_ = 0.0;

  // Command input
  realtime_tools::RealtimeThreadSafeBox<Command> command_box_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

  // Odometry state
  double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
  double prev_linear_cmd_ = 0.0, prev_angular_cmd_ = 0.0;
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> rt_odom_pub_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int8MultiArray>> slip_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Int8MultiArray>> rt_slip_pub_;
  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> tf_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> rt_tf_pub_;
};

}  // namespace rover_diff_controller

#endif  // ROVER_DIFF_CONTROLLER__ROVER_DIFF_CONTROLLER_HPP_
