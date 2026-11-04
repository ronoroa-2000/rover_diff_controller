#include "rover_diff_controller/rover_diff_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace rover_diff_controller
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;
using controller_interface::return_type;

namespace
{
double yaw_to_qz(double yaw) {return std::sin(yaw * 0.5);}
double yaw_to_qw(double yaw) {return std::cos(yaw * 0.5);}
}  // namespace

InterfaceConfiguration RoverDiffController::command_interface_configuration() const
{
  InterfaceConfiguration cfg{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto * side : {&left_, &right_}) {
    for (const auto & n : side->names) {
      cfg.names.push_back(n + "/" + hardware_interface::HW_IF_VELOCITY);
    }
  }
  return cfg;
}

InterfaceConfiguration RoverDiffController::state_interface_configuration() const
{
  // Velocity for every wheel (left wheels first, then right). Effort, in the same order, is
  // only claimed when slip detection is on (e.g. the Gazebo hardware interface has no effort).
  InterfaceConfiguration cfg{interface_configuration_type::INDIVIDUAL, {}};
  for (const char * type : {hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT}) {
    if (std::string(type) == hardware_interface::HW_IF_EFFORT && !slip_.enabled) {break;}
    for (const auto * side : {&left_, &right_}) {
      for (const auto & n : side->names) {
        cfg.names.push_back(n + "/" + type);
      }
    }
  }
  return cfg;
}

CallbackReturn RoverDiffController::on_init()
{
  auto node = get_node();
  try {
    node->declare_parameter<std::vector<std::string>>("left_wheel_names", std::vector<std::string>{});
    node->declare_parameter<std::vector<std::string>>("right_wheel_names", std::vector<std::string>{});
    node->declare_parameter<double>("wheel_radius", 0.0);
    node->declare_parameter<double>("wheel_separation", 0.0);
    node->declare_parameter<double>("left_wheel_radius_multiplier", 1.0);
    node->declare_parameter<double>("right_wheel_radius_multiplier", 1.0);
    node->declare_parameter<double>("wheel_separation_multiplier", 1.0);
    node->declare_parameter<double>("cmd_vel_timeout", 0.25);
    node->declare_parameter<double>("publish_rate", 50.0);
    node->declare_parameter<bool>("use_stamped_vel", true);
    node->declare_parameter<bool>("enable_odom_tf", false);
    node->declare_parameter<std::string>("odom_frame_id", "odom");
    node->declare_parameter<std::string>("base_frame_id", "base_link");
    node->declare_parameter<std::vector<double>>(
      "pose_covariance_diagonal", {0.001, 0.001, 1e-3, 1e-3, 1e-3, 0.01});
    node->declare_parameter<std::vector<double>>(
      "twist_covariance_diagonal", {0.001, 0.001, 1e-3, 1e-3, 1e-3, 0.01});
    node->declare_parameter<bool>("slip_detection.enabled", false);
    for (const char * name : {"min_wheel_speed", "min_torque_ref", "torque_ratio",
        "velocity_excess", "confirm_time", "release_time", "command_scale"})
    {
      node->declare_parameter<double>(std::string("slip_detection.") + name, 0.0);
    }
    for (const char * axis : {"linear.x", "angular.z"}) {
      const std::string p = axis;
      node->declare_parameter<double>(p + ".max_velocity", 0.0);
      node->declare_parameter<double>(p + ".min_velocity", 0.0);
      node->declare_parameter<double>(p + ".max_acceleration", 0.0);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn RoverDiffController::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  auto logger = node->get_logger();

  left_.names = node->get_parameter("left_wheel_names").as_string_array();
  right_.names = node->get_parameter("right_wheel_names").as_string_array();

  if (left_.names.empty() || right_.names.empty()) {
    RCLCPP_ERROR(logger, "left_wheel_names and right_wheel_names must be non-empty");
    return CallbackReturn::ERROR;
  }
  wheel_radius_ = node->get_parameter("wheel_radius").as_double();
  wheel_separation_ = node->get_parameter("wheel_separation").as_double();
  if (wheel_radius_ <= 0.0 || wheel_separation_ <= 0.0) {
    RCLCPP_ERROR(logger, "wheel_radius and wheel_separation must be > 0");
    return CallbackReturn::ERROR;
  }
  left_.radius_multiplier = node->get_parameter("left_wheel_radius_multiplier").as_double();
  right_.radius_multiplier = node->get_parameter("right_wheel_radius_multiplier").as_double();
  r_left_ = wheel_radius_ * left_.radius_multiplier;
  r_right_ = wheel_radius_ * right_.radius_multiplier;
  sep_ = wheel_separation_ * node->get_parameter("wheel_separation_multiplier").as_double();

  cmd_vel_timeout_ = node->get_parameter("cmd_vel_timeout").as_double();
  publish_rate_ = node->get_parameter("publish_rate").as_double();
  use_stamped_vel_ = node->get_parameter("use_stamped_vel").as_bool();
  enable_odom_tf_ = node->get_parameter("enable_odom_tf").as_bool();
  odom_frame_id_ = node->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = node->get_parameter("base_frame_id").as_string();

  auto read_limits = [&](const std::string & p) {
      Limits l;
      l.max_vel = node->get_parameter(p + ".max_velocity").as_double();
      l.min_vel = node->get_parameter(p + ".min_velocity").as_double();
      l.max_acc = node->get_parameter(p + ".max_acceleration").as_double();
      return l;
    };
  linear_limits_ = read_limits("linear.x");
  angular_limits_ = read_limits("angular.z");

  auto read_cov = [&](const std::string & p, std::array<double, 36> & cov) {
      const auto diag = node->get_parameter(p).as_double_array();
      if (diag.size() != 6) {
        RCLCPP_ERROR(logger, "%s must have 6 entries", p.c_str());
        return false;
      }
      cov.fill(0.0);
      for (size_t i = 0; i < 6; ++i) {cov[i * 7] = diag[i];}
      return true;
    };
  if (!read_cov("pose_covariance_diagonal", pose_cov_) ||
    !read_cov("twist_covariance_diagonal", twist_cov_))
  {
    return CallbackReturn::ERROR;
  }

  slip_.enabled = node->get_parameter("slip_detection.enabled").as_bool();
  if (slip_.enabled) {
    auto get = [&](const char * n) {
        return node->get_parameter(std::string("slip_detection.") + n).as_double();
      };
    slip_.min_wheel_speed = get("min_wheel_speed");
    slip_.min_torque_ref = get("min_torque_ref");
    slip_.torque_ratio = get("torque_ratio");
    slip_.velocity_excess = get("velocity_excess");
    slip_.confirm_time = get("confirm_time");
    slip_.release_time = get("release_time");
    slip_.command_scale = get("command_scale");
    if (slip_.command_scale < 0.0 || slip_.command_scale > 1.0) {
      RCLCPP_ERROR(logger, "slip_detection.command_scale must be in [0, 1]");
      return CallbackReturn::ERROR;
    }
  }
  const size_t n_wheels = left_.names.size() + right_.names.size();
  wheel_slip_.assign(n_wheels, WheelSlip{});
  wheel_vel_.assign(n_wheels, 0.0);
  wheel_torque_.assign(n_wheels, 0.0);
  scratch_.assign(n_wheels, 0.0);

  slip_pub_ = node->create_publisher<std_msgs::msg::Int8MultiArray>(
    "~/wheel_slip", rclcpp::SystemDefaultsQoS());
  rt_slip_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Int8MultiArray>>(slip_pub_);
  rt_slip_pub_->msg_.data.assign(n_wheels, 0);

  // Command subscription
  stamped_sub_.reset();
  twist_sub_.reset();
  auto clock = node->get_clock();
  if (use_stamped_vel_) {
    stamped_sub_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
      "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) {
        Command c;
        c.linear = msg->twist.linear.x;
        c.angular = msg->twist.angular.z;
        c.stamp = get_node()->now();
        command_box_.set(c);
      });
  } else {
    twist_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "~/cmd_vel_unstamped", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
        Command c;
        c.linear = msg->linear.x;
        c.angular = msg->angular.z;
        c.stamp = get_node()->now();
        command_box_.set(c);
      });
  }

  // Odometry / TF publishers
  odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>(
    "~/odom", rclcpp::SystemDefaultsQoS());
  rt_odom_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_pub_);
  {
    auto & m = rt_odom_pub_->msg_;
    m.header.frame_id = odom_frame_id_;
    m.child_frame_id = base_frame_id_;
    m.pose.covariance = pose_cov_;
    m.twist.covariance = twist_cov_;
  }
  if (enable_odom_tf_) {
    tf_pub_ = node->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::SystemDefaultsQoS());
    rt_tf_pub_ =
      std::make_unique<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(tf_pub_);
    rt_tf_pub_->msg_.transforms.resize(1);
    rt_tf_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
    rt_tf_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
  }

  RCLCPP_INFO(
    logger, "Configured: %zu left / %zu right wheels, r=%.3f (L x%.2f, R x%.2f), sep=%.3f",
    left_.names.size(), right_.names.size(), wheel_radius_,
    left_.radius_multiplier, right_.radius_multiplier, sep_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn RoverDiffController::on_activate(const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != left_.names.size() + right_.names.size() ||
    state_interfaces_.size() != (slip_.enabled ? 2 : 1) * command_interfaces_.size())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Unexpected number of claimed interfaces");
    return CallbackReturn::ERROR;
  }
  wheel_slip_.assign(wheel_slip_.size(), WheelSlip{});
  x_ = y_ = theta_ = 0.0;
  prev_linear_cmd_ = prev_angular_cmd_ = 0.0;
  last_publish_ = get_node()->now();
  // Drop any command received while inactive.
  command_box_.set(Command{});
  return CallbackReturn::SUCCESS;
}

CallbackReturn RoverDiffController::on_deactivate(const rclcpp_lifecycle::State &)
{
  set_wheels_zero();
  return CallbackReturn::SUCCESS;
}

void RoverDiffController::set_wheels_zero()
{
  for (auto & ci : command_interfaces_) {
    (void)ci.set_value(0.0);
  }
}

double RoverDiffController::limit(double target, double previous, const Limits & l, double dt)
{
  if (l.max_vel > 0.0 || l.min_vel < 0.0) {
    target = std::clamp(target, l.min_vel, l.max_vel);
  }
  if (l.max_acc > 0.0 && dt > 0.0) {
    const double dv = l.max_acc * dt;
    target = std::clamp(target, previous - dv, previous + dv);
  }
  return target;
}

double RoverDiffController::median(std::vector<double> & v)
{
  if (v.empty()) {return 0.0;}
  const size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  double m = v[mid];
  if (v.size() % 2 == 0) {  // even count: mean of the two middle values
    m = 0.5 * (m + *std::max_element(v.begin(), v.begin() + mid));
  }
  return m;
}

// Flags wheels on [begin, end) that are unloaded (lifted / free-spinning: torque far below
// the side median) or over-spinning (faster than the side median). Debounced both ways.
void RoverDiffController::update_slip(size_t begin, size_t end, double dt)
{
  const size_t count = end - begin;
  scratch_.resize(count);
  for (size_t i = 0; i < count; ++i) {scratch_[i] = std::abs(wheel_vel_[begin + i]);}
  const double speed_ref = median(scratch_);
  scratch_.resize(count);
  for (size_t i = 0; i < count; ++i) {scratch_[i] = std::abs(wheel_torque_[begin + i]);}
  const double torque_ref = median(scratch_);

  const bool active = slip_.enabled && speed_ref >= slip_.min_wheel_speed &&
    torque_ref >= slip_.min_torque_ref;

  for (size_t i = begin; i < end; ++i) {
    auto & ws = wheel_slip_[i];
    const double torque = std::abs(wheel_torque_[i]);
    const bool candidate = active && std::isfinite(torque) &&
      (torque < slip_.torque_ratio * torque_ref ||
      std::abs(wheel_vel_[i]) - speed_ref > slip_.velocity_excess);
    if (candidate) {
      ws.clear_time = 0.0;
      ws.candidate_time += dt;
      if (ws.candidate_time >= slip_.confirm_time) {ws.slipping = true;}
    } else {
      ws.candidate_time = 0.0;
      if (ws.slipping) {
        ws.clear_time += dt;
        if (ws.clear_time >= slip_.release_time) {
          ws.slipping = false;
          ws.clear_time = 0.0;
        }
      }
    }
  }
}

return_type RoverDiffController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  const size_t n_left = left_.names.size();
  const size_t n = n_left + right_.names.size();

  // ---- Wheel feedback (left wheels first, then right) -------------------
  bool feedback_ok = true;
  for (size_t i = 0; i < n; ++i) {
    const auto v = state_interfaces_[i].get_optional<double>();
    const auto t = slip_.enabled ? state_interfaces_[n + i].get_optional<double>() :
      std::optional<double>{};
    if (!v.has_value() || !std::isfinite(v.value())) {
      feedback_ok = false;
    } else {
      wheel_vel_[i] = v.value();
    }
    wheel_torque_[i] = t.value_or(std::numeric_limits<double>::quiet_NaN());
  }

  if (feedback_ok && slip_.enabled) {
    update_slip(0, n_left, dt);
    update_slip(n_left, n, dt);
  }

  // ---- Odometry from per-side median wheel velocity ----------------------
  // Median is robust to a single slipping, lifted or faulty wheel; wheels already flagged
  // as slipping are excluded first (falling back to all wheels if every one is flagged).
  auto side_speed = [&](size_t begin, size_t end, double radius) {
      scratch_.clear();
      for (size_t i = begin; i < end; ++i) {
        if (!wheel_slip_[i].slipping) {scratch_.push_back(wheel_vel_[i]);}
      }
      if (scratch_.empty()) {
        for (size_t i = begin; i < end; ++i) {scratch_.push_back(wheel_vel_[i]);}
      }
      return median(scratch_) * radius;  // m/s
    };

  double lin_fb = 0.0, ang_fb = 0.0;
  if (feedback_ok) {
    const double v_l = side_speed(0, n_left, r_left_);
    const double v_r = side_speed(n_left, n, r_right_);
    lin_fb = 0.5 * (v_l + v_r);
    ang_fb = (v_r - v_l) / sep_;
    if (dt > 0.0) {
      const double mid = theta_ + 0.5 * ang_fb * dt;
      x_ += lin_fb * std::cos(mid) * dt;
      y_ += lin_fb * std::sin(mid) * dt;
      theta_ = std::atan2(std::sin(theta_ + ang_fb * dt), std::cos(theta_ + ang_fb * dt));
    }
  }

  // ---- Command: timeout, limit, convert to wheel speeds ------------------
  Command cmd;
  command_box_.try_get([&](const Command & c) {cmd = c;});
  double lin_cmd = cmd.linear, ang_cmd = cmd.angular;
  if (cmd.stamp.nanoseconds() == 0 || (time - cmd.stamp).seconds() > cmd_vel_timeout_) {
    lin_cmd = ang_cmd = 0.0;
  }
  lin_cmd = limit(lin_cmd, prev_linear_cmd_, linear_limits_, dt);
  ang_cmd = limit(ang_cmd, prev_angular_cmd_, angular_limits_, dt);
  prev_linear_cmd_ = lin_cmd;
  prev_angular_cmd_ = ang_cmd;

  const double left_w = (lin_cmd - ang_cmd * sep_ * 0.5) / r_left_;
  const double right_w = (lin_cmd + ang_cmd * sep_ * 0.5) / r_right_;
  for (size_t i = 0; i < n; ++i) {
    const double w = i < n_left ? left_w : right_w;
    (void)command_interfaces_[i].set_value(wheel_slip_[i].slipping ? w * slip_.command_scale : w);
  }

  // ---- Publish ----------------------------------------------------------
  if (publish_rate_ > 0.0 && (time - last_publish_).seconds() < 1.0 / publish_rate_) {
    return return_type::OK;
  }
  last_publish_ = time;

  if (rt_slip_pub_ && rt_slip_pub_->trylock()) {
    for (size_t i = 0; i < n; ++i) {
      rt_slip_pub_->msg_.data[i] = wheel_slip_[i].slipping ? 1 : 0;
    }
    rt_slip_pub_->unlockAndPublish();
  }
  if (rt_odom_pub_ && rt_odom_pub_->trylock()) {
    auto & m = rt_odom_pub_->msg_;
    m.header.stamp = time;
    m.pose.pose.position.x = x_;
    m.pose.pose.position.y = y_;
    m.pose.pose.orientation.z = yaw_to_qz(theta_);
    m.pose.pose.orientation.w = yaw_to_qw(theta_);
    m.twist.twist.linear.x = lin_fb;
    m.twist.twist.angular.z = ang_fb;
    rt_odom_pub_->unlockAndPublish();
  }
  if (rt_tf_pub_ && rt_tf_pub_->trylock()) {
    auto & t = rt_tf_pub_->msg_.transforms[0];
    t.header.stamp = time;
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.rotation.z = yaw_to_qz(theta_);
    t.transform.rotation.w = yaw_to_qw(theta_);
    rt_tf_pub_->unlockAndPublish();
  }
  return return_type::OK;
}

}  // namespace rover_diff_controller

PLUGINLIB_EXPORT_CLASS(
  rover_diff_controller::RoverDiffController, controller_interface::ControllerInterface)
