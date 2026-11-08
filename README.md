# rover_diff_controller

A `ros2_control` differential-drive controller for multi-wheel skid-steer / rocker-bogie
rovers (N wheels per side, one velocity command per side). Developed for a 6-wheel rover
driven by ODrive S1 controllers through the
[`ros_odrive`](https://github.com/odriverobotics/ros_odrive) `odrive_ros2_control` hardware
interface, but it works with any hardware interface that exposes wheel joints with a
`velocity` command interface and a `velocity` state interface (Gazebo's
`gz_ros2_control` included).

**What it does differently from the stock `diff_drive_controller`**

- Odometry uses the **median wheel velocity of each side**, so one slipping, lifted or
  faulty wheel does not corrupt the pose.
- Optional **lifted / slipping wheel detection** from per-wheel torque (`effort`) and
  velocity, with an `Int8MultiArray` flag topic. Flagged wheels are excluded from odometry
  and their command is scaled down.
- Per-side radius multipliers and a separation multiplier for calibration.

Tested with ROS 2 **Jazzy**.

## Install

```bash
mkdir -p ~/rover_ws/src && cd ~/rover_ws/src
git clone <this-repo-url> rover_diff_controller
cd ~/rover_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select rover_diff_controller
source install/setup.bash
```

Check the plugin is registered (with a controller manager running, `ros2 control
list_controller_types` also lists it):

```bash
ls install/rover_diff_controller/share/ament_index/resource_index/controller_interface__pluginlib__plugin/
```

## Use

1. Your URDF needs a `ros2_control` block whose wheel joints have a `velocity` command
   interface and `velocity` state interface (plus `effort` state if you enable slip detection).
2. Register and configure the controller in your controller YAML. A complete example is in
   [`config/rover_diff_controller.yaml`](config/rover_diff_controller.yaml):

   ```yaml
   controller_manager:
     ros__parameters:
       update_rate: 100
       joint_state_broadcaster:
         type: joint_state_broadcaster/JointStateBroadcaster
       rover_controller:
         type: rover_diff_controller/RoverDiffController

   rover_controller:
     ros__parameters:
       left_wheel_names: ['L_1_Wheel_Joint', 'L_2_Wheel_Joint', 'L_3_Wheel_Joint']
       right_wheel_names: ['R_1_Wheel_Joint', 'R_2_Wheel_Joint', 'R_3_Wheel_Joint']
       wheel_radius: 0.110
       wheel_separation: 0.728
   ```
3. Spawn it: `ros2 run controller_manager spawner rover_controller`.
4. Drive it: publish `geometry_msgs/TwistStamped` on `/rover_controller/cmd_vel`
   (or `Twist` on `/rover_controller/cmd_vel_unstamped` with `use_stamped_vel: false`).

Parameters are read when the controller is configured; changing one needs a reconfigure.

## Topics

| Topic (`~` = controller name) | Type | Direction |
|---|---|---|
| `~/cmd_vel` | `geometry_msgs/TwistStamped` | in (`use_stamped_vel: true`) |
| `~/cmd_vel_unstamped` | `geometry_msgs/Twist` | in (`use_stamped_vel: false`) |
| `~/odom` | `nav_msgs/Odometry` | out |
| `~/wheel_slip` | `std_msgs/Int8MultiArray` | out (left wheels first, then right; 1 = flagged) |
| `/tf` (`odom_frame_id` → `base_frame_id`) | `tf2_msgs/TFMessage` | out, only if `enable_odom_tf: true` |

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `left_wheel_names`, `right_wheel_names` | required | Joint names per side. |
| `wheel_radius` | required | Wheel radius [m]. |
| `wheel_separation` | required | Distance between left and right wheel centres [m]. |
| `left_wheel_radius_multiplier`, `right_wheel_radius_multiplier` | 1.0 | Per-side radius calibration. |
| `wheel_separation_multiplier` | 1.0 | Effective track-width calibration (skid-steer scrub). |
| `use_stamped_vel` | true | Subscribe to TwistStamped vs Twist. |
| `cmd_vel_timeout` | 0.25 | Seconds without a command before the rover stops. |
| `publish_rate` | 50.0 | Odometry / flag publish rate [Hz]. |
| `odom_frame_id`, `base_frame_id` | `odom`, `base_link` | Frames in the odometry message. |
| `enable_odom_tf` | false | Broadcast odom → base TF. |
| `pose_covariance_diagonal`, `twist_covariance_diagonal` | see source | 6 values each. |
| `linear.x.{max_velocity,min_velocity,max_acceleration}` | 0 (off) | Limits [m/s, m/s²]. 0 disables. |
| `angular.z.{max_velocity,min_velocity,max_acceleration}` | 0 (off) | Limits [rad/s, rad/s²]. |
| `slip_detection.enabled` | false | Needs an `effort` state interface per wheel. |
| `slip_detection.*` | see YAML | `min_wheel_speed`, `min_torque_ref`, `torque_ratio`, `velocity_excess`, `confirm_time`, `release_time`, `command_scale`. |

## How odometry is computed

Per side, the median of the wheel angular velocities (excluding flagged wheels) is
converted to a ground speed `v_L`, `v_R` using that side's effective radius. Then

```
v = (v_R + v_L) / 2          w = (v_R - v_L) / separation
```

and the pose is integrated with the midpoint rule (heading at `θ + w·dt/2`).

## Slip / lifted-wheel detection

When enabled, each wheel is compared with the median of its side, but only while the side
is actually driving (`min_wheel_speed`, `min_torque_ref`). A wheel is a candidate if its
|torque| is below `torque_ratio` × the side median (unloaded, e.g. lifted) or it spins
faster than the side median by more than `velocity_excess`. A candidate is flagged after
`confirm_time` and released after `release_time` of being clear. Thresholds are untuned
defaults; log real torques on your robot first.

## ODrive notes

- With `ros_odrive`'s hardware interface, `effort` is the ODrive's torque estimate
  (Nm, sign-corrected by the joint `direction` parameter), not motor current.
- Per-joint `node_id` and `direction` are configured in your URDF's `ros2_control` block,
  not here.

## License

MIT, see [LICENSE](LICENSE).
