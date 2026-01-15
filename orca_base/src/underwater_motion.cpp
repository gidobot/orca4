// MIT License
//
// Copyright (c) 2022 Clyde McQueen
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <memory>
#include <string>

#include "orca_base/underwater_motion.hpp"
#include "orca_shared/util.hpp"

namespace orca_base
{

// Purpose: given cmd_vel, calculate motion.
//
// Pose is in the world frame; velocity, acceleration and thrust are in the robot frame.
//
// There are several sources of cmd_vel:
// -- orca_nav2::PurePursuitController3D, the primary path following controller.
//    PurePursuitController3D builds a constant acceleration motion (trapezoidal velocity) plan,
//    so [in theory] there's no need to clamp acceleration and velocity.
// -- recovery controllers (spin, wait) interrupt PurePursuitController3D and may cause rapid
//    acceleration or deceleration. These will be clamped.

UnderwaterMotion::UnderwaterMotion(
  const rclcpp::Time & t, const rclcpp::Logger & logger,
  const BaseContext & cxt, const geometry_msgs::msg::Pose & initial_pose)
: logger_{logger}, cxt_{cxt}
{
  motion_.header.stamp = t;
  motion_.header.frame_id = cxt_.odom_frame_id_;
  motion_.pose.position.x = initial_pose.position.x;
  motion_.pose.position.y = initial_pose.position.y;
  motion_.pose.position.z = initial_pose.position.z;
  auto initial_yaw = orca::get_yaw(initial_pose.orientation);
  orca::set_yaw(motion_.pose.orientation, initial_yaw);
  RCLCPP_INFO(
    logger_, "initialize odometry to {{%g, %g, %g}, {0, 0, %g}}",
    motion_.pose.position.x, motion_.pose.position.y, motion_.pose.position.z, initial_yaw);
}

// Loud vs quiet clamp functions
// #define CLAMP(v, minmax) report_and_clamp(__func__, #v, v, minmax)
#define CLAMP(v, minmax) orca::clamp(v, minmax)
#define EPSILON 0.00001

double
UnderwaterMotion::report_and_clamp(std::string func, std::string name, double v, double minmax)
{
  if (v > minmax + EPSILON) {
    RCLCPP_INFO(
      logger_, "%s: {%s} %g too high, clamp to %g", func.c_str(), name.c_str(), v,
      minmax);
    return minmax;
  } else if (v < -minmax - EPSILON) {
    RCLCPP_INFO(
      logger_, "%s: {%s} %g too low, clamp to %g", func.c_str(), name.c_str(), v,
      -minmax);
    return -minmax;
  } else {
    return v;
  }
}

// a = (v1 - v0) / dt
geometry_msgs::msg::Accel UnderwaterMotion::calc_accel(
  const geometry_msgs::msg::Twist & v0,
  const geometry_msgs::msg::Twist & v1) const
{
  geometry_msgs::msg::Accel result;
  result.linear.x = CLAMP((v1.linear.x - v0.linear.x) / motion_.dt, cxt_.x_accel_);
  result.linear.y = CLAMP((v1.linear.y - v0.linear.y) / motion_.dt, cxt_.y_accel_);
  result.linear.z = CLAMP((v1.linear.z - v0.linear.z) / motion_.dt, cxt_.z_accel_);
  result.angular.z = CLAMP((v1.angular.z - v0.angular.z) / motion_.dt, cxt_.yaw_accel_);
  return result;
}

// v = v0 + a * dt
geometry_msgs::msg::Twist UnderwaterMotion::calc_vel(
  const geometry_msgs::msg::Twist & v0,
  const geometry_msgs::msg::Accel & a) const
{
  geometry_msgs::msg::Twist result;
  result.linear.x = CLAMP(v0.linear.x + a.linear.x * motion_.dt, cxt_.x_vel_);
  result.linear.y = CLAMP(v0.linear.y + a.linear.y * motion_.dt, cxt_.y_vel_);
  result.linear.z = CLAMP(v0.linear.z + a.linear.z * motion_.dt, cxt_.z_vel_);
  result.angular.z = CLAMP(v0.angular.z + a.angular.z * motion_.dt, cxt_.yaw_vel_);
  return result;
}

// p = p0 + v * dt
geometry_msgs::msg::Pose UnderwaterMotion::calc_pose(
  const geometry_msgs::msg::Pose & p0,
  const geometry_msgs::msg::Twist & v) const
{
  geometry_msgs::msg::Pose result;
  auto yaw = orca::get_yaw(p0.orientation);
  result.position.x = p0.position.x + (v.linear.x * cos(yaw) + v.linear.y * sin(-yaw)) * motion_.dt;
  result.position.y = p0.position.y + (v.linear.x * sin(yaw) + v.linear.y * cos(-yaw)) * motion_.dt;
  result.position.z = p0.position.z + v.linear.z * motion_.dt;
  yaw += v.angular.z * motion_.dt;
  orca::set_yaw(result.orientation, yaw);

  if (result.position.z > 0) {
    // Don't go above the surface
    result.position.z = 0;
  }

  return result;
}

nav_msgs::msg::Odometry UnderwaterMotion::odometry() const
{
  static std::array<double, 36> covariance{
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
  };

  nav_msgs::msg::Odometry result;
  result.header.stamp = motion_.header.stamp;
  result.header.frame_id = cxt_.odom_frame_id_;
  result.child_frame_id = cxt_.base_frame_id_;
  result.pose.pose = motion_.pose;
  result.pose.covariance = covariance;
  result.twist.twist =
    orca::robot_to_world_frame(motion_.vel, orca::get_yaw(motion_.pose.orientation));
  result.twist.covariance = covariance;
  return result;
}

// Coast: if cmd_vel is 0, then force acceleration to 0. Only applies to x, y and yaw.
// Helpful for some ROV operations. TODO(clyde)
void coast(const geometry_msgs::msg::Twist & cmd_vel, geometry_msgs::msg::Accel & model_plus_drag)
{
  static double epsilon = 0.01;
  if (std::abs(cmd_vel.linear.x) < epsilon) {
    model_plus_drag.linear.x = 0;
  }
  if (std::abs(cmd_vel.linear.y) < epsilon) {
    model_plus_drag.linear.y = 0;
  }
  if (std::abs(cmd_vel.angular.z) < epsilon) {
    model_plus_drag.angular.z = 0;
  }
}

void UnderwaterMotion::update(const rclcpp::Time & t, const geometry_msgs::msg::Twist & cmd_vel)
{
  motion_.header.stamp = t;
  motion_.dt = 1.0 / cxt_.timer_rate_;
  motion_.cmd_vel = cmd_vel;
  
  // Reset DVL fresh flag at the beginning of each update cycle
  dvl_updated_this_step_ = false;

  // Pose and vel don't honor coast TODO(clyde)
  motion_.pose = calc_pose(motion_.pose, motion_.vel);
  
  // Calculate velocity using physics model
  auto physics_vel = calc_vel(motion_.vel, motion_.accel_model);
  
  // Temporal DVL fusion: use DVL when fresh, physics model between updates
  if (is_dvl_valid(t) && is_dvl_fresh(t)) {
    // Fresh DVL data available - use it directly
    auto dvl_vel = get_dvl_velocity_in_robot_frame();
    motion_.vel = fuse_velocities(physics_vel, dvl_vel);
    RCLCPP_INFO(logger_, "dvl=[%.3f, %.3f, %.3f]", 
                dvl_vel.linear.x, dvl_vel.linear.y, dvl_vel.linear.z);
    RCLCPP_INFO(logger_, "phy=[%.3f, %.3f, %.3f]", 
                physics_vel.linear.x, physics_vel.linear.y, physics_vel.linear.z);
    RCLCPP_INFO(logger_, "using_dvl");
    dvl_updated_this_step_ = true;
    RCLCPP_DEBUG(logger_, "DVL FRESH: physics=[%.3f, %.3f, %.3f], dvl=[%.3f, %.3f, %.3f], using_dvl", 
                physics_vel.linear.x, physics_vel.linear.y, physics_vel.linear.z,
                dvl_vel.linear.x, dvl_vel.linear.y, dvl_vel.linear.z);
  } else if (is_dvl_valid(t) && !is_dvl_fresh(t)) {
    // DVL data is valid but not fresh - use physics model for interpolation
    motion_.vel = physics_vel;
    dvl_updated_this_step_ = false;
    RCLCPP_DEBUG(logger_, "DVL STALE: physics=[%.3f, %.3f, %.3f], dvl=[%.3f, %.3f, %.3f], using_physics", 
                physics_vel.linear.x, physics_vel.linear.y, physics_vel.linear.z,
                dvl_velocity_.velocity.x, dvl_velocity_.velocity.y, dvl_velocity_.velocity.z);
  } else {
    // No valid DVL data - use physics model
    motion_.vel = physics_vel;
    dvl_updated_this_step_ = false;
    if (dvl_available_) {
      RCLCPP_DEBUG(logger_, "DVL INVALID: using physics velocity: [%.3f, %.3f, %.3f]", 
                  motion_.vel.linear.x, motion_.vel.linear.y, motion_.vel.linear.z);
    }
  }

  // Calculate acceleration from physics velocity to avoid feedback loops
  motion_.accel_model = calc_accel(motion_.vel, cmd_vel);
  
  // Counteract drag
  motion_.accel_drag = -cxt_.drag_accel(motion_.vel);

  // Combine model and drag
  auto accel_total = motion_.accel_model + motion_.accel_drag;

  // Experiment for ROV operations
  if (cxt_.coast_) {
    coast(cmd_vel, accel_total);
  }

  motion_.accel_total = accel_total;
  motion_.force = cxt_.accel_to_wrench(accel_total);
  motion_.effort = cxt_.wrench_to_effort(motion_.force);
}

// DVL integration methods

bool UnderwaterMotion::is_dvl_valid(const rclcpp::Time & current_time) const
{
  if (!dvl_available_) {
    return false;
  }

  // Check if DVL data is recent enough
  auto dvl_age = current_time - dvl_velocity_time_;
  if (dvl_age.seconds() > cxt_.dvl_timeout_ms_ / 1000.0) {
    RCLCPP_DEBUG(logger_, "DVL data too old: %.3f seconds", dvl_age.seconds());
    return false;
  }

  // Check DVL validity flags
  if (!dvl_velocity_.velocity_valid) {
    RCLCPP_DEBUG(logger_, "DVL velocity not valid");
    return false;
  }

  // Check figure of merit
  if (dvl_velocity_.figure_of_merit < cxt_.dvl_min_figure_of_merit_) {
    RCLCPP_DEBUG(logger_, "DVL figure of merit too low: %.3f < %.3f", 
                 dvl_velocity_.figure_of_merit, cxt_.dvl_min_figure_of_merit_);
    return false;
  }

  // Check for reasonable velocity values (sanity check)
  double vel_magnitude = sqrt(dvl_velocity_.velocity.x * dvl_velocity_.velocity.x +
                             dvl_velocity_.velocity.y * dvl_velocity_.velocity.y +
                             dvl_velocity_.velocity.z * dvl_velocity_.velocity.z);
  if (vel_magnitude > 5.0) {  // 5 m/s is very fast for an AUV
    RCLCPP_WARN(logger_, "DVL velocity magnitude too high: %.3f m/s", vel_magnitude);
    return false;
  }

  // Check altitude (DVL should have reasonable altitude)
  if (dvl_velocity_.altitude < 0.1 || dvl_velocity_.altitude > 100.0) {
    RCLCPP_DEBUG(logger_, "DVL altitude out of range: %.3f m", dvl_velocity_.altitude);
    return false;
  }

  return true;
}

bool UnderwaterMotion::is_dvl_fresh(const rclcpp::Time & current_time) const
{
  if (!dvl_available_) {
    return false;
  }

  // Check if DVL data is very recent (within one motion model update cycle)
  auto dvl_age = current_time - dvl_velocity_time_;
  double fresh_threshold = 1.0 / cxt_.timer_rate_;  // One update cycle
  
  bool is_fresh = dvl_age.seconds() <= fresh_threshold;
  
  RCLCPP_DEBUG(logger_, "DVL freshness check: age=%.3f, threshold=%.3f, fresh=%s",
               dvl_age.seconds(), fresh_threshold, is_fresh ? "true" : "false");
  
  return is_fresh;
}

geometry_msgs::msg::Twist UnderwaterMotion::get_dvl_velocity_in_robot_frame() const
{
  geometry_msgs::msg::Twist result;
  
  // DVL velocities are already in vehicle frame, so direct assignment
  result.linear.x = dvl_velocity_.velocity.x;
  result.linear.y = dvl_velocity_.velocity.y;
  result.linear.z = dvl_velocity_.velocity.z;
  result.angular.x = 0.0;
  result.angular.y = 0.0;
  result.angular.z = 0.0;  // DVL doesn't measure angular velocity

  RCLCPP_DEBUG(logger_, "DVL velocity (vehicle frame): [%.3f, %.3f, %.3f]",
               result.linear.x, result.linear.y, result.linear.z);

  return result;
}

geometry_msgs::msg::Twist UnderwaterMotion::fuse_velocities(
  const geometry_msgs::msg::Twist & physics_vel,
  const geometry_msgs::msg::Twist & dvl_vel) const
{
  geometry_msgs::msg::Twist result;
  double weight = cxt_.dvl_velocity_fusion_weight_;

  // Fuse linear velocities
  result.linear.x = weight * dvl_vel.linear.x + (1.0 - weight) * physics_vel.linear.x;
  result.linear.y = weight * dvl_vel.linear.y + (1.0 - weight) * physics_vel.linear.y;
  result.linear.z = weight * dvl_vel.linear.z + (1.0 - weight) * physics_vel.linear.z;

  // Angular velocity only from physics model (DVL doesn't measure it)
  result.angular.x = physics_vel.angular.x;
  result.angular.y = physics_vel.angular.y;
  result.angular.z = physics_vel.angular.z;

  return result;
}

void UnderwaterMotion::update_dvl_velocity(const ros_gz_dvl_bridge::msg::DVLVelocity & dvl_msg)
{
  dvl_velocity_ = dvl_msg;
  dvl_velocity_time_ = rclcpp::Time(dvl_msg.header.stamp);
  dvl_available_ = true;
  dvl_updated_this_step_ = true;  // Mark that we have fresh DVL data

  RCLCPP_INFO(
    logger_, "DVL velocity updated: [%.3f, %.3f, %.3f], valid=%s, fom=%.3f, alt=%.3f, mode=%d",
    dvl_msg.velocity.x, dvl_msg.velocity.y, dvl_msg.velocity.z,
    dvl_msg.velocity_valid ? "true" : "false",
    dvl_msg.figure_of_merit, dvl_msg.altitude, dvl_msg.tracking_mode);
}

}  // namespace orca_base
