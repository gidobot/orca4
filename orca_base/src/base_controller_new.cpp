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

#include <algorithm>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "marine_acoustic_msgs/msg/dvl.hpp"
#include "orca_base/base_context.hpp"
#include "orca_shared/util.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace orca_base
{

// Purpose: Simple base controller that forwards cmd_vel directly to ArduSub
// for 6DOF velocity control without any smoothing or physics modeling.
//
// This controller assumes that cmd_vel values have been appropriately
// smoothed for limited acceleration and respected velocity limits upstream.

class BaseControllerNew : public rclcpp::Node
{
  // Parameters
  BaseContext cxt_;
  rclcpp::Duration transform_expiration_{std::chrono::milliseconds{0}};

  // We are in control
  bool conn_{false};

  // Most recent messages
  geometry_msgs::msg::PoseStamped ardu_pose_;   // Pose from ArduSub EKF
  geometry_msgs::msg::Twist cmd_vel_;            // Twist from Nav2
  marine_acoustic_msgs::msg::Dvl dvl_velocity_;  // DVL velocity measurements
  nav_msgs::msg::Odometry gazebo_odom_;         // Ground truth from Gazebo (when vision_pose_from_gazebo_odom)
  bool have_gazebo_odom_{false};

  // Dynamic transforms
  tf2::Transform tf_map_odom_;
  tf2::Transform tf_odom_base_;

  // Dead-reckoned position (integrated DVL/cmd_vel from 0,0,0, no absolute reference)
  tf2::Vector3 dr_position_{0.0, 0.0, 0.0};
  rclcpp::Time last_dr_time_;

  // EKF warm-up state
  enum class EKFState
  {
    WAITING_FOR_EKF,    // Waiting for EKF to start publishing
    EKF_RUNNING         // EKF is running and publishing
  };
  EKFState ekf_state_{EKFState::WAITING_FOR_EKF};
  rclcpp::Time last_ekf_pose_time_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Service provided by this node
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr conn_srv_;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ardu_pose_sub_;
  rclcpp::Subscription<marine_acoustic_msgs::msg::Dvl>::SharedPtr dvl_velocity_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gazebo_odom_sub_;

  // Publications
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr velocity_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mavros_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_pub_;

  // TF2
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


  geometry_msgs::msg::Pose compute_dead_reckoned_pose()
  {
    auto now_time = now();
    double dt = 0.0;
    if (last_dr_time_.nanoseconds() > 0) {
      dt = (now_time - last_dr_time_).seconds();
    }
    last_dr_time_ = now_time;

    // Get velocity: DVL when valid, else cmd_vel
    geometry_msgs::msg::Twist vel_body;
    auto dvl_age = now_time - rclcpp::Time(dvl_velocity_.header.stamp);
    double figure_of_merit = dvl_velocity_.num_good_beams / 4.0;
    bool dvl_ok = dvl_velocity_.num_good_beams > 0 && dvl_age.seconds() < 1.0 &&
                  dvl_velocity_.altitude > 0.5 && figure_of_merit >= 0.6;
    if (dvl_ok) {
      vel_body.linear.x = dvl_velocity_.velocity.x;
      vel_body.linear.y = dvl_velocity_.velocity.y;
      vel_body.linear.z = dvl_velocity_.velocity.z;
    } else {
      vel_body.linear = cmd_vel_.linear;
      vel_body.angular = cmd_vel_.angular;
    }

    // Get yaw from EKF (compass) when available
    double yaw = 0.0;
    if (ardu_pose_.header.stamp.sec != 0 || ardu_pose_.header.stamp.nanosec != 0) {
      yaw = orca::get_yaw(ardu_pose_.pose.orientation);
    }

    // Transform velocity to world frame and integrate
    auto vel_world = orca::robot_to_world_frame(vel_body, yaw);
    dr_position_.setX(dr_position_.x() + vel_world.linear.x * dt);
    dr_position_.setY(dr_position_.y() + vel_world.linear.y * dt);
    dr_position_.setZ(dr_position_.z() + vel_world.linear.z * dt);

    geometry_msgs::msg::Pose pose;
    pose.position.x = dr_position_.x();
    pose.position.y = dr_position_.y();
    pose.position.z = dr_position_.z();
    if (ardu_pose_.header.stamp.sec != 0 || ardu_pose_.header.stamp.nanosec != 0) {
      pose.orientation = ardu_pose_.pose.orientation;
    } else {
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;
    }
    return pose;
  }

  void publish_vision_pose(const geometry_msgs::msg::Pose & pose)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = cxt_.map_frame_id_;
    msg.pose = pose;
    vision_pose_pub_->publish(msg);
  }

  void publish_velocity(const geometry_msgs::msg::Twist & cmd_vel)
  {
    if (conn_) {
      // Publish velocity command to ArduSub using body frame
      mavros_msgs::msg::PositionTarget msg;
      msg.header.stamp = now();
      msg.header.frame_id = "fcu";
      msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_OFFSET_NED;  // Body frame
      msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX | 
                     mavros_msgs::msg::PositionTarget::IGNORE_PY | 
                     mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                     mavros_msgs::msg::PositionTarget::IGNORE_AFX | 
                     mavros_msgs::msg::PositionTarget::IGNORE_AFY | 
                     mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                     mavros_msgs::msg::PositionTarget::IGNORE_YAW;
      
      // Set velocity in body frame (forward, right, down)
      msg.velocity.x = cmd_vel.linear.x;   // Forward
      msg.velocity.y = cmd_vel.linear.y;   // Right  
      msg.velocity.z = cmd_vel.linear.z;   // Down
      msg.yaw_rate = cmd_vel.angular.z;    // Yaw rate
      
      velocity_pub_->publish(msg);
      
      RCLCPP_DEBUG(get_logger(), "Publishing velocity (body frame): [%.3f, %.3f, %.3f, %.3f]",
                   cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z, cmd_vel.angular.z);
    }
  }

  void publish_dvl_odometry()
  {
    // Send DVL velocity to EKF for fusion (EK3_SRC1_VELXY=6). With EK3_SRC1_POSXY=0,
    // EKF ignores position and dead reckons from velocity. Send as soon as DVL is valid.
    // Check if DVL data is fresh and valid
    auto now_time = now();
    auto dvl_age = now_time - rclcpp::Time(dvl_velocity_.header.stamp);
    
    // Use DVL when quality is good; fall back to cmd_vel when DVL unavailable (e.g. sim).
    bool dvl_fresh = dvl_age.seconds() < 1.0;
    double figure_of_merit = dvl_velocity_.num_good_beams / 4.0;
    bool dvl_quality_ok = dvl_velocity_.num_good_beams > 0 &&
                         dvl_velocity_.altitude > 0.1 &&   // Relaxed for sim (was 0.5)
                         figure_of_merit >= 0.3;  // Relaxed for sim (was 0.6)
    bool use_dvl = dvl_fresh && dvl_quality_ok;

    geometry_msgs::msg::Twist vel_body;
    double vel_cov = 0.01;
    if (use_dvl) {
      vel_body.linear.x = dvl_velocity_.velocity.x;
      vel_body.linear.y = dvl_velocity_.velocity.y;
      vel_body.linear.z = dvl_velocity_.velocity.z;
      vel_body.angular.x = vel_body.angular.y = vel_body.angular.z = 0.0;
      vel_cov = std::max({dvl_velocity_.velocity_covar[0],
                         dvl_velocity_.velocity_covar[4],
                         dvl_velocity_.velocity_covar[8], 0.01});
    } else {
      vel_body.linear = cmd_vel_.linear;
      vel_body.angular = cmd_vel_.angular;
      vel_cov = 0.5;  // Higher uncertainty for commanded vs measured velocity
    }

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose = orca::transform_to_pose_msg(tf_odom_base_);
    odom_msg.pose.covariance[0] = odom_msg.pose.covariance[7] = odom_msg.pose.covariance[14] = 100.0;
    odom_msg.twist.twist = vel_body;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    if (use_dvl) {
      // Map 3x3 velocity_covar to 6x6 twist covariance (linear block)
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          odom_msg.twist.covariance[i * 6 + j] = dvl_velocity_.velocity_covar[i * 3 + j];
        }
      }
    } else {
      odom_msg.twist.covariance[0] = odom_msg.twist.covariance[7] =
        odom_msg.twist.covariance[14] = vel_cov;
    }
    mavros_odom_pub_->publish(odom_msg);
  }

  void publish_tf(std::string parent, std::string child, const tf2::Transform & tf)
  {
    geometry_msgs::msg::TransformStamped tm;
    tm.header.frame_id = std::move(parent);
    tm.child_frame_id = std::move(child);
    tm.transform = orca::transform_to_transform_msg(tf);
    // Adding time to the transform avoids problems and improves rviz2 display
    tm.header.stamp = now() + transform_expiration_;
    tf_broadcaster_->sendTransform(tm);
  }

  void publish_odometry()
  {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now();
    odom_msg.header.frame_id = cxt_.odom_frame_id_;
    odom_msg.child_frame_id = cxt_.base_frame_id_;
    odom_msg.pose.pose = orca::transform_to_pose_msg(tf_odom_base_);
    odom_msg.twist.twist = cmd_vel_;  // Use current cmd_vel as twist
    odom_pub_->publish(odom_msg);
  }

  void timer_cb()
  {
    // Check EKF state for warm-up completion (manager needs this for have_pose_)
    if (ekf_state_ == EKFState::WAITING_FOR_EKF) {
      if (ardu_pose_.header.stamp.sec != 0 || ardu_pose_.header.stamp.nanosec != 0) {
        auto time_since_last_pose = now() - rclcpp::Time(ardu_pose_.header.stamp);
        if (time_since_last_pose.seconds() < 1.0) {
          ekf_state_ = EKFState::EKF_RUNNING;
          RCLCPP_INFO(get_logger(), "EKF is now running%s",
            cxt_.vision_pose_for_ekf_ ? " - continuing vision_pose for position aiding" : " (GPS/internal position)");
        }
      }
    }

    // Send vision_pose only when EKF expects it (EK3_SRC1_POSXY=6). When vision_pose_for_ekf=false,
    // EKF uses GPS/internal (EK3_SRC1_POSXY=3) and we should not send vision_pose.
    if (cxt_.vision_pose_for_ekf_) {
      geometry_msgs::msg::Pose pose;
      if (cxt_.vision_pose_from_gazebo_odom_ && have_gazebo_odom_) {
        pose = gazebo_odom_.pose.pose;  // Ground truth from Gazebo (no GPS)
      } else {
        // Dead reckoning: integrate DVL/cmd_vel from (0,0,0), no absolute position reference
        pose = compute_dead_reckoned_pose();
      }
      publish_vision_pose(pose);
    }

    // Update transforms based on ArduSub pose
    if (ardu_pose_.header.stamp.sec != 0 || ardu_pose_.header.stamp.nanosec != 0) {
      // ArduSub pose is in odom frame, so we need to create odom->base transform
      // The pose from ArduSub represents the vehicle position in the odom frame
      tf_odom_base_ = orca::pose_msg_to_transform(ardu_pose_.pose);
    }

    // For simple navigation without SLAM, map->odom should be identity
    // This means the vehicle's odom frame is aligned with the global map frame
    tf_map_odom_.setIdentity();

    // Publish essential transforms for navigation (same as base_controller.cpp)
    publish_tf(cxt_.map_frame_id_, cxt_.odom_frame_id_, tf_map_odom_);
    publish_tf(cxt_.odom_frame_id_, cxt_.base_frame_id_, tf_odom_base_);

    // Publish odometry
    publish_odometry();

    // Publish DVL odometry to EKF (bypasses motion model)
    publish_dvl_odometry();
  }

  void ardu_pose_cb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
  {
    ardu_pose_ = *msg;
  }

  void dvl_velocity_cb(const marine_acoustic_msgs::msg::Dvl::ConstSharedPtr & msg)
  {
    dvl_velocity_ = *msg;
    double figure_of_merit = msg->num_good_beams / 4.0;
    RCLCPP_DEBUG(get_logger(), "Received DVL velocity: [%.3f, %.3f, %.3f] beams=%d mode=%d alt=%.3f fom=%.6f",
                 msg->velocity.x, msg->velocity.y, msg->velocity.z,
                 msg->num_good_beams, msg->velocity_mode, msg->altitude, figure_of_merit);
  }

  void init_parameters()
  {
    // Get parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(BASE_ALL_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), cxt_, BASE_ALL_PARAMS, validate_parameters)

    // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    BASE_ALL_PARAMS

    // Check that all command line parameters are defined
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), BASE_ALL_PARAMS)
  }

  void validate_parameters()
  {
    transform_expiration_ = {std::chrono::milliseconds{cxt_.transform_expiration_ms_}};

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / cxt_.timer_rate_), [this]
      {
        timer_cb();
      });
  }

public:
  BaseControllerNew()
  : Node("base_controller_new")
  {
    // Suppress IDE warnings
    (void) ardu_pose_sub_;
    (void) gazebo_odom_sub_;
    (void) timer_;

    init_parameters();

    // Initial transforms
    tf_map_odom_.setIdentity();
    tf_odom_base_.setIdentity();
    // Set initial pose at surface (z=0)
    tf_odom_base_.getOrigin().setZ(0.0);

    if (cxt_.vision_pose_for_ekf_) {
      RCLCPP_INFO(get_logger(), "Sending VISION_POSITION_ESTIMATE via vision_pose for EKF position aiding%s",
        cxt_.vision_pose_from_gazebo_odom_ ? " (from Gazebo ground truth)"
        : " (dead reckoning from DVL/cmd_vel, no absolute position ref)");
    } else {
      RCLCPP_INFO(get_logger(), "vision_pose disabled - EKF velocity-only (EK3_SRC1_POSXY=0). Run set_ekf_origin.py for warmup.");
    }

    // Initialize TF2
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // QoS settings
    rclcpp::QoS reliable(10);
    reliable.reliable();

    rclcpp::QoS best_effort(10);
    best_effort.best_effort();

    // Create publishers
    velocity_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
      "/mavros/setpoint_raw/local", reliable);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", reliable);
    mavros_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/mavros/odometry/out", reliable);
    vision_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/vision_pose/pose", reliable);

    // Create service
    conn_srv_ = create_service<std_srvs::srv::SetBool>(
      "conn",
      [this](
        const std::shared_ptr<rmw_request_id_t>,
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
      {
        if (conn_ != request->data) {
          if (request->data) {
            RCLCPP_INFO(get_logger(), "Starting velocity control");
          } else {
            RCLCPP_INFO(get_logger(), "Stopping velocity control");
            // Send zero velocity when stopping
            geometry_msgs::msg::Twist zero_vel;
            publish_velocity(zero_vel);
          }
          conn_ = request->data;
        }
        response->success = true;
      },
      rclcpp::QoS(10).reliable());

    // Create subscriptions
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", reliable,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) -> void
      {
        RCLCPP_DEBUG(get_logger(), "Received cmd_vel: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                     msg->linear.x, msg->linear.y, msg->linear.z,
                     msg->angular.x, msg->angular.y, msg->angular.z);
        
        cmd_vel_ = *msg;
        // Forward immediately to ArduSub
        publish_velocity(*msg);
      });

    ardu_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", best_effort,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) -> void
      {
        ardu_pose_cb(msg);
      });

    dvl_velocity_sub_ = create_subscription<marine_acoustic_msgs::msg::Dvl>(
      "/dvl/velocity", best_effort,
      [this](marine_acoustic_msgs::msg::Dvl::ConstSharedPtr msg) -> void
      {
        dvl_velocity_cb(msg);
      });

    gazebo_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/model/orca4/odometry", best_effort,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) -> void
      {
        gazebo_odom_ = *msg;
        have_gazebo_odom_ = true;
      });

    RCLCPP_INFO(get_logger(), "base_controller_new ready");
    RCLCPP_INFO(get_logger(), "Publishing to:");
    RCLCPP_INFO(get_logger(), "  - /mavros/setpoint_raw/local (velocity commands to ArduSub)");
    RCLCPP_INFO(get_logger(), "  - odom (odometry for Nav2 navigation)");
    RCLCPP_INFO(get_logger(), "  - /mavros/odometry/in (DVL velocity measurements to ArduSub EKF)");
    RCLCPP_INFO(get_logger(), "Subscribing to: cmd_vel, /mavros/local_position/pose, /dvl/velocity");
    RCLCPP_INFO(get_logger(), "Service: conn");
    RCLCPP_INFO(get_logger(), "Publishing transforms: map->odom, odom->base_link");
    RCLCPP_INFO(get_logger(), "DVL integration: External velocity input via odometry (bypasses motion model)");
  }
};

}  // namespace orca_base

//=============================================================================
// Main
//=============================================================================

int main(int argc, char ** argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orca_base::BaseControllerNew>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}