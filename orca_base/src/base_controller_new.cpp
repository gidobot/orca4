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

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "ros_gz_dvl_bridge/msg/dvl_velocity.hpp"
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
  geometry_msgs::msg::Twist cmd_vel_;           // Twist from Nav2
  ros_gz_dvl_bridge::msg::DVLVelocity dvl_velocity_;  // DVL velocity measurements

  // Dynamic transforms
  tf2::Transform tf_map_odom_;
  tf2::Transform tf_odom_base_;

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
  rclcpp::Subscription<ros_gz_dvl_bridge::msg::DVLVelocity>::SharedPtr dvl_velocity_sub_;

  // Publications
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr velocity_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mavros_odom_pub_;

  // TF2
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


  void publish_warmup_odometry(const tf2::Transform & pose)
  {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now();
    odom_msg.header.frame_id = cxt_.odom_frame_id_;
    odom_msg.child_frame_id = cxt_.base_frame_id_;
    
    // Set pose (position and orientation)
    odom_msg.pose.pose = orca::transform_to_pose_msg(pose);
    
    // Set velocity to zero (vehicle is stationary during warm-up)
    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;
    
    // Set covariances for warm-up
    // Position covariance: moderate uncertainty (1m standard deviation)
    odom_msg.pose.covariance[0] = 1.0;   // x position variance
    odom_msg.pose.covariance[7] = 1.0;   // y position variance  
    odom_msg.pose.covariance[14] = 0.1;  // z position variance (more accurate)
    odom_msg.pose.covariance[21] = 0.1;  // roll variance
    odom_msg.pose.covariance[28] = 0.1;  // pitch variance
    odom_msg.pose.covariance[35] = 0.5;  // yaw variance
    
    // Velocity covariance: low uncertainty (vehicle stationary)
    odom_msg.twist.covariance[0] = 0.01;  // x velocity variance
    odom_msg.twist.covariance[7] = 0.01;  // y velocity variance
    odom_msg.twist.covariance[14] = 0.01; // z velocity variance
    odom_msg.twist.covariance[21] = 0.01; // roll rate variance
    odom_msg.twist.covariance[28] = 0.01; // pitch rate variance
    odom_msg.twist.covariance[35] = 0.01; // yaw rate variance
    
    mavros_odom_pub_->publish(odom_msg);
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
    // Only publish DVL data after EKF has initialized
    if (ekf_state_ != EKFState::EKF_RUNNING) {
      RCLCPP_DEBUG(get_logger(), "DVL odometry: Skipping - EKF not yet initialized");
      return;
    }
    
    // Check if DVL data is fresh and valid
    auto now_time = now();
    auto dvl_age = now_time - rclcpp::Time(dvl_velocity_.header.stamp);
    
    // Only publish if DVL data is fresh (less than 1 second old) and valid
    // Additional quality checks: valid velocity, reasonable altitude, good figure of merit
    bool quality_ok = dvl_velocity_.velocity_valid && 
                      dvl_velocity_.altitude > 0.5 &&  // At least 0.5m altitude
                      dvl_velocity_.figure_of_merit >= 0.6;  // Good quality (high FOM is better)
    
    if (dvl_age.seconds() < 1.0 && quality_ok) {
      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = now();
      odom_msg.header.frame_id = "odom";  // Local frame for EKF
      odom_msg.child_frame_id = "base_link";  // Body frame
      
      // Set position to current EKF position (we only have velocity from DVL)
      odom_msg.pose.pose = orca::transform_to_pose_msg(tf_odom_base_);
      
      // Twist should be expressed in the child frame (body frame)
      // DVL velocities are already in vehicle frame (forward, right, down)
      // No transformation needed - use DVL velocities directly in body frame
      odom_msg.twist.twist.linear.x = dvl_velocity_.velocity.x;   // Forward
      odom_msg.twist.twist.linear.y = dvl_velocity_.velocity.y;   // Right  
      odom_msg.twist.twist.linear.z = dvl_velocity_.velocity.z;   // Down
      odom_msg.twist.twist.angular.x = 0.0;  // DVL doesn't provide angular velocity
      odom_msg.twist.twist.angular.y = 0.0;
      odom_msg.twist.twist.angular.z = 0.0;
      
      // Set covariance matrices (DVL velocity uncertainty)
      // Position covariance (high uncertainty since we're not using DVL for position)
      odom_msg.pose.covariance[0] = 100.0;   // x
      odom_msg.pose.covariance[7] = 100.0;    // y  
      odom_msg.pose.covariance[14] = 100.0;  // z
      
      // Velocity covariance - use DVL-provided covariance if available, otherwise use default
      if (dvl_velocity_.velocity_covariance.size() >= 9) {
        // Use DVL-provided velocity covariance matrix
        for (int i = 0; i < 9; i++) {
          odom_msg.twist.covariance[i] = dvl_velocity_.velocity_covariance[i];
        }
      } else {
        // Use constant covariance for DVL velocity (DVL is generally accurate)
        double base_covariance = 0.01;  // Constant covariance for DVL velocity
        odom_msg.twist.covariance[0] = base_covariance;   // x velocity
        odom_msg.twist.covariance[7] = base_covariance;    // y velocity
        odom_msg.twist.covariance[14] = base_covariance;  // z velocity
      }
      
      mavros_odom_pub_->publish(odom_msg);
    } else {
      // Log why DVL data was rejected
      if (dvl_age.seconds() >= 1.0) {
        RCLCPP_DEBUG(get_logger(), "DVL data rejected: too old (%.3f seconds)", dvl_age.seconds());
      } else if (!dvl_velocity_.velocity_valid) {
        RCLCPP_DEBUG(get_logger(), "DVL data rejected: velocity not valid");
      } else if (dvl_velocity_.altitude <= 0.5) {
        RCLCPP_DEBUG(get_logger(), "DVL data rejected: altitude too low (%.3f m)", dvl_velocity_.altitude);
      } else if (dvl_velocity_.figure_of_merit < 0.6) {
        RCLCPP_DEBUG(get_logger(), "DVL data rejected: poor quality (FOM=%.6f, need >=0.6)", dvl_velocity_.figure_of_merit);
      }
    }
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
    // Check EKF state and warm-up if needed
    if (ekf_state_ == EKFState::WAITING_FOR_EKF) {
      // Check if EKF has started publishing
      if (ardu_pose_.header.stamp.sec != 0 || ardu_pose_.header.stamp.nanosec != 0) {
        auto time_since_last_pose = now() - rclcpp::Time(ardu_pose_.header.stamp);
        if (time_since_last_pose.seconds() < 1.0) {  // EKF is actively publishing
          ekf_state_ = EKFState::EKF_RUNNING;
          RCLCPP_INFO(get_logger(), "EKF is now running - stopping warm-up poses");
        }
      }
      
      // Send default odometry to warm up EKF
      if (ekf_state_ == EKFState::WAITING_FOR_EKF) {
        RCLCPP_DEBUG(get_logger(), "Sending default odometry to warm up EKF");
        publish_warmup_odometry(tf_odom_base_);
      }
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

  void dvl_velocity_cb(const ros_gz_dvl_bridge::msg::DVLVelocity::ConstSharedPtr & msg)
  {
    dvl_velocity_ = *msg;
    RCLCPP_DEBUG(get_logger(), "Received DVL velocity: [%.3f, %.3f, %.3f] valid=%d mode=%d alt=%.3f fom=%.6f",
                 msg->velocity.x, msg->velocity.y, msg->velocity.z, 
                 msg->velocity_valid, msg->tracking_mode, msg->altitude, msg->figure_of_merit);
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
    (void) timer_;

    init_parameters();

    // Initial transforms
    tf_map_odom_.setIdentity();
    tf_odom_base_.setIdentity();
    // Set initial pose at surface (z=0)
    tf_odom_base_.getOrigin().setZ(0.0);

    // Initialize EKF warm-up
    RCLCPP_INFO(get_logger(), "EKF warm-up: Sending default odometry until EKF initializes");

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

    dvl_velocity_sub_ = create_subscription<ros_gz_dvl_bridge::msg::DVLVelocity>(
      "/dvl/velocity", best_effort,
      [this](ros_gz_dvl_bridge::msg::DVLVelocity::ConstSharedPtr msg) -> void
      {
        dvl_velocity_cb(msg);
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