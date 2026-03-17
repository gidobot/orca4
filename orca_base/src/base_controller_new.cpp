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
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
// THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "orca_base/base_context.hpp"
#include "orca_shared/util.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace orca_base
{

// Purpose: Base controller that forwards cmd_vel to ArduSub and uses external
// odometry (e.g. uwlocalization) for pose. Publishes TF, odom for Nav2, and
// vision_pose for ArduPilot EKF (position only; baro used for vertical velocity).

class BaseControllerNew : public rclcpp::Node
{
  BaseContext cxt_;
  rclcpp::Duration transform_expiration_{std::chrono::milliseconds{0}};

  bool conn_{false};
  geometry_msgs::msg::Twist cmd_vel_;

  tf2::Transform tf_map_odom_;
  tf2::Transform tf_odom_base_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr conn_srv_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr external_odom_sub_;

  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr velocity_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr vision_pose_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void publish_vision_pose(const geometry_msgs::msg::Pose & pose, rclcpp::Time stamp = {})
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = stamp.nanoseconds() ? stamp : now();
    msg.header.frame_id = cxt_.map_frame_id_;
    msg.pose.pose = pose;
    for (int i = 0; i < 36; ++i) msg.pose.covariance[i] = 0.0;
    msg.pose.covariance[0] = msg.pose.covariance[7] = 0.01;   // x, y
    msg.pose.covariance[14] = 0.0001;  // z (tight - trust uwlocalization depth)
    msg.pose.covariance[21] = msg.pose.covariance[28] = msg.pose.covariance[35] = 0.01;  // rpy
    vision_pose_pub_->publish(msg);
  }

  void publish_velocity(const geometry_msgs::msg::Twist & vel)
  {
    if (conn_) {
      mavros_msgs::msg::PositionTarget msg;
      msg.header.stamp = now();
      msg.header.frame_id = "fcu";
      msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_OFFSET_NED;
      msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
                     mavros_msgs::msg::PositionTarget::IGNORE_PY |
                     mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                     mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                     mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                     mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                     mavros_msgs::msg::PositionTarget::IGNORE_YAW;
      msg.velocity.x = vel.linear.x;
      msg.velocity.y = vel.linear.y;
      msg.velocity.z = vel.linear.z;
      msg.yaw_rate = vel.angular.z;
      velocity_pub_->publish(msg);
    }
  }

  void publish_tf(const std::string & parent, const std::string & child, const tf2::Transform & tf)
  {
    geometry_msgs::msg::TransformStamped tm;
    tm.header.frame_id = parent;
    tm.child_frame_id = child;
    tm.transform = orca::transform_to_transform_msg(tf);
    tm.header.stamp = now() + transform_expiration_;
    tf_broadcaster_->sendTransform(tm);
  }

  void external_odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
  {
    tf_odom_base_ = orca::pose_msg_to_transform(msg->pose.pose);
    tf_map_odom_.setIdentity();

    publish_tf(cxt_.map_frame_id_, cxt_.odom_frame_id_, tf_map_odom_);
    publish_tf(cxt_.odom_frame_id_, cxt_.base_frame_id_, tf_odom_base_);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now();
    odom_msg.header.frame_id = cxt_.odom_frame_id_;
    odom_msg.child_frame_id = cxt_.base_frame_id_;
    odom_msg.pose.pose = msg->pose.pose;
    odom_msg.twist.twist = msg->twist.twist;
    odom_pub_->publish(odom_msg);

    publish_vision_pose(msg->pose.pose, rclcpp::Time(msg->header.stamp));
  }

  void init_parameters()
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(BASE_ALL_PARAMS, validate_parameters)

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), cxt_, BASE_ALL_PARAMS, validate_parameters)

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    BASE_ALL_PARAMS

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), BASE_ALL_PARAMS)
  }

  void validate_parameters()
  {
    transform_expiration_ = {std::chrono::milliseconds{cxt_.transform_expiration_ms_}};
  }

public:
  BaseControllerNew()
  : Node("base_controller_new")
  {
    init_parameters();

    tf_map_odom_.setIdentity();
    tf_odom_base_.setIdentity();
    tf_odom_base_.getOrigin().setZ(0.0);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    rclcpp::QoS reliable(10);
    reliable.reliable();
    rclcpp::QoS best_effort(10);
    best_effort.best_effort();

    velocity_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
      "/mavros/setpoint_raw/local", reliable);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", reliable);
    vision_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/mavros/vision_pose/pose_cov", reliable);

    conn_srv_ = create_service<std_srvs::srv::SetBool>(
      "conn",
      [this](const std::shared_ptr<rmw_request_id_t>,
             const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response)
      {
        if (conn_ != request->data) {
          conn_ = request->data;
          if (!conn_) {
            geometry_msgs::msg::Twist zero;
            publish_velocity(zero);
          }
          RCLCPP_INFO(get_logger(), conn_ ? "Velocity control started" : "Velocity control stopped");
        }
        response->success = true;
      },
      rclcpp::QoS(10).reliable());

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", reliable,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg)
      {
        cmd_vel_ = *msg;
        publish_velocity(*msg);
      });

    external_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", best_effort,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { external_odom_cb(msg); });

    RCLCPP_INFO(get_logger(), "base_controller ready (external odometry from /odometry/filtered)");
    RCLCPP_INFO(get_logger(), "Publishing: /mavros/setpoint_raw/local, odom, /mavros/vision_pose/pose_cov");
    RCLCPP_INFO(get_logger(), "Subscribing: cmd_vel, /odometry/filtered");
    RCLCPP_INFO(get_logger(), "TF: map->odom->base_link");
  }
};

}  // namespace orca_base

int main(int argc, char ** argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<orca_base::BaseControllerNew>());
  rclcpp::shutdown();
  return 0;
}
