// MIT License
//
// Copyright (c) 2024 ACFR
//
// StructureSurveyController
// =========================
// Receives a StructureSurvey action goal and runs a parallel control loop:
//
//   1. Distance hold  – PID on sensor range → cmd_vel.linear.x (surge)
//   2. Traverse       – constant velocity along traverse_direction → cmd_vel.linear.y/x
//   3. Yaw hold       – PID on heading error → cmd_vel.angular.z
//
// Sensor abstraction
// ------------------
// Set the parameter `range_sensor_type` to choose the range source:
//
//   "range"       – subscribe to sensor_msgs/Range on `range_topic`
//                   (forward-facing single-beam sonar or ultrasonic sensor)
//
//   "depth_image" – subscribe to sensor_msgs/Image on `depth_image_topic`
//                   (forward-facing depth camera, 32FC1 or 16UC1 encoding)
//                   The controller takes the median of a centre ROI as range.

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "orca_msgs/action/structure_survey.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace orca_base
{

using StructureSurvey = orca_msgs::action::StructureSurvey;
using GoalHandleSS = rclcpp_action::ServerGoalHandle<StructureSurvey>;

// ---------------------------------------------------------------------------
// Simple PID with integral anti-windup and symmetric output clamping
// ---------------------------------------------------------------------------

struct Pid
{
  double kp{1.0}, ki{0.0}, kd{0.1};
  double integral_limit{1.0};   // clamp integral contribution
  double output_limit{1.0};     // clamp output

  double integral{0.0};
  double prev_error{0.0};
  bool first{true};

  double compute(double error, double dt)
  {
    if (first) {
      prev_error = error;
      first = false;
    }
    integral += error * dt;
    integral = std::clamp(integral, -integral_limit, integral_limit);
    double derivative = (error - prev_error) / (dt > 1e-6 ? dt : 1e-6);
    prev_error = error;
    double output = kp * error + ki * integral + kd * derivative;
    return std::clamp(output, -output_limit, output_limit);
  }

  void reset()
  {
    integral = 0.0;
    prev_error = 0.0;
    first = true;
  }
};

// ---------------------------------------------------------------------------
// StructureSurveyController node
// ---------------------------------------------------------------------------

class StructureSurveyController : public rclcpp::Node
{
  // ---- Parameters ---------------------------------------------------------

  std::string range_sensor_type_;   // "range" | "depth_image"
  std::string range_topic_;
  std::string depth_image_topic_;
  int depth_roi_half_{5};           // half-size of centre ROI (pixels)

  double max_surge_vel_;            // m/s
  double max_sway_vel_;             // m/s
  double max_yaw_rate_;             // rad/s
  double traverse_vel_;             // m/s – constant lateral speed during traverse

  double control_hz_;

  // ---- Sensor state -------------------------------------------------------

  std::mutex sensor_mutex_;
  std::optional<double> current_range_;   // metres, from whichever sensor is active

  // ---- Odometry state -----------------------------------------------------

  std::mutex odom_mutex_;
  std::optional<nav_msgs::msg::Odometry> current_odom_;

  // ---- Mission state ------------------------------------------------------

  std::atomic<bool> mission_active_{false};

  // Set at mission start, read from control loop
  double setpoint_distance_{1.5};
  double traverse_distance_{5.0};
  double setpoint_yaw_{0.0};
  double traverse_dir_world_x_{0.0};   // world-frame traverse unit vector
  double traverse_dir_world_y_{1.0};

  double start_x_{0.0}, start_y_{0.0};

  Pid pid_distance_;
  Pid pid_yaw_;

  std::shared_ptr<GoalHandleSS> active_goal_handle_;
  std::thread execute_thread_;

  // ---- ROS interfaces -----------------------------------------------------

  rclcpp_action::Server<StructureSurvey>::SharedPtr action_server_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  // ---- Sensor callbacks ---------------------------------------------------

  void range_cb(sensor_msgs::msg::Range::ConstSharedPtr msg)
  {
    if (std::isfinite(msg->range) &&
      msg->range >= msg->min_range &&
      msg->range <= msg->max_range)
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      current_range_ = msg->range;
    }
  }

  // Extract the median of valid centre-ROI pixels from a depth image.
  // Supports 32FC1 (float metres) and 16UC1 (uint16 millimetres) encodings.
  void depth_image_cb(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    const int cx = static_cast<int>(msg->width) / 2;
    const int cy = static_cast<int>(msg->height) / 2;
    const int r = depth_roi_half_;

    const int x0 = std::max(0, cx - r);
    const int x1 = std::min(static_cast<int>(msg->width) - 1, cx + r);
    const int y0 = std::max(0, cy - r);
    const int y1 = std::min(static_cast<int>(msg->height) - 1, cy + r);

    std::vector<double> samples;
    samples.reserve(static_cast<size_t>((x1 - x0 + 1) * (y1 - y0 + 1)));

    if (msg->encoding == "32FC1") {
      for (int row = y0; row <= y1; ++row) {
        const float * row_ptr = reinterpret_cast<const float *>(
          msg->data.data() + static_cast<size_t>(row) * msg->step);
        for (int col = x0; col <= x1; ++col) {
          float d = row_ptr[col];
          if (std::isfinite(d) && d > 0.0f) {
            samples.push_back(static_cast<double>(d));
          }
        }
      }
    } else if (msg->encoding == "16UC1") {
      for (int row = y0; row <= y1; ++row) {
        const uint16_t * row_ptr = reinterpret_cast<const uint16_t *>(
          msg->data.data() + static_cast<size_t>(row) * msg->step);
        for (int col = x0; col <= x1; ++col) {
          uint16_t raw = row_ptr[col];
          if (raw > 0) {
            samples.push_back(raw / 1000.0);   // mm → m
          }
        }
      }
    } else {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "Unsupported depth image encoding '%s'. Expected 32FC1 or 16UC1.",
        msg->encoding.c_str());
      return;
    }

    if (samples.empty()) {
      return;
    }

    // Median is more robust than mean against specular noise
    const size_t mid = samples.size() / 2;
    std::nth_element(samples.begin(), samples.begin() + mid, samples.end());
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    current_range_ = samples[mid];
  }

  void odom_cb(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    current_odom_ = *msg;
  }

  // ---- Control loop (runs at control_hz_) ---------------------------------

  void control_tick()
  {
    if (!mission_active_) {
      return;
    }

    // --- Gather state ---
    std::optional<double> range;
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      range = current_range_;
    }

    std::optional<nav_msgs::msg::Odometry> odom;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      odom = current_odom_;
    }

    auto feedback = std::make_shared<StructureSurvey::Feedback>();
    feedback->phase = range ? "controlling" : "waiting_for_sensor";

    if (!range) {
      feedback->current_distance = 0.0;
      feedback->traverse_progress_m = 0.0;
      feedback->heading_error_rad = 0.0;
      if (active_goal_handle_) {
        active_goal_handle_->publish_feedback(feedback);
      }
      // Publish zero velocity while waiting for sensor
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
      return;
    }

    const double dt = 1.0 / control_hz_;

    // --- Distance hold (surge) ---
    const double dist_error = setpoint_distance_ - *range;
    const double surge = pid_distance_.compute(dist_error, dt);

    // --- Yaw hold ---
    double current_yaw = 0.0;
    double traverse_progress = 0.0;

    if (odom) {
      current_yaw = tf2::getYaw(odom->pose.pose.orientation);
      const double dx = odom->pose.pose.position.x - start_x_;
      const double dy = odom->pose.pose.position.y - start_y_;
      traverse_progress = dx * traverse_dir_world_x_ + dy * traverse_dir_world_y_;
    }

    double yaw_error = setpoint_yaw_ - current_yaw;
    // Normalise to [-pi, pi]
    while (yaw_error > M_PI) {yaw_error -= 2.0 * M_PI;}
    while (yaw_error < -M_PI) {yaw_error += 2.0 * M_PI;}
    const double yaw_rate = pid_yaw_.compute(yaw_error, dt);

    // --- Traverse (sway / surge blend) ---
    // Apply traverse velocity only until distance goal is reached
    const double remaining = traverse_distance_ - traverse_progress;
    double sway = 0.0;

    if (remaining > 0.01) {
      // Blend body-frame traverse direction with traverse_vel_
      // Note: surge component of traverse direction is added on top of the
      // distance-hold surge, so they combine naturally.
      sway = traverse_vel_ * traverse_dir_world_y_;
      // (traverse_dir_world_x_ contributes to surge below)
    }

    // Compose cmd_vel
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::clamp(
      surge + traverse_vel_ * traverse_dir_world_x_ * (remaining > 0.01 ? 1.0 : 0.0),
      -max_surge_vel_, max_surge_vel_);
    cmd.linear.y = std::clamp(sway, -max_sway_vel_, max_sway_vel_);
    cmd.angular.z = std::clamp(yaw_rate, -max_yaw_rate_, max_yaw_rate_);

    cmd_vel_pub_->publish(cmd);

    // --- Feedback ---
    feedback->current_distance = *range;
    feedback->traverse_progress_m = traverse_progress;
    feedback->heading_error_rad = yaw_error;

    if (remaining <= 0.01) {
      feedback->phase = "complete";
    }

    if (active_goal_handle_) {
      active_goal_handle_->publish_feedback(feedback);
    }

    // Traverse complete – signal execute thread
    if (remaining <= 0.01 && mission_active_) {
      mission_active_ = false;
    }
  }

  // ---- Action server callbacks --------------------------------------------

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const StructureSurvey::Goal>)
  {
    if (mission_active_) {
      RCLCPP_WARN(get_logger(), "Survey already active, rejecting new goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandleSS>)
  {
    RCLCPP_INFO(get_logger(), "Cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(std::shared_ptr<GoalHandleSS> goal_handle)
  {
    active_goal_handle_ = goal_handle;
    if (execute_thread_.joinable()) {
      execute_thread_.join();
    }
    execute_thread_ = std::thread([this, goal_handle]() { run_mission(goal_handle); });
  }

  // ---- Mission execution (runs in dedicated thread) -----------------------

  void run_mission(std::shared_ptr<GoalHandleSS> goal_handle)
  {
    const auto goal = goal_handle->get_goal();

    // Capture start pose
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (current_odom_) {
        start_x_ = current_odom_->pose.pose.position.x;
        start_y_ = current_odom_->pose.pose.position.y;
        const double initial_yaw = tf2::getYaw(current_odom_->pose.pose.orientation);
        // Rotate body-frame traverse direction to world frame using initial yaw
        const double c = std::cos(initial_yaw);
        const double s = std::sin(initial_yaw);
        const double bx = goal->traverse_direction.x;
        const double by = goal->traverse_direction.y;
        traverse_dir_world_x_ = c * bx - s * by;
        traverse_dir_world_y_ = s * bx + c * by;
        // Normalise world-frame vector
        const double mag = std::hypot(traverse_dir_world_x_, traverse_dir_world_y_);
        if (mag > 1e-6) {
          traverse_dir_world_x_ /= mag;
          traverse_dir_world_y_ /= mag;
        }
      } else {
        // No odometry yet – use body-frame direction directly
        traverse_dir_world_x_ = goal->traverse_direction.x;
        traverse_dir_world_y_ = goal->traverse_direction.y;
        start_x_ = 0.0;
        start_y_ = 0.0;
      }
    }

    setpoint_distance_ = goal->setpoint_distance;
    traverse_distance_ = goal->traverse_distance;
    setpoint_yaw_ = goal->setpoint_yaw;

    pid_distance_.reset();
    pid_yaw_.reset();
    mission_active_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Structure survey started: hold %.2f m, traverse %.2f m, yaw %.2f rad",
      setpoint_distance_, traverse_distance_, setpoint_yaw_);

    // Block until mission completes or is cancelled
    while (mission_active_) {
      if (goal_handle->is_canceling()) {
        mission_active_ = false;
        // Stop vehicle
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});

        double traversed = 0.0;
        {
          std::lock_guard<std::mutex> lock(odom_mutex_);
          if (current_odom_) {
            const double dx = current_odom_->pose.pose.position.x - start_x_;
            const double dy = current_odom_->pose.pose.position.y - start_y_;
            traversed = dx * traverse_dir_world_x_ + dy * traverse_dir_world_y_;
          }
        }

        auto result = std::make_shared<StructureSurvey::Result>();
        result->success = false;
        result->message = "Cancelled";
        result->distance_traversed = traversed;
        goal_handle->canceled(result);
        active_goal_handle_ = nullptr;
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Stop vehicle
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});

    double traversed = 0.0;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (current_odom_) {
        const double dx = current_odom_->pose.pose.position.x - start_x_;
        const double dy = current_odom_->pose.pose.position.y - start_y_;
        traversed = dx * traverse_dir_world_x_ + dy * traverse_dir_world_y_;
      }
    }

    auto result = std::make_shared<StructureSurvey::Result>();
    result->success = true;
    result->message = "Survey complete";
    result->distance_traversed = traversed;
    goal_handle->succeed(result);
    active_goal_handle_ = nullptr;

    RCLCPP_INFO(get_logger(), "Structure survey complete (%.2f m traversed)", traversed);
  }

public:
  StructureSurveyController()
  : Node("structure_survey_controller")
  {
    // Parameters
    range_sensor_type_ = declare_parameter<std::string>("range_sensor_type", "range");
    range_topic_ = declare_parameter<std::string>("range_topic", "forward_range");
    depth_image_topic_ =
      declare_parameter<std::string>("depth_image_topic", "forward_camera/depth/image_raw");
    depth_roi_half_ = declare_parameter<int>("depth_image_roi_half", 5);

    max_surge_vel_ = declare_parameter<double>("max_surge_vel", 0.3);
    max_sway_vel_ = declare_parameter<double>("max_sway_vel", 0.3);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 0.25);
    traverse_vel_ = declare_parameter<double>("traverse_vel", 0.2);
    control_hz_ = declare_parameter<double>("control_hz", 20.0);

    pid_distance_.kp = declare_parameter<double>("pid_distance_kp", 1.0);
    pid_distance_.ki = declare_parameter<double>("pid_distance_ki", 0.0);
    pid_distance_.kd = declare_parameter<double>("pid_distance_kd", 0.1);
    pid_distance_.integral_limit = declare_parameter<double>("pid_distance_integral_limit", 0.5);
    pid_distance_.output_limit = max_surge_vel_;

    pid_yaw_.kp = declare_parameter<double>("pid_yaw_kp", 2.0);
    pid_yaw_.ki = declare_parameter<double>("pid_yaw_ki", 0.0);
    pid_yaw_.kd = declare_parameter<double>("pid_yaw_kd", 0.2);
    pid_yaw_.integral_limit = declare_parameter<double>("pid_yaw_integral_limit", 0.5);
    pid_yaw_.output_limit = max_yaw_rate_;

    rclcpp::QoS reliable(10);
    reliable.reliable();
    rclcpp::QoS best_effort(10);
    best_effort.best_effort();

    cmd_vel_pub_ =
      create_publisher<geometry_msgs::msg::Twist>("cmd_vel", reliable);

    // Odometry for pose feedback
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", best_effort,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_cb(msg); });

    // Sensor subscription — chosen by parameter
    if (range_sensor_type_ == "depth_image") {
      depth_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_image_topic_, best_effort,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { depth_image_cb(msg); });
      RCLCPP_INFO(
        get_logger(), "Range source: depth_image on '%s'", depth_image_topic_.c_str());
    } else {
      if (range_sensor_type_ != "range") {
        RCLCPP_WARN(
          get_logger(),
          "Unknown range_sensor_type '%s', defaulting to 'range'",
          range_sensor_type_.c_str());
      }
      range_sub_ = create_subscription<sensor_msgs::msg::Range>(
        range_topic_, best_effort,
        [this](sensor_msgs::msg::Range::ConstSharedPtr msg) { range_cb(msg); });
      RCLCPP_INFO(
        get_logger(), "Range source: sensor_msgs/Range on '%s'", range_topic_.c_str());
    }

    // Control timer
    const auto period = std::chrono::duration<double>(1.0 / control_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { control_tick(); });

    // Action server
    auto cb_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    action_server_ = rclcpp_action::create_server<StructureSurvey>(
      this,
      "structure_survey",
      [this](const auto & uuid, auto goal) { return handle_goal(uuid, goal); },
      [this](auto gh) { return handle_cancel(gh); },
      [this](auto gh) { handle_accepted(gh); },
      rcl_action_server_get_default_options(),
      cb_group);

    RCLCPP_INFO(
      get_logger(),
      "structure_survey_controller ready (range_sensor_type=%s, control_hz=%.0f)",
      range_sensor_type_.c_str(), control_hz_);
  }

  ~StructureSurveyController()
  {
    mission_active_ = false;
    if (execute_thread_.joinable()) {
      execute_thread_.join();
    }
  }
};

}  // namespace orca_base

int main(int argc, char ** argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orca_base::StructureSurveyController>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
