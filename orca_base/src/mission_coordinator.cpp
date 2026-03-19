// MIT License
//
// Copyright (c) 2024 ACFR
//
// Dispatch incoming ExecuteMission action goals to the appropriate mission
// controller (Nav2, StructureSurveyController, etc.) based on mission_type.
// New mission types are registered by adding a MissionHandler to handlers_.

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "orca_msgs/action/execute_mission.hpp"
#include "orca_msgs/action/structure_survey.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "yaml-cpp/yaml.h"

namespace orca_base
{

using ExecuteMission = orca_msgs::action::ExecuteMission;
using GoalHandleEM = rclcpp_action::ServerGoalHandle<ExecuteMission>;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using StructureSurvey = orca_msgs::action::StructureSurvey;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static geometry_msgs::msg::Quaternion yaw_to_quat(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

// Abort with a human-readable message and log it.
static void abort_with_msg(
  std::shared_ptr<GoalHandleEM> gh,
  rclcpp::Logger logger,
  const std::string & msg)
{
  RCLCPP_ERROR(logger, "%s", msg.c_str());
  auto result = std::make_shared<ExecuteMission::Result>();
  result->success = false;
  result->message = msg;
  gh->abort(result);
}

// ---------------------------------------------------------------------------
// MissionHandler base
// ---------------------------------------------------------------------------

class MissionHandler
{
public:
  virtual ~MissionHandler() = default;

  // Called from the execute thread. Must call gh->succeed/abort/canceled before returning.
  virtual void execute(
    const std::string & params_yaml,
    std::shared_ptr<GoalHandleEM> gh) = 0;
};

// ---------------------------------------------------------------------------
// Shared wait-for-result utility
//
// Polls a future in a tight loop, forwarding cancellations and publishing
// periodic feedback. Returns true if the sub-action succeeded.
// ---------------------------------------------------------------------------

template<typename ClientT>
static bool wait_for_result(
  std::shared_future<typename rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult> result_future,
  std::shared_ptr<rclcpp_action::ClientGoalHandle<ClientT>> & sub_goal_handle,
  rclcpp_action::Client<ClientT> & client,
  std::shared_ptr<GoalHandleEM> gh,
  std::shared_ptr<ExecuteMission::Feedback> feedback)
{
  while (true) {
    if (result_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
      auto result = result_future.get();
      return result.code == rclcpp_action::ResultCode::SUCCEEDED;
    }

    if (gh->is_canceling()) {
      if (sub_goal_handle) {
        client.async_cancel_goal(sub_goal_handle);
      }
      auto em_result = std::make_shared<ExecuteMission::Result>();
      em_result->success = false;
      em_result->message = "Cancelled";
      gh->canceled(em_result);
      return false;
    }

    gh->publish_feedback(feedback);
  }
}

// ---------------------------------------------------------------------------
// WaypointNavHandler — navigate_to_pose (single pose)
// ---------------------------------------------------------------------------

class WaypointNavHandler : public MissionHandler
{
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::Logger logger_;

public:
  WaypointNavHandler(
    rclcpp_action::Client<NavigateToPose>::SharedPtr client,
    rclcpp::Logger logger)
  : client_(client), logger_(logger) {}

  void execute(const std::string & params_yaml, std::shared_ptr<GoalHandleEM> gh) override
  {
    YAML::Node p;
    try {
      p = YAML::Load(params_yaml);
    } catch (const YAML::Exception & e) {
      abort_with_msg(gh, logger_, std::string("waypoint_nav YAML error: ") + e.what());
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = p["frame_id"] ? p["frame_id"].as<std::string>() : "map";
    goal.pose.pose.position.x = p["x"] ? p["x"].as<double>() : 0.0;
    goal.pose.pose.position.y = p["y"] ? p["y"].as<double>() : 0.0;
    goal.pose.pose.position.z = p["z"] ? p["z"].as<double>() : 0.0;
    goal.pose.pose.orientation = yaw_to_quat(p["yaw"] ? p["yaw"].as<double>() : 0.0);

    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      abort_with_msg(gh, logger_, "navigate_to_pose action server unavailable");
      return;
    }

    auto feedback = std::make_shared<ExecuteMission::Feedback>();
    feedback->status = "navigating";

    std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>> sub_gh;
    std::atomic<bool> sub_gh_ready{false};

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    options.goal_response_callback =
      [this, &sub_gh, &sub_gh_ready](auto goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(logger_, "navigate_to_pose rejected goal");
        }
        sub_gh = goal_handle;
        // Release store — execute thread acquire-loads this before reading sub_gh
        sub_gh_ready.store(true, std::memory_order_release);
      };

    options.feedback_callback =
      [feedback](auto, auto fb) {
        feedback->status = "navigating";
        feedback->progress_pct = 0.0f;
        (void) fb;
      };

    options.result_callback =
      [](auto) {};  // result retrieved via async_get_result below

    client_->async_send_goal(goal, options);

    // Wait for goal acceptance — acquire-load ensures sub_gh write is visible
    while (!sub_gh_ready.load(std::memory_order_acquire) && !gh->is_canceling()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!sub_gh) {
      abort_with_msg(gh, logger_, "navigate_to_pose goal not accepted");
      return;
    }

    auto wrapped_future = client_->async_get_result(sub_gh);

    bool succeeded = wait_for_result<NavigateToPose>(
      wrapped_future, sub_gh, *client_, gh, feedback);

    if (gh->is_canceling()) {
      return;  // already handled by wait_for_result
    }

    auto em_result = std::make_shared<ExecuteMission::Result>();
    em_result->success = succeeded;
    em_result->message = succeeded ? "Waypoint reached" : "Navigation failed";
    if (succeeded) {
      gh->succeed(em_result);
    } else {
      gh->abort(em_result);
    }
  }
};

// ---------------------------------------------------------------------------
// WaypointSeqHandler — follow_waypoints (ordered sequence)
// ---------------------------------------------------------------------------

class WaypointSeqHandler : public MissionHandler
{
  rclcpp_action::Client<FollowWaypoints>::SharedPtr client_;
  rclcpp::Logger logger_;

public:
  WaypointSeqHandler(
    rclcpp_action::Client<FollowWaypoints>::SharedPtr client,
    rclcpp::Logger logger)
  : client_(client), logger_(logger) {}

  void execute(const std::string & params_yaml, std::shared_ptr<GoalHandleEM> gh) override
  {
    YAML::Node p;
    try {
      p = YAML::Load(params_yaml);
    } catch (const YAML::Exception & e) {
      abort_with_msg(gh, logger_, std::string("waypoint_seq YAML error: ") + e.what());
      return;
    }

    std::string frame_id = p["frame_id"] ? p["frame_id"].as<std::string>() : "map";

    FollowWaypoints::Goal goal;
    if (p["waypoints"]) {
      for (const auto & wp : p["waypoints"]) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = frame_id;
        ps.pose.position.x = wp["x"] ? wp["x"].as<double>() : 0.0;
        ps.pose.position.y = wp["y"] ? wp["y"].as<double>() : 0.0;
        ps.pose.position.z = wp["z"] ? wp["z"].as<double>() : 0.0;
        ps.pose.orientation = yaw_to_quat(wp["yaw"] ? wp["yaw"].as<double>() : 0.0);
        goal.poses.push_back(ps);
      }
    }

    if (goal.poses.empty()) {
      abort_with_msg(gh, logger_, "waypoint_seq: no waypoints provided");
      return;
    }

    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      abort_with_msg(gh, logger_, "follow_waypoints action server unavailable");
      return;
    }

    const float n = static_cast<float>(goal.poses.size());
    auto feedback = std::make_shared<ExecuteMission::Feedback>();
    feedback->status = "following_waypoints";

    std::shared_ptr<rclcpp_action::ClientGoalHandle<FollowWaypoints>> sub_gh;
    std::atomic<bool> sub_gh_ready{false};

    auto options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();

    options.goal_response_callback =
      [this, &sub_gh, &sub_gh_ready](auto goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(logger_, "follow_waypoints rejected goal");
        }
        sub_gh = goal_handle;
        sub_gh_ready.store(true, std::memory_order_release);
      };

    options.feedback_callback =
      [feedback, n](auto, auto fb) {
        feedback->progress_pct = n > 0
          ? static_cast<float>(fb->current_waypoint) / n * 100.0f
          : 0.0f;
      };

    options.result_callback = [](auto) {};

    client_->async_send_goal(goal, options);

    while (!sub_gh_ready.load(std::memory_order_acquire) && !gh->is_canceling()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!sub_gh) {
      abort_with_msg(gh, logger_, "follow_waypoints goal not accepted");
      return;
    }

    auto wrapped_future = client_->async_get_result(sub_gh);

    // Wait for follow_waypoints to finish
    while (true) {
      if (wrapped_future.wait_for(std::chrono::milliseconds(100)) ==
        std::future_status::ready)
      {
        break;
      }
      if (gh->is_canceling()) {
        if (sub_gh) {
          client_->async_cancel_goal(sub_gh);
        }
        auto em_result = std::make_shared<ExecuteMission::Result>();
        em_result->success = false;
        em_result->message = "Cancelled";
        gh->canceled(em_result);
        return;
      }
      gh->publish_feedback(feedback);
    }

    if (gh->is_canceling()) {
      return;
    }

    auto fw_result = wrapped_future.get();
    const bool nav_succeeded =
      fw_result.code == rclcpp_action::ResultCode::SUCCEEDED;
    const size_t missed =
      (nav_succeeded && fw_result.result)
      ? fw_result.result->missed_waypoints.size()
      : goal.poses.size();

    auto em_result = std::make_shared<ExecuteMission::Result>();
    em_result->success = nav_succeeded && (missed == 0);
    em_result->message = em_result->success
      ? "All waypoints reached"
      : (nav_succeeded
        ? "Waypoint sequence completed with " + std::to_string(missed) + " missed waypoint(s)"
        : "Waypoint sequence failed");

    if (em_result->success) {
      gh->succeed(em_result);
    } else {
      gh->abort(em_result);
    }
  }
};

// ---------------------------------------------------------------------------
// StructureSurveyHandler — dispatches to StructureSurveyController action
// ---------------------------------------------------------------------------

class StructureSurveyHandler : public MissionHandler
{
  rclcpp_action::Client<StructureSurvey>::SharedPtr client_;
  rclcpp::Logger logger_;

public:
  StructureSurveyHandler(
    rclcpp_action::Client<StructureSurvey>::SharedPtr client,
    rclcpp::Logger logger)
  : client_(client), logger_(logger) {}

  void execute(const std::string & params_yaml, std::shared_ptr<GoalHandleEM> gh) override
  {
    YAML::Node p;
    try {
      p = YAML::Load(params_yaml);
    } catch (const YAML::Exception & e) {
      abort_with_msg(gh, logger_, std::string("structure_survey YAML error: ") + e.what());
      return;
    }

    StructureSurvey::Goal goal;
    goal.setpoint_distance =
      p["setpoint_distance"] ? p["setpoint_distance"].as<double>() : 1.5;
    goal.traverse_distance =
      p["traverse_distance"] ? p["traverse_distance"].as<double>() : 5.0;
    goal.setpoint_yaw =
      p["setpoint_yaw"] ? p["setpoint_yaw"].as<double>() : 0.0;
    goal.traverse_direction.x =
      p["traverse_direction_x"] ? p["traverse_direction_x"].as<double>() : 0.0;
    goal.traverse_direction.y =
      p["traverse_direction_y"] ? p["traverse_direction_y"].as<double>() : 1.0;
    goal.traverse_direction.z =
      p["traverse_direction_z"] ? p["traverse_direction_z"].as<double>() : 0.0;

    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      abort_with_msg(gh, logger_, "structure_survey action server unavailable");
      return;
    }

    auto feedback = std::make_shared<ExecuteMission::Feedback>();
    feedback->status = "structure_survey:waiting_for_sensor";

    std::shared_ptr<rclcpp_action::ClientGoalHandle<StructureSurvey>> sub_gh;
    std::atomic<bool> sub_gh_ready{false};

    auto options = rclcpp_action::Client<StructureSurvey>::SendGoalOptions();

    options.goal_response_callback =
      [this, &sub_gh, &sub_gh_ready](auto goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(logger_, "structure_survey rejected goal");
        }
        sub_gh = goal_handle;
        sub_gh_ready.store(true, std::memory_order_release);
      };

    options.feedback_callback =
      [feedback](auto, auto fb) {
        feedback->status = "structure_survey:" + fb->phase;
        feedback->progress_pct = 0.0f;
      };

    options.result_callback = [](auto) {};

    client_->async_send_goal(goal, options);

    while (!sub_gh_ready.load(std::memory_order_acquire) && !gh->is_canceling()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!sub_gh) {
      abort_with_msg(gh, logger_, "structure_survey goal not accepted");
      return;
    }

    auto wrapped_future = client_->async_get_result(sub_gh);

    bool succeeded = wait_for_result<StructureSurvey>(
      wrapped_future, sub_gh, *client_, gh, feedback);

    if (gh->is_canceling()) {
      return;
    }

    auto em_result = std::make_shared<ExecuteMission::Result>();
    em_result->success = succeeded;
    em_result->message = succeeded ? "Structure survey complete" : "Structure survey failed";
    if (succeeded) {
      gh->succeed(em_result);
    } else {
      gh->abort(em_result);
    }
  }
};

// ---------------------------------------------------------------------------
// MissionCoordinator node
// ---------------------------------------------------------------------------

class MissionCoordinator : public rclcpp::Node
{
  rclcpp_action::Server<ExecuteMission>::SharedPtr execute_mission_srv_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_client_;
  rclcpp_action::Client<StructureSurvey>::SharedPtr structure_survey_client_;

  std::map<std::string, std::unique_ptr<MissionHandler>> handlers_;

  std::shared_ptr<GoalHandleEM> active_goal_handle_;
  std::thread execute_thread_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteMission::Goal> goal)
  {
    if (handlers_.find(goal->mission_type) == handlers_.end()) {
      RCLCPP_WARN(
        get_logger(), "Rejecting unknown mission type '%s'", goal->mission_type.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (active_goal_handle_ && active_goal_handle_->is_active()) {
      RCLCPP_WARN(get_logger(), "Mission already active, rejecting new goal");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(
      get_logger(), "Accepting mission '%s'", goal->mission_type.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandleEM>)
  {
    RCLCPP_INFO(get_logger(), "Cancel requested for active mission");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(std::shared_ptr<GoalHandleEM> goal_handle)
  {
    active_goal_handle_ = goal_handle;
    if (execute_thread_.joinable()) {
      execute_thread_.join();
    }
    execute_thread_ = std::thread(
      [this, goal_handle]() {
        const auto goal = goal_handle->get_goal();
        handlers_.at(goal->mission_type)->execute(goal->mission_params_yaml, goal_handle);
        active_goal_handle_ = nullptr;
      });
  }

public:
  MissionCoordinator()
  : Node("mission_coordinator")
  {
    execute_mission_srv_ = rclcpp_action::create_server<ExecuteMission>(
      this,
      "execute_mission",
      [this](const auto & uuid, auto goal) { return handle_goal(uuid, goal); },
      [this](auto gh) { return handle_cancel(gh); },
      [this](auto gh) { handle_accepted(gh); });

    nav_to_pose_client_ =
      rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    follow_waypoints_client_ =
      rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
    structure_survey_client_ =
      rclcpp_action::create_client<StructureSurvey>(this, "structure_survey");

    handlers_["waypoint_nav"] = std::make_unique<WaypointNavHandler>(
      nav_to_pose_client_, get_logger());
    handlers_["waypoint_seq"] = std::make_unique<WaypointSeqHandler>(
      follow_waypoints_client_, get_logger());
    handlers_["structure_survey"] = std::make_unique<StructureSurveyHandler>(
      structure_survey_client_, get_logger());

    RCLCPP_INFO(get_logger(), "mission_coordinator ready");
    RCLCPP_INFO(
      get_logger(),
      "Registered mission types: waypoint_nav, waypoint_seq, structure_survey");
  }

  ~MissionCoordinator()
  {
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
  rclcpp::spin(std::make_shared<orca_base::MissionCoordinator>());
  rclcpp::shutdown();
  return 0;
}
