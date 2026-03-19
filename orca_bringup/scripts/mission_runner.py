#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2022 Clyde McQueen
# Copyright (c) 2024 ACFR
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Run missions via the MissionCoordinator.

The script talks to two action servers:
  /set_target_mode  (orca_msgs/action/TargetMode)   -- arms the vehicle (unchanged)
  /execute_mission  (orca_msgs/action/ExecuteMission) -- dispatches to sub-controllers

Mission types accepted by /execute_mission:
  "waypoint_nav"      Single pose goal via Nav2 navigate_to_pose
  "waypoint_seq"      Ordered pose sequence via Nav2 follow_waypoints
  "structure_survey"  Hold distance from structure while traversing

Usage:
  ros2 run orca_bringup mission_runner.py
  ros2 run orca_bringup mission_runner.py --mission huge_loop
  ros2 run orca_bringup mission_runner.py --mission structure_survey
"""

import argparse
import sys
import textwrap
from enum import Enum

import rclpy
import rclpy.logging
import yaml
from action_msgs.msg import GoalStatus
from orca_msgs.action import ExecuteMission, TargetMode
from rclpy.action import ActionClient


# ---------------------------------------------------------------------------
# Mission parameter helpers
# ---------------------------------------------------------------------------

def waypoint_nav_params(x: float, y: float, z: float,
                        yaw: float = 0.0, frame_id: str = 'map') -> str:
    """Return a YAML string for a single-pose waypoint_nav mission."""
    return yaml.dump({'frame_id': frame_id, 'x': x, 'y': y, 'z': z, 'yaw': yaw})


def waypoint_seq_params(waypoints: list, frame_id: str = 'map') -> str:
    """
    Return a YAML string for a waypoint_seq mission.

    waypoints -- list of dicts with keys x, y, z, and optionally yaw.
    Example:
        waypoint_seq_params([
            {'x': 0.0,  'y': 0.0,  'z': -7.0},
            {'x': 20.0, 'y': -13.0,'z': -7.0},
        ])
    """
    pts = [{'x': float(w.get('x', 0)), 'y': float(w.get('y', 0)),
            'z': float(w.get('z', 0)), 'yaw': float(w.get('yaw', 0))}
           for w in waypoints]
    return yaml.dump({'frame_id': frame_id, 'waypoints': pts})


def structure_survey_params(setpoint_distance: float,
                            traverse_distance: float,
                            traverse_direction_y: float = 1.0,
                            traverse_direction_x: float = 0.0,
                            setpoint_yaw: float = 0.0) -> str:
    """Return a YAML string for a structure_survey mission."""
    return yaml.dump({
        'setpoint_distance': float(setpoint_distance),
        'traverse_distance': float(traverse_distance),
        'traverse_direction_x': float(traverse_direction_x),
        'traverse_direction_y': float(traverse_direction_y),
        'traverse_direction_z': 0.0,
        'setpoint_yaw': float(setpoint_yaw),
    })


# ---------------------------------------------------------------------------
# Pre-built mission goals
# ---------------------------------------------------------------------------

# Arm the vehicle for autonomous operation
go_auv = TargetMode.Goal()
go_auv.target_mode = TargetMode.Goal.ORCA_MODE_AUV

# Return to pilot control
go_rov = TargetMode.Goal()
go_rov.target_mode = TargetMode.Goal.ORCA_MODE_ROV


def make_execute_mission(mission_type: str, params_yaml: str) -> ExecuteMission.Goal:
    goal = ExecuteMission.Goal()
    goal.mission_type = mission_type
    goal.mission_params_yaml = params_yaml
    return goal


# "huge loop" mission — replicates the original mission_runner.py trajectory
huge_loop_waypoints = [{'x': 0.0, 'y': 0.0, 'z': -7.0}]
for _ in range(2):
    huge_loop_waypoints += [
        {'x': 20.0, 'y': -13.0, 'z': -7.0},
        {'x': 10.0, 'y': -23.0, 'z': -7.0},
        {'x': -10.0, 'y': -8.0, 'z': -7.0},
        {'x': 0.0,  'y':  0.0,  'z': -7.0},
    ]

mission_huge_loop = make_execute_mission(
    'waypoint_seq',
    waypoint_seq_params(huge_loop_waypoints)
)

# Structure survey mission — hold 1.5 m from forward structure, sway 10 m
mission_structure_survey = make_execute_mission(
    'structure_survey',
    structure_survey_params(
        setpoint_distance=1.5,
        traverse_distance=10.0,
        traverse_direction_y=1.0,  # sway left
        setpoint_yaw=0.0,
    )
)

# Go home first (single pose, 1 m deep)
mission_go_home = make_execute_mission(
    'waypoint_nav',
    waypoint_nav_params(x=0.0, y=0.0, z=-1.0)
)

MISSIONS = {
    'huge_loop':       mission_huge_loop,
    'structure_survey': mission_structure_survey,
    'go_home':         mission_go_home,
}


# ---------------------------------------------------------------------------
# Action client helper (identical pattern to original)
# ---------------------------------------------------------------------------

class SendGoalResult(Enum):
    SUCCESS = 0
    FAILURE = 1
    CANCELED = 2


def send_goal(node, action_client, send_goal_msg, print_feedback=True) -> SendGoalResult:
    goal_handle = None

    try:
        action_client.wait_for_server()

        print('Sending goal...')
        goal_future = action_client.send_goal_async(
            send_goal_msg,
            feedback_callback=lambda fb: print(
                f'  feedback: {fb.feedback.status}  {fb.feedback.progress_pct:.0f}%'
            ) if print_feedback else None
        )
        rclpy.spin_until_future_complete(node, goal_future)
        goal_handle = goal_future.result()

        if goal_handle is None:
            raise RuntimeError(
                'Exception while sending goal: {!r}'.format(goal_future.exception()))

        if not goal_handle.accepted:
            print('Goal rejected')
            return SendGoalResult.FAILURE

        print('Goal accepted with ID: {}'.format(bytes(goal_handle.goal_id.uuid).hex()))
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)

        result = result_future.result()

        if result is None:
            raise RuntimeError(
                'Exception while getting result: {!r}'.format(result_future.exception()))

        success = result.result.success if hasattr(result.result, 'success') else (
            result.status == GoalStatus.STATUS_SUCCEEDED)
        msg = getattr(result.result, 'message', '')
        print(f'Goal completed: success={success}  "{msg}"')
        return SendGoalResult.SUCCESS if success else SendGoalResult.FAILURE

    except KeyboardInterrupt:
        if (goal_handle is not None and
                goal_handle.status in (GoalStatus.STATUS_ACCEPTED,
                                       GoalStatus.STATUS_EXECUTING)):
            print('Canceling goal...')
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel_future)
            cancel_response = cancel_future.result()

            if cancel_response is None:
                raise RuntimeError(
                    'Exception while canceling: {!r}'.format(cancel_future.exception()))
            if len(cancel_response.goals_canceling) == 0:
                raise RuntimeError('Failed to cancel goal')

            print('Goal canceled')
            return SendGoalResult.CANCELED


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description=textwrap.dedent("""\
            Run a mission via the MissionCoordinator.
            Available missions: {}
        """.format(', '.join(MISSIONS.keys())))
    )
    parser.add_argument(
        '--mission', default='huge_loop',
        choices=list(MISSIONS.keys()),
        help='Which mission to run (default: huge_loop)',
    )
    # Strip ROS args before parsing
    args = parser.parse_args(
        [a for a in sys.argv[1:] if not a.startswith('--ros-args')])

    node = None
    set_target_mode_client = None
    execute_mission_client = None

    rclpy.init()

    try:
        node = rclpy.create_node('mission_runner')

        set_target_mode_client = ActionClient(node, TargetMode, '/set_target_mode')
        execute_mission_client = ActionClient(node, ExecuteMission, '/execute_mission')

        print(f'>>> Setting mode to AUV <<<')
        if send_goal(node, set_target_mode_client, go_auv,
                     print_feedback=False) == SendGoalResult.SUCCESS:

            print(f'>>> Executing mission: {args.mission} <<<')
            result = send_goal(node, execute_mission_client, MISSIONS[args.mission])

            if result == SendGoalResult.CANCELED:
                print('>>> Mission canceled <<<')
            elif result == SendGoalResult.FAILURE:
                print('>>> Mission failed <<<')
            else:
                print('>>> Mission complete <<<')

            print('>>> Setting mode to ROV <<<')
            send_goal(node, set_target_mode_client, go_rov, print_feedback=False)

        else:
            print('>>> Failed to set mode to AUV, quit <<<')

    finally:
        if execute_mission_client is not None:
            execute_mission_client.destroy()
        if set_target_mode_client is not None:
            set_target_mode_client.destroy()
        if node is not None:
            node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
