#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Wimblerobotics
#
# Diagnostic node: runs alongside patrol_using_waypoints_launch.py
# Logs events to /tmp/patrol_diag_<timestamp>.log:
#
#   [SCAN_CLOSE] - nearest lidar return drops below 0.60 m
#   [STALL]      - nav feedback: distance_remaining not decreasing for 3+ s
#   [RECOVERY]   - recovery behavior started
#   [POSE]       - AMCL pose sampled every 2 s (for post-mortem path replay)
#   [NAV]        - navigation goal state changes
#
# NOTE: collision_monitor is not launched in this stack.
#
# Usage: runs automatically when included in patrol launch.
# To read the log in real time:  tail -f /tmp/patrol_diag_*.log

import math
import os
import time
import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from action_msgs.msg import GoalStatusArray
from sensor_msgs.msg import LaserScan


class PatrolDiagnostics(Node):
    def __init__(self):
        super().__init__('patrol_diagnostics')

        # Log file
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self._log_path = f'/tmp/patrol_diag_{ts}.log'
        self._log_file = open(self._log_path, 'w', buffering=1)  # line-buffered
        self._log(f'PatrolDiagnostics started. Log: {self._log_path}')
        self.get_logger().info(f'Diagnostic log: {self._log_path}')

        # ------------------------------------------------------------------
        # 1. AMCL pose — sampled at 0.5 Hz, also records on large jumps
        # ------------------------------------------------------------------
        amcl_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_cb,
            amcl_qos)
        self._last_pose_x = None
        self._last_pose_y = None
        self._last_pose_yaw = None
        self._last_pose_logged_time = 0.0
        self._pose_log_interval = 2.0    # seconds between periodic pose logs
        self._pose_jump_threshold = 0.5  # log immediately on AMCL jump > 0.5 m

        # ------------------------------------------------------------------
        # 2. Lidar scan — log when the nearest return is dangerously close.
        # ------------------------------------------------------------------
        self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_cb,
            10)
        self._scan_close_threshold = 0.60   # metres — log below this distance
        self._scan_last_min = 9999.0
        self._scan_last_log_time = 0.0
        self._scan_close_log_interval = 1.0  # log at most every 1 s when close

        # ------------------------------------------------------------------
        # 3. Nav2 action goal status
        # ------------------------------------------------------------------
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._nav_status_cb,
            10)
        self._nav_active_goals = set()

        # ------------------------------------------------------------------
        # 4. Nav2 feedback — stall detection via NavigateToPose feedback
        # ------------------------------------------------------------------
        try:
            from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
            self.create_subscription(
                NavigateToPose_FeedbackMessage,
                '/navigate_to_pose/_action/feedback',
                self._nav_feedback_cb,
                10)
        except Exception as e:
            self.get_logger().warn(f'Could not subscribe to nav feedback: {e}')

        self._last_distance = None
        self._last_distance_progress_time = None
        self._stall_logged = False
        self._stall_threshold_sec = 8.0  # was 4.0 — raised to avoid false alarms during in-place rotation
        self._last_yaw = None  # track heading to suppress stall during rotation

        # ------------------------------------------------------------------
        # 5. Behavior server — detects recovery behavior starts
        # ------------------------------------------------------------------
        self.create_subscription(
            GoalStatusArray,
            '/behavior_server/_action/status',
            self._behavior_status_cb,
            10)
        self._behavior_active_goals = set()

        self.get_logger().info('PatrolDiagnostics ready — monitoring 5 topics')

    # -----------------------------------------------------------------------
    def _log(self, msg: str):
        ts = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
        line = f'[{ts}] {msg}'
        print(line)
        self._log_file.write(line + '\n')

    def _pose_str(self) -> str:
        if self._last_pose_x is None:
            return ''
        return (f'  robot=({self._last_pose_x:.3f}, {self._last_pose_y:.3f})'
                f' yaw={math.degrees(self._last_pose_yaw):.1f}°')

    # -----------------------------------------------------------------------
    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        now = time.monotonic()
        jump = 0.0
        if self._last_pose_x is not None:
            jump = math.hypot(x - self._last_pose_x, y - self._last_pose_y)

        self._last_pose_x = x
        self._last_pose_y = y
        self._last_pose_yaw = yaw

        if jump > self._pose_jump_threshold:
            self._log(
                f'[POSE] AMCL JUMP {jump:.2f}m  pos=({x:.3f}, {y:.3f})'
                f' yaw={math.degrees(yaw):.1f}°')
        elif now - self._last_pose_logged_time >= self._pose_log_interval:
            self._log(f'[POSE] ({x:.3f}, {y:.3f}) yaw={math.degrees(yaw):.1f}°')
            self._last_pose_logged_time = now

    # -----------------------------------------------------------------------
    def _scan_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges
                 if not math.isinf(r) and not math.isnan(r)
                 and msg.range_min <= r <= msg.range_max]
        if not valid:
            return
        min_range = min(valid)
        self._scan_last_min = min_range

        now = time.monotonic()
        if min_range < self._scan_close_threshold:
            if now - self._scan_last_log_time >= self._scan_close_log_interval:
                idx = msg.ranges.index(min_range) if min_range in msg.ranges else -1
                angle_deg = (math.degrees(msg.angle_min + idx * msg.angle_increment)
                             if idx >= 0 else 0.0)
                self._log(
                    f'[SCAN_CLOSE] nearest={min_range:.3f}m @{angle_deg:.0f}°'
                    f'{self._pose_str()}')
                self._scan_last_log_time = now

    # -----------------------------------------------------------------------
    def _nav_status_cb(self, msg: GoalStatusArray):
        current_goals = {
            (bytes(s.goal_info.goal_id.uuid), s.status)
            for s in msg.status_list
        }
        new_goals = current_goals - self._nav_active_goals
        for goal_id_bytes, status in new_goals:
            status_str = {1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
                          4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(status, str(status))
            self._log(f'[NAV] goal → {status_str}{self._pose_str()}')
            if status in (1, 2):
                self._stall_logged = False
                self._last_distance = None
                self._last_distance_progress_time = None
                self._last_yaw = None
        self._nav_active_goals = current_goals

    # -----------------------------------------------------------------------
    def _nav_feedback_cb(self, msg):
        try:
            distance = msg.feedback.distance_remaining
            now = time.monotonic()

            if self._last_distance is None:
                self._last_distance = distance
                self._last_distance_progress_time = now
                self._last_yaw = None
                return

            # Reset stall timer on linear progress OR active yaw rotation.
            # Rotation shim controller does in-place turns that produce zero
            # positional progress; without this guard every heading correction
            # fires a spurious STALL.
            if distance < self._last_distance - 0.05:
                self._last_distance_progress_time = now
                self._stall_logged = False
            else:
                # Check if robot is actively rotating via feedback current_pose
                try:
                    q = msg.feedback.current_pose.pose.orientation
                    yaw = math.atan2(
                        2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                    if self._last_yaw is not None:
                        dyaw = abs(yaw - self._last_yaw)
                        if dyaw > math.pi:
                            dyaw = 2.0 * math.pi - dyaw
                        if dyaw > 0.02:  # ~1° per feedback cycle = actively rotating
                            self._last_distance_progress_time = now
                            self._stall_logged = False
                    self._last_yaw = yaw
                except Exception:
                    pass

            self._last_distance = distance

            stall_sec = now - self._last_distance_progress_time
            if stall_sec >= self._stall_threshold_sec and not self._stall_logged:
                self._log(
                    f'[STALL] no forward progress for {stall_sec:.1f}s'
                    f'  dist_remaining={distance:.3f}m'
                    f'  nearest_scan={self._scan_last_min:.3f}m'
                    f'{self._pose_str()}')
                self._stall_logged = True
        except Exception:
            pass

    # -----------------------------------------------------------------------
    def _behavior_status_cb(self, msg: GoalStatusArray):
        current_goals = {
            (bytes(s.goal_info.goal_id.uuid), s.status)
            for s in msg.status_list
        }
        new_goals = current_goals - self._behavior_active_goals
        for goal_id_bytes, status in new_goals:
            if status in (1, 2):
                self._log(f'[RECOVERY] behavior_server started recovery{self._pose_str()}')
        self._behavior_active_goals = current_goals

    # -----------------------------------------------------------------------
    def destroy_node(self):
        self._log('PatrolDiagnostics shutting down')
        self._log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PatrolDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
