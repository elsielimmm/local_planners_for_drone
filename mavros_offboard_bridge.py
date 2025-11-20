#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import PositionTarget


class OffboardPosFollower(Node):
    """
    ROS2 + MAVROS2:
      - Subscribe: /drone_0_planning/pos_cmdtrong (geometry_msgs/PoseStamped)  -> lệnh đích người dùng
      - Relay    : /mavros/setpoint_position/local      -> gửi setpoint cho PX4
      - State    : /mavros/state                        -> theo dõi mode/armed
      - Arming   : /mavros/cmd/arming (CommandBool)
      - Mode     : /mavros/set_mode (SetMode)
      - Optional : /mavros/local_position/pose          -> lấy pose hiện tại làm setpoint ban đầu
    """

    def __init__(self):
        super().__init__('offboard_pos_follower')

        # QoS gợi ý cho MAVROS2 (reliable, volatile, depth 10)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- Publishers / Subscribers ---
        # removed setpoint_position/local publisher; use raw setpoint only
        self.raw_sp_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', reliable_qos
        )
        self.pos_cmd_sub = self.create_subscription(
            PositionCommand, '/drone_0_planning/pos_cmd', self._pos_cmd_cb, reliable_qos
        )
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self._state_cb, reliable_qos
        )
        self.local_pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._local_pose_cb, reliable_qos
        )

        # --- Service clients ---
        self.arming_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')

        # --- Internal state ---
        self.current_state: Optional[State] = None
        self.current_pose: Optional[PoseStamped] = None
        self.latest_cmd: Optional[PoseStamped] = None
        self.latest_pos_cmd: Optional[PositionCommand] = None

        self.offboard_requested = False
        self.armed_requested = False

        # Timer phát setpoint liên tục (50 Hz ~ 0.02s)
        self.timer = self.create_timer(0.02, self._publish_setpoint_loop)

        # Sau khi node lên, chờ dịch vụ sẵn sàng rồi khởi động quy trình OFFBOARD
        self.create_timer(0.5, self._ensure_services_ready_once)

        self.init_done = False
        self.init_start_time = None

        self.get_logger().info("OffboardPosFollower started. Waiting for /pos_cmd ...")

    # ===== Callbacks =====
    def _state_cb(self, msg: State):
        self.current_state = msg

    def _local_pose_cb(self, msg: PoseStamped):
        # lưu pose hiện tại để dùng làm setpoint ban đầu nếu chưa có /pos_cmd
        self.current_pose = msg

    def _pos_cmd_cb(self, msg: PositionCommand):
        # Lưu PositionCommand để phát raw setpoint đầy đủ
        self.latest_pos_cmd = msg
        # Đồng thời dựng PoseStamped cho kênh position/local (tương thích)
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x = float(msg.position.x)
        sp.pose.position.y = float(msg.position.y)
        sp.pose.position.z = float(msg.position.z)
        import math
        half_yaw = float(msg.yaw) * 0.5
        sp.pose.orientation.x = 0.0
        sp.pose.orientation.y = 0.0
        sp.pose.orientation.z = math.sin(half_yaw)
        sp.pose.orientation.w = math.cos(half_yaw)
        self.latest_cmd = sp

    # ===== Service helpers =====
    def _ensure_services_ready_once(self):
        if self.init_done:
            return

        # Chờ services của MAVROS sẵn sàng
        if not self.arming_cli.service_is_ready():
            if not self.arming_cli.wait_for_service(timeout_sec=0.1):
                return
        if not self.set_mode_cli.service_is_ready():
            if not self.set_mode_cli.wait_for_service(timeout_sec=0.1):
                return

        # Khi services sẵn sàng, khởi động pha streaming setpoint trước OFFBOARD
        if self.init_start_time is None:
            self.init_start_time = time.time()
            self.get_logger().info("Services ready. Priming setpoints before OFFBOARD...")

        # Sau ~2 giây phát setpoint đều, ta yêu cầu ARM và OFFBOARD
        if time.time() - self.init_start_time > 2.0:
            self._try_arm()
            self._try_set_offboard()
            self.init_done = True

    def _try_arm(self):
        if self.current_state and self.current_state.armed:
            self.armed_requested = True
            return

        if not self.armed_requested:
            req = CommandBool.Request()
            req.value = True
            self.get_logger().info("Arming...")
            future = self.arming_cli.call_async(req)
            future.add_done_callback(self._arm_done)
            self.armed_requested = True

    def _arm_done(self, future):
        try:
            resp = future.result()
            if resp and resp.success:
                self.get_logger().info("Armed successfully.")
            else:
                self.get_logger().warn("Arming failed. Will retry.")
                self.armed_requested = False
        except Exception as e:
            self.get_logger().error(f"Arming service call failed: {e}")
            self.armed_requested = False

    def _try_set_offboard(self):
        if self.current_state and self.current_state.mode == "OFFBOARD":
            self.offboard_requested = True
            return

        if not self.offboard_requested:
            req = SetMode.Request()
            req.custom_mode = "OFFBOARD"
            self.get_logger().info("Switching to OFFBOARD...")
            future = self.set_mode_cli.call_async(req)
            future.add_done_callback(self._setmode_done)
            self.offboard_requested = True

    def _setmode_done(self, future):
        try:
            resp = future.result()
            # In MAVROS2, resp.mode_sent indicates if request sent successfully.
            if resp and resp.mode_sent:
                self.get_logger().info("OFFBOARD request sent.")
            else:
                self.get_logger().warn("OFFBOARD request failed. Will retry.")
                self.offboard_requested = False
        except Exception as e:
            self.get_logger().error(f"SetMode service call failed: {e}")
            self.offboard_requested = False

    # ===== Setpoint loop =====
    def _publish_setpoint_loop(self):
        """
        Luồng định kỳ 50 Hz:
          - Phát setpoint raw (PositionTarget) đầy đủ pos/vel/acc/yaw/yaw_rate.
          - Đồng thời phát PoseStamped để tương thích kênh position/local.
          - Nếu OFFBOARD rớt hoặc chưa ARM, thử lại nhẹ nhàng.
        """
        # 1) setpoint_raw/local
        sp_raw = PositionTarget()
        sp_raw.header.stamp = self.get_clock().now().to_msg()
        sp_raw.header.frame_id = "map"
        sp_raw.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        # Mặc định dùng đủ trường (type_mask=0). Nếu thiếu dữ liệu sẽ đặt mask tương ứng.
        sp_raw.type_mask = 0

        if self.latest_pos_cmd is not None:
            pc = self.latest_pos_cmd
            sp_raw.position.x = float(pc.position.x)
            sp_raw.position.y = float(pc.position.y)
            sp_raw.position.z = float(pc.position.z)

            sp_raw.velocity.x = float(pc.velocity.x)
            sp_raw.velocity.y = float(pc.velocity.y)
            sp_raw.velocity.z = float(pc.velocity.z)

            sp_raw.acceleration_or_force.x = float(pc.acceleration.x)
            sp_raw.acceleration_or_force.y = float(pc.acceleration.y)
            sp_raw.acceleration_or_force.z = float(pc.acceleration.z)

            sp_raw.yaw = float(pc.yaw)
            sp_raw.yaw_rate = float(pc.yaw_dot)
        elif self.current_pose is not None:
            # Giữ chỗ bằng vị trí hiện tại, bỏ qua vel/acc/yaw_rate
            sp_raw.position.x = float(self.current_pose.pose.position.x)
            sp_raw.position.y = float(self.current_pose.pose.position.y)
            sp_raw.position.z = float(self.current_pose.pose.position.z)
            sp_raw.type_mask = (
                PositionTarget.IGNORE_VX |
                PositionTarget.IGNORE_VY |
                PositionTarget.IGNORE_VZ |
                PositionTarget.IGNORE_AFX |
                PositionTarget.IGNORE_AFY |
                PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE
            )
        else:
            # Fallback: đứng ở (0,0,1)
            sp_raw.position.x = 0.0
            sp_raw.position.y = 0.0
            sp_raw.position.z = 1.0
            sp_raw.type_mask = (
                PositionTarget.IGNORE_VX |
                PositionTarget.IGNORE_VY |
                PositionTarget.IGNORE_VZ |
                PositionTarget.IGNORE_AFX |
                PositionTarget.IGNORE_AFY |
                PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE
            )

        self.raw_sp_pub.publish(sp_raw)

        # removed position/local publish for optimized raw offboard control

        # Giữ OFFBOARD & ARMED (nếu bị rớt mode do mất setpoint/chưa đủ tần số)
        if self.current_state:
            if self.current_state.mode != "OFFBOARD":
                # Thử lại OFFBOARD sau khi đã phát setpoint đều
                self._try_set_offboard()
            if not self.current_state.armed:
                self._try_arm()


def main():
    rclpy.init()
    node = OffboardPosFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down OffboardPosFollower...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
