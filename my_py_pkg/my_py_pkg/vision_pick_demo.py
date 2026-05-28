#!/usr/bin/env python3
import math
import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Float64MultiArray

from cv_bridge import CvBridge

import tf2_ros
from tf2_geometry_msgs import do_transform_point


class VisionPickDemo(Node):
    def __init__(self):
        super().__init__("vision_pick_demo")

        self.bridge = CvBridge()

        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None
        self.depth_frame = None

        self.rgb_sub = self.create_subscription(
            Image,
            "/camera/rgb/image",
            self.rgb_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            "/camera/depth/image",
            self.depth_callback,
            10
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            "/camera/depth/camera_info",
            self.info_callback,
            10
        )

        self.arm_pub = self.create_publisher(
            Float64MultiArray,
            "/arm_controller/commands",
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 机械臂参数
        self.arm_base_x = 0.1
        self.arm_base_y = 0.0
        self.arm_base_z = 0.2

        self.L1 = 0.2
        self.L2 = 0.2

        self.q1_min = 0.0
        self.q1_max = 9.0 * math.pi / 20.0
        self.q2_min = 0.0
        self.q2_max = 9.0 * math.pi / 20.0

        self.gripper_open = 0.03
        self.gripper_close = 0.0

        # 小车自动靠近参数
        # 目标：让红色方块最终出现在 base_link 前方约 0.42m 处，y 接近 0
        self.desired_object_x = 0.42
        self.x_tolerance = 0.025
        self.y_tolerance = 0.025

        self.k_linear = 0.45
        self.k_angular = 1.2

        self.max_linear = 0.05
        self.max_angular = 0.25

        self.min_linear = -0.03

        self.has_picked = False
        self.is_picking = False

        # 10Hz 控制频率
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Vision pick demo with visual approach started.")

    def rgb_callback(self, msg: Image):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"RGB convert failed: {e}")

    def depth_callback(self, msg: Image):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_frame = msg.header.frame_id
        except Exception as e:
            self.get_logger().error(f"Depth convert failed: {e}")

    def info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def stop_robot(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

    def get_valid_depth(self, u, v, window=6):
        if self.depth_image is None:
            return None

        h, w = self.depth_image.shape[:2]

        u_min = max(0, u - window)
        u_max = min(w, u + window + 1)
        v_min = max(0, v - window)
        v_max = min(h, v + window + 1)

        patch = self.depth_image[v_min:v_max, u_min:u_max].astype(np.float32)

        valid = patch[np.isfinite(patch)]
        valid = valid[valid > 0.05]

        if valid.size == 0:
            return None

        return float(np.median(valid))

    def detect_red_object(self):
        if self.rgb_image is None or self.depth_image is None or self.camera_info is None:
            return None

        hsv = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 80, 80])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 80, 80])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < 150:
            return None

        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None

        u = int(M["m10"] / M["m00"])
        v = int(M["m01"] / M["m00"])

        depth = self.get_valid_depth(u, v, window=6)
        if depth is None:
            self.get_logger().warn(f"Red object detected at pixel ({u}, {v}), but depth invalid.")
            return None

        K = self.camera_info.k
        fx = K[0]
        fy = K[4]
        cx = K[2]
        cy = K[5]

        # optical frame 下的点：
        # X 右，Y 下，Z 前
        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy
        Z = depth

        p_camera = PointStamped()
        p_camera.header.stamp = rclpy.time.Time().to_msg()
        p_camera.header.frame_id = self.depth_frame
        p_camera.point.x = float(X)
        p_camera.point.y = float(Y)
        p_camera.point.z = float(Z)

        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                p_camera.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )

            p_base = do_transform_point(p_camera, transform)

            return p_base.point.x, p_base.point.y, p_base.point.z

        except Exception as e:
            self.get_logger().warn(f"TF to base_link failed: {e}")
            return None

    def solve_ik(self, x, y, z):
        dx = x - self.arm_base_x
        dy = y - self.arm_base_y
        dz = z - self.arm_base_z

        q0 = math.atan2(dy, dx)

        r = math.sqrt(dx * dx + dy * dy)
        h = dz

        d = math.sqrt(r * r + h * h)

        if d > self.L1 + self.L2:
            raise ValueError(f"目标太远，机械臂够不到: d={d:.3f}")

        if d < abs(self.L1 - self.L2):
            raise ValueError(f"目标太近，机械臂够不到: d={d:.3f}")

        cos_q2 = (d * d - self.L1 * self.L1 - self.L2 * self.L2) / (2.0 * self.L1 * self.L2)
        cos_q2 = max(-1.0, min(1.0, cos_q2))

        q2 = math.acos(cos_q2)

        # 你的机械臂初始主要沿 z 方向，使用相对竖直方向的角度
        gamma = math.atan2(r, h)
        beta = math.atan2(
            self.L2 * math.sin(q2),
            self.L1 + self.L2 * math.cos(q2)
        )

        q1 = gamma - beta

        if not (self.q1_min <= q1 <= self.q1_max):
            raise ValueError(f"base_arm_arm1_joint 超限: q1={q1:.3f}")

        if not (self.q2_min <= q2 <= self.q2_max):
            raise ValueError(f"arm1_arm2_joint 超限: q2={q2:.3f}")

        return q0, q1, q2

    def send_arm_command(self, q0, q1, q2, gripper):
        msg = Float64MultiArray()
        msg.data = [
            float(q0),
            float(q1),
            float(q2),
            float(gripper),
            float(gripper),
        ]

        self.arm_pub.publish(msg)

        self.get_logger().info(
            f"Arm command: q0={q0:.3f}, q1={q1:.3f}, q2={q2:.3f}, gripper={gripper:.3f}"
        )

    def visual_approach(self, obj):
        x, y, z = obj

        x_error = x - self.desired_object_x
        y_error = y

        aligned_x = abs(x_error) < self.x_tolerance
        aligned_y = abs(y_error) < self.y_tolerance

        self.get_logger().info(
            f"Object base_link: x={x:.3f}, y={y:.3f}, z={z:.3f} | "
            f"x_error={x_error:.3f}, y_error={y_error:.3f}"
        )

        if aligned_x and aligned_y:
            self.stop_robot()
            self.get_logger().info("Robot is in grasping range. Stop and start picking.")
            return True

        cmd = Twist()

        # x 太远就前进，太近就后退
        cmd.linear.x = self.clamp(
            self.k_linear * x_error,
            self.min_linear,
            self.max_linear
        )

        # y 为正说明物体在机器人左边，正角速度左转，把物体转到中间
        cmd.angular.z = self.clamp(
            self.k_angular * y_error,
            -self.max_angular,
            self.max_angular
        )

        # 如果左右偏差很大，先少前进，优先对准
        if abs(y_error) > 0.08:
            cmd.linear.x *= 0.3

        self.cmd_pub.publish(cmd)

        return False

    def pick_sequence(self, obj):
        self.is_picking = True
        self.stop_robot()
        time.sleep(0.3)

        x, y, z = obj

        # 这里可以按实际夹爪位置微调
        target_x = x - 0.02
        target_y = y
        target_z = z - 0.02

        self.get_logger().info(
            f"Pick target: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}"
        )

        try:
            q0, q1, q2 = self.solve_ik(target_x, target_y, target_z)
        except ValueError as e:
            self.get_logger().error(str(e))
            self.is_picking = False
            return

        self.has_picked = True

        self.get_logger().info("Open gripper")
        self.send_arm_command(0.0, 0.0, 0.0, self.gripper_open)
        time.sleep(1.0)

        self.get_logger().info("Move to object")
        self.send_arm_command(q0, q1, q2, self.gripper_open)
        time.sleep(2.0)

        self.get_logger().info("Close gripper")
        self.send_arm_command(q0, q1, q2, self.gripper_close)
        time.sleep(2.0)

        lift_z = target_z + 0.1

        try:
            q0_lift, q1_lift, q2_lift = self.solve_ik(target_x, target_y, lift_z)
        except ValueError as e:
            self.get_logger().warn(f"Lift pose invalid, use current pose: {e}")
            q0_lift, q1_lift, q2_lift = q0, q1, q2

        self.get_logger().info("Lift object")
        self.send_arm_command(q0_lift, q1_lift, q2_lift, self.gripper_close)
        time.sleep(2.0)

        self.stop_robot()
        self.get_logger().info("Vision-guided pick finished.")
        self.is_picking = False

    def control_loop(self):
        if self.has_picked or self.is_picking:
            return

        obj = self.detect_red_object()

        if obj is None:
            self.stop_robot()
            self.get_logger().warn("No red object detected. Robot stopped.")
            return

        ready_to_pick = self.visual_approach(obj)

        if ready_to_pick:
            # 停车后重新检测一次，避免最后一帧误差
            time.sleep(0.2)
            obj2 = self.detect_red_object()
            if obj2 is None:
                self.get_logger().warn("Object lost before picking.")
                return

            self.pick_sequence(obj2)


def main(args=None):
    rclpy.init(args=args)
    node = VisionPickDemo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()