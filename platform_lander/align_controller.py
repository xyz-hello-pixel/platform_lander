#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped, PointStamped
from std_msgs.msg import Bool
import time
from collections import deque

class AlignController(Node):
    def __init__(self):
        super().__init__('align_controller')
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('kp_xy', 0.002)   # pixels -> m/s scale, tune this
        self.declare_parameter('max_lin', 0.5)   # max linear speed m/s
        self.declare_parameter('threshold_px', 20)
        self.declare_parameter('stable_frames_needed', 5)

        self.width = self.get_parameter('frame_width').value
        self.height = self.get_parameter('frame_height').value
        self.kp = self.get_parameter('kp_xy').value
        self.max_lin = self.get_parameter('max_lin').value
        self.threshold = self.get_parameter('threshold_px').value
        self.stable_needed = self.get_parameter('stable_frames_needed').value

        self.center_sub = self.create_subscription(PointStamped, '/platform/center', self.center_cb, 10)
        self.vel_sub = self.create_subscription(TwistStamped, '/platform/velocity', self.vel_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.land_pub = self.create_publisher(Bool, '/platform/land', 10)

        self.last_center = None
        self.last_vel = None
        self.stable_count = 0

    def center_cb(self, msg: PointStamped):
        cx = msg.point.x
        cy = msg.point.y
        err_x = (cx - self.width/2.0)
        err_y = (cy - self.height/2.0)
        vx_cmd = -self.kp * err_x
        vy_cmd = -self.kp * err_y
        vx_cmd = max(-self.max_lin, min(self.max_lin, vx_cmd))
        vy_cmd = max(-self.max_lin, min(self.max_lin, vy_cmd))

        twist = Twist()
        twist.linear.x = vx_cmd  # forward/back mapped to x — adapt mapping as needed for drone
        twist.linear.y = vy_cmd
        twist.linear.z = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        if abs(err_x) < self.threshold and abs(err_y) < self.threshold:
            self.stable_count += 1
        else:
            self.stable_count = 0

        if self.stable_count >= self.stable_needed:
            b = Bool()
            b.data = True
            self.land_pub.publish(b)
            self.get_logger().info('Platform centered: publishing land signal')

    def vel_cb(self, msg: TwistStamped):
        self.last_vel = msg.twist

def main(args=None):
    rclpy.init(args=args)
    node = AlignController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
