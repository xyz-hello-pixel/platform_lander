#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, TwistStamped
from std_msgs.msg import Header
import cv2
import numpy as np
from cv_bridge import CvBridge
import time

class VisionTracker(Node):
    def __init__(self):
        super().__init__('vision_tracker')
        self.declare_parameter('use_ros_image', False)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('show_window', True)
        self.bridge = CvBridge()
        self.last_center = None
        self.last_time = None

        use_ros_image = self.get_parameter('use_ros_image').value
        camera_topic = self.get_parameter('camera_topic').value
        cam_id = self.get_parameter('camera_id').value
        self.show_window = self.get_parameter('show_window').value

        self.pub_center = self.create_publisher(PointStamped, '/platform/center', 10)
        self.pub_vel = self.create_publisher(TwistStamped, '/platform/velocity', 10)

        if use_ros_image:
            self.sub = self.create_subscription(Image, camera_topic, self.image_callback, 10)
            self.get_logger().info(f'Using ROS image topic: {camera_topic}')
        else:
            self.cap = cv2.VideoCapture(cam_id)
            if not self.cap.isOpened():
                self.get_logger().error('Failed to open camera')
                raise RuntimeError('Camera open failed')
            self.timer = self.create_timer(0.03, self.timer_cb)

    def find_circle(self, gray):
        gray_blur = cv2.medianBlur(gray, 5)
        circles = cv2.HoughCircles(gray_blur, cv2.HOUGH_GRADIENT, dp=1.2, minDist=100,
                                   param1=100, param2=30, minRadius=10, maxRadius=0)
        if circles is None:
            return None
        circles = np.uint16(np.around(circles))
        x, y, r = circles[0,0]
        return (int(x), int(y), int(r))

    def publish_center_vel(self, cx, cy, r, stamp=None):
        now = time.time()
        if self.last_time is None:
            vx = 0.0; vy = 0.0
        else:
            dt = max(1e-3, now - self.last_time)
            vx = (cx - self.last_center[0]) / dt
            vy = (cy - self.last_center[1]) / dt
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        pt = PointStamped()
        pt.header = hdr
        pt.point.x = float(cx)
        pt.point.y = float(cy)
        pt.point.z = float(r)
        twist = TwistStamped()
        twist.header = hdr
        twist.twist.linear.x = vx
        twist.twist.linear.y = vy
        twist.twist.linear.z = 0.0
        self.pub_center.publish(pt)
        self.pub_vel.publish(twist)
        self.last_center = (cx, cy)
        self.last_time = now

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        c = self.find_circle(gray)
        if c:
            cx, cy, r = c
            self.publish_center_vel(cx, cy, r)
            cv2.circle(frame, (cx, cy), r, (0,255,0), 2)
            cv2.circle(frame, (cx, cy), 2, (0,0,255), 3)
        if self.show_window:
            cv2.imshow('vision', frame)
            cv2.waitKey(1)

    def image_callback(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        c = self.find_circle(gray)
        if c:
            cx, cy, r = c
            self.publish_center_vel(cx, cy, r)
        if self.show_window:
            if c:
                cv2.circle(cv_img, (cx, cy), r, (0,255,0), 2)
                cv2.circle(cv_img, (cx, cy), 2, (0,0,255), 3)
            cv2.imshow('vision', cv_img)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisionTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
