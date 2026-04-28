#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PoseStamped
from cv_bridge import CvBridge
import numpy as np
import cv2
from pathlib import Path
import tf2_ros
import message_filters
from ultralytics import YOLO
from ament_index_python import get_package_share_directory

from handle_estimation.utils import (
    compute_stable_orientation,
    smooth_orientation,
    axes_to_euler,
    apply_ema_smoothing,
)


class HandlePoseEstimationNode(Node):
    def __init__(self):
        super().__init__('handle_pose_estimation')

        self.declare_parameter('model_path', 'handler_model.pt')
        self.declare_parameter('handle_class', 0)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('ema_alpha', 0.3)
        self.declare_parameter('ema_orientation', 0.15)
        self.declare_parameter('target_fps', 15)
        self.declare_parameter('min_eigenvalue_ratio', 3.0)
        self.declare_parameter('frame_id', 'handle_frame')
        self.declare_parameter('base_frame', 'camera_color_optical_frame')

        self.model_path = self.get_parameter('model_path').value
        self.handle_class = self.get_parameter('handle_class').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.ema_alpha = self.get_parameter('ema_alpha').value
        self.ema_orientation = self.get_parameter('ema_orientation').value
        self.target_fps = self.get_parameter('target_fps').value
        self.min_eigenvalue_ratio = self.get_parameter('min_eigenvalue_ratio').value
        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value

        self.get_logger().info(f'Loading model: {self.model_path}')
        if not Path(self.model_path).exists():
            pkg_share = get_package_share_directory('handle_estimation')
            if pkg_share:
                model_path = Path(pkg_share) / 'model' / self.model_path
                if model_path.exists():
                    self.model_path = str(model_path)
                    self.get_logger().info(f'Found model at: {self.model_path}')
            if not Path(self.model_path).exists():
                src_path = Path(__file__).parent.parent / 'model' / self.model_path
                if src_path.exists():
                    self.model_path = str(src_path)
                    self.get_logger().info(f'Found model at: {self.model_path}')
            if not Path(self.model_path).exists():
                self.get_logger().error(f'Model not found: {self.model_path}')
                return

        self.model = YOLO(self.model_path)

        self.bridge = CvBridge()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.color_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/depth/image_rect_raw')

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10)

        self.overlay_pub = self.create_publisher(Image, '/handle_estimation/overlay', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/handle_estimation/pose', 10)

        sync_policy = message_filters.TimeSynchronizer(
            [self.color_sub, self.depth_sub],
            10)
        sync_policy.registerCallback(self.image_callback)

        self.camera_info_received = False
        self.camera_model = None
        self.prev_axes = None
        self.pose_history = []

        self.get_logger().info('Handle pose estimation node started')

    def camera_info_callback(self, msg):
        if self.camera_info_received:
            return

        self.K = np.array(msg.k).reshape(3, 3)
        self.D = np.array(msg.d)
        self.P = np.array(msg.p).reshape(3, 4)
        self.width = msg.width
        self.height = msg.height
        self.camera_info_received = True
        self.get_logger().info('Camera intrinsics received')

    def deproject_pixel_to_point(self, u, v, depth):
        if not self.camera_info_received:
            return None

        x = (u - self.P[0, 2]) / self.P[0, 0]
        y = (v - self.P[1, 2]) / self.P[1, 1]

        x = x * depth
        y = y * depth
        
        return [x, y, depth]

    def image_callback(self, color_msg, depth_msg):
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        h, w = color_image.shape[:2]
        depth_h, depth_w = depth_image.shape[:2]
        if depth_h != h or depth_w != w:
            depth_image = cv2.resize(depth_image, (w, h), interpolation=cv2.INTER_NEAREST)

        results = self.model.predict(color_image, verbose=False)
        result = results[0]

        overlay = color_image.copy()

        pose_data = None
        detected = False

        if result.masks is not None and len(result.masks) > 0:
            masks = result.masks.data
            classes = result.boxes.cls.cpu().numpy().astype(int)
            confs = result.boxes.conf.cpu().numpy()

            for mask, cls, conf in zip(masks, classes, confs):
                if cls != self.handle_class or conf < self.confidence_threshold:
                    continue

                detected = True

                mask_np = mask.cpu().numpy()
                mask_resized = cv2.resize(mask_np, (w, h), interpolation=cv2.INTER_LINEAR)
                mask_binary = (mask_resized > 0.5).astype(np.uint8)

                timestamp = depth_msg.header.stamp

                points = []
                for py in range(0, h, 2):
                    for px in range(0, w, 2):
                        if mask_binary[py, px] > 0:
                            depth = depth_image[py, px]
                            if depth > 0 and depth < 10000:
                                d = depth / 1000.0
                                point = self.deproject_pixel_to_point(px, py, d)
                                if point:
                                    points.append(point)

                if len(points) < 20:
                    continue

                points = np.array(points)

                axes, centroid = compute_stable_orientation(
                    points, self.prev_axes, self.min_eigenvalue_ratio)

                if self.prev_axes is not None:
                    axes = smooth_orientation(axes, self.prev_axes, self.ema_orientation)

                self.prev_axes = axes

                roll, pitch, yaw = axes_to_euler(axes)

                new_pose = [
                    float(centroid[0]),
                    float(centroid[1]),
                    float(centroid[2]),
                    float(roll),
                    float(pitch),
                    float(yaw)
                ]
                pose_data = apply_ema_smoothing(self.pose_history, new_pose, self.ema_alpha)
                self.pose_history.append(pose_data)
                if len(self.pose_history) > 30:
                    self.pose_history.pop(0)

                color = (0, 255, 0)

                colored_mask = np.zeros_like(color_image)
                colored_mask[mask_binary == 1] = color
                overlay = cv2.addWeighted(overlay, 1.0, colored_mask, 0.5, 0)

                contours, _ = cv2.findContours(
                    mask_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(overlay, contours, -1, (255, 255, 255), 2)

                self.publish_tf(pose_data, timestamp)
                self.publish_pose(pose_data, timestamp)

        try:
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, 'bgr8')
            overlay_msg.header.stamp = depth_msg.header.stamp
            overlay_msg.header.frame_id = self.base_frame
            self.overlay_pub.publish(overlay_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish overlay: {e}')

    def publish_tf(self, pose_data, timestamp):
        if not pose_data:
            return

        x, y, z, roll, pitch, yaw = pose_data

        transform = TransformStamped()
        transform.header.stamp = timestamp
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.frame_id

        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.translation.z = float(z)

        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        transform.transform.rotation.x = sr * cp * cy - cr * sp * sy
        transform.transform.rotation.y = cr * sp * cy + sr * cp * sy
        transform.transform.rotation.z = cr * cp * sy - sr * sp * cy
        transform.transform.rotation.w = cr * cp * cy + sr * sp * sy

        self.tf_broadcaster.sendTransform(transform)

    def publish_pose(self, pose_data, timestamp):
        if not pose_data:
            return

        x, y, z, roll, pitch, yaw = pose_data

        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = self.base_frame

        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = float(z)

        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        pose_msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose_msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose_msg.pose.orientation.z = cr * cp * sy - sr * sp * cy
        pose_msg.pose.orientation.w = cr * cp * cy + sr * sp * sy

        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HandlePoseEstimationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()