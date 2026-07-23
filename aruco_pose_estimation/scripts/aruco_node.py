#!/usr/bin/env python3
"""
ROS2 wrapper code taken from:
https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco/tree/main

This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /output/aruco/markers (aruco_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding
       marker ids.

    /aruco_image (sensor_msgs.msg.Image)
       Annotated image with marker locations and ids, with markers drawn on it

Parameters:
    marker_size - size of the markers in meters (default .065)
    aruco_dictionary_id - dictionary that was used to generate markers (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /camera/color/image_raw)
    camera_info_topic - camera info topic to subscribe to (default /camera/camera_info)
    camera_frame - camera optical frame to use (default "camera_depth_optical_frame")
    detected_markers_topic - topic to publish detected markers (default /output/aruco/markers)
    markers_visualization_topic - topic to publish markers visualization (default /aruco_poses)
    output_image_topic - topic to publish annotated image (default /aruco_image)

Author: Simone Giampà
Version: 2024-01-29

"""

# ROS2 imports
import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
import message_filters

# Python imports
import threading
import time

import numpy as np
import cv2

# Local imports for custom defined functions
from aruco_pose_estimation.utils import ARUCO_DICT
from aruco_pose_estimation.pose_estimation import pose_estimation
from aruco_pose_estimation.image_processing import (
    extract_nv12_y_plane,
    scale_camera_matrix,
)
from aruco_pose_estimation.camera_calibration import (
    SUPPORTED_DISTORTION_LENGTHS,
    load_vita_camera_calibration,
)

# ROS2 message imports
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class ArucoNode(rclpy.node.Node):
    IMAGE_TOPIC_STATUS_PERIOD_SECONDS = 1.0
    IMAGE_TOPIC_RECEIVING_LOG_LIMIT = 3
    POSE_LOG_PERIOD_SECONDS = 0.5
    POSE_LOG_MAX_AGE_SECONDS = 2.0

    def __init__(self):
        super().__init__("aruco_node")

        self.initialize_parameters()
        self.image_geometry_logged = False
        self.input_image_geometry_logged = False

        image_monitor_started_at = time.monotonic()
        self.image_monitor_started_at = image_monitor_started_at
        self.image_monitor_last_sample_at = image_monitor_started_at
        self.image_monitor_last_frame_at = None
        self.image_monitor_total_frames = 0
        self.image_monitor_last_sample_total_frames = 0
        self.image_monitor_sample_first_frame_at = None
        self.image_monitor_sample_last_frame_at = None
        self.image_monitor_receiving_log_count = 0
        self.image_monitor_complete = False
        self.image_monitor_timer = None

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(self.dictionary_id_name)
            # check if the dictionary_id is a valid dictionary inside ARUCO_DICT values
            if dictionary_id not in ARUCO_DICT.values():
                raise AttributeError
        except AttributeError:
            self.get_logger().error(
                "bad aruco_dictionary_id: {}".format(self.dictionary_id_name)
            )
            options = "\n".join([s for s in ARUCO_DICT])
            self.get_logger().error("valid options: {}".format(options))

        # Camera calibration is activated by the first valid configured source.
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None
        self.camera_calibration_source = None
        self.info_sub = None
        self.camera_info_wait_timer = None
        self.camera_info_wait_started_at = time.monotonic()
        self.camera_file_fallback_attempted = False

        # Set up subscriptions to the camera info and camera image topics

        if self.camera_calibration_mode in ("auto", "topic"):
            self.info_sub = self.create_subscription(
                CameraInfo,
                self.info_topic,
                self.info_callback,
                qos_profile_sensor_data,
            )

        # select the type of input to use for the pose estimation
        if (bool(self.use_depth_input)):
            # use both rgb and depth image topics for the pose estimation

            # create a message filter to synchronize the image and depth image topics
            self.image_sub = message_filters.Subscriber(self, Image, self.image_topic,
                                                        qos_profile=qos_profile_sensor_data)
            self.image_sub.registerCallback(self.observe_input_image)
            self.depth_image_sub = message_filters.Subscriber(self, Image, self.depth_image_topic,
                                                              qos_profile=qos_profile_sensor_data)

            # create synchronizer between the 2 topics using message filters and approximate time policy
            # slop is the maximum time difference between messages that are considered synchronized
            self.synchronizer = message_filters.ApproximateTimeSynchronizer(
                [self.image_sub, self.depth_image_sub], queue_size=10, slop=0.05
            )
            self.synchronizer.registerCallback(self.rgb_depth_sync_callback)
        else:
            # rely only on the rgb image topic for the pose estimation

            # create a subscription to the image topic
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self.image_callback, qos_profile_sensor_data
            )

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, self.markers_visualization_topic, 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, self.detected_markers_topic, 10)
        self.image_pub = self.create_publisher(Image, self.output_image_topic, 10)

        self.latest_pose_log_text = None
        self.latest_pose_update_time = None
        self.pose_log_lock = threading.Lock()
        self.pose_log_stop_event = threading.Event()
        self.context.on_shutdown(self.pose_log_stop_event.set)

        # code for updated version of cv2 (4.7.0)
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)

        if self.camera_calibration_mode == "file":
            if not self.load_camera_calibration_file():
                raise RuntimeError(
                    "file calibration mode could not load the configured file"
                )
        else:
            self.camera_info_wait_timer = self.create_timer(
                1.0, self.log_camera_info_waiting
            )

        self.image_monitor_timer = self.create_timer(
            self.IMAGE_TOPIC_STATUS_PERIOD_SECONDS,
            self.log_image_topic_status,
        )

        self.pose_log_thread = threading.Thread(
            target=self.pose_log_loop,
            name="aruco_pose_terminal_log",
            daemon=True,
        )
        self.pose_log_thread.start()

        # old code version
        # self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        # self.aruco_parameters = cv2.aruco.DetectorParameters_create()

    def log_camera_info_waiting(self):
        """Report topic wait state and trigger auto-mode file fallback."""

        if self.info_msg is not None:
            return

        elapsed = time.monotonic() - self.camera_info_wait_started_at
        elapsed_seconds = max(1, int(elapsed))
        self.get_logger().info(
            "Waiting for camera information... "
            f"已等待 {elapsed_seconds} 秒"
        )

        if (
            self.camera_calibration_mode == "auto"
            and elapsed >= self.camera_info_timeout_sec
            and not self.camera_file_fallback_attempted
        ):
            self.camera_file_fallback_attempted = True
            self.get_logger().info(
                f"{self.camera_info_timeout_sec:g} 秒内未收到内参话题，"
                "开始读取本地内参文件。"
            )
            if not self.load_camera_calibration_file():
                self.get_logger().error(
                    "本地内参文件读取失败，将继续等待内参话题。"
                )

    def info_callback(self, info_msg):
        """Accept the first valid CameraInfo message."""

        if self.info_msg is not None:
            return

        try:
            self.activate_camera_calibration(info_msg, "内参话题")
        except ValueError as error:
            self.get_logger().error(f"收到的摄像头内参无效：{error}")

    @staticmethod
    def validate_camera_info(info_msg: CameraInfo):
        """Validate CameraInfo fields used by solvePnP."""

        if info_msg.width <= 0 or info_msg.height <= 0:
            raise ValueError("width and height must be positive")

        camera_matrix = np.asarray(info_msg.k, dtype=np.float64)
        if camera_matrix.size != 9 or not np.all(np.isfinite(camera_matrix)):
            raise ValueError("K must contain 9 finite values")
        camera_matrix = camera_matrix.reshape(3, 3)
        if camera_matrix[0, 0] <= 0.0 or camera_matrix[1, 1] <= 0.0:
            raise ValueError("fx and fy must be positive")

        distortion = np.asarray(info_msg.d, dtype=np.float64).reshape(-1)
        allowed_lengths = SUPPORTED_DISTORTION_LENGTHS | {0}
        if distortion.size not in allowed_lengths:
            expected = ", ".join(
                str(length) for length in sorted(allowed_lengths)
            )
            raise ValueError(
                f"D contains {distortion.size} values; expected one of: {expected}"
            )
        if not np.all(np.isfinite(distortion)):
            raise ValueError("D must contain only finite values")

        return camera_matrix, distortion

    def activate_camera_calibration(
        self,
        info_msg: CameraInfo,
        source_description: str,
    ) -> bool:
        """Activate the first valid topic or file calibration."""

        if self.info_msg is not None:
            return False

        camera_matrix, distortion = self.validate_camera_info(info_msg)
        self.info_msg = info_msg
        self.intrinsic_mat = camera_matrix
        self.distortion = distortion
        self.camera_calibration_source = source_description

        if self.camera_info_wait_timer is not None:
            self.camera_info_wait_timer.cancel()

        if self.info_sub is not None:
            info_sub = self.info_sub
            self.info_sub = None
            self.destroy_subscription(info_sub)

        self.get_logger().info(
            f"已经获得摄像头内参！来源：{source_description}"
        )
        self.get_logger().info("摄像头内参矩阵为：{}".format(self.intrinsic_mat))
        self.get_logger().info("摄像头畸变系数为：{}".format(self.distortion))
        self.get_logger().info(
            "摄像头硬件分辨率为：{}x{}".format(
                self.info_msg.width, self.info_msg.height
            )
        )
        return True

    def load_camera_calibration_file(self) -> bool:
        """Load configured local calibration and activate it as CameraInfo."""

        try:
            calibration = load_vita_camera_calibration(
                self.camera_calibration_file,
                self.camera_calibration_label,
            )
        except ValueError as error:
            self.get_logger().error(f"读取本地摄像头内参失败：{error}")
            return False

        info_msg = CameraInfo()
        info_msg.header.frame_id = self.camera_frame or calibration.label
        info_msg.width = calibration.width
        info_msg.height = calibration.height
        info_msg.distortion_model = calibration.distortion_model
        info_msg.d = list(calibration.distortion)
        info_msg.k = list(calibration.camera_matrix)
        info_msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        fx = calibration.camera_matrix[0]
        cx = calibration.camera_matrix[2]
        fy = calibration.camera_matrix[4]
        cy = calibration.camera_matrix[5]
        info_msg.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        return self.activate_camera_calibration(
            info_msg,
            (
                f"本地文件 {self.camera_calibration_file}，"
                f"摄像头标签 {self.camera_calibration_label}"
            ),
        )

    def image_callback(self, img_msg: Image):
        self.observe_input_image(img_msg)

        if self.info_msg is None:
            return

        try:
            # NV12 stores its full-resolution luminance first, followed by UV data.
            # ArUco only needs luminance, so avoid an unnecessary RGB conversion.
            if img_msg.encoding.lower() == "nv12":
                cv_image = extract_nv12_y_plane(
                    img_msg.data,
                    width=img_msg.width,
                    height=img_msg.height,
                    step=img_msg.step,
                )
            else:
                cv_image = self.get_cv_bridge().imgmsg_to_cv2(
                    img_msg, desired_encoding="mono8"
                )

            source_height, source_width = cv_image.shape[:2]
            if (source_width, source_height) != (
                self.resize_width,
                self.resize_height,
            ):
                cv_image = cv2.resize(
                    cv_image,
                    (self.resize_width, self.resize_height),
                    interpolation=cv2.INTER_LINEAR,
                )

            processing_intrinsic_mat = scale_camera_matrix(
                self.intrinsic_mat,
                calibration_width=self.info_msg.width,
                calibration_height=self.info_msg.height,
                image_width=self.resize_width,
                image_height=self.resize_height,
            )
        except (ValueError, TypeError, cv2.error) as error:
            self.get_logger().error(f"Failed to prepare input image: {error}")
            return

        if not self.image_geometry_logged:
            self.get_logger().info(
                "图像处理过程为："
                f"{source_width}x{source_height}（{img_msg.encoding}）-> "
                f"{self.resize_width}x{self.resize_height}（mono8 灰度图）"
            )
            self.get_logger().info(
                "图像处理使用的内参矩阵为：{}".format(
                    processing_intrinsic_mat
                )
            )
            self.image_geometry_logged = True

        # create the ArucoMarkers and PoseArray messages
        markers = ArucoMarkers()
        pose_array = PoseArray()

        # Set the frame id and timestamp for the markers and pose array
        if self.camera_frame == "":
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame

        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        """
        # OVERRIDE: use calibrated intrinsic matrix and distortion coefficients
        self.intrinsic_mat = np.reshape([615.95431, 0., 325.26983,
                                         0., 617.92586, 257.57722,
                                         0., 0., 1.], (3, 3))
        self.distortion = np.array([0.142588, -0.311967, 0.003950, -0.006346, 0.000000])
        """
        
        # call the pose estimation function
        frame, pose_array, markers = pose_estimation(rgb_frame=cv_image, depth_frame=None,
                                                     aruco_detector=self.aruco_detector,
                                                     marker_size=self.marker_size, matrix_coefficients=processing_intrinsic_mat,
                                                     distortion_coefficients=self.distortion, pose_array=pose_array, markers=markers)

        # if some markers are detected
        if len(markers.marker_ids) > 0:
            # Publish the results with the poses and markes positions
            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)
            self.update_latest_pose_log(markers)
        else:
            self.clear_latest_pose_log()

        # publish the image frame with computed markers positions over the image
        output_msg = self.rgb_frame_to_image_msg(frame, img_msg.header)
        self.image_pub.publish(output_msg)

    @staticmethod
    def get_cv_bridge():
        """Load cv_bridge only for non-NV12 compatibility paths."""

        from cv_bridge import CvBridge

        return CvBridge()

    @staticmethod
    def rgb_frame_to_image_msg(frame: np.ndarray, header) -> Image:
        """Create an rgb8 ROS Image without relying on cv_bridge."""

        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError(
                f"Expected an RGB frame with shape HxWx3, got {frame.shape}"
            )

        contiguous_frame = np.ascontiguousarray(frame, dtype=np.uint8)
        output_msg = Image()
        output_msg.header = header
        output_msg.height = contiguous_frame.shape[0]
        output_msg.width = contiguous_frame.shape[1]
        output_msg.encoding = "rgb8"
        output_msg.is_bigendian = 0
        output_msg.step = contiguous_frame.shape[1] * 3
        output_msg.data = contiguous_frame.tobytes()
        return output_msg

    def depth_image_callback(self, depth_msg: Image):
        if self.info_msg is None:
            return

    def rgb_depth_sync_callback(self, rgb_msg: Image, depth_msg: Image):
        if self.info_msg is None:
            return

        # convert the image messages to cv2 format
        bridge = self.get_cv_bridge()
        cv_depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
        cv_image = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")

        # create the ArucoMarkers and PoseArray messages
        markers = ArucoMarkers()
        pose_array = PoseArray()

        # Set the frame id and timestamp for the markers and pose array
        if self.camera_frame == "":
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame

        markers.header.stamp = rgb_msg.header.stamp
        pose_array.header.stamp = rgb_msg.header.stamp

        # call the pose estimation function
        frame, pose_array, markers = pose_estimation(rgb_frame=cv_image, depth_frame=cv_depth_image,
                                                     aruco_detector=self.aruco_detector,
                                                     marker_size=self.marker_size, matrix_coefficients=self.intrinsic_mat,
                                                     distortion_coefficients=self.distortion, pose_array=pose_array, markers=markers)

        # if some markers are detected
        if len(markers.marker_ids) > 0:
            # Publish the results with the poses and markes positions
            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)
            self.update_latest_pose_log(markers)
        else:
            self.clear_latest_pose_log()

        # publish the image frame with computed markers positions over the image
        self.image_pub.publish(self.rgb_frame_to_image_msg(frame, rgb_msg.header))

    def observe_input_image(self, image_msg: Image):
        """Record an input image independently of calibration and decoding."""

        if not self.image_monitor_complete:
            received_at = time.monotonic()
            if self.image_monitor_sample_first_frame_at is None:
                self.image_monitor_sample_first_frame_at = received_at
            self.image_monitor_sample_last_frame_at = received_at
            self.image_monitor_total_frames += 1
            self.image_monitor_last_frame_at = received_at

        self.log_input_image_geometry(image_msg)

    def log_image_topic_status(self):
        """Report startup image availability and three receive-rate samples."""

        if self.image_monitor_complete:
            return

        now = time.monotonic()
        sample_elapsed = max(now - self.image_monitor_last_sample_at, 1e-9)
        received_frames = (
            self.image_monitor_total_frames
            - self.image_monitor_last_sample_total_frames
        )

        if received_frames <= 0:
            self.image_monitor_last_sample_at = now
            self.image_monitor_last_sample_total_frames = (
                self.image_monitor_total_frames
            )
            self.image_monitor_sample_first_frame_at = None
            self.image_monitor_sample_last_frame_at = None
            wait_started_at = (
                self.image_monitor_last_frame_at
                if self.image_monitor_last_frame_at is not None
                else self.image_monitor_started_at
            )
            waited_seconds = max(1, int(now - wait_started_at))
            self.get_logger().info(
                f"等待“{self.image_topic}”图像数据... "
                f"已经等待了{waited_seconds}秒"
            )
            self.image_monitor_receiving_log_count = 0
            return

        first_frame_at = self.image_monitor_sample_first_frame_at
        last_frame_at = self.image_monitor_sample_last_frame_at
        if (
            received_frames == 1
            and first_frame_at is not None
            and now - first_frame_at
            < self.IMAGE_TOPIC_STATUS_PERIOD_SECONDS
        ):
            return

        if (
            received_frames >= 2
            and first_frame_at is not None
            and last_frame_at is not None
            and last_frame_at > first_frame_at
        ):
            frequency_hz = (received_frames - 1) / (
                last_frame_at - first_frame_at
            )
        else:
            frequency_hz = received_frames / sample_elapsed

        self.image_monitor_last_sample_at = now
        self.image_monitor_last_sample_total_frames = (
            self.image_monitor_total_frames
        )
        self.image_monitor_sample_first_frame_at = None
        self.image_monitor_sample_last_frame_at = None
        self.get_logger().info(
            f"在持续接收“{self.image_topic}”图像数据，"
            f"频率为：{frequency_hz:.2f} Hz"
        )
        self.image_monitor_receiving_log_count += 1

        if (
            self.image_monitor_receiving_log_count
            >= self.IMAGE_TOPIC_RECEIVING_LOG_LIMIT
        ):
            self.image_monitor_complete = True
            if self.image_monitor_timer is not None:
                self.image_monitor_timer.cancel()

    def log_input_image_geometry(self, image_msg: Image):
        """Log the actual dimensions carried by the first input image."""

        if self.input_image_geometry_logged:
            return

        self.get_logger().info(
            "输入的图像话题尺寸为："
            f"{image_msg.width}x{image_msg.height}（{image_msg.encoding}）"
        )
        self.input_image_geometry_logged = True

    def update_latest_pose_log(self, markers: ArucoMarkers):
        """Prepare the latest marker pose for the fixed-rate terminal display."""

        pose_texts = []
        for marker_id, pose in zip(markers.marker_ids, markers.poses):
            # OpenCV optical coordinates are X right, Y down, Z forward.
            # Present them as robot-friendly axes: forward, left and up positive.
            forward = float(pose.position.z)
            left = -float(pose.position.x)
            up = -float(pose.position.y)

            quaternion = np.array(
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
                dtype=np.float64,
            )
            quaternion_norm = np.linalg.norm(quaternion)
            if quaternion_norm > 0.0:
                qx, qy, qz, qw = quaternion / quaternion_norm
                marker_normal_x = 2.0 * (qx * qz + qy * qw)
                marker_normal_z = 1.0 - 2.0 * (qx * qx + qy * qy)
                horizontal_angle = float(
                    np.degrees(np.arctan2(marker_normal_x, -marker_normal_z))
                )
            else:
                horizontal_angle = float("nan")

            pose_texts.append(
                f"二维码ID {int(marker_id)}，"
                f"二维码在前面：{forward:+.3f} m，"
                f"在左边：{left:+.3f} m，"
                f"在上面：{up:+.3f} m，"
                f"在偏左：{horizontal_angle:+.2f} 度"
            )

        if pose_texts:
            with self.pose_log_lock:
                self.latest_pose_log_text = (
                    "最新位姿（二维码相对摄像头，前左上右手坐标系）："
                    + " | ".join(pose_texts)
                )
                self.latest_pose_update_time = time.monotonic()

    def clear_latest_pose_log(self):
        """Stop terminal pose output when the latest frame has no marker."""

        with self.pose_log_lock:
            self.latest_pose_log_text = None
            self.latest_pose_update_time = None

    def pose_log_loop(self):
        """Print the latest valid pose at a fixed 0.5-second interval."""

        next_log_time = time.monotonic() + self.POSE_LOG_PERIOD_SECONDS
        while True:
            wait_seconds = max(0.0, next_log_time - time.monotonic())
            if self.pose_log_stop_event.wait(wait_seconds):
                return

            now = time.monotonic()
            while next_log_time <= now:
                next_log_time += self.POSE_LOG_PERIOD_SECONDS

            with self.pose_log_lock:
                pose_text = self.latest_pose_log_text
                update_time = self.latest_pose_update_time

            if pose_text is None or update_time is None:
                continue
            if now - update_time > self.POSE_LOG_MAX_AGE_SECONDS:
                continue
            if not self.context.ok():
                return

            try:
                self.get_logger().info(pose_text)
            except Exception:
                return

    def stop_pose_log_thread(self):
        """Stop the terminal display thread before destroying the ROS node."""

        self.pose_log_stop_event.set()
        if self.pose_log_thread.is_alive():
            self.pose_log_thread.join(timeout=1.0)

    def initialize_parameters(self):
        # Declare and read parameters from aruco_params.yaml
        self.declare_parameter(
            name="marker_size",
            value=0.0625,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_5X5_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )

        self.declare_parameter(
            name="use_depth_input",
            value=True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Use depth camera input for pose estimation instead of RGB image",
            ),
        )

        self.declare_parameter(
            name="image_topic",
            value="/camera/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="depth_image_topic",
            value="/camera/depth/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Depth camera topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_calibration_mode",
            value="auto",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera calibration source: auto, topic, or file.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/camera/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_info_timeout_sec",
            value=5.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Auto-mode wait before loading local calibration.",
            ),
        )

        self.declare_parameter(
            name="camera_calibration_file",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Path to the local VITA calibration YAML.",
            ),
        )

        self.declare_parameter(
            name="camera_calibration_label",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera label selected from the calibration YAML.",
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
            ),
        )

        self.declare_parameter(
            name="resize_width",
            value=1920,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Width used for ArUco detection and pose estimation.",
            ),
        )

        self.declare_parameter(
            name="resize_height",
            value=1080,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Height used for ArUco detection and pose estimation.",
            ),
        )

        self.declare_parameter(
            name="detected_markers_topic",
            value="/output/aruco/markers",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish detected markers as array of marker ids and poses",
            ),
        )

        self.declare_parameter(
            name="markers_visualization_topic",
            value="/aruco_poses",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish markers as pose array",
            ),
        )

        self.declare_parameter(
            name="output_image_topic",
            value="/aruco_image",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish annotated images with markers drawn on them",
            ),
        )

        # read parameters from aruco_params.yaml and store them
        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"二维码实际边长为：{self.marker_size} 米")

        self.dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )
        self.get_logger().info(f"ArUco 字典类型为：{self.dictionary_id_name}")

        self.use_depth_input = (
            self.get_parameter("use_depth_input").get_parameter_value().bool_value
        )
        self.get_logger().info(f"是否使用深度图：{self.use_depth_input}")

        self.image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"输入的图像话题为：{self.image_topic}")

        self.depth_image_topic = (
            self.get_parameter("depth_image_topic").get_parameter_value().string_value
        )
        if self.use_depth_input:
            self.get_logger().info(f"输入的深度图话题为：{self.depth_image_topic}")

        self.camera_calibration_mode = (
            self.get_parameter("camera_calibration_mode")
            .get_parameter_value()
            .string_value
            .lower()
        )
        if self.camera_calibration_mode not in {"auto", "topic", "file"}:
            raise ValueError(
                "camera_calibration_mode must be one of: auto, topic, file"
            )
        self.get_logger().info(
            f"摄像头内参读取模式为：{self.camera_calibration_mode}"
        )

        self.info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.camera_info_timeout_sec = (
            self.get_parameter("camera_info_timeout_sec")
            .get_parameter_value()
            .double_value
        )
        if self.camera_info_timeout_sec <= 0.0:
            raise ValueError("camera_info_timeout_sec must be positive")

        self.camera_calibration_file = (
            self.get_parameter("camera_calibration_file")
            .get_parameter_value()
            .string_value
        )
        self.camera_calibration_label = (
            self.get_parameter("camera_calibration_label")
            .get_parameter_value()
            .string_value
        )

        if self.camera_calibration_mode in {"auto", "topic"}:
            self.get_logger().info(
                f"输入的摄像头硬件内参话题为：{self.info_topic}"
            )
        if self.camera_calibration_mode == "auto":
            self.get_logger().info(
                f"若 {self.camera_info_timeout_sec:g} 秒内未收到内参话题，"
                f"则读取：{self.camera_calibration_file} "
                f"（标签：{self.camera_calibration_label}）"
            )
        elif self.camera_calibration_mode == "file":
            self.get_logger().info(
                f"直接读取本地内参文件：{self.camera_calibration_file} "
                f"（标签：{self.camera_calibration_label}）"
            )

        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )
        self.get_logger().info(f"摄像头坐标系为：{self.camera_frame}")

        self.resize_width = (
            self.get_parameter("resize_width").get_parameter_value().integer_value
        )
        self.resize_height = (
            self.get_parameter("resize_height").get_parameter_value().integer_value
        )
        if self.resize_width <= 0 or self.resize_height <= 0:
            raise ValueError(
                "resize_width and resize_height must both be positive integers"
            )
        self.get_logger().info(
            f"图像处理目标分辨率为：{self.resize_width}x{self.resize_height}"
        )

        # Output topics
        self.detected_markers_topic = (
            self.get_parameter("detected_markers_topic").get_parameter_value().string_value
        )

        self.markers_visualization_topic = (
            self.get_parameter("markers_visualization_topic").get_parameter_value().string_value
        )

        self.output_image_topic = (
            self.get_parameter("output_image_topic").get_parameter_value().string_value
        )


def main():
    rclpy.init()
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_pose_log_thread()
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):
            pass
        try:
            rclpy.try_shutdown()
        except (Exception, KeyboardInterrupt):
            pass


if __name__ == "__main__":
    main()
