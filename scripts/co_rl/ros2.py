from __future__ import annotations

import os
import sys
import glob
import importlib
import math

DEFAULT_FRONT_DEPTH_CAMERA_PRIM = "/World/envs/env_0/Robot/F_camera_link/front_depth_cam"
DEFAULT_AUTO_DEPTH_CAMERA_PRIM = "/World/envs/env_0/Robot/A_camera_link/adas_camera"
DEFAULT_AUTO_RGB_CAMERA_PRIM = DEFAULT_AUTO_DEPTH_CAMERA_PRIM
DEFAULT_AUTO_LEFT_CAMERA_PRIM = "/World/envs/env_0/Robot/A_camera_link/adas_left_camera"
DEFAULT_AUTO_RIGHT_CAMERA_PRIM = "/World/envs/env_0/Robot/A_camera_link/adas_right_camera"
DEFAULT_DEPTH_CAMERA_PRIM = DEFAULT_FRONT_DEPTH_CAMERA_PRIM
DEFAULT_GRAPH_PATH = "/ROS_FrontDepthCamera"
DEFAULT_FRONT_GRAPH_PATH = "/ROS_FrontDepthCamera"
DEFAULT_AUTO_DEPTH_GRAPH_PATH = "/ROS_AutoDepthCamera"
DEFAULT_AUTO_RGB_GRAPH_PATH = "/ROS_AutoRgbCamera"
DEFAULT_AUTO_STEREO_GRAPH_PATH = "/ROS_AutoStereoCamera"
DEFAULT_DEPTH_TOPIC = "/f4/front_camera/depth/image_rect_raw"
DEFAULT_CAMERA_INFO_TOPIC = "/f4/front_camera/depth/camera_info"
DEFAULT_FRAME_ID = "f4/front_camera_depth_optical_frame"
DEFAULT_FRONT_DEPTH_TOPIC = DEFAULT_DEPTH_TOPIC
DEFAULT_FRONT_CAMERA_INFO_TOPIC = DEFAULT_CAMERA_INFO_TOPIC
DEFAULT_FRONT_FRAME_ID = DEFAULT_FRAME_ID
DEFAULT_AUTO_DEPTH_TOPIC = "/f4/adas_camera/depth/image_rect_raw"
DEFAULT_AUTO_DEPTH_CAMERA_INFO_TOPIC = "/f4/adas_camera/depth/camera_info"
DEFAULT_AUTO_DEPTH_FRAME_ID = "A_camera_link"
DEFAULT_AUTO_RGB_TOPIC = "/f4/adas_camera/rgb/image_raw"
DEFAULT_AUTO_RGB_CAMERA_INFO_TOPIC = "/f4/adas_camera/rgb/camera_info"
DEFAULT_AUTO_RGB_FRAME_ID = "A_camera_link"
DEFAULT_AUTO_LEFT_IMAGE_TOPIC = "/f4/adas_camera/left/image_raw"
DEFAULT_AUTO_RIGHT_IMAGE_TOPIC = "/f4/adas_camera/right/image_raw"
DEFAULT_AUTO_LEFT_CAMERA_INFO_TOPIC = "/f4/adas_camera/left/camera_info"
DEFAULT_AUTO_RIGHT_CAMERA_INFO_TOPIC = "/f4/adas_camera/right/camera_info"
DEFAULT_AUTO_LEFT_FRAME_ID = "A_camera_link_left_optical"
DEFAULT_AUTO_RIGHT_FRAME_ID = "A_camera_link_right_optical"
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_IMU_GRAPH_PATH = "/ROS_Imu"
DEFAULT_IMU_TOPIC = "/f4/imu"
DEFAULT_IMU_FRAME_ID = "f4/base_link"
DEFAULT_FRONT_CAMERA_IMU_GRAPH_PATH = "/ROS_FrontCameraImu"
DEFAULT_FRONT_CAMERA_IMU_TOPIC = "/f4/front_camera/imu"
DEFAULT_FRONT_CAMERA_IMU_FRAME_ID = "f4/front_camera_link"
DEFAULT_CLOCK_GRAPH_PATH = "/ROS_Clock"
DEFAULT_TIME_TOPIC = "time"
DEFAULT_JOINT_STATES_TOPIC = "/f4/joint_states"
DEFAULT_WORLD_FRAME_ID = "world"
DEFAULT_BASE_FRAME_ID = "f4/base_link"
DEFAULT_PATH_GT_TOPIC = "/path_gt"
DEFAULT_COMMAND_USER_TOPIC = "/control_command/user_odom"
DEFAULT_HEIGHT_MAP_TOPIC = "/f4/height_map/points"
DEFAULT_HEIGHT_MAP_FRAME_ID = DEFAULT_WORLD_FRAME_ID
DEFAULT_MID360_LIDAR_TOPIC = "/f4/lidar/points"
DEFAULT_MID360_FRAME_ID = "f4/lidar_link"
DEFAULT_MID360_RTX_PRIM = "/World/envs/env_0/Robot/base_link/lidar_rtx"
DEFAULT_MID360_RTX_GRAPH_PATH = "/ROS_LidarRtx"
DEFAULT_MID360_RTX_CONFIG = "OS1_REV6_32ch10hz512res"


def _import_ros2_command_user_type():
    """Import core.msg.CommandUser without being shadowed by scripts/co_rl/core."""

    candidate_paths: list[str] = []
    prefixes = [prefix for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if prefix]
    prefixes += ["/root/ros2_ws/install/core", "/root/ros2_ws/install", "/opt/ros/humble"]

    for prefix in prefixes:
        candidate_paths.extend(glob.glob(os.path.join(prefix, "local", "lib", "python*", "dist-packages")))
        candidate_paths.extend(glob.glob(os.path.join(prefix, "lib", "python*", "dist-packages")))
        candidate_paths.extend(glob.glob(os.path.join(prefix, "lib", "python*", "site-packages")))

    candidate_paths.extend(glob.glob("/root/ros2_ws/install/*/local/lib/python*/dist-packages"))
    candidate_paths.extend(glob.glob("/opt/ros/humble/local/lib/python*/dist-packages"))
    candidate_paths.extend(glob.glob("/opt/ros/humble/lib/python*/dist-packages"))

    unique_paths = []
    for path in candidate_paths:
        if os.path.isdir(path) and path not in unique_paths:
            unique_paths.append(path)

    ros2_core_paths = [path for path in unique_paths if os.path.isdir(os.path.join(path, "core", "msg"))]
    if not ros2_core_paths:
        raise ModuleNotFoundError(
            "Could not find ROS 2 Python package 'core.msg'. "
            "Expected a path like /root/ros2_ws/install/core/local/lib/python*/dist-packages/core/msg."
        )

    # Only promote the custom core package. Promoting every AMENT Python path
    # would put the system Jazzy Python 3.12 messages ahead of Isaac Sim's
    # bundled Python 3.11 messages and break their native type support.
    for path in reversed(ros2_core_paths):
        if path in sys.path:
            sys.path.remove(path)
        sys.path.insert(0, path)

    for module_name in list(sys.modules):
        if module_name == "core" or module_name.startswith("core."):
            del sys.modules[module_name]

    importlib.invalidate_caches()
    CommandUser = importlib.import_module("core.msg").CommandUser

    return CommandUser


class Ros2ImuGraphPublisher:
    """Small helper that updates the ROS 2 IMU OmniGraph inputs each step."""

    def __init__(self, graph_path: str):
        import omni.graph.core as og

        self._og = og
        self._orientation_attr = og.Controller.attribute(f"{graph_path}/PublishImu.inputs:orientation")
        self._angular_velocity_attr = og.Controller.attribute(f"{graph_path}/PublishImu.inputs:angularVelocity")
        self._linear_acceleration_attr = og.Controller.attribute(f"{graph_path}/PublishImu.inputs:linearAcceleration")
        self._timestamp_attr = og.Controller.attribute(f"{graph_path}/PublishImu.inputs:timeStamp")

    def publish(
        self,
        orientation_wxyz,
        angular_velocity_xyz,
        linear_acceleration_xyz,
        timestamp_s: float,
    ) -> None:
        # ROS2PublishImu expects quaternion ordering as (i, j, k, r) i.e. (x, y, z, w).
        q_w, q_x, q_y, q_z = [float(v) for v in orientation_wxyz]
        self._orientation_attr.set((q_x, q_y, q_z, q_w))
        self._angular_velocity_attr.set(tuple(float(v) for v in angular_velocity_xyz))
        self._linear_acceleration_attr.set(tuple(float(v) for v in linear_acceleration_xyz))
        self._timestamp_attr.set(float(timestamp_s))


class Ros2ImuPublisher:
    """ROS 2 sensor_msgs/Imu publisher using rclpy."""

    def __init__(
        self,
        topic_name: str,
        frame_id: str,
        node_name: str = "isaac_imu_bridge",
        derive_angular_velocity: bool = False,
    ):
        import math
        import rclpy
        from builtin_interfaces.msg import Time
        from sensor_msgs.msg import Imu

        self._rclpy = rclpy
        self._math = math
        self._time_type = Time
        self._imu_type = Imu
        self._frame_id = frame_id
        self._derive_angular_velocity = bool(derive_angular_velocity)
        self._previous_orientation = None
        self._previous_timestamp_s = None
        self._owns_context = False

        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_context = True

        self._node = rclpy.create_node(node_name)
        self._publisher = self._node.create_publisher(Imu, topic_name, 50)

    def publish(
        self,
        orientation_wxyz,
        angular_velocity_xyz,
        linear_acceleration_xyz,
        timestamp_s: float,
    ) -> None:
        orientation = self._normalize_quaternion(tuple(float(value) for value in orientation_wxyz))
        angular_velocity = tuple(float(value) for value in angular_velocity_xyz)
        if self._derive_angular_velocity:
            angular_velocity = self._angular_velocity_from_orientation(orientation, float(timestamp_s))

        msg = self._imu_type()
        msg.header.stamp = self._stamp(timestamp_s)
        msg.header.frame_id = self._frame_id
        msg.orientation.w = orientation[0]
        msg.orientation.x = orientation[1]
        msg.orientation.y = orientation[2]
        msg.orientation.z = orientation[3]
        msg.angular_velocity.x = angular_velocity[0]
        msg.angular_velocity.y = angular_velocity[1]
        msg.angular_velocity.z = angular_velocity[2]
        msg.linear_acceleration.x = float(linear_acceleration_xyz[0])
        msg.linear_acceleration.y = float(linear_acceleration_xyz[1])
        msg.linear_acceleration.z = float(linear_acceleration_xyz[2])
        self._publisher.publish(msg)
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    @staticmethod
    def _normalize_quaternion(quaternion):
        norm = sum(value * value for value in quaternion) ** 0.5
        if norm <= 1.0e-12:
            return (1.0, 0.0, 0.0, 0.0)
        return tuple(value / norm for value in quaternion)

    @staticmethod
    def _quaternion_multiply(first, second):
        aw, ax, ay, az = first
        bw, bx, by, bz = second
        return (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )

    def _angular_velocity_from_orientation(self, orientation, timestamp_s: float):
        previous = self._previous_orientation
        previous_timestamp_s = self._previous_timestamp_s
        self._previous_orientation = orientation
        self._previous_timestamp_s = timestamp_s
        if previous is None or previous_timestamp_s is None:
            return (0.0, 0.0, 0.0)

        dt = timestamp_s - previous_timestamp_s
        if dt <= 1.0e-6 or dt > 0.2:
            return (0.0, 0.0, 0.0)
        if sum(first * second for first, second in zip(previous, orientation)) < 0.0:
            orientation = tuple(-value for value in orientation)
            self._previous_orientation = orientation

        relative = self._normalize_quaternion(
            self._quaternion_multiply(
                (previous[0], -previous[1], -previous[2], -previous[3]), orientation
            )
        )
        vector_norm = sum(value * value for value in relative[1:]) ** 0.5
        if vector_norm <= 1.0e-12:
            return (0.0, 0.0, 0.0)
        angle = 2.0 * self._math.atan2(vector_norm, max(relative[0], 0.0))
        scale = angle / (vector_norm * dt)
        return tuple(value * scale for value in relative[1:])

    def close(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()

    def _stamp(self, timestamp_s: float):
        stamp = self._time_type()
        sec = int(timestamp_s)
        stamp.sec = sec
        stamp.nanosec = int((float(timestamp_s) - sec) * 1.0e9)
        return stamp


class Ros2TimePublisher:
    """ROS 2 Float64 publisher for simulation time using rclpy."""

    def __init__(self, topic_name: str):
        import rclpy
        from std_msgs.msg import Float64

        self._rclpy = rclpy
        self._float64_type = Float64
        self._owns_context = False

        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_context = True

        self._node = rclpy.create_node("isaac_sim_time_publisher")
        self._publisher = self._node.create_publisher(Float64, topic_name, 10)

    def publish(self, value: float) -> None:
        msg = self._float64_type()
        msg.data = float(value)
        self._publisher.publish(msg)
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def close(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()


class Ros2ClockPublisher:
    """Publish authoritative ROS /clock messages from Isaac simulation time."""

    def __init__(self, topic_name: str = "/clock"):
        import rclpy
        from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
        from rosgraph_msgs.msg import Clock

        self._rclpy = rclpy
        self._clock_type = Clock
        self._owns_context = False
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_context = True
        self._node = rclpy.create_node("isaac_ros_clock_publisher")
        clock_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._publisher = self._node.create_publisher(Clock, topic_name, clock_qos)

    def publish(self, timestamp_s: float) -> None:
        msg = self._clock_type()
        msg.clock.sec = int(timestamp_s)
        msg.clock.nanosec = int((float(timestamp_s) - msg.clock.sec) * 1.0e9)
        self._publisher.publish(msg)
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def close(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()


class Ros2HeightMapPointCloudPublisher:
    """Publish IsaacLab RayCaster hit points as a ROS 2 PointCloud2 message."""

    def __init__(
        self,
        topic_name: str = DEFAULT_HEIGHT_MAP_TOPIC,
        frame_id: str = DEFAULT_HEIGHT_MAP_FRAME_ID,
        node_name: str = "isaac_height_map_pointcloud_bridge",
        max_range_m: float | None = None,
        include_intensity_time: bool = False,
    ):
        import numpy as np
        import rclpy
        from builtin_interfaces.msg import Time
        from sensor_msgs.msg import PointCloud2, PointField

        self._np = np
        self._rclpy = rclpy
        self._time_type = Time
        self._pointcloud2_type = PointCloud2
        self._pointfield_type = PointField
        self._owns_context = False
        self._frame_id = frame_id
        self._max_range_m = max_range_m
        self._include_intensity_time = bool(include_intensity_time)

        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_context = True

        self._node = rclpy.create_node(node_name)
        self._publisher = self._node.create_publisher(PointCloud2, topic_name, 10)
        self._fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        if self._include_intensity_time:
            # Super-LIO accepts the standard Velodyne PointCloud2 layout. A
            # simulated RayCaster scan is instantaneous, so both extra values
            # are intentionally zero.
            self._fields.extend(
                [
                    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
                    PointField(name="time", offset=16, datatype=PointField.FLOAT32, count=1),
                ]
            )

    def publish(self, ray_hits_w, timestamp_s: float, sensor_pos_w=None, sensor_quat_wxyz=None) -> None:
        if ray_hits_w is None:
            return

        if hasattr(ray_hits_w, "detach"):
            points = ray_hits_w[0].detach().cpu().numpy()
        else:
            points = self._np.asarray(ray_hits_w)
            if points.ndim == 3:
                points = points[0]

        points = self._np.asarray(points, dtype=self._np.float32).reshape(-1, 3)
        finite_mask = self._np.isfinite(points).all(axis=1)
        points = points[finite_mask]
        if sensor_pos_w is not None and sensor_quat_wxyz is not None and points.size > 0:
            points = self._points_w_to_sensor(points, sensor_pos_w, sensor_quat_wxyz)
        if self._max_range_m is not None and points.size > 0:
            ranges = self._np.linalg.norm(points, axis=1)
            points = points[ranges < (float(self._max_range_m) * 0.995)]

        msg = self._pointcloud2_type()
        msg.header.stamp = self._stamp(timestamp_s)
        msg.header.frame_id = self._frame_id
        msg.height = 1
        msg.width = int(points.shape[0])
        msg.fields = self._fields
        msg.is_bigendian = False
        if self._include_intensity_time:
            payload = self._np.zeros((points.shape[0], 5), dtype=self._np.float32)
            payload[:, :3] = points
        else:
            payload = points.astype(self._np.float32, copy=False)
        msg.point_step = int(payload.shape[1] * payload.dtype.itemsize)
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = payload.tobytes()

        self._publisher.publish(msg)
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def close(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()

    def _stamp(self, timestamp_s: float):
        stamp = self._time_type()
        sec = int(timestamp_s)
        stamp.sec = sec
        stamp.nanosec = int((float(timestamp_s) - sec) * 1.0e9)
        return stamp

    def _points_w_to_sensor(self, points_w, sensor_pos_w, sensor_quat_wxyz):
        pos_w = self._to_numpy(sensor_pos_w).reshape(3)
        quat = self._to_numpy(sensor_quat_wxyz).reshape(4)
        quat = quat / self._np.linalg.norm(quat)
        w, x, y, z = quat
        rotation_sensor_to_world = self._np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=self._np.float32,
        )
        return (points_w - pos_w) @ rotation_sensor_to_world

    def _to_numpy(self, value):
        if hasattr(value, "detach"):
            value = value.detach().cpu().numpy()
        return self._np.asarray(value, dtype=self._np.float32)


class Ros2Mid360PointCloudPublisher(Ros2HeightMapPointCloudPublisher):
    """Publish RayCaster lidar hit points as a ROS 2 PointCloud2 message."""

    def __init__(
        self,
        topic_name: str = DEFAULT_MID360_LIDAR_TOPIC,
        frame_id: str = DEFAULT_MID360_FRAME_ID,
        node_name: str = "isaac_lidar_pointcloud_bridge",
        max_range_m: float = 15.0,
    ):
        super().__init__(
            topic_name=topic_name,
            frame_id=frame_id,
            node_name=node_name,
            max_range_m=max_range_m,
            include_intensity_time=True,
        )


class Ros2RobotStatePublisher:
    """Publish env_0 joint states and lidar-referenced ground truth."""

    def __init__(
        self,
        joint_states_topic: str = DEFAULT_JOINT_STATES_TOPIC,
        world_frame_id: str = DEFAULT_WORLD_FRAME_ID,
        base_frame_id: str = DEFAULT_BASE_FRAME_ID,
        path_gt_topic: str = DEFAULT_PATH_GT_TOPIC,
        gt_odom_topic: str = "/gt/lidar_odom",
        gt_child_frame_id: str = DEFAULT_MID360_FRAME_ID,
        path_gt_max_poses: int = 5000,
        publish_root_tf: bool = True,
        relative_to_initial_pose: bool = False,
        diagnostics_csv_path: str = "",
        lio_odom_topic: str = "/lio/odom",
        lio_map_odom_topic: str = "/lio/odom_map",
    ):
        import csv
        import rclpy
        from builtin_interfaces.msg import Time
        from geometry_msgs.msg import TransformStamped
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState
        from tf2_ros import TransformBroadcaster

        self._rclpy = rclpy
        self._time_type = Time
        self._transform_type = TransformStamped
        self._pose_stamped_type = PoseStamped
        self._path_type = Path
        self._odom_type = Odometry
        self._joint_state_type = JointState
        self._world_frame_id = world_frame_id
        self._base_frame_id = base_frame_id
        self._gt_child_frame_id = gt_child_frame_id
        self._path_gt_max_poses = max(int(path_gt_max_poses), 1)
        self._path_gt_poses = []
        self._publish_root_tf = bool(publish_root_tf)
        self._relative_to_initial_pose = bool(relative_to_initial_pose)
        self._initial_root_position = None
        self._initial_root_orientation = None
        self._latest_lio_odom = None
        self._latest_lio_map_odom = None
        self._diagnostics_file = None
        self._diagnostics_writer = None
        self._last_diagnostics_flush_s = -1.0
        self._owns_context = False

        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_context = True

        self._node = rclpy.create_node("isaac_robot_state_bridge")
        self._joint_state_pub = self._node.create_publisher(JointState, joint_states_topic, 10)
        self._path_gt_pub = self._node.create_publisher(Path, path_gt_topic, 10)
        self._gt_odom_pub = self._node.create_publisher(Odometry, gt_odom_topic, 10)
        self._lio_odom_sub = self._node.create_subscription(
            Odometry, lio_odom_topic, self._on_lio_odom, 20
        )
        self._lio_map_odom_sub = self._node.create_subscription(
            Odometry, lio_map_odom_topic, self._on_lio_map_odom, 20
        )
        self._tf_broadcaster = TransformBroadcaster(self._node) if self._publish_root_tf else None
        if diagnostics_csv_path:
            self._diagnostics_file = open(diagnostics_csv_path, "w", newline="", encoding="utf-8")
            self._diagnostics_writer = csv.writer(self._diagnostics_file)
            self._diagnostics_writer.writerow(
                [
                    "sim_time_s",
                    "gt_x", "gt_y", "gt_z", "gt_qx", "gt_qy", "gt_qz", "gt_qw",
                    "lio_odom_x", "lio_odom_y", "lio_odom_z",
                    "lio_odom_vx", "lio_odom_vy", "lio_odom_vz",
                    "lio_map_x", "lio_map_y", "lio_map_z",
                    "global_position_error_m",
                ]
            )
            self._diagnostics_file.flush()

    def publish(
        self,
        joint_names,
        joint_positions,
        joint_velocities,
        root_position_xyz,
        root_orientation_wxyz,
        timestamp_s: float,
        gt_position_xyz=None,
        gt_orientation_wxyz=None,
    ) -> None:
        # Drain LIO callbacks before choosing the relative GT origin. In SLAM
        # mode the filter starts only after stationary-IMU initialization;
        # using Isaac's earlier un-settled pose would create a permanent GT
        # offset even when both trajectories are otherwise identical.
        self._rclpy.spin_once(self._node, timeout_sec=0.0)
        stamp = self._stamp(timestamp_s)
        root_position_xyz = tuple(float(value) for value in root_position_xyz)
        root_orientation_wxyz = self._normalize_quaternion(
            tuple(float(value) for value in root_orientation_wxyz)
        )

        joint_msg = self._joint_state_type()
        joint_msg.header.stamp = stamp
        joint_msg.name = [str(name) for name in joint_names]
        joint_msg.position = [float(value) for value in joint_positions]
        joint_msg.velocity = [float(value) for value in joint_velocities]
        self._joint_state_pub.publish(joint_msg)

        if self._relative_to_initial_pose and self._latest_lio_map_odom is None:
            return

        gt_position_xyz, gt_orientation_wxyz = self._path_pose(
            gt_position_xyz if gt_position_xyz is not None else root_position_xyz,
            gt_orientation_wxyz if gt_orientation_wxyz is not None else root_orientation_wxyz,
            reference_orientation_wxyz=root_orientation_wxyz,
        )

        if self._tf_broadcaster is not None:
            tf_msg = self._transform_type()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self._world_frame_id
            tf_msg.child_frame_id = self._base_frame_id
            tf_msg.transform.translation.x = root_position_xyz[0]
            tf_msg.transform.translation.y = root_position_xyz[1]
            tf_msg.transform.translation.z = root_position_xyz[2]
            tf_msg.transform.rotation.w = root_orientation_wxyz[0]
            tf_msg.transform.rotation.x = root_orientation_wxyz[1]
            tf_msg.transform.rotation.y = root_orientation_wxyz[2]
            tf_msg.transform.rotation.z = root_orientation_wxyz[3]
            self._tf_broadcaster.sendTransform(tf_msg)

        pose_msg = self._pose_stamped_type()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self._world_frame_id
        pose_msg.pose.position.x = gt_position_xyz[0]
        pose_msg.pose.position.y = gt_position_xyz[1]
        pose_msg.pose.position.z = gt_position_xyz[2]
        pose_msg.pose.orientation.w = gt_orientation_wxyz[0]
        pose_msg.pose.orientation.x = gt_orientation_wxyz[1]
        pose_msg.pose.orientation.y = gt_orientation_wxyz[2]
        pose_msg.pose.orientation.z = gt_orientation_wxyz[3]
        self._path_gt_poses.append(pose_msg)
        if len(self._path_gt_poses) > self._path_gt_max_poses:
            self._path_gt_poses = self._path_gt_poses[-self._path_gt_max_poses :]

        path_msg = self._path_type()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = self._world_frame_id
        path_msg.poses = list(self._path_gt_poses)
        self._path_gt_pub.publish(path_msg)

        gt_odom = self._odom_type()
        gt_odom.header.stamp = stamp
        gt_odom.header.frame_id = self._world_frame_id
        gt_odom.child_frame_id = self._gt_child_frame_id
        gt_odom.pose.pose = pose_msg.pose
        self._gt_odom_pub.publish(gt_odom)

        self._write_diagnostics(timestamp_s, gt_odom)

        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def _on_lio_odom(self, msg) -> None:
        self._latest_lio_odom = msg

    def _on_lio_map_odom(self, msg) -> None:
        self._latest_lio_map_odom = msg

    def _write_diagnostics(self, timestamp_s: float, gt_odom) -> None:
        if self._diagnostics_writer is None:
            return

        def pose_xyz(msg):
            if msg is None:
                return ("", "", "")
            position = msg.pose.pose.position
            return (position.x, position.y, position.z)

        def velocity_xyz(msg):
            if msg is None:
                return ("", "", "")
            velocity = msg.twist.twist.linear
            return (velocity.x, velocity.y, velocity.z)

        gt_pose = gt_odom.pose.pose
        local_xyz = pose_xyz(self._latest_lio_odom)
        map_xyz = pose_xyz(self._latest_lio_map_odom)
        global_error = ""
        if self._latest_lio_map_odom is not None:
            global_error = sum(
                (map_xyz[index] - (gt_pose.position.x, gt_pose.position.y, gt_pose.position.z)[index]) ** 2
                for index in range(3)
            ) ** 0.5
        self._diagnostics_writer.writerow(
            [
                timestamp_s,
                gt_pose.position.x, gt_pose.position.y, gt_pose.position.z,
                gt_pose.orientation.x, gt_pose.orientation.y,
                gt_pose.orientation.z, gt_pose.orientation.w,
                *local_xyz,
                *velocity_xyz(self._latest_lio_odom),
                *map_xyz,
                global_error,
            ]
        )
        if self._diagnostics_file is not None and (
            self._last_diagnostics_flush_s < 0.0
            or timestamp_s - self._last_diagnostics_flush_s >= 1.0
        ):
            self._diagnostics_file.flush()
            self._last_diagnostics_flush_s = timestamp_s

    def _path_pose(self, position, orientation_wxyz, reference_orientation_wxyz=None):
        position = tuple(float(value) for value in position)
        orientation = self._normalize_quaternion(tuple(float(value) for value in orientation_wxyz))
        reference_orientation = self._normalize_quaternion(
            tuple(float(value) for value in (
                reference_orientation_wxyz
                if reference_orientation_wxyz is not None
                else orientation_wxyz
            ))
        )
        if not self._relative_to_initial_pose:
            return position, orientation
        if self._initial_root_position is None:
            self._initial_root_position = position
            # The tracked target can be an upside-down lidar_link. Align only
            # initial heading with map: cancelling the robot's initial roll or
            # pitch would disagree with gravity-aligned LIO coordinates.
            self._initial_root_orientation = self._yaw_only_quaternion(reference_orientation)

        initial_inverse = self._quaternion_conjugate(self._initial_root_orientation)
        delta = tuple(position[index] - self._initial_root_position[index] for index in range(3))
        relative_position = self._rotate_vector(initial_inverse, delta)
        relative_orientation = self._normalize_quaternion(
            self._quaternion_multiply(initial_inverse, orientation)
        )
        return relative_position, relative_orientation

    @staticmethod
    def _normalize_quaternion(quaternion):
        norm = sum(value * value for value in quaternion) ** 0.5
        if norm <= 1.0e-12:
            return (1.0, 0.0, 0.0, 0.0)
        return tuple(value / norm for value in quaternion)

    @staticmethod
    def _yaw_only_quaternion(quaternion):
        w, x, y, z = quaternion
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return (math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw))

    @staticmethod
    def _quaternion_conjugate(quaternion):
        w, x, y, z = quaternion
        return (w, -x, -y, -z)

    @staticmethod
    def _quaternion_multiply(first, second):
        aw, ax, ay, az = first
        bw, bx, by, bz = second
        return (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )

    @classmethod
    def _rotate_vector(cls, quaternion, vector):
        rotated = cls._quaternion_multiply(
            cls._quaternion_multiply(quaternion, (0.0, *vector)),
            cls._quaternion_conjugate(quaternion),
        )
        return rotated[1:]

    def close(self) -> None:
        if self._diagnostics_file is not None:
            self._diagnostics_file.flush()
            self._diagnostics_file.close()
            self._diagnostics_file = None
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()

    def _stamp(self, timestamp_s: float):
        stamp = self._time_type()
        sec = int(timestamp_s)
        stamp.sec = sec
        stamp.nanosec = int((float(timestamp_s) - sec) * 1.0e9)
        return stamp


class Ros2CameraPublisher:
    """Publish RGB/depth camera tensors as ROS 2 Image and CameraInfo."""

    def __init__(
        self,
        rgb_topic: str,
        depth_topic: str,
        camera_info_topic: str,
        frame_id: str,
        node_name: str = "isaac_front_camera_bridge",
    ):
        import numpy as np
        import rclpy
        from builtin_interfaces.msg import Time
        from sensor_msgs.msg import CameraInfo, Image

        self._np = np
        self._rclpy = rclpy
        self._time_type = Time
        self._image_type = Image
        self._camera_info_type = CameraInfo
        self._frame_id = frame_id
        self._owns_context = False

        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_context = True

        self._node = rclpy.create_node(node_name)
        self._rgb_pub = self._node.create_publisher(Image, rgb_topic, 10)
        self._depth_pub = self._node.create_publisher(Image, depth_topic, 10)
        self._info_pub = self._node.create_publisher(CameraInfo, camera_info_topic, 10)

    def publish(self, camera_data, timestamp_s: float) -> None:
        output = getattr(camera_data, "output", None)
        if not output:
            return

        stamp = self._stamp(timestamp_s)
        if "rgb" in output:
            rgb = self._to_numpy(output["rgb"][0])
            if rgb.dtype != self._np.uint8:
                rgb = self._np.clip(rgb, 0, 255).astype(self._np.uint8)
            self._rgb_pub.publish(self._image_msg(rgb, stamp, "rgb8"))

        depth_tensor = output.get("distance_to_image_plane", output.get("depth"))
        if depth_tensor is not None:
            depth = self._to_numpy(depth_tensor[0]).astype(self._np.float32, copy=False)
            self._depth_pub.publish(self._image_msg(depth, stamp, "32FC1"))

        intrinsics = getattr(camera_data, "intrinsic_matrices", None)
        image_shape = getattr(camera_data, "image_shape", None)
        if intrinsics is not None and image_shape is not None:
            k = self._to_numpy(intrinsics[0]).reshape(3, 3)
            info = self._camera_info_type()
            info.header.stamp = stamp
            info.header.frame_id = self._frame_id
            info.height = int(image_shape[0])
            info.width = int(image_shape[1])
            info.k = [float(v) for v in k.reshape(-1)]
            info.p = [
                float(k[0, 0]), float(k[0, 1]), float(k[0, 2]), 0.0,
                float(k[1, 0]), float(k[1, 1]), float(k[1, 2]), 0.0,
                float(k[2, 0]), float(k[2, 1]), float(k[2, 2]), 0.0,
            ]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.distortion_model = "plumb_bob"
            info.d = []
            self._info_pub.publish(info)

        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def close(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()

    def _image_msg(self, image, stamp, encoding: str):
        msg = self._image_type()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = False
        channels = 1 if image.ndim == 2 else int(image.shape[2])
        msg.step = int(msg.width * channels * image.dtype.itemsize)
        msg.data = image.tobytes()
        return msg

    def _stamp(self, timestamp_s: float):
        stamp = self._time_type()
        sec = int(timestamp_s)
        stamp.sec = sec
        stamp.nanosec = int((float(timestamp_s) - sec) * 1.0e9)
        return stamp

    def _to_numpy(self, value):
        if hasattr(value, "detach"):
            return value.detach().cpu().numpy()
        return self._np.asarray(value)


class Ros2CommandUserBridge:
    """Publish keyboard and subscribe actuator-facing core/msg/CommandUser commands."""

    def __init__(
        self,
        topic_name: str = DEFAULT_COMMAND_USER_TOPIC,
        command_dim: int = 3,
        axis_names: list[str] | None = None,
        node_name: str = "isaac_command_user_bridge",
        odom_frame_id: str = "odom",
        child_frame_id: str = DEFAULT_BASE_FRAME_ID,
        publish_enabled: bool = False,
    ):
        import math
        import numpy as np
        import rclpy
        from builtin_interfaces.msg import Time
        from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
        CommandUser = _import_ros2_command_user_type()

        self._math = math
        self._np = np
        self._rclpy = rclpy
        self._time_type = Time
        self._command_user_type = CommandUser
        self._owns_context = False
        self._topic_name = topic_name
        self._axis_names = list(axis_names) if axis_names is not None else []
        self._command_dim = int(command_dim)
        self._odom_frame_id = odom_frame_id
        self._child_frame_id = child_frame_id
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._latest_command = np.zeros(self._command_dim, dtype=np.float32)
        self._received_count = 0

        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_context = True

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=16,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._node = rclpy.create_node(node_name)
        self._publisher = (
            self._node.create_publisher(CommandUser, topic_name, qos) if publish_enabled else None
        )
        self._subscription = self._node.create_subscription(CommandUser, topic_name, self._on_command_user, qos)

    @property
    def received_count(self) -> int:
        return self._received_count

    @property
    def topic_name(self) -> str:
        return self._topic_name

    def publish(self, command, timestamp_s: float, dt: float = 0.0) -> None:
        if self._publisher is None:
            raise RuntimeError("CommandUser publishing is disabled for this bridge")

        command_np = self._to_command_np(command)
        lin_x, lin_y, ang_z, pos_z = self._command_to_motion(command_np)

        self._yaw += ang_z * float(dt)
        self._x += (lin_x * self._math.cos(self._yaw) - lin_y * self._math.sin(self._yaw)) * float(dt)
        self._y += (lin_x * self._math.sin(self._yaw) + lin_y * self._math.cos(self._yaw)) * float(dt)

        msg = self._command_user_type()
        msg.odom.header.stamp = self._stamp(timestamp_s)
        msg.odom.header.frame_id = self._odom_frame_id
        msg.odom.child_frame_id = self._child_frame_id
        msg.odom.pose.pose.position.x = self._x
        msg.odom.pose.pose.position.y = self._y
        msg.odom.pose.pose.position.z = pos_z
        msg.odom.pose.pose.orientation.z = self._math.sin(self._yaw * 0.5)
        msg.odom.pose.pose.orientation.w = self._math.cos(self._yaw * 0.5)
        msg.odom.twist.twist.linear.x = lin_x
        msg.odom.twist.twist.linear.y = lin_y
        msg.odom.twist.twist.linear.z = 0.0
        msg.odom.twist.twist.angular.z = ang_z
        msg.event.estop = False
        msg.event.wake = False
        msg.event.sleep = False
        msg.event.rough_drive_toggle = False

        self._publisher.publish(msg)
        self.spin_once()

    def get_latest_command(self):
        return self._latest_command.copy()

    def spin_once(self) -> None:
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def close(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()

    def _on_command_user(self, msg) -> None:
        command = self._np.zeros(self._command_dim, dtype=self._np.float32)
        if bool(msg.event.estop) or bool(msg.event.sleep):
            self._latest_command = command
            self._received_count += 1
            return

        values = {
            "lin_vel_x": float(msg.odom.twist.twist.linear.x),
            "x": float(msg.odom.twist.twist.linear.x),
            "vx": float(msg.odom.twist.twist.linear.x),
            "vel_x": float(msg.odom.twist.twist.linear.x),
            "lin_vel_y": float(msg.odom.twist.twist.linear.y),
            "y": float(msg.odom.twist.twist.linear.y),
            "vy": float(msg.odom.twist.twist.linear.y),
            "vel_y": float(msg.odom.twist.twist.linear.y),
            "ang_vel_z": float(msg.odom.twist.twist.angular.z),
            "yaw": float(msg.odom.twist.twist.angular.z),
            "wz": float(msg.odom.twist.twist.angular.z),
            "omega_z": float(msg.odom.twist.twist.angular.z),
            "yaw_rate": float(msg.odom.twist.twist.angular.z),
            "pos_z": float(msg.odom.pose.pose.position.z),
            "z": float(msg.odom.pose.pose.position.z),
            "height": float(msg.odom.pose.pose.position.z),
            "base_height": float(msg.odom.pose.pose.position.z),
        }
        for i in range(self._command_dim):
            axis_name = self._axis_names[i].lower() if i < len(self._axis_names) else ""
            command[i] = values.get(axis_name, 0.0)
        self._latest_command = command
        self._received_count += 1

    def _command_to_motion(self, command_np) -> tuple[float, float, float, float]:
        values = {}
        for i in range(min(self._command_dim, command_np.shape[0])):
            axis_name = self._axis_names[i].lower() if i < len(self._axis_names) else ""
            values[axis_name] = float(command_np[i])
        return (
            values.get("lin_vel_x", values.get("x", values.get("vx", values.get("vel_x", 0.0)))),
            values.get("lin_vel_y", values.get("y", values.get("vy", values.get("vel_y", 0.0)))),
            values.get("ang_vel_z", values.get("yaw", values.get("wz", values.get("yaw_rate", 0.0)))),
            values.get("pos_z", values.get("z", values.get("height", values.get("base_height", 0.0)))),
        )

    def _to_command_np(self, command):
        if hasattr(command, "detach"):
            command = command[0].detach().cpu().numpy()
        command_np = self._np.asarray(command, dtype=self._np.float32).reshape(-1)
        if command_np.shape[0] < self._command_dim:
            command_np = self._np.pad(command_np, (0, self._command_dim - command_np.shape[0]))
        return command_np[: self._command_dim]

    def _stamp(self, timestamp_s: float):
        stamp = self._time_type()
        sec = int(timestamp_s)
        stamp.sec = sec
        stamp.nanosec = int((float(timestamp_s) - sec) * 1.0e9)
        return stamp


def create_depth_camera_ros_graph(
    camera_prim_path: str = DEFAULT_DEPTH_CAMERA_PRIM,
    graph_path: str = DEFAULT_GRAPH_PATH,
    depth_topic: str = DEFAULT_DEPTH_TOPIC,
    camera_info_topic: str = DEFAULT_CAMERA_INFO_TOPIC,
    frame_id: str = DEFAULT_FRAME_ID,
    width: int = DEFAULT_WIDTH,
    height: int = DEFAULT_HEIGHT,
) -> str:
    """Create a ROS 2 OmniGraph that publishes the env_0 front depth camera."""

    import omni.graph.core as og
    from isaacsim.core.utils import extensions

    extensions.enable_extension("isaacsim.ros2.bridge")

    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("Ros2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("DepthCameraHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthCameraInfoHelper", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", camera_prim_path),
                ("CreateRenderProduct.inputs:enabled", True),
                ("CreateRenderProduct.inputs:width", width),
                ("CreateRenderProduct.inputs:height", height),
                ("DepthCameraHelper.inputs:topicName", depth_topic),
                ("DepthCameraHelper.inputs:frameId", frame_id),
                ("DepthCameraHelper.inputs:type", "depth"),
                ("DepthCameraHelper.inputs:resetSimulationTimeOnStop", False),
                ("DepthCameraInfoHelper.inputs:topicName", camera_info_topic),
                ("DepthCameraInfoHelper.inputs:frameId", frame_id),
                ("DepthCameraInfoHelper.inputs:resetSimulationTimeOnStop", False),
            ],
            keys.CONNECT: [
                ("Ros2Context.outputs:context", "DepthCameraHelper.inputs:context"),
                ("Ros2Context.outputs:context", "DepthCameraInfoHelper.inputs:context"),
                ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "DepthCameraHelper.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "DepthCameraInfoHelper.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "DepthCameraHelper.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "DepthCameraInfoHelper.inputs:renderProductPath"),
            ],
        },
    )
    return graph_path


def create_rgb_camera_ros_graph(
    camera_prim_path: str = DEFAULT_AUTO_RGB_CAMERA_PRIM,
    graph_path: str = DEFAULT_AUTO_RGB_GRAPH_PATH,
    rgb_topic: str = DEFAULT_AUTO_RGB_TOPIC,
    camera_info_topic: str = DEFAULT_AUTO_RGB_CAMERA_INFO_TOPIC,
    frame_id: str = DEFAULT_AUTO_RGB_FRAME_ID,
    width: int = DEFAULT_WIDTH,
    height: int = DEFAULT_HEIGHT,
) -> str:
    """Create a ROS 2 OmniGraph that publishes RGB images from a camera."""

    import omni.graph.core as og
    from isaacsim.core.utils import extensions

    extensions.enable_extension("isaacsim.ros2.bridge")

    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("Ros2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RgbCameraHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("RgbCameraInfoHelper", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", camera_prim_path),
                ("CreateRenderProduct.inputs:enabled", True),
                ("CreateRenderProduct.inputs:width", width),
                ("CreateRenderProduct.inputs:height", height),
                ("RgbCameraHelper.inputs:topicName", rgb_topic),
                ("RgbCameraHelper.inputs:frameId", frame_id),
                ("RgbCameraHelper.inputs:type", "rgb"),
                ("RgbCameraHelper.inputs:resetSimulationTimeOnStop", False),
                ("RgbCameraInfoHelper.inputs:topicName", camera_info_topic),
                ("RgbCameraInfoHelper.inputs:frameId", frame_id),
                ("RgbCameraInfoHelper.inputs:resetSimulationTimeOnStop", False),
            ],
            keys.CONNECT: [
                ("Ros2Context.outputs:context", "RgbCameraHelper.inputs:context"),
                ("Ros2Context.outputs:context", "RgbCameraInfoHelper.inputs:context"),
                ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "RgbCameraHelper.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "RgbCameraInfoHelper.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "RgbCameraHelper.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "RgbCameraInfoHelper.inputs:renderProductPath"),
            ],
        },
    )
    return graph_path


def create_auto_camera_ros_graphs(
    auto_depth_camera_prim_path: str = DEFAULT_AUTO_DEPTH_CAMERA_PRIM,
    auto_rgb_camera_prim_path: str = DEFAULT_AUTO_RGB_CAMERA_PRIM,
    auto_depth_topic: str = DEFAULT_AUTO_DEPTH_TOPIC,
    auto_rgb_topic: str = DEFAULT_AUTO_RGB_TOPIC,
    auto_depth_camera_info_topic: str = DEFAULT_AUTO_DEPTH_CAMERA_INFO_TOPIC,
    auto_rgb_camera_info_topic: str = DEFAULT_AUTO_RGB_CAMERA_INFO_TOPIC,
    auto_depth_frame_id: str = DEFAULT_AUTO_DEPTH_FRAME_ID,
    auto_rgb_frame_id: str = DEFAULT_AUTO_RGB_FRAME_ID,
    width: int = DEFAULT_WIDTH,
    height: int = DEFAULT_HEIGHT,
) -> tuple[str, str]:
    """Create ROS 2 OmniGraphs for A_camera_link depth and RGB cameras."""

    if auto_depth_camera_prim_path == auto_rgb_camera_prim_path:
        return create_depth_rgb_camera_ros_graph(
            camera_prim_path=auto_depth_camera_prim_path,
            graph_path=DEFAULT_AUTO_DEPTH_GRAPH_PATH,
            depth_topic=auto_depth_topic,
            rgb_topic=auto_rgb_topic,
            depth_camera_info_topic=auto_depth_camera_info_topic,
            rgb_camera_info_topic=auto_rgb_camera_info_topic,
            depth_frame_id=auto_depth_frame_id,
            rgb_frame_id=auto_rgb_frame_id,
            width=width,
            height=height,
        )

    auto_depth_graph_path = create_depth_camera_ros_graph(
        camera_prim_path=auto_depth_camera_prim_path,
        graph_path=DEFAULT_AUTO_DEPTH_GRAPH_PATH,
        depth_topic=auto_depth_topic,
        camera_info_topic=auto_depth_camera_info_topic,
        frame_id=auto_depth_frame_id,
        width=width,
        height=height,
    )
    auto_rgb_graph_path = create_rgb_camera_ros_graph(
        camera_prim_path=auto_rgb_camera_prim_path,
        graph_path=DEFAULT_AUTO_RGB_GRAPH_PATH,
        rgb_topic=auto_rgb_topic,
        camera_info_topic=auto_rgb_camera_info_topic,
        frame_id=auto_rgb_frame_id,
        width=width,
        height=height,
    )
    return auto_depth_graph_path, auto_rgb_graph_path


def create_depth_rgb_camera_ros_graph(
    camera_prim_path: str,
    graph_path: str,
    depth_topic: str,
    rgb_topic: str,
    depth_camera_info_topic: str,
    rgb_camera_info_topic: str,
    depth_frame_id: str,
    rgb_frame_id: str,
    width: int = DEFAULT_WIDTH,
    height: int = DEFAULT_HEIGHT,
) -> tuple[str, str]:
    """Create one render product and publish depth/RGB streams from the same camera."""

    import omni.graph.core as og
    from isaacsim.core.utils import extensions

    extensions.enable_extension("isaacsim.ros2.bridge")

    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("Ros2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("DepthCameraHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthCameraInfoHelper", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("RgbCameraHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("RgbCameraInfoHelper", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", camera_prim_path),
                ("CreateRenderProduct.inputs:enabled", True),
                ("CreateRenderProduct.inputs:width", width),
                ("CreateRenderProduct.inputs:height", height),
                ("DepthCameraHelper.inputs:topicName", depth_topic),
                ("DepthCameraHelper.inputs:frameId", depth_frame_id),
                ("DepthCameraHelper.inputs:type", "depth"),
                ("DepthCameraHelper.inputs:resetSimulationTimeOnStop", False),
                ("DepthCameraInfoHelper.inputs:topicName", depth_camera_info_topic),
                ("DepthCameraInfoHelper.inputs:frameId", depth_frame_id),
                ("DepthCameraInfoHelper.inputs:resetSimulationTimeOnStop", False),
                ("RgbCameraHelper.inputs:topicName", rgb_topic),
                ("RgbCameraHelper.inputs:frameId", rgb_frame_id),
                ("RgbCameraHelper.inputs:type", "rgb"),
                ("RgbCameraHelper.inputs:resetSimulationTimeOnStop", False),
                ("RgbCameraInfoHelper.inputs:topicName", rgb_camera_info_topic),
                ("RgbCameraInfoHelper.inputs:frameId", rgb_frame_id),
                ("RgbCameraInfoHelper.inputs:resetSimulationTimeOnStop", False),
            ],
            keys.CONNECT: [
                ("Ros2Context.outputs:context", "DepthCameraHelper.inputs:context"),
                ("Ros2Context.outputs:context", "DepthCameraInfoHelper.inputs:context"),
                ("Ros2Context.outputs:context", "RgbCameraHelper.inputs:context"),
                ("Ros2Context.outputs:context", "RgbCameraInfoHelper.inputs:context"),
                ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "DepthCameraHelper.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "DepthCameraInfoHelper.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "RgbCameraHelper.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "RgbCameraInfoHelper.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "DepthCameraHelper.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "DepthCameraInfoHelper.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "RgbCameraHelper.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "RgbCameraInfoHelper.inputs:renderProductPath"),
            ],
        },
    )
    return graph_path, graph_path


def create_auto_stereo_camera_ros_graphs(
    left_camera_prim_path: str = DEFAULT_AUTO_LEFT_CAMERA_PRIM,
    right_camera_prim_path: str = DEFAULT_AUTO_RIGHT_CAMERA_PRIM,
    graph_path: str = DEFAULT_AUTO_STEREO_GRAPH_PATH,
    left_image_topic: str = DEFAULT_AUTO_LEFT_IMAGE_TOPIC,
    right_image_topic: str = DEFAULT_AUTO_RIGHT_IMAGE_TOPIC,
    left_camera_info_topic: str = DEFAULT_AUTO_LEFT_CAMERA_INFO_TOPIC,
    right_camera_info_topic: str = DEFAULT_AUTO_RIGHT_CAMERA_INFO_TOPIC,
    left_frame_id: str = DEFAULT_AUTO_LEFT_FRAME_ID,
    right_frame_id: str = DEFAULT_AUTO_RIGHT_FRAME_ID,
    width: int = DEFAULT_WIDTH,
    height: int = DEFAULT_HEIGHT,
) -> str:
    """Create ROS 2 OmniGraph for ADAS stereo RGB images and stereo CameraInfo."""

    import omni.graph.core as og
    from isaacsim.core.utils import extensions

    extensions.enable_extension("isaacsim.ros2.bridge")

    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("Ros2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateLeftRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CreateRightRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("LeftImageHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("RightImageHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("StereoCameraInfoHelper", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                ("CreateLeftRenderProduct.inputs:cameraPrim", left_camera_prim_path),
                ("CreateLeftRenderProduct.inputs:enabled", True),
                ("CreateLeftRenderProduct.inputs:width", width),
                ("CreateLeftRenderProduct.inputs:height", height),
                ("CreateRightRenderProduct.inputs:cameraPrim", right_camera_prim_path),
                ("CreateRightRenderProduct.inputs:enabled", True),
                ("CreateRightRenderProduct.inputs:width", width),
                ("CreateRightRenderProduct.inputs:height", height),
                ("LeftImageHelper.inputs:topicName", left_image_topic),
                ("LeftImageHelper.inputs:frameId", left_frame_id),
                ("LeftImageHelper.inputs:type", "rgb"),
                ("LeftImageHelper.inputs:resetSimulationTimeOnStop", False),
                ("RightImageHelper.inputs:topicName", right_image_topic),
                ("RightImageHelper.inputs:frameId", right_frame_id),
                ("RightImageHelper.inputs:type", "rgb"),
                ("RightImageHelper.inputs:resetSimulationTimeOnStop", False),
                ("StereoCameraInfoHelper.inputs:topicName", left_camera_info_topic),
                ("StereoCameraInfoHelper.inputs:topicNameRight", right_camera_info_topic),
                ("StereoCameraInfoHelper.inputs:frameId", left_frame_id),
                ("StereoCameraInfoHelper.inputs:frameIdRight", right_frame_id),
                ("StereoCameraInfoHelper.inputs:resetSimulationTimeOnStop", False),
            ],
            keys.CONNECT: [
                ("Ros2Context.outputs:context", "LeftImageHelper.inputs:context"),
                ("Ros2Context.outputs:context", "RightImageHelper.inputs:context"),
                ("Ros2Context.outputs:context", "StereoCameraInfoHelper.inputs:context"),
                ("OnPlaybackTick.outputs:tick", "CreateLeftRenderProduct.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "CreateRightRenderProduct.inputs:execIn"),
                ("CreateLeftRenderProduct.outputs:execOut", "LeftImageHelper.inputs:execIn"),
                ("CreateRightRenderProduct.outputs:execOut", "RightImageHelper.inputs:execIn"),
                ("CreateLeftRenderProduct.outputs:execOut", "StereoCameraInfoHelper.inputs:execIn"),
                ("CreateLeftRenderProduct.outputs:renderProductPath", "LeftImageHelper.inputs:renderProductPath"),
                ("CreateRightRenderProduct.outputs:renderProductPath", "RightImageHelper.inputs:renderProductPath"),
                (
                    "CreateLeftRenderProduct.outputs:renderProductPath",
                    "StereoCameraInfoHelper.inputs:renderProductPath",
                ),
                (
                    "CreateRightRenderProduct.outputs:renderProductPath",
                    "StereoCameraInfoHelper.inputs:renderProductPathRight",
                ),
            ],
        },
    )
    return graph_path


def create_front_depth_camera_ros_graphs(
    front_camera_prim_path: str = DEFAULT_FRONT_DEPTH_CAMERA_PRIM,
    front_depth_topic: str = DEFAULT_FRONT_DEPTH_TOPIC,
    front_camera_info_topic: str = DEFAULT_FRONT_CAMERA_INFO_TOPIC,
    front_frame_id: str = DEFAULT_FRONT_FRAME_ID,
    width: int = DEFAULT_WIDTH,
    height: int = DEFAULT_HEIGHT,
) -> tuple[str, str]:
    """Create ROS 2 OmniGraphs that publish env_0 front depth cameras."""

    front_graph_path = create_depth_camera_ros_graph(
        camera_prim_path=front_camera_prim_path,
        graph_path=DEFAULT_FRONT_GRAPH_PATH,
        depth_topic=front_depth_topic,
        camera_info_topic=front_camera_info_topic,
        frame_id=front_frame_id,
        width=width,
        height=height,
    )
    return front_graph_path


def create_imu_ros_graph(
    graph_path: str = DEFAULT_IMU_GRAPH_PATH,
    topic_name: str = DEFAULT_IMU_TOPIC,
    frame_id: str = DEFAULT_IMU_FRAME_ID,
    derive_angular_velocity: bool = False,
) -> Ros2ImuPublisher:
    """Create a ROS 2 publisher for IMU data updated from Python."""

    node_name = graph_path.strip("/").lower() or "isaac_imu_bridge"
    return Ros2ImuPublisher(
        topic_name=topic_name,
        frame_id=frame_id,
        node_name=node_name,
        derive_angular_velocity=derive_angular_velocity,
    )


def create_clock_ros_graph(
    graph_path: str = DEFAULT_CLOCK_GRAPH_PATH,
) -> str:
    """Create a ROS 2 OmniGraph that publishes /clock from simulation time."""

    import omni.graph.core as og
    from isaacsim.core.utils import extensions

    extensions.enable_extension("isaacsim.ros2.bridge")

    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("Ros2Context", "isaacsim.ros2.bridge.ROS2Context"),
            ],
            keys.SET_VALUES: [
                ("ReadSimTime.inputs:resetOnStop", False),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Ros2Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
        },
    )
    return graph_path


def create_time_ros_graph(topic_name: str = DEFAULT_TIME_TOPIC) -> Ros2TimePublisher:
    """Create a ROS 2 publisher that publishes the simulation time as std_msgs/Float64."""

    return Ros2TimePublisher(topic_name)


def create_clock_publisher(topic_name: str = "/clock") -> Ros2ClockPublisher:
    """Create the rclpy publisher that owns simulation /clock."""

    return Ros2ClockPublisher(topic_name)


def create_front_camera_publisher(
    rgb_topic: str = "/f4/front_camera/rgb/image_raw",
    depth_topic: str = DEFAULT_FRONT_DEPTH_TOPIC,
    camera_info_topic: str = DEFAULT_FRONT_CAMERA_INFO_TOPIC,
    frame_id: str = DEFAULT_FRONT_FRAME_ID,
) -> Ros2CameraPublisher:
    """Create a ROS 2 Python publisher for front RGB/depth images."""

    return Ros2CameraPublisher(
        rgb_topic=rgb_topic,
        depth_topic=depth_topic,
        camera_info_topic=camera_info_topic,
        frame_id=frame_id,
    )


def create_command_user_bridge(
    topic_name: str = DEFAULT_COMMAND_USER_TOPIC,
    command_dim: int = 3,
    axis_names: list[str] | None = None,
    odom_frame_id: str = "odom",
    child_frame_id: str = DEFAULT_BASE_FRAME_ID,
    publish_enabled: bool = False,
) -> Ros2CommandUserBridge:
    """Create a ROS 2 CommandUser publisher/subscriber bridge."""

    return Ros2CommandUserBridge(
        topic_name=topic_name,
        command_dim=command_dim,
        axis_names=axis_names,
        odom_frame_id=odom_frame_id,
        child_frame_id=child_frame_id,
        publish_enabled=publish_enabled,
    )


def create_height_map_pointcloud_publisher(
    topic_name: str = DEFAULT_HEIGHT_MAP_TOPIC,
    frame_id: str = DEFAULT_HEIGHT_MAP_FRAME_ID,
) -> Ros2HeightMapPointCloudPublisher:
    """Create a ROS 2 publisher for env_0 RayCaster height-map hit points."""

    return Ros2HeightMapPointCloudPublisher(topic_name=topic_name, frame_id=frame_id)


def create_mid360_pointcloud_publisher(
    topic_name: str = DEFAULT_MID360_LIDAR_TOPIC,
    frame_id: str = DEFAULT_MID360_FRAME_ID,
    max_range_m: float = 15.0,
) -> Ros2Mid360PointCloudPublisher:
    """Create a ROS 2 publisher for env_0 RayCaster lidar hits."""

    return Ros2Mid360PointCloudPublisher(topic_name=topic_name, frame_id=frame_id, max_range_m=max_range_m)


def create_mid360_rtx_lidar_ros_graph(
    lidar_prim_path: str = DEFAULT_MID360_RTX_PRIM,
    graph_path: str = DEFAULT_MID360_RTX_GRAPH_PATH,
    topic_name: str = DEFAULT_MID360_LIDAR_TOPIC,
    frame_id: str = DEFAULT_MID360_FRAME_ID,
    config_name: str = DEFAULT_MID360_RTX_CONFIG,
    translation: tuple[float, float, float] = (0.24, 0.0, 0.12),
    orientation_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
) -> str:
    """Create an RTX LiDAR prim and ROS 2 PointCloud2 graph."""

    import omni.graph.core as og
    import omni.kit.commands
    from isaacsim.core.utils import extensions
    from isaacsim.core.utils.prims import get_prim_at_path
    from isaacsim.core.utils.prims import set_prim_visibility
    from pxr import Gf

    extensions.enable_extension("isaacsim.sensors.rtx")
    extensions.enable_extension("isaacsim.ros2.bridge")

    parent_path, lidar_name = lidar_prim_path.rsplit("/", 1)
    lidar_prim = get_prim_at_path(lidar_prim_path)
    if not lidar_prim.IsValid():
        result, lidar_prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=f"/{lidar_name}",
            parent=parent_path,
            config=config_name,
            translation=Gf.Vec3d(*translation),
            orientation=Gf.Quatd(*orientation_wxyz),
        )
        if not result:
            raise RuntimeError(f"Failed to create RTX LiDAR prim at {lidar_prim_path}")
    set_prim_visibility(prim=lidar_prim, visible=False)

    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("Ros2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PointCloudHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            keys.SET_VALUES: [
                ("RenderProduct.inputs:cameraPrim", lidar_prim_path),
                ("PointCloudHelper.inputs:topicName", topic_name),
                ("PointCloudHelper.inputs:type", "point_cloud"),
                ("PointCloudHelper.inputs:frameId", frame_id),
                ("PointCloudHelper.inputs:resetSimulationTimeOnStop", False),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "PointCloudHelper.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "PointCloudHelper.inputs:renderProductPath"),
                ("Ros2Context.outputs:context", "PointCloudHelper.inputs:context"),
            ],
        },
    )
    return graph_path


def create_robot_state_publisher(
    joint_states_topic: str = DEFAULT_JOINT_STATES_TOPIC,
    world_frame_id: str = DEFAULT_WORLD_FRAME_ID,
    base_frame_id: str = DEFAULT_BASE_FRAME_ID,
    path_gt_topic: str = DEFAULT_PATH_GT_TOPIC,
    gt_odom_topic: str = "/gt/lidar_odom",
    gt_child_frame_id: str = DEFAULT_MID360_FRAME_ID,
    path_gt_max_poses: int = 5000,
    publish_root_tf: bool = True,
    relative_to_initial_pose: bool = False,
    diagnostics_csv_path: str = "",
) -> Ros2RobotStatePublisher:
    """Create a ROS 2 publisher for JointState, root TF, and ground-truth path."""

    return Ros2RobotStatePublisher(
        joint_states_topic=joint_states_topic,
        world_frame_id=world_frame_id,
        base_frame_id=base_frame_id,
        path_gt_topic=path_gt_topic,
        gt_odom_topic=gt_odom_topic,
        gt_child_frame_id=gt_child_frame_id,
        path_gt_max_poses=path_gt_max_poses,
        publish_root_tf=publish_root_tf,
        relative_to_initial_pose=relative_to_initial_pose,
        diagnostics_csv_path=diagnostics_csv_path,
    )
