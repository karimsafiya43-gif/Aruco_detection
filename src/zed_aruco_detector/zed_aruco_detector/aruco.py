
#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rclpy
import message_filters

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Int32MultiArray, Float32MultiArray

from cv_bridge import CvBridge


class ZEDArucoDepthNode(Node):

    def __init__(self):
        super().__init__('zed_aruco_depth_node')

        # --------------------------------------------------
        # PARAMETERS
        # --------------------------------------------------

        self.declare_parameter('marker_size', 0.20)
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('show_window', True)

        self.declare_parameter('rgb_topic',
            '/zed/zed_node/rgb/color/rect/image')
        self.declare_parameter('depth_topic',
            '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('camera_info_topic',
            '/zed/zed_node/rgb/color/rect/camera_info')

        self.declare_parameter('sync_slop', 0.10)
        self.declare_parameter('min_valid_depth_pixels', 5)
        self.declare_parameter('max_reprojection_error', 2.0)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('log_every_n_frames', 10)

        self.marker_size = float(
            self.get_parameter('marker_size').value)
        self.show_window = bool(
            self.get_parameter('show_window').value)

        self.rgb_topic = str(
            self.get_parameter('rgb_topic').value)
        self.depth_topic = str(
            self.get_parameter('depth_topic').value)
        self.camera_info_topic = str(
            self.get_parameter('camera_info_topic').value)

        self.sync_slop = float(
            self.get_parameter('sync_slop').value)
        self.min_valid_depth_pixels = int(
            self.get_parameter('min_valid_depth_pixels').value)
        self.max_reprojection_error = float(
            self.get_parameter('max_reprojection_error').value)
        self.max_range = float(
            self.get_parameter('max_range').value)
        self.log_every_n_frames = max(
            1, int(self.get_parameter('log_every_n_frames').value))

        if self.marker_size <= 0:
            raise ValueError('marker_size must be positive')

        if self.sync_slop < 0:
            raise ValueError('sync_slop cannot be negative')

        if self.max_reprojection_error <= 0:
            raise ValueError('max_reprojection_error must be positive')

        self.bridge = CvBridge()
        self.frame_count = 0

        # --------------------------------------------------
        # ARUCO DICTIONARY AND DETECTOR
        # --------------------------------------------------

        dictionary_name = str(
            self.get_parameter('dictionary').value)

        if not hasattr(cv2.aruco, dictionary_name):
            raise ValueError(
                f'Unsupported ArUco dictionary: {dictionary_name}')

        dictionary_id = getattr(cv2.aruco, dictionary_name)

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            dictionary_id)

        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.detector_parameters = (
                cv2.aruco.DetectorParameters())
        else:
            self.detector_parameters = (
                cv2.aruco.DetectorParameters_create())

        p = self.detector_parameters

        # Candidate marker size and shape
        p.minMarkerPerimeterRate = 0.03
        p.maxMarkerPerimeterRate = 1.0
        p.minDistanceToBorder = 5
        p.polygonalApproxAccuracyRate = 0.03

        # Adaptive thresholding
        p.adaptiveThreshWinSizeMin = 3
        p.adaptiveThreshWinSizeMax = 27
        p.adaptiveThreshWinSizeStep = 4
        p.adaptiveThreshConstant = 7

        # Subpixel corner localization
        p.cornerRefinementMethod = (
            cv2.aruco.CORNER_REFINE_SUBPIX)
        p.cornerRefinementMaxIterations = 50
        p.cornerRefinementMinAccuracy = 0.01

        # Small-marker decoding
        p.perspectiveRemovePixelPerCell = 6
        p.perspectiveRemoveIgnoredMarginPerCell = 0.13
        p.errorCorrectionRate = 0.5
        p.maxErroneousBitsInBorderRate = 0.25

        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(
                self.dictionary, p)
        else:
            self.detector = None

        # IPPE_SQUARE requires this exact object-point order.
        half = self.marker_size / 2.0

        self.object_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0]
        ], dtype=np.float32)

        # --------------------------------------------------
        # SYNCHRONIZED CAMERA SUBSCRIBERS
        # --------------------------------------------------

        self.rgb_sub = message_filters.Subscriber(
            self,
            Image,
            self.rgb_topic,
            qos_profile=qos_profile_sensor_data)

        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=qos_profile_sensor_data)

        self.info_sub = message_filters.Subscriber(
            self,
            CameraInfo,
            self.camera_info_topic,
            qos_profile=qos_profile_sensor_data)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.info_sub],
            queue_size=10,
            slop=self.sync_slop)

        self.sync.registerCallback(self.image_callback)

        # --------------------------------------------------
        # PUBLISHERS
        # --------------------------------------------------

        self.pose_pub = self.create_publisher(
            PoseArray, '/aruco/poses', 10)

        self.ids_pub = self.create_publisher(
            Int32MultiArray, '/aruco/ids', 10)

        # Each array entry has this order:
        # [id, pnp_range, pnp_z, depth_range, depth_z,
        #  reprojection_error, valid_depth_pixel_count]
        # Unavailable depth values are NaN.
        self.measurements_pub = self.create_publisher(
            Float32MultiArray, '/aruco/measurements', 10)

        self.get_logger().info(
            'ZED ArUco node started | '
            f'dictionary={dictionary_name} | '
            f'marker_size={self.marker_size:.3f} m')

    # ==================================================
    # DEPTH CONVERSION
    # ==================================================

    def convert_depth(self, msg):

        depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

        if msg.encoding == '32FC1':
            return np.asarray(depth, dtype=np.float32)

        if msg.encoding == '16UC1':
            # ZED 16-bit registered depth is interpreted here
            # as millimetres. Verify the encoding/units on your
            # actual driver before using this branch.
            return np.asarray(depth, dtype=np.float32) / 1000.0

        raise ValueError(
            f'Unsupported depth encoding: {msg.encoding}')

    # ==================================================
    # ROBUST DEPTH SAMPLE INSIDE THE MARKER
    # ==================================================

    def get_marker_depth(self, depth, corners):

        height, width = depth.shape[:2]
        points = corners.reshape(4, 2).astype(np.float32)

        if not np.all(np.isfinite(points)):
            return None, 0

        polygon = np.round(points).astype(np.int32)

        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(mask, [polygon], 255)

        # Avoid the black border and surrounding background.
        # Reduce erosion for very small image-space markers.
        side_pixels = np.linalg.norm(
            points[1] - points[0])

        erosion_size = 5 if side_pixels >= 40 else 3
        kernel = np.ones(
            (erosion_size, erosion_size), dtype=np.uint8)

        mask = cv2.erode(mask, kernel, iterations=1)

        values = depth[mask > 0]

        valid = values[
            np.isfinite(values) &
            (values > 0.0) &
            (values <= self.max_range)
        ]

        if valid.size < self.min_valid_depth_pixels:
            return None, int(valid.size)

        return float(np.median(valid)), int(valid.size)

    # ==================================================
    # PIXEL + OPTICAL-AXIS DEPTH TO CAMERA XYZ
    # ==================================================

    @staticmethod
    def depth_to_3d(u, v, z, K):

        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]

        if fx <= 0 or fy <= 0 or z <= 0:
            return None

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        return np.array([x, y, z], dtype=np.float64)

    # ==================================================
    # ROTATION MATRIX TO QUATERNION
    # ==================================================

    @staticmethod
    def rotation_to_quaternion(R):

        trace = np.trace(R)

        if trace > 0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s

        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = math.sqrt(
                1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s

        elif R[1, 1] > R[2, 2]:
            s = math.sqrt(
                1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] + R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s

        else:
            s = math.sqrt(
                1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] + R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s

        quaternion = np.array([qx, qy, qz, qw])

        norm = np.linalg.norm(quaternion)
        if norm > 0:
            quaternion /= norm

        return tuple(float(x) for x in quaternion)

    # ==================================================
    # MAIN CALLBACK
    # ==================================================

    def image_callback(self, rgb_msg, depth_msg, info_msg):

        self.frame_count += 1

        try:
            image = self.bridge.imgmsg_to_cv2(
                rgb_msg, desired_encoding='bgr8')

            depth = self.convert_depth(depth_msg)

        except Exception as exc:
            self.get_logger().error(
                f'Image conversion failed: {exc}')
            return

        if depth.shape[:2] != image.shape[:2]:
            self.get_logger().error(
                'RGB/depth dimensions differ. '
                'Use registered depth aligned to rectified RGB.')
            return

        K = np.asarray(
            info_msg.k, dtype=np.float64).reshape(3, 3)

        if K[0, 0] <= 0 or K[1, 1] <= 0:
            self.get_logger().warning(
                'Invalid camera intrinsics in CameraInfo')
            return

        # The selected image is rectified. Its matching CameraInfo
        # must correspond to this same rectified image.
        D = np.zeros((5, 1), dtype=np.float64)

        if self.detector is not None:
            corners, ids, rejected = (
                self.detector.detectMarkers(image))
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                image, self.dictionary,
                parameters=self.detector_parameters)

        poses = PoseArray()
        poses.header = rgb_msg.header
        poses.header.frame_id = (
            info_msg.header.frame_id or
            rgb_msg.header.frame_id)

        marker_ids = []
        measurement_data = []

        if ids is not None:

            cv2.aruco.drawDetectedMarkers(image, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):

                image_points = corners[i].reshape(
                    4, 2).astype(np.float32)

                center = image_points.mean(axis=0)
                u, v = np.round(center).astype(int)

                if not (
                    0 <= u < image.shape[1] and
                    0 <= v < image.shape[0]
                ):
                    continue

                depth_z, valid_depth_count = (
                    self.get_marker_depth(depth, corners[i]))

                depth_position = None
                depth_range = float('nan')

                if depth_z is not None:
                    depth_position = self.depth_to_3d(
                        float(center[0]),
                        float(center[1]),
                        depth_z,
                        K)

                    if depth_position is not None:
                        depth_range = float(
                            np.linalg.norm(depth_position))

                # ------------------------------------------
                # PNP POSE: inspect planar candidate solutions
                # ------------------------------------------

                try:
                    result = cv2.solvePnPGeneric(
                        self.object_points,
                        image_points,
                        K,
                        D,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE)

                    success = bool(result[0])

                    if not success:
                        continue

                    rvecs = result[1]
                    tvecs = result[2]

                except (cv2.error, IndexError, TypeError) as exc:
                    self.get_logger().warning(
                        f'PnP failed for ID {marker_id}: {exc}')
                    continue

                candidates = []

                for rvec_candidate, tvec_candidate in zip(
                    rvecs, tvecs
                ):
                    t = np.asarray(
                        tvec_candidate, dtype=np.float64).reshape(3)

                    if not np.all(np.isfinite(t)):
                        continue

                    if t[2] <= 0:
                        continue

                    if np.linalg.norm(t) > self.max_range:
                        continue

                    projected, _ = cv2.projectPoints(
                        self.object_points,
                        rvec_candidate,
                        tvec_candidate,
                        K,
                        D)

                    projected = projected.reshape(4, 2)

                    errors = np.linalg.norm(
                        projected - image_points, axis=1)

                    reprojection_error = float(np.mean(errors))

                    if not np.isfinite(reprojection_error):
                        continue

                    candidates.append((
                        reprojection_error,
                        rvec_candidate,
                        tvec_candidate))

                if not candidates:
                    continue

                # Choose the positive-depth candidate with the
                # lowest reprojection error.
                candidates.sort(key=lambda item: item[0])

                reprojection_error, rvec, tvec = candidates[0]

                if reprojection_error > self.max_reprojection_error:
                    self.get_logger().warning(
                        f'ID={marker_id} rejected: '
                        f'reprojection error='
                        f'{reprojection_error:.2f}px')
                    continue

                R, _ = cv2.Rodrigues(rvec)

                x, y, z = np.asarray(
                    tvec, dtype=np.float64).reshape(3)

                pnp_range = float(np.linalg.norm(tvec))

                pose = Pose()
                pose.position.x = float(x)
                pose.position.y = float(y)
                pose.position.z = float(z)

                qx, qy, qz, qw = (
                    self.rotation_to_quaternion(R))

                pose.orientation.x = qx
                pose.orientation.y = qy
                pose.orientation.z = qz
                pose.orientation.w = qw

                poses.poses.append(pose)
                marker_ids.append(int(marker_id))

                measurement_data.extend([
                    float(marker_id),
                    pnp_range,
                    float(z),
                    depth_range,
                    depth_z if depth_z is not None else float('nan'),
                    reprojection_error,
                    float(valid_depth_count)
                ])

                cv2.drawFrameAxes(
                    image, K, D, rvec, tvec,
                    self.marker_size * 0.5)

                cv2.circle(
                    image, (u, v), 4, (0, 255, 0), -1)

                depth_text = (
                    f'{depth_z:.2f}m'
                    if depth_z is not None else 'N/A')

                label = (
                    f'ID:{marker_id} '
                    f'PnP:{pnp_range:.2f}m '
                    f'Depth:{depth_text}')

                cv2.putText(
                    image, label,
                    (max(5, u - 100), max(25, v - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 255, 0), 2)

                if (
                    self.frame_count % self.log_every_n_frames == 0
                ):
                    if depth_position is not None:
                        self.get_logger().info(
                            f'ID={marker_id} | '
                            f'PnP XYZ=({x:.3f},{y:.3f},{z:.3f})m | '
                            f'PnP range={pnp_range:.3f}m | '
                            f'Depth Z={depth_z:.3f}m | '
                            f'Depth range={depth_range:.3f}m | '
                            f'|PnP Z-depth Z|={abs(z-depth_z):.3f}m | '
                            f'Reprojection={reprojection_error:.2f}px | '
                            f'Valid depth pixels={valid_depth_count}'
                        )
                    else:
                        self.get_logger().info(
                            f'ID={marker_id} | '
                            f'PnP range={pnp_range:.3f}m | '
                            f'Depth unavailable | '
                            f'Reprojection={reprojection_error:.2f}px'
                        )

        self.pose_pub.publish(poses)

        id_msg = Int32MultiArray()
        id_msg.data = marker_ids
        self.ids_pub.publish(id_msg)

        measurement_msg = Float32MultiArray()
        measurement_msg.data = measurement_data
        self.measurements_pub.publish(measurement_msg)

        if self.show_window:
            cv2.imshow('ZED 2i ArUco Range Estimation', image)
            cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)
    node = None

    try:
        node = ZEDArucoDepthNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        print(f'Fatal ArUco node error: {exc}')
        raise

    finally:
        cv2.destroyAllWindows()

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

