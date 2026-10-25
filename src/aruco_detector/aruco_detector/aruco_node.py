
import cv2
import cv2.aruco as aruco
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge


def rotation_matrix_to_quaternion(R):
    """Convert a 3x3 rotation matrix to quaternion (x, y, z, w)."""
    trace = np.trace(R)

    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    return x, y, z, w


class ArucoDetectorNode(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        # Parameters
        self.declare_parameter('image_topic',
                               '/zed/zed_node/rgb/color/rect/image')
        self.declare_parameter(
            'camera_info_topic',
            '/zed/zed_node/rgb/color/rect/camera_info'
        )
        self.declare_parameter('marker_size', 0.107)
        self.declare_parameter('dictionary', 'DICT_7X7_250')

        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.marker_size = float(
            self.get_parameter('marker_size').value
        )
        dictionary_name = self.get_parameter('dictionary').value

        # OpenCV ArUco detector
        dictionary_id = getattr(aruco, dictionary_name, None)
        if dictionary_id is None:
            raise ValueError(f'Unknown ArUco dictionary: {dictionary_name}')

        self.dictionary = aruco.getPredefinedDictionary(dictionary_id)
        self.detector_parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(
            self.dictionary, self.detector_parameters
        )

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.last_log_time = 0.0

        # Publishers
        self.pose_pub = self.create_publisher(
            PoseArray, '/aruco/poses', 10
        )
        self.image_pub = self.create_publisher(
            Image, '/aruco/annotated_image', 10
        )

        # Subscribe using sensor-data QoS for camera compatibility
        self.info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data
        )
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info('ArUco detector started')
        self.get_logger().info(f'Image topic: {image_topic}')
        self.get_logger().info(f'Camera info topic: {camera_info_topic}')
        self.get_logger().info(
            f'Marker size: {self.marker_size} m; dictionary: {dictionary_name}'
        )

    def camera_info_callback(self, msg):
        # P is the projection matrix for the rectified image.
        P = np.array(msg.p, dtype=np.float64).reshape(3, 4)
        K = P[:, :3]

        if K[0, 0] > 0 and K[1, 1] > 0:
            self.camera_matrix = K

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
        except Exception as exc:
            self.get_logger().error(f'Image conversion failed: {exc}')
            return

        annotated = frame.copy()
        poses_msg = PoseArray()
        poses_msg.header = msg.header

        if self.camera_matrix is not None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)

            if ids is not None:
                aruco.drawDetectedMarkers(annotated, corners, ids)

                half = self.marker_size / 2.0

                # Corner order required by SOLVEPNP_IPPE_SQUARE:
                # top-left, top-right, bottom-right, bottom-left.
                object_points = np.array([
                    [-half,  half, 0.0],
                    [ half,  half, 0.0],
                    [ half, -half, 0.0],
                    [-half, -half, 0.0]
                ], dtype=np.float32)

                detections = []

                # Rectified images use zero distortion here.
                distortion = np.zeros((5, 1), dtype=np.float64)

                for marker_corners, marker_id in zip(corners, ids.flatten()):
                    image_points = marker_corners.reshape(4, 2).astype(
                        np.float32
                    )

                    success, rvec, tvec = cv2.solvePnP(
                        object_points,
                        image_points,
                        self.camera_matrix,
                        distortion,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )

                    if not success:
                        continue

                    x, y, z = tvec.flatten()

                    # Skip invalid or behind-camera estimates.
                    if not np.isfinite([x, y, z]).all() or z <= 0:
                        continue

                    R, _ = cv2.Rodrigues(rvec)
                    qx, qy, qz, qw = rotation_matrix_to_quaternion(R)

                    pose = Pose()
                    pose.position.x = float(x)
                    pose.position.y = float(y)
                    pose.position.z = float(z)
                    pose.orientation.x = float(qx)
                    pose.orientation.y = float(qy)
                    pose.orientation.z = float(qz)
                    pose.orientation.w = float(qw)
                    poses_msg.poses.append(pose)

                    cv2.drawFrameAxes(
                        annotated,
                        self.camera_matrix,
                        distortion,
                        rvec,
                        tvec,
                        self.marker_size * 0.5
                    )

                    cv2.putText(
                        annotated,
                        f'ID: {int(marker_id)}  '
                        f'X:{x:.2f} Y:{y:.2f} Z:{z:.2f} m',
                        tuple(image_points[0].astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2
                    )

                    detections.append(
                        f'ID={int(marker_id)} '
                        f'x={x:.3f} y={y:.3f} z={z:.3f} m'
                    )

                # Avoid flooding the terminal at camera frame rate.
                now = self.get_clock().now().nanoseconds / 1e9
                if detections and now - self.last_log_time >= 1.0:
                    for detection in detections:
                        self.get_logger().info(detection)
                    self.last_log_time = now

        else:
            cv2.putText(
                annotated,
                'Waiting for camera calibration...',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

        self.pose_pub.publish(poses_msg)

        try:
            image_msg = Image()
            image_msg.header = msg.header
            image_msg.height = annotated.shape[0]
            image_msg.width = annotated.shape[1]
            image_msg.encoding = 'bgr8'
            image_msg.is_bigendian = 0
            image_msg.step = annotated.shape[1] * 3
            image_msg.data = annotated.tobytes()

            self.image_pub.publish(image_msg)

        except Exception as exc:
            self.get_logger().error(
                f'Annotated image publish failed: {type(exc).__name__}: {repr(exc)}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
