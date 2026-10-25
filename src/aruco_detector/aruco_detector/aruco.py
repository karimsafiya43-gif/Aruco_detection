import math

import cv2
import cv2.aruco as aruco
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Quaternion

from cv_bridge import CvBridge


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = CvBridge()

        self.camera_matrix = None
        self.dist_coeffs = None

        # Actual side length of the printed marker, in metres.
        self.declare_parameter('marker_size', 0.107)
        self.marker_size = float(
            self.get_parameter('marker_size').value
        )

        # Set these to the actual topics published by your ZED wrapper.
        self.declare_parameter(
            'image_topic',
            '/zed/zed_node/rgb/color/rect/image'
        )

        self.declare_parameter(
            'camera_info_topic',
            '/zed/zed_node/rgb/color/rect/camera_info'
        )

        image_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value

        # Subscribe to the camera image.
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        # Subscribe to the matching camera calibration.
        self.info_sub = self.create_subscription(
            CameraInfo,
            info_topic,
            self.info_callback,
            qos_profile_sensor_data
        )

        # ArUco detector.
        self.aruco_dict = aruco.getPredefinedDictionary(
            aruco.DICT_7X7_250
        )

        self.detector_params = aruco.DetectorParameters()

        self.detector = aruco.ArucoDetector(
            self.aruco_dict,
            self.detector_params
        )

        # Publish marker poses.
        self.pose_pub = self.create_publisher(
            PoseArray,
            '/aruco/poses',
            10
        )

        self.get_logger().info(
            'ArUco detection and pose estimation started.'
        )

        self.get_logger().info(
            f'Image topic: {image_topic}'
        )

        self.get_logger().info(
            f'Camera info topic: {info_topic}'
        )

    def info_callback(self, msg):

        # Intrinsic camera matrix.
        self.camera_matrix = np.array(
            msg.k, dtype=np.float64
        ).reshape(3, 3)

        # Distortion coefficients.
        self.dist_coeffs = np.array(
            msg.d, dtype=np.float64
        )

    @staticmethod
    def rotation_matrix_to_rpy(R):

        # Convert rotation matrix to roll, pitch, yaw.
        sy = math.sqrt(
            R[0, 0] ** 2 + R[1, 0] ** 2
        )

        singular = sy < 1e-6

        if not singular:

            roll = math.atan2(
                R[2, 1], R[2, 2]
            )

            pitch = math.atan2(
                -R[2, 0], sy
            )

            yaw = math.atan2(
                R[1, 0], R[0, 0]
            )

        else:

            roll = math.atan2(
                -R[1, 2], R[1, 1]
            )

            pitch = math.atan2(
                -R[2, 0], sy
            )

            yaw = 0.0

        return roll, pitch, yaw

    @staticmethod
    def rotation_matrix_to_quaternion(R):

        # Convert rotation matrix to a quaternion.
        trace = np.trace(R)

        if trace > 0:

            s = math.sqrt(trace + 1.0) * 2.0

            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s

        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:

            s = math.sqrt(
                1.0 + R[0, 0] - R[1, 1] - R[2, 2]
            ) * 2.0

            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s

        elif R[1, 1] > R[2, 2]:

            s = math.sqrt(
                1.0 + R[1, 1] - R[0, 0] - R[2, 2]
            ) * 2.0

            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s

        else:

            s = math.sqrt(
                1.0 + R[2, 2] - R[0, 0] - R[1, 1]
            ) * 2.0

            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s

        q = np.array([qx, qy, qz, qw], dtype=float)

        # Normalize to avoid numerical rounding errors.
        q /= np.linalg.norm(q)

        return q

    def image_callback(self, msg):

        if self.camera_matrix is None:
            return

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as e:

            self.get_logger().error(
                f'Image conversion failed: {e}'
            )
            return

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        corners, ids, _ = self.detector.detectMarkers(gray)

        # Preserve the image's frame and timestamp.
        pose_array = PoseArray()
        pose_array.header = msg.header

        if ids is not None:

            ids = ids.flatten()

            aruco.drawDetectedMarkers(
                frame,
                corners,
                ids
            )

            try:

                rvecs, tvecs, _ = (
                    aruco.estimatePoseSingleMarkers(
                        corners,
                        self.marker_size,
                        self.camera_matrix,
                        self.dist_coeffs
                    )
                )

                for i, marker_id in enumerate(ids):

                    rvec = rvecs[i][0]
                    tvec = tvecs[i][0]

                    # 3D translation.
                    x = float(tvec[0])
                    y = float(tvec[1])
                    z = float(tvec[2])

                    # Rotation vector -> rotation matrix.
                    R, _ = cv2.Rodrigues(rvec)

                    # Rotation matrix -> roll, pitch, yaw.
                    roll, pitch, yaw = (
                        self.rotation_matrix_to_rpy(R)
                    )

                    # Rotation matrix -> quaternion.
                    qx, qy, qz, qw = (
                        self.rotation_matrix_to_quaternion(R)
                    )

                    # Draw the marker's coordinate axes.
                    cv2.drawFrameAxes(
                        frame,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec,
                        tvec,
                        self.marker_size * 0.5
                    )

                    # Add position and orientation to the image.
                    c = corners[i][0]

                    center_x = int(np.mean(c[:, 0]))
                    center_y = int(np.mean(c[:, 1]))

                    cv2.putText(
                        frame,
                        f'ID: {marker_id}',
                        (center_x, center_y - 35),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )

                    cv2.putText(
                        frame,
                        f'XYZ: {x:.3f}, {y:.3f}, {z:.3f} m',
                        (center_x - 80, center_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 255),
                        2
                    )

                    # Pose message.
                    pose = Pose()

                    pose.position.x = x
                    pose.position.y = y
                    pose.position.z = z

                    pose.orientation = Quaternion(
                        x=float(qx),
                        y=float(qy),
                        z=float(qz),
                        w=float(qw)
                    )

                    pose_array.poses.append(pose)

                    # Log orientation in degrees for readability.
                    self.get_logger().info(
                        f'ID={marker_id} | '
                        f'Position: X={x:.3f}, '
                        f'Y={y:.3f}, Z={z:.3f} m | '
                        f'Roll={math.degrees(roll):.1f}, '
                        f'Pitch={math.degrees(pitch):.1f}, '
                        f'Yaw={math.degrees(yaw):.1f} deg'
                    )

            except Exception as e:

                self.get_logger().error(
                    f'Pose estimation failed: {e}'
                )

        # Publish an empty PoseArray if no markers were found.
        self.pose_pub.publish(pose_array)

        cv2.imshow(
            'ArUco Detection and Pose Estimation',
            frame
        )

        cv2.waitKey(1)

    def destroy_node(self):

        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = ArucoDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
