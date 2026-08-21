##!/usr/bin/env python3

import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped


def quaternion_to_matrix(qx, qy, qz, qw):
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    q = q / np.linalg.norm(q)

    x, y, z, w = q

    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ])

    return R


def matrix_to_quaternion(R):
    trace = np.trace(R)

    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s

    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s

    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s

    else:
        s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    return np.array([qx, qy, qz, qw])


def pose_to_matrix(pose):
    T = np.eye(4)

    T[:3, :3] = quaternion_to_matrix(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    )

    T[0, 3] = pose.position.x
    T[1, 3] = pose.position.y
    T[2, 3] = pose.position.z

    return T


class OptitrackTransform(Node):

    def __init__(self):
        super().__init__('optitrack_transform')

        self.aruco_pose = None
        self.optitrack_pose = None

        # Camera -> OptiTrack world
        self.C_T_O = None

        # --------------------------------------------------
        # KNOWN RIGID BODY -> ARUCO MARKER TRANSFORM
        #
        # CHANGE THESE TO YOUR MEASURED VALUES.
        #
        # Example below:
        # marker is 10 cm along rigid body's +X axis
        # and axes are aligned.
        # --------------------------------------------------

        self.B_T_M = np.eye(4)
        
        theta = np.deg2rad(90)

        self.B_T_M[:3, :3] = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0,              0,             1]
        ])

        self.B_T_M[0, 3] = 0.19
        self.B_T_M[1, 3] = 0.075
        self.B_T_M[2, 3] = 0.0146

        # --------------------------------------------------
        # ArUco subscription
        # --------------------------------------------------

        self.create_subscription(
            PoseStamped,
            '/aruco/pose',
            self.aruco_callback,
            10
        )

        # --------------------------------------------------
        # OptiTrack QoS
        # VRPN publisher is BEST_EFFORT
        # --------------------------------------------------

        optitrack_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/rigidbody1/pose',
            self.optitrack_callback,
            optitrack_qos
        )
        
        self.body_pose_pub = self.create_publisher(
            PoseStamped,
            '/optitrack/body_pose_camera',
            10
        )

        # --------------------------------------------------
        # Output:
        # OptiTrack estimate of MARKER pose in CAMERA frame
        # --------------------------------------------------

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/optitrack/marker_pose_camera',
            10
        )

        self.get_logger().info('OptiTrack comparison node started')
        self.get_logger().info('Waiting for ArUco and OptiTrack poses...')


    def aruco_callback(self, msg):
        self.aruco_pose = msg


    def optitrack_callback(self, msg):
        self.optitrack_pose = msg

        if self.aruco_pose is None:
            return

        # ArUco:
        # camera -> marker
        C_T_M = pose_to_matrix(
            self.aruco_pose.pose
        )

        # OptiTrack:
        # OptiTrack world -> rigid body
        
        O_T_B = pose_to_matrix(
            self.optitrack_pose.pose
        )
        
        # TEST
        
        # O_T_B_raw = pose_to_matrix(
        #     self.optitrack_pose.pose
        # )

        # # Axis correction between OptiTrack/VRPN and camera-style coordinates
        # R_axis = np.array([
        #     [1,  0,  0],
        #     [0,  0, 1],
        #     [0,  -1,  0]
        # ])

        # T_axis = np.eye(4)
        # T_axis[:3, :3] = R_axis

        # O_T_B = T_axis @ O_T_B_raw

        # --------------------------------------------------
        # CALIBRATION
        #
        # C_T_M = C_T_O * O_T_B * B_T_M
        #
        # therefore:
        #
        # C_T_O =
        # C_T_M * inv(B_T_M) * inv(O_T_B)
        # --------------------------------------------------

        if self.C_T_O is None:

    # --------------------------------------------------
    # 1. FIXED AXIS MAPPING
    # OptiTrack world -> camera frame
    # --------------------------------------------------

            R_C_O = np.array([
                [1, 0, 0],
                [0, -1, 0],
                [0, 0, -1]
            ], dtype=np.float64)

            # --------------------------------------------------
            # 2. Find marker position according to OptiTrack
            #    BEFORE translating into camera frame
            #
            # O_T_M = O_T_B * B_T_M
            # --------------------------------------------------

            O_T_M = O_T_B @ self.B_T_M

            p_O_M = O_T_M[:3, 3]
            p_C_M = C_T_M[:3, 3]

            # --------------------------------------------------
            # 3. Calculate TRANSLATION ONLY
            #
            # p_C_M = R_C_O * p_O_M + t_C_O
            #
            # therefore:
            #
            # t_C_O = p_C_M - R_C_O * p_O_M
            # --------------------------------------------------

            t_C_O = p_C_M - (R_C_O @ p_O_M)

            # --------------------------------------------------
            # 4. Construct complete C_T_O
            # --------------------------------------------------

            self.C_T_O = np.eye(4)

            self.C_T_O[:3, :3] = R_C_O
            self.C_T_O[:3, 3] = t_C_O

            self.get_logger().info(
                f'Calibration complete\nC_T_O =\n{self.C_T_O}'
            )

        # --------------------------------------------------
        # Transform OptiTrack rigid body into camera frame
        #
        # C_T_B = C_T_O * O_T_B
        # --------------------------------------------------

        C_T_B = (
            self.C_T_O
            @ O_T_B
        )

        # --------------------------------------------------
        # Move from rigid-body origin to marker origin
        #
        # C_T_M_opti = C_T_B * B_T_M
        # --------------------------------------------------
        
        # --------------------------------------------------
        # TEST MAKER OFFSET IN RVIZ - START
        
        # B_T_M_test = self.B_T_M.copy()

        # B_T_M_test[0, 3] += 1.0

        # C_T_M_opti = (
        #     C_T_B
        #     @ B_T_M_test
        # )
        
        # TEST MAKER OFFSET IN RVIZ - END
        # --------------------------------------------------

        C_T_M_opti = (
            C_T_B
            @ self.B_T_M
        )
        
        # Publish rigid-body origin in camera frame
        body_output = PoseStamped()

        body_output.header.stamp = msg.header.stamp
        body_output.header.frame_id = self.aruco_pose.header.frame_id

        body_output.pose.position.x = float(C_T_B[0, 3])
        body_output.pose.position.y = float(C_T_B[1, 3])
        body_output.pose.position.z = float(C_T_B[2, 3])

        q_body = matrix_to_quaternion(C_T_B[:3, :3])

        body_output.pose.orientation.x = float(q_body[0])
        body_output.pose.orientation.y = float(q_body[1])
        body_output.pose.orientation.z = float(q_body[2])
        body_output.pose.orientation.w = float(q_body[3])

        self.body_pose_pub.publish(body_output)

        # --------------------------------------------------
        # Publish
        # --------------------------------------------------

        output = PoseStamped()

        output.header.stamp = msg.header.stamp

        # Same frame as ArUco
        output.header.frame_id = (
            self.aruco_pose.header.frame_id
        )

        output.pose.position.x = float(
            C_T_M_opti[0, 3]
        )

        output.pose.position.y = float(
            C_T_M_opti[1, 3]
        )

        output.pose.position.z = float(
            C_T_M_opti[2, 3]
        )

        q = matrix_to_quaternion(
            C_T_M_opti[:3, :3]
        )

        output.pose.orientation.x = float(q[0])
        output.pose.orientation.y = float(q[1])
        output.pose.orientation.z = float(q[2])
        output.pose.orientation.w = float(q[3])

        self.pose_pub.publish(output)


def main(args=None):

    rclpy.init(args=args)

    node = OptitrackTransform()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()