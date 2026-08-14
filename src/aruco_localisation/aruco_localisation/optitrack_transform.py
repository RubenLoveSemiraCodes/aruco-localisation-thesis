#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf_transformations import quaternion_matrix, quaternion_from_matrix


def transform_to_matrix(t):
    q = [
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w
    ]

    T = quaternion_matrix(q)

    T[0, 3] = t.transform.translation.x
    T[1, 3] = t.transform.translation.y
    T[2, 3] = t.transform.translation.z

    return T


def pose_to_matrix(pose):
    q = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ]

    T = quaternion_matrix(q)

    T[0, 3] = pose.position.x
    T[1, 3] = pose.position.y
    T[2, 3] = pose.position.z

    return T


class OptitrackTransform(Node):

    def __init__(self):
        super().__init__('optitrack_transform')

        # Change this to your actual OptiTrack topic
        self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/rigidbody1/pose',
            self.optitrack_callback,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.br = TransformBroadcaster(self)

        # -------------------------------------------------
        # KNOWN RIGID BODY -> ARUCO MARKER OFFSET
        #
        # Example:
        # marker centre is 10 cm along rigid body's +X axis
        #
        # B_T_M
        # -------------------------------------------------

        self.B_T_M = np.eye(4)

        self.B_T_M[0, 3] = 0.10
        self.B_T_M[1, 3] = 0.00
        self.B_T_M[2, 3] = 0.00

        # Camera -> OptiTrack world calibration
        self.C_T_O = None

    def optitrack_callback(self, msg):

        # ---------------------------------------------
        # OptiTrack world -> rigid body
        # ---------------------------------------------

        O_T_B = pose_to_matrix(msg.pose)

        # ---------------------------------------------
        # CALIBRATION
        #
        # Get camera -> marker from your ArUco TF
        # ---------------------------------------------

        if self.C_T_O is None:

            try:
                tf_marker = self.tf_buffer.lookup_transform(
                    'camera',
                    'marker_frame',
                    rclpy.time.Time()
                )

            except Exception:
                self.get_logger().info(
                    'Waiting for camera -> marker_frame...'
                )
                return

            C_T_M = transform_to_matrix(tf_marker)

            # -----------------------------------------
            # Solve:
            #
            # C_T_M = C_T_O * O_T_B * B_T_M
            #
            # therefore:
            #
            # C_T_O =
            # C_T_M * inv(B_T_M) * inv(O_T_B)
            # -----------------------------------------

            self.C_T_O = (
                C_T_M
                @ np.linalg.inv(self.B_T_M)
                @ np.linalg.inv(O_T_B)
            )

            self.get_logger().info(
                'Camera -> OptiTrack world calibrated!'
            )

        # ---------------------------------------------
        # NORMAL OPERATION
        #
        # camera -> rigid body
        #
        # C_T_B = C_T_O * O_T_B
        # ---------------------------------------------

        C_T_B = self.C_T_O @ O_T_B

        # ---------------------------------------------
        # Publish camera -> rigid_body
        # ---------------------------------------------

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera'
        t.child_frame_id = 'rigid_body_camera'

        t.transform.translation.x = float(C_T_B[0, 3])
        t.transform.translation.y = float(C_T_B[1, 3])
        t.transform.translation.z = float(C_T_B[2, 3])

        q = quaternion_from_matrix(C_T_B)

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    node = OptitrackTransform()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()