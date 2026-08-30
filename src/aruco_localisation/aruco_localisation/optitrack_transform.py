#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
import numpy as np


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def quaternion_to_rotation_matrix(qx, qy, qz, qw):

    return np.array([
        [
            1 - 2 * (qy*qy + qz*qz),
            2 * (qx*qy - qz*qw),
            2 * (qx*qz + qy*qw)
        ],
        [
            2 * (qx*qy + qz*qw),
            1 - 2 * (qx*qx + qz*qz),
            2 * (qy*qz - qx*qw)
        ],
        [
            2 * (qx*qz - qy*qw),
            2 * (qy*qz + qx*qw),
            1 - 2 * (qx*qx + qy*qy)
        ]
    ])


def rotation_matrix_to_quaternion(R):

    trace = np.trace(R)

    if trace > 0:

        s = np.sqrt(trace + 1.0) * 2

        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s

    elif (
        R[0, 0] > R[1, 1]
        and R[0, 0] > R[2, 2]
    ):

        s = np.sqrt(
            1.0
            + R[0, 0]
            - R[1, 1]
            - R[2, 2]
        ) * 2

        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s

    elif R[1, 1] > R[2, 2]:

        s = np.sqrt(
            1.0
            + R[1, 1]
            - R[0, 0]
            - R[2, 2]
        ) * 2

        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s

    else:

        s = np.sqrt(
            1.0
            + R[2, 2]
            - R[0, 0]
            - R[1, 1]
        ) * 2

        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    q = np.array([qx, qy, qz, qw])

    q = q / np.linalg.norm(q)

    return q


def pose_to_matrix(pose):

    qx = pose.orientation.x
    qy = pose.orientation.y
    qz = pose.orientation.z
    qw = pose.orientation.w

    R = quaternion_to_rotation_matrix(
        qx,
        qy,
        qz,
        qw
    )

    T = np.eye(4)

    T[:3, :3] = R

    T[:3, 3] = np.array([
        pose.position.x,
        pose.position.y,
        pose.position.z
    ])

    return T


def matrix_to_pose_stamped(T, stamp, frame_id):

    msg = PoseStamped()

    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    msg.pose.position.x = float(T[0, 3])
    msg.pose.position.y = float(T[1, 3])
    msg.pose.position.z = float(T[2, 3])

    q = rotation_matrix_to_quaternion(
        T[:3, :3]
    )

    msg.pose.orientation.x = float(q[0])
    msg.pose.orientation.y = float(q[1])
    msg.pose.orientation.z = float(q[2])
    msg.pose.orientation.w = float(q[3])

    return msg


# ============================================================
# NODE
# ============================================================

class OptiTrackTransform(Node):

    def __init__(self):

        super().__init__('optitrack_transform')
        
        # TF broadcaster for RViz
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # ============================================================
        # OPTITRACK WORLD TF FOR RVIZ
        # ============================================================

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        world_tf = TransformStamped()

        world_tf.header.stamp = self.get_clock().now().to_msg()

        # RViz root frame
        world_tf.header.frame_id = 'map'

        # OptiTrack world frame
        world_tf.child_frame_id = 'world'

        # Identity transform
        world_tf.transform.translation.x = 0.0
        world_tf.transform.translation.y = 0.0
        world_tf.transform.translation.z = 0.0

        world_tf.transform.rotation.x = 0.0
        world_tf.transform.rotation.y = 0.0
        world_tf.transform.rotation.z = 0.0
        world_tf.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(world_tf)

        # ====================================================
        # CHANGE THESE TOPICS TO MATCH MOTIVE
        # ====================================================
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.camera_rigid_body_topic = \
            '/vrpn_mocap/camera_rigidbody/pose'

        self.marker_rigid_body_topic = \
            '/vrpn_mocap/marker_rigidbody/pose'

        self.aruco_topic = \
            '/aruco/pose'


        # ====================================================
        # LATEST CAMERA RIGID BODY POSE
        # ====================================================

        self.O_T_BC = None


        # ====================================================
        # SUBSCRIBERS
        # ====================================================

        self.create_subscription(
            PoseStamped,
            self.camera_rigid_body_topic,
            self.camera_rigid_body_callback,
            qos
        )

        self.create_subscription(
            PoseStamped,
            self.marker_rigid_body_topic,
            self.marker_rigid_body_callback,
            qos
        )

        self.create_subscription(
            PoseStamped,
            self.aruco_topic,
            self.aruco_callback,
            10
        )


        # ====================================================
        # PUBLISHERS
        # ====================================================

        # ArUco pose converted into OptiTrack world
        self.aruco_world_pub = self.create_publisher(
            PoseStamped,
            '/aruco/pose_optitrack',
            10
        )

        # Raw marker rigid body pose from OptiTrack
        # republished with a convenient comparison topic
        self.marker_world_pub = self.create_publisher(
            PoseStamped,
            '/optitrack/marker_pose',
            10
        )


        self.get_logger().info(
            'New OptiTrack transform pipeline running'
        )

        self.get_logger().warn(
            'Camera rigid-body -> camera optical offset '
            'is currently assumed to be identity.'
        )
        
    def publish_tf(self, T, parent_frame, child_frame):

        tf_msg = TransformStamped()

        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = parent_frame
        tf_msg.child_frame_id = child_frame

        tf_msg.transform.translation.x = float(T[0, 3])
        tf_msg.transform.translation.y = float(T[1, 3])
        tf_msg.transform.translation.z = float(T[2, 3])

        q = rotation_matrix_to_quaternion(T[:3, :3])

        tf_msg.transform.rotation.x = float(q[0])
        tf_msg.transform.rotation.y = float(q[1])
        tf_msg.transform.rotation.z = float(q[2])
        tf_msg.transform.rotation.w = float(q[3])

        self.tf_broadcaster.sendTransform(tf_msg)


    # ========================================================
    # CAMERA RIGID BODY
    # ========================================================

    def camera_rigid_body_callback(self, msg):

        self.O_T_BC = pose_to_matrix(msg.pose)

        self.publish_tf(
            self.O_T_BC,
            'world',
            'camera_rigidbody'
        )


    # ========================================================
    # MARKER RIGID BODY
    # ========================================================

    def marker_rigid_body_callback(self, msg):

        # For now, simply republish the marker rigid-body pose.
        #
        # This is your OptiTrack ground truth.
        #
        # Later, if the rigid-body origin is offset from
        # the physical ArUco centre, you can add B_M_T_M here.

        output = PoseStamped()

        output.header = msg.header

        output.header.frame_id = 'world'

        output.pose = msg.pose
        
        O_T_BM = pose_to_matrix(msg.pose)
        
        self.publish_tf(
            O_T_BM,
            'world',
            'marker_rigidbody'
        )

        self.marker_world_pub.publish(
            output
        )


    # ========================================================
    # ARUCO
    # ========================================================

    def aruco_callback(self, msg):

        if self.O_T_BC is None:

            self.get_logger().warn(
                'Waiting for camera rigid-body pose...',
                throttle_duration_sec=2.0
            )

            return


        # ----------------------------------------------------
        # ArUco detector gives:
        #
        # C_T_M
        #
        # marker pose relative to camera optical frame
        # ----------------------------------------------------

        C_T_M = pose_to_matrix(
            msg.pose
        )


        # ====================================================
        # TEMPORARY CAMERA EXTRINSIC
        #
        # Camera rigid body -> camera optical frame
        #
        # FOR NOW:
        #
        # BC_T_C = Identity
        #
        # Later replace this with your calibrated transform.
        # ====================================================

        BC_T_C = np.eye(4)


        # ====================================================
        # NEW TRANSFORM CHAIN
        #
        # O_T_M =
        #
        # O_T_BC
        # *
        # BC_T_C
        # *
        # C_T_M
        #
        # Currently BC_T_C = I
        # ====================================================

        O_T_M = (
            self.O_T_BC
            @ BC_T_C
            @ C_T_M
        )
        
        self.publish_tf(
            O_T_M,
            'world',
            'aruco_marker'
        )


        # ----------------------------------------------------
        # Publish ArUco estimate in OptiTrack world
        # ----------------------------------------------------

        output = matrix_to_pose_stamped(
            O_T_M,
            msg.header.stamp,
            'world'
        )

        self.aruco_world_pub.publish(
            output
        )


def main(args=None):

    rclpy.init(args=args)

    node = OptiTrackTransform()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':

    main()