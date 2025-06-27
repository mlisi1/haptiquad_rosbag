#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from anymal_msgs.msg import AnymalState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster



class MsgConverter(Node):

    def __init__(self):

        super().__init__("MsgConverter")

        self.state_sub = self.create_subscription(AnymalState, "/state_estimator/anymal_state", self.state_callback, 10)

        self.state_pub = self.create_publisher(JointState, "/joint_states", 10)

        self.joint_state_msg = JointState()

        self.br = TransformBroadcaster(self)

        self.first_msg = True



    def state_callback(self, msg):

        if self.first_msg:

            self.joint_state_msg.name = msg.joints.name
            self.first_msg = False

        self.joint_state_msg.header = msg.header
        self.joint_state_msg.position = msg.joints.position
        self.joint_state_msg.velocity = msg.joints.velocity
        self.joint_state_msg.effort = msg.joints.effort

        self.state_pub.publish(self.joint_state_msg)

        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base'

        # Translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Rotation (identity quaternion)
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        self.br.sendTransform(t)








def main(args=None):
    rclpy.init(args=args)
    initial_pose_publisher = MsgConverter()
    rclpy.spin(initial_pose_publisher)
    initial_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()