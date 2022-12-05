import numpy as np

import rclpy
from rclpy.node import Node


from tf2_ros import TransformBroadcaster

import tf_transformations
from geometry_msgs.msg import Twist, Vector3, TransformStamped, Pose
from nav_msgs.msg import Odometry


class AnafiDriver(Node):
    def __init__(self):
        super().__init__("anafi_driver")
        self.hover_sub = self.create_subscription(
            Odometry, "anafi/odom", self.handle_drone_pose, 10
        )

        self.br = TransformBroadcaster(self)

    def handle_drone_pose(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        current_time = self.get_clock().now().to_msg()
        last_time = msg.header.stamp
        t.header.stamp = current_time
        t.header.frame_id = "anafi"
        t.child_frame_id = "camera"  # NOT SURE

        th = msg.pose.rotation.x

        dt = (current_time - last_time).toSec()

        # Don't realy know drone position
        # TO DO
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        vz = msg.twist.linear.z
        delta_x = (vx * np.cos(th) - vy * np.sin(th)) * dt
        delta_y = (vx * np.sin(th) + vy * np.cos(th)) * dt
        t.transform.translation.x = msg.x + delta_x
        t.transform.translation.y = msg.y + delta_y
        t.transform.translation.z = 0.0

        q = tf_transformations.quaternion_from_euler(
            msg.pose.rotation.x, msg.pose.rotation.y, msg.pose.rotation.z, 0
        )
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation

    def timer_callback(self):

        odom = Odometry()

        measured_speed = self.anafi.get_speed()

        vx = measured_speed["speedX"]
        vy = measured_speed["speedY"]
        vz = measured_speed["speedZ"]
        measured_attitude = self.anafi.get_attitude()

        pose = Pose()
        twist = Twist()

        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 0
        pose.rotation.x = measured_attitude["roll"]
        pose.rotation.y = measured_attitude["pitch"]
        pose.rotation.z = measured_attitude["yaw"]
        pose.rotation.w = 0

        twist.twist.linear = Twist(Vector3(vx, vy, vz), Vector3(0, 0, 0))
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "anafi"
        self.odom_publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)

    anafi_node = AnafiDriver()
    anafi_node.get_clock().now()

    rclpy.spin(anafi_node)


if __name__ == "__main__":
    main()
