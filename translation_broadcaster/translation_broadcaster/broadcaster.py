from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

import tf_transformations

from math import pi

class FramePublisher(Node):

    def __init__(self):
        super().__init__('drone_tf_frame_publisher')

        # Declare and acquire `dronename` parameter
        self.declare_parameter('dronename', 'anafi')
        self.dronename = self.get_parameter(
            'dronename').get_parameter_value().string_value

        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        ## timer 
        self.timer = self.create_timer(0.1, self.handle_anafi_pose)
        self.i = 0
        self.speed = -0.01
        

    def handle_anafi_pose(self):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base'
        t.child_frame_id = self.dronename

        # anafi only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = self.i*self.speed
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.0

        q = tf_transformations.quaternion_from_euler(0, -pi*0.5, 0)
        
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.br.sendTransform(t)

        self.i+=1


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()