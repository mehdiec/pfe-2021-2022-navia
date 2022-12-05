import cv2
import numpy as np


import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32

import rosbag2_py


class DataGeneratorNode(Node):
    def __init__(self):
        super().__init__("data_generator_node")

        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py._storage.StorageOptions(
            uri="my_bags", storage_id="sqlite3"
        )
        converter_options = rosbag2_py._storage.ConverterOptions("", "")
        self.writer.open(storage_options, converter_options)

        topic_image = rosbag2_py._storage.TopicMetadata(
            name="initial_image",
            type="sensor_msgs/msg/CompressedImage",
            serialization_format="cdr",
        )
        self.writer.create_topic(topic_image)

        topic_lin_z = rosbag2_py._storage.TopicMetadata(
            name="linear_z", type="std_msgs/msg/Float32", serialization_format="cdr"
        )
        self.writer.create_topic(topic_lin_z)

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):

        output = cv2.imread(
            "/home/dehk/sdi/pfe/pfe-2021-2022-navia/bag_recorder_nodes_py/bag_recorder_nodes_py/couloir_test.jpg"
        )
        print(output)
        output = cv2.cvtColor(output, cv2.COLOR_RGB2BGR)
        msg_img = CompressedImage()
        msg_img.header.stamp = self.get_clock().now().to_msg()
        msg_img.format = "jpeg"
        msg_img.data = np.array(cv2.imencode(".jpg", output)[1]).tostring()

        msg_h = Float32()
        msg_h.data = 1.0

        self.writer.write(
            "initial_image",
            serialize_message(msg_img),
            self.get_clock().now().nanoseconds,
        )
        self.writer.write(
            "linear_z", serialize_message(msg_h), self.get_clock().now().nanoseconds
        )


def main(args=None):
    rclpy.init(args=args)
    dgn = DataGeneratorNode()
    rclpy.spin(dgn)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
