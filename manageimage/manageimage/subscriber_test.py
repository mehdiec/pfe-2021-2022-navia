#!/usr/bin/env python3

# External modules
import cv2
import numpy as np
# Local modules
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class subscriber_test(Node):
    def __init__(self):
        super().__init__("subscriber_test")
        self.image_subscriber =self.create_subscription(CompressedImage, "anafi/segmented_image", self.on_image, 10)

    def on_image(self,msg):
        img = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(img, cv2.IMREAD_COLOR)
        imS = cv2.resize(img, (960, 540))  
        cv2.imshow("segmented image",imS)
        cv2.waitKey(10000)
        cv2.destroyAllWindows()



def main(args=None):
    rclpy.init(args=args)

    subscriber_node = subscriber_test()
    subscriber_node.get_clock().now()

    rclpy.spin(subscriber_node)



if __name__ == '__main__':
    main()