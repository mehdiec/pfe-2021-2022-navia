#!/usr/bin/env python3

# External modules
import cv2
import numpy as np
# Local modules
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import os
class publisher_test(Node):
    def __init__(self):
        super().__init__("publisher_test")
        self.image_publisher = self.create_publisher(CompressedImage, "anafi/image_raw", 10)

        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    def timer_callback(self):
        #if self.i ==0:
        file_path = os.path.join(os.getcwd(),'manageimage','manageimage','couloir_2.jpg')
        #output = cv2.imread('/home/rudolf/Documents/pfe-2021-2022-navia/manageimage/manageimage/couloir_1.jpg')
        output = cv2.imread(file_path)
        output = cv2.cvtColor(output, cv2.COLOR_RGB2BGR)

        #output = cv2.imread('/home/rudolf/Documents/pfe-2021-2022-navia/manageimage/manageimage/output.jpg',0)
        #output = cv2.cvtColor(output, cv2.COLOR_RGB2BGR)
        ### resize image
        #output = cv2.resize(output, (600,400), interpolation = cv2.INTER_AREA)
        
        msg_img = CompressedImage()
        msg_img.header.stamp = self.get_clock().now().to_msg()
        msg_img.format = "jpeg"
        msg_img.data = np.array(cv2.imencode(".jpg", output)[1]).tostring()
        self.image_publisher.publish(msg_img)

        #    self.i+=1



def main(args=None):
    rclpy.init(args=args)

    publisher_node = publisher_test()
    publisher_node.get_clock().now()

    rclpy.spin(publisher_node)



if __name__ == '__main__':
    main()