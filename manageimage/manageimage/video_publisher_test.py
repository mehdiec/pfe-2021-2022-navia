#!/usr/bin/env python3

# External modules
import cv2
import numpy as np
import time
# Local modules
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import os

class video_publisher_test(Node):
    def __init__(self):
        super().__init__("video_publisher_test")
        self.image_publisher = self.create_publisher(CompressedImage, "anafi/image_raw", 1000)

        cap = cv2.VideoCapture(os.path.join(os.getcwd(),'manageimage','manageimage','video_bag_1.mp4'))
        #cap = cv2.VideoCapture('/home/rudolf/Documents/pfe-2021-2022-navia/manageimage/manageimage/video_bag_1.mp4')

        # Check if camera opened successfully

        self.i = 0 

        if (cap.isOpened()== False):
            print("Error opening video stream or file")

        # Read until video is completed

        while(cap.isOpened()):

            # Capture frame-by-frame

            ret, output = cap.read()

            if ret == True:
            # Display the resulting frame
                output = cv2.cvtColor(output, cv2.COLOR_RGB2BGR)
                output = cv2.resize(output,(320,240),interpolation = cv2.INTER_AREA)
                msg_img = CompressedImage()
                msg_img.header.stamp = self.get_clock().now().to_msg()
                msg_img.format = "jpeg"
                msg_img.data = np.array(cv2.imencode(".jpg", output)[1]).tostring()

                self.image_publisher.publish(msg_img)

                # save image 
                #im_path = "/home/rudolf/Documents/images_vid/orig/image"+ str(self.i) + ".jpg"
                #cv2.imwrite(im_path, output)
                self.i+=1

                time.sleep(0.2)
            else:
                break
            
        self.get_logger().info("finished segmentation of the video")



def main(args=None):
    rclpy.init(args=args)

    publisher_node = video_publisher_test()
    publisher_node.get_clock().now()

    rclpy.spin(publisher_node)



if __name__ == '__main__':
    main()