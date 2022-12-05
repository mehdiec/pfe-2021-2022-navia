#!/usr/bin/env python3

# Standard modules
import argparse
import socket
import sys
import time
import functools
# External modules
import cv2
import numpy as np
# Local modules
from deeplearning_demos import video_grabber
from deeplearning_demos import utils
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class image_segmentation(Node):
    def __init__(self):
        super().__init__("image_segmentation")
        host_descriptor = ParameterDescriptor(description="The IP of the echo server")
        port_descriptor = ParameterDescriptor(description="The port on which the server is listening")
        jpeg_quality_descriptor = ParameterDescriptor(description="The JPEG quality for compressing the reply")
        resize_descriptor = ParameterDescriptor(description="Resize factor of the image")
        encoder_descriptor = ParameterDescriptor(description="Library to use to encode/decode in JPEG the images")


        self.declare_parameter("host", "localhost", host_descriptor)
        self.declare_parameter("port", 6008, port_descriptor)
        self.declare_parameter("jpeg_quality", 50, jpeg_quality_descriptor)
        self.declare_parameter("resize", 1.0, resize_descriptor)
        self.declare_parameter("encoder", "cv2", encoder_descriptor)

        
        self.host= self.get_parameter("host").value
        self.port= self.get_parameter("port").value
        self.jpeg_quality= self.get_parameter("jpeg_quality").value
        self.resize= self.get_parameter("resize").value
        self.encoder= self.get_parameter("encoder").value



        self.incl_x_subscriber = self.create_subscription(CompressedImage, "initial_image", self.on_image, 1000)
        self.image_publisher = self.create_publisher(Image, "segmented_image", 1000)

    def on_image(self,msg):
        
        self.get_logger().info( f" received an image")
        jpeg_handler = utils.make_jpeg_handler(self.encoder, self.jpeg_quality)

        
        img = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(img, cv2.IMREAD_COLOR)

        
        
        
        get_buffer = functools.partial(jpeg_handler.compress, cv2_img=img)

        # A temporary buffer in which the received data will be copied
        # this prevents creating a new buffer all the time
        tmp_buf = bytearray(7)
        # this allows to get a reference to a slice of tmp_buf
        tmp_view = memoryview(tmp_buf)

        # Creates a temporary buffer which can hold the largest image we can transmit
        img_buf = bytearray(9999999)
        img_view = memoryview(img_buf)

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((self.host, self.port))
            

            # Grab and encode the image
            img_buffer, orig_img = get_buffer()
            # print(type(orig_img), orig_img.shape)
            if img_buffer is None:
                self.get_logger().error("NO BUFFER ")

            # Prepare the message with the number of bytes going to be sent
            msg = bytes("image{:07}".format(len(img_buffer)), "ascii")

            utils.send_data(sock, msg)

            # Send the buffer
            utils.send_data(sock, img_buffer)

            # Read the reply command
            utils.recv_data_into(sock, tmp_view[:5], 5)
            cmd = tmp_buf[:5].decode('ascii')

            if cmd != 'image':
                raise RuntimeError("Unexpected server reply")

            # Read the image buffer size
            utils.recv_data_into(sock, tmp_view, 7)
            img_size = int(tmp_buf.decode('ascii'))

            # Read the image buffer
            utils.recv_data_into(sock, img_view[:img_size], img_size)

            # Read the final handshake
            cmd = utils.recv_data(sock, 5).decode('ascii')
            if cmd != 'enod!':
                raise RuntimeError("Unexpected server reply. Expected 'enod!'"
                                ", got '{}'".format(cmd))

            # Transaction is done, we now process/display the received image
            output = jpeg_handler.decompress(img_view[:img_size])
            self.get_logger().info( f" the image shape {output.shape}")
            if(len(output.shape) == 2):
                #output = cv2.cvtColor(output,cv2.IMREAD_GRAYSCALE)
                #output = output.astype(np.uint8)
                #output = output[:, :, np.newaxis]
                bridge = CvBridge()
                msg_img = bridge.cv2_to_imgmsg(output, "mono8")
                #msg_img.header.stamp = self.get_clock().now().to_msg()
                #msg_img.format = "jpeg"
                #msg_img.data = np.array(cv2.imencode(".jpg", output)[1]).tostring()
                self.image_publisher.publish(msg_img)
            sock.sendall('quit!'.encode('ascii'))
        self.get_logger().info( "Image sucessfully segmented ")



def main(args=None):
    rclpy.init(args=args)

    segmentation_node = image_segmentation()
    segmentation_node.get_clock().now()

    rclpy.spin(segmentation_node)



if __name__ == '__main__':
    main()