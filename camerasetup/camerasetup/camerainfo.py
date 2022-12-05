import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo
from rcl_interfaces.msg import ParameterDescriptor

class camerainfo(Node):
    def __init__(self):
        super().__init__("camerainfo")

        self.camera_info_publisher = self.create_publisher(CameraInfo, "/sensor_msgs/CameraInfo", 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        width_descriptor = ParameterDescriptor(description="Image width")
        self.declare_parameter("image_width", 320, width_descriptor)
        self.image_width = self.get_parameter("image_width").value

        height_descriptor = ParameterDescriptor(description="Image height")
        self.declare_parameter("image_height", 240, height_descriptor)
        self.image_height = self.get_parameter("image_height").value

        distortion_model_descriptor = ParameterDescriptor(description="distortion model")
        self.declare_parameter("distortion_model", "plumb_bob" , distortion_model_descriptor)
        self.distortion_model = self.get_parameter("distortion_model").get_parameter_value().string_value

        camera_matrix_descriptor = ParameterDescriptor(description="camera_matrix K")
        self.declare_parameter("camera_matrix K.data", [326.173156, 0.000000, 142.152465,0.000000, 325.057891, 100.344205, 0.000000, 0.000000, 1.000000], camera_matrix_descriptor)
        self.camera_matrix = self.get_parameter("camera_matrix K.data").value

        #self.get_logger().info(f" camera k val {self.camera_matrix}")

        distortion_coefficients_descriptor = ParameterDescriptor(description="distortion_coefficients D")
        self.declare_parameter("distortion_coefficients D.data", [-0.090308, 0.298195, -0.008055, -0.007610, 0.0000], distortion_coefficients_descriptor)
        self.distortion_coefficients = self.get_parameter("distortion_coefficients D.data").value

        rectification_matrix_descriptor = ParameterDescriptor(description="rectification_matrix R")
        self.declare_parameter("rectification_matrix R.data", [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000], rectification_matrix_descriptor)
        self.rectification_matrix = self.get_parameter("rectification_matrix R.data").value

        projection_matrix_descriptor = ParameterDescriptor(description="projection_matrix P")
        self.declare_parameter("projection_matrix P.data", [327.801178, 0.000000, 140.863108, 0.000000, 0.000000, 326.170898, 99.016313, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000], projection_matrix_descriptor)
        self.projection_matrix = self.get_parameter("projection_matrix P.data").value



    def timer_callback(self):
        camera_info_msg = CameraInfo()
        camera_info_msg.width = self.image_width
        camera_info_msg.height = self.image_height
        camera_info_msg.k = self.camera_matrix
        camera_info_msg.d = self.distortion_coefficients
        camera_info_msg.r = self.rectification_matrix
        camera_info_msg.p = self.projection_matrix
        camera_info_msg.distortion_model = self.distortion_model
        
        self.camera_info_publisher.publish(camera_info_msg)

def main(args=None):
    rclpy.init(args=args)

    anafi_node = camerainfo()
    anafi_node.get_clock().now()

    rclpy.spin(anafi_node)


if __name__ == "__main__":
    main()
