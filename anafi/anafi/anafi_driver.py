import cv2
import numpy as np

import rclpy
from rclpy.node import Node


from tf2_ros import TransformBroadcaster

import tf_transformations
from geometry_msgs.msg import Twist, Vector3, TransformStamped, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Empty
from sensor_msgs.msg import CompressedImage
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter
import olympe
from sys import maxsize
from multiprocessing import Lock
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, PCMD, moveBy
from olympe.messages.ardrone3.PilotingState import (
    FlyingStateChanged,
    SpeedChanged,
    AttitudeChanged,
)
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.SpeedSettings import MaxVerticalSpeed
from olympe.messages.ardrone3.SpeedSettings import MaxRotationSpeed

from numpy import pi


class Anafi:
    def __init__(
        self,
        frame_cb,
        max_tilt,
        max_rotation_speed,
        max_vertical_speed,
        ip="192.168.42.1",
    ):

        ## set Ip,and frame
        self.DRONE_IP = ip
        self.drone = olympe.Drone(self.DRONE_IP)
        self.frame_cb = frame_cb
        self.max_tilt = max_tilt
        self.max_vertical_speed = max_vertical_speed
        self.max_rotation_speed = max_rotation_speed
        ## set maxtilt in radian
        self.set_max_tilt()
        self.set_max_vertical_speed()
        self.set_max_rotation_speed()

        self.measured_speed = None
        self.measured_attitude = None

        self.frame_mutex = Lock()
        self.img = None
        self.width = None
        self.height = (None,)
        self.cv2_cvt_color_flag = None

        self.stamp = 0

    def __str__(self):
        if self.measured_speed is None:
            return "???"
        else:
            return (
                "[vx = {}, vy = {}, vz = {}, roll = {}, pitch = {}, yaw = {}]".format(
                    self.measured_speed["speedX"],
                    self.measured_speed["speedY"],
                    self.measured_speed["speedZ"],
                    self.measured_attitude["roll"],
                    self.measured_attitude["pitch"],
                    self.measured_attitude["yaw"],
                )
            )

    def connect(self):
        self.drone.connect()
        self.drone.streaming.set_callbacks(raw_cb=self.on_yuv)
        self.drone.streaming.start()

    def disconnect(self):
        self.drone.streaming.stop()
        self.drone.disconnect()

    def takeoff(self):
        self.drone(
            TakeOff() >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait().success()

    def land(self):
        self.drone(Landing()).wait().success()

    def hover(self):
        assert (
            self.drone(FlyingStateChanged(state="hovering", _timeout=5))
            .wait()
            .success()
        )

    def get_image(self):

        return self.img

    def get_speed(self):
        return self.drone.get_state(SpeedChanged)

    def get_attitude(self):
        return self.drone.get_state(AttitudeChanged)

    def set_max_tilt(self):
        ## maxtilt is converted into degrees
        self.drone(MaxTilt(self.max_tilt * 180 / pi))

    def set_max_vertical_speed(self):
        self.drone(MaxVerticalSpeed(self.max_vertical_speed))

    def set_max_rotation_speed(self):
        ## radian/s coverted into degree/s
        self.drone(MaxRotationSpeed(self.max_rotation_speed * 180 / pi))

    def move(self, roll, pitch, yaw, gaz, flag=1):

        """
        roll, pitch are in radian
        yaw is in radian/s  ( can be negative )

        """

        self.drone(
            PCMD(
                flag=1,
                roll=int(roll * 100 / self.max_tilt),
                pitch=int(pitch * 100 / self.max_tilt),
                yaw=int(yaw * 100 / self.max_rotation_speed),
                gaz=int(gaz),
                timestampAndSeqNum=self.stamp,
            )
        )

    def set_pitch(self, pitch):
        """
        pitch must be in radian
        if pitch is higher than maxtilt it is set to maxtilt

        """
        if pitch > self.max_tilt:
            pitch = self.max_tilt

        current_attitude = self.get_attitude()
        roll = current_attitude["roll"]

        yaw = current_attitude["yaw"]

        ## -100 is equivalent to 0m/s
        gaz = self.get_speed()["speedZ"] / self.max_vertical_speed * 200 - 100

        ### issue yaw is in radian ???
        self.move(roll, pitch, yaw, gaz)

    def set_roll(self, roll):

        """
        roll must be in radian
        if roll is higher than maxtilt it is set to maxtilt

        """
        if roll > self.max_tilt:
            roll = self.max_tilt

        current_attitude = self.get_attitude()

        pitch = current_attitude["pitch"]
        yaw = current_attitude["yaw"]

        ## -100 is equivalent to 0m/s
        gaz = self.get_speed()["speedZ"] / self.max_vertical_speed * 200 - 100

        self.move(roll, pitch, yaw, gaz)

    def set_yaw(self, yaw):
        """
        yaw must be in radian/s
        if yaw is higher than max_rotation_speed it is set to max_rotation_speed

        """
        if yaw > self.max_rotation_speed:
            yaw = self.max_rotation_speed

        current_attitude = self.get_attitude()
        roll = current_attitude["roll"]
        pitch = current_attitude["pitch"]

        ## -100 is equivalent to 0m/s
        gaz = self.get_speed()["speedZ"] / self.max_vertical_speed * 200 - 100

        self.move(roll, pitch, yaw, gaz)

    def set_gaz(self, gaz):
        """
        gaz must be in m/s

        """
        if gaz > self.max_vertical_speed:
            yaw = self.max_vertical_speed

        ## -100 is equivalent to 0m/s
        gaz = gaz / self.max_vertical_speed * 200 - 100

        current_attitude = self.get_attitude()
        roll = current_attitude["roll"]
        pitch = current_attitude["pitch"]
        yaw = current_attitude["yaw"]
        self.move(roll, pitch, yaw, gaz)

    def on_yuv(self, yuv_frame):
        yuv_frame.ref()
        if self.width is None:
            info = yuv_frame.info()
            self.height, self.width = (
                info["raw"]["frame"]["info"]["height"],
                info["raw"]["frame"]["info"]["width"],
            )
            self.cv2_cvt_color_flag = {
                olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
                olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
            }[yuv_frame.format()]
        with self.frame_mutex:
            self.img = cv2.cvtColor(yuv_frame.as_ndarray(), self.cv2_cvt_color_flag)
        yuv_frame.unref()


class AnafiDriver(Node):
    def __init__(self):
        super().__init__("anafi_driver")
        self.incl_x_subscriber = self.create_subscription(
            Float32, "anafi/incl_x", self.on_incl_x, 10
        )
        self.incl_y_subscriber = self.create_subscription(
            Float32, "anafi/incl_y", self.on_incl_y, 10
        )
        self.lin_z_sub = self.create_subscription(
            Float32, "anafi/linear_z", self.on_linear_z, 10
        )
        self.ang_z_sub = self.create_subscription(
            Float32, "anafi/angular_z", self.on_angular_z, 10
        )
        self.land_sub = self.create_subscription(Empty, "anafi/land", self.on_land, 10)
        self.takeoff_sub = self.create_subscription(
            Empty, "anafi/takeoff", self.on_takeoff, 10
        )
        self.hover_sub = self.create_subscription(
            Empty, "anafi/hover", self.on_hover, 10
        )

        self.odom_publisher = self.create_publisher(Odometry, "anafi/odom", 10)
        self.image_publisher = self.create_publisher(
            CompressedImage, "anafi/image_raw", 10
        )

        param_ip_descriptor = ParameterDescriptor(description="IP of the drone")
        max_tilt_descriptor = ParameterDescriptor(description="max tilt in rad")
        max_vert_spd_descriptor = ParameterDescriptor(
            description="max vertical speed in m/s"
        )
        max_vert_spd_descriptor = ParameterDescriptor(
            description="max rotation speed speed in m/s"
        )
        self.declare_parameter("drone_IP", '192.168.42.1', param_ip_descriptor)
        self.declare_parameter("max_tilt", 0.77, max_tilt_descriptor)
        self.declare_parameter("max_vert_spd", 0.25, max_vert_spd_descriptor)
        self.declare_parameter("max_rotation_speed", 0.30, max_vert_spd_descriptor)

        self.param_ip = self.get_parameter("drone_IP").value
        self.param_max_tilt = self.get_parameter("max_tilt").value
        self.param_max_vert_spd = self.get_parameter("max_vert_spd").value
        self.max_rotation_speed = self.get_parameter("max_rotation_speed").value

        self.get_logger().info(
            str(self.param_ip)
            + str(self.param_max_tilt)
            + str(self.param_max_vert_spd)
            + str(self.max_rotation_speed)
        )

        def on_image(img):  # This is called in the main thread.
            cv2.imshow("Drone view", img)

        self.anafi = Anafi(
            frame_cb=on_image,
            ip=self.param_ip,
            max_tilt=self.param_max_tilt,
            max_vertical_speed=self.param_max_vert_spd,
            max_rotation_speed=self.max_rotation_speed,
        )
        self.anafi.connect()

    def on_hover(self, msg):
        self.anafi.hover()

    def on_takeoff(self, msg):
        self.anafi.takeoff()

    def on_land(self, msg):
        self.anafi.land()

    def on_linear_z(self, msg):
        self.anafi.set_gaz(msg.data)

    def on_angular_z(self, msg):
        self.anafi.set_yaw(msg.data)

    def on_incl_x(self, msg):
        self.anafi.set_roll(msg.data)

    def on_incl_y(self, msg):
        self.anafi.set_pitch(msg.data)

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

        twist.twist = Twist(Vector3(vx, vy, vz), Vector3(0, 0, 0))
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "anafi"
        self.odom_publisher.publish(odom)

        if self.anafi.on_yuv():
            msg_img = CompressedImage()
            msg_img.header.stamp = self.get_clock().now().to_msg()
            msg_img.format = "jpeg"
            msg_img.data = np.array(cv2.imencode(".jpg", self.anafi.img)[1]).tostring()
            self.image_publisher.publish(msg_img)


def main(args=None):
    rclpy.init(args=args)

    anafi_node = AnafiDriver()
    anafi_node.get_clock().now()

    rclpy.spin(anafi_node)


if __name__ == "__main__":
    main()
