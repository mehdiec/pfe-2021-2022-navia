import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty, Float32
import math

## Buttons id
BUTTON_A = 0
BUTTON_B = 1

## axes id
R2 = 5
L2 = 2
LEFT_STICK_HORIZONTAL = 0
LEFT_STICK_VERTICAL  = 1
RIGHT_STICK_HORIZONTAL = 3
RIGHT_STICK_VERTICAL  = 4

class AnafiController(Node):

    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameters(
        namespace = '',
        parameters = [('speed',0.25),('delta_speed',0.08),('tolerance',0.5)]
        )
        
        self.joy_sub = self.create_subscription(Joy,"/joy", self.on_joy ,10 )

        self.incl_x = self.create_publisher(Float32, 'anafi/incl_x', 10)
        self.incl_y = self.create_publisher(Float32, 'anafi/incl_y', 10)
        self.angular_z = self.create_publisher(Float32, 'anafi/angular_z', 10)
        self.linear_z = self.create_publisher(Float32, 'anafi/linear_z', 10)

        self.land = self.create_publisher(Empty, 'anafi/land', 10)
        self.takeoff = self.create_publisher(Empty, 'anafi/takeoff', 10)
        self.hover = self.create_publisher(Empty, 'anafi/hover', 10)

        self.tolerance = self.get_parameter('tolerance').value
        self.is_grounded = True
        self.speed = self.get_parameter('speed').value 
        self.delta_speed = self.get_parameter('delta_speed').value

    
    def on_joy(self,msg):
        ## command options 
        if msg.axes[R2] < - self.tolerance:
            self.takeoff.publish(Empty())
            if self.is_grounded:
                self.is_grounded =False
            


        if msg.axes[R2]  > self.tolerance:
            self.land.publish(Empty())
            self.is_grounded = True

        if msg.axes[L2] < - self.tolerance:
            self.hover.publish(Empty())

        ## use the unit circle 
        ##x = RIGHT_STICK_HORIZONTAL (inverted axis)
        ##y = RIGHT_STICK_VERTICAL
        ## angular = -tan-1(y/x)
        angular_z = 0.0
        x = msg.axes[RIGHT_STICK_HORIZONTAL]
        if abs(x)>1e-5:
            angular_z = -math.atan(msg.axes[RIGHT_STICK_VERTICAL]/x)

        float_ang_z =Float32()
        float_ang_z.data = angular_z
        self.angular_z.publish(float_ang_z)

        ## sin-1 will be used for the incl_x (inverted) and inc_y

        float_incl_x = Float32()
        float_incl_y = Float32()

        float_incl_x.data = -math.asin(msg.axes[LEFT_STICK_HORIZONTAL])
        float_incl_y.data = math.asin(msg.axes[LEFT_STICK_VERTICAL])
        self.incl_x.publish(float_incl_x)

        self.incl_y.publish(float_incl_y)

        # speed gaz control
        ## One button will increase speed, the second will decrease

        if msg.buttons[BUTTON_A]:
            self.speed+=self.delta_speed
        if msg.buttons[BUTTON_B]:
            self.speed-=self.delta_speed
            if self.speed < 0:  ## no negative speed
                self.speed =0.0
        
        float_lin_z = Float32()
        float_lin_z.data =self.speed

        self.linear_z.publish(float_lin_z)
        


def main(args=None):
    rclpy.init(args=args)
    

    controller = AnafiController()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()