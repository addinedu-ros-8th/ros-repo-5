from geometry_msgs.msg import Twist
from rclpy.node import Node

class CmdVelSubscriber(Node):
    def __init__(self, gui_callback):
        super().__init__('cmd_vel_listener')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            gui_callback,
            10
        )
