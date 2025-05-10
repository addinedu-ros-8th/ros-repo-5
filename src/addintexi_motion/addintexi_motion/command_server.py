import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class CommandServer(Node):
    def __init__(self):
        super().__init__('command_server')
        self.publisher = self.create_publisher(Int32, '/motion_command', 10)
        self.get_logger().info("Command Server Initialized. Use '0: Forward, 1: Left, 2: Right, 3: Stop'")

    def send_command(self, command):
        msg = Int32()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent Command: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = CommandServer()
    
    try:
        while rclpy.ok():
            cmd = int(input("\nEnter command (0: Forward, 1: Left, 2: Right, 3: Stop): ").strip())
            if cmd in [0, 1, 2, 3]:
                node.send_command(cmd)
            else:
                print("Invalid command. Please enter 0, 1, 2, 3.")
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

