import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

class JoyMsgSubPub(Node):
    def __init__(self):
        super().__init__('joy_msg_sub_pub')
        self.sub = self.create_subscription(Joy, "joy", self.joy_callback,10)
        self.sub # prevent unused variable warning
    
    def joy_callback(self, msg):

        self.get_logger().info("data:%f" % msg.axes[1])

def main(args=None):
    rclpy.init(args=args)

    joy_msg_sub_pub = JoyMsgSubPub()

    rclpy.spin(joy_msg_sub_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joy_msg_sub_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()