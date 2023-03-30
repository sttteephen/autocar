# listens to the /cmd topic
# records the current steering angle and acceleration

import datetime
import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped

class CmdSubscriber(Node):

    def __init__(self):
        super().__init__('cmd_subscriber')
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'cmd',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.record = open('cmd_recording.csv', 'w')

    def listener_callback(self, msg):
        save = f'{datetime.datetime.now()},{msg.drive.steering_angle},{msg.drive.acceleration}\n'
        print(save)
        self.record.write(save)
        #print(msg.drive.steering_angle, msg.drive.acceleration)
        #self.get_logger().info(msg)


def main(args=None):
    rclpy.init(args=args)

    cmd_subscriber = CmdSubscriber()

    rclpy.spin(cmd_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cmd_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
