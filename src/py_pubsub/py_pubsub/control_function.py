from eufs_msgs.msg import WaypointArrayStamped, CarState
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
import rclpy
from rclpy.node import Node
from math import sqrt, asin, degrees, atan, sin

import numpy as np

class Control(Node):
    def __init__(self, name):
        super().__init__(name)
        self.speed = 0
        self.error_buffer = [0]
        self.period = 0.04      # the time between updates to the path

        # Declare ROS parameters
        #self.look_ahead = self.declare_parameter("look_ahead", 4.0).value
        self.L = self.declare_parameter("L", 1.5).value
        #self.K_p = self.declare_parameter("K_p", 1.0).value
        #self.K_i = self.declare_parameter("K_i", 1.0).value
        #self.K_d = self.declare_parameter("K_d", 1.0).value
        #self.max_lat_acc = self.declare_parameter("max_lat_acc", 5.0).value
        #self.safe_speed = self.declare_parameter("safe_speed", 1.5).value
        #self.max_speed = self.declare_parameter("max_speed", 4.5).value
        #self.buffer_len = self.declare_parameter("buffer_len", 30).value

        # Create subscribers
        self.path_sub = self.create_subscription(WaypointArrayStamped, "/trajectory", self.path_callback, 1)
        self.car_state_sub = self.create_subscription(CarState, "/ground_truth/state", self.state_callback, 1)

        # Create publishers
        self.comand_pub = self.create_publisher(AckermannDriveStamped, "/cmd", 1)
        self.viz_pub = self.create_publisher(Marker, "/control/viz", 1)

    def state_callback(self, msg):
        self.speed = msg.twist.twist.linear.x

    def path_callback(self, msg):
        path = self.convert(msg) # If you prefer to use complex numbers specify the optional parameter struct="complex"
        if len(path) == 0: return
        print(path)
        # Index of the waypoint to the look ahead distance
        look_ahead_index = self.get_look_ahead_index(path)

        # Steering control
        steering_cmd = self.get_steering(path, look_ahead_index)

        # Speed control
        speed_target = self.get_speed_target(path, look_ahead_index)

        # PID control for acceleration
        acceleration_cmd = self.get_acceleration(speed_target)

        # Publish commands
        self.pubish_command(acceleration_cmd, steering_cmd)

    def get_look_ahead_index(self, path):
        return 0

    def get_steering(self, path, look_ahead_ind):
        """
        IMPLEMENT YOURSELF
        note: the wheelbase of the car L is saved in the self.L variable
        :param path: np array
        :param look_ahead_ind:
        :return: steering angle to be sent to the car
        """
        print(path[look_ahead_ind])
        distance = sqrt(path[look_ahead_ind][0]**2 + path[look_ahead_ind][1]**2)
        angle = degrees(asin(abs(path[look_ahead_ind][1]) / distance))
        if path[look_ahead_ind][1] > 0:
            angle = -angle

        steering_angle = atan((2*self.L*sin(angle)/distance))
        print(distance, angle, steering_angle)

        return steering_angle

    def get_speed_target(self, path, look_ahead_ind):
        """
        IMPLEMENT YOURSELF
        note: You might want to use the max_lat_acc variable to limit lateral acceleration
        and max_speed to limit the maximum speed
        :param path: array of complex numbers
        :param look_ahead_ind:
        :return: speed we want to reach
        """

        return 1.0

    def get_acceleration(self, speed_target):
        """
        IMPLEMENT YOURSELF
        Note: the current speed of the car is saved in self.speed
        the PID gains are saved in self.K_p, self.K_i, self.K_d
        :param speed_target: speed we want to achieve
        :return: acceleration command to be sent to the car
        """

        return 1.0

    def pubish_command(self, acceleration, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pure_pursuit"
        msg.drive.steering_angle = steering
        msg.drive.acceleration = acceleration

        self.comand_pub.publish(msg)



    def convert(self, waypoints, struct = ''):
        """
        Converts a waypoints array message into a 2d np array or np array of complex
        :param waypoints: WaypointsArrayStamped message
        :param struct: Type of output list
        :return:
        """
        if struct == "complex":
            return np.array([p.position.x + 1j * p.position.y for p in waypoints.waypoints])
        else:
            return np.array([[p.position.x, p.position.y] for p in waypoints.waypoints])


def main():
    rclpy.init(args=None)
    node = Control("pure_pursuit")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
