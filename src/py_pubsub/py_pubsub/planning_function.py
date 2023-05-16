# listens to the /ground_truth/cones topic
# records positions of currently visible cones

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from eufs_msgs.msg import WaypointArrayStamped, Waypoint, ConeArrayWithCovariance, CarState, ConeWithCovariance
import numpy as np
from math import sqrt, asin, degrees


class Planner(Node):


    def __init__(self):
        super().__init__('cone_subscriber')

        # subscribe to cone positions
        self.subscription = self.create_subscription(
            ConeArrayWithCovariance,
            '/ground_truth/cones',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # publish waypoints
        self.track_line_pub = self.create_publisher(WaypointArrayStamped, "/trajectory", 1)


    # saves the positions of currently visible cones
    def listener_callback(self, msg):

        # loop over every visible cone
        blue_cones = self.convert(msg.blue_cones)
        yellow_cones = self.convert(msg.yellow_cones)
        print("lovely blu cones ", blue_cones, "yelly cones ", yellow_cones)
        #orange_cones = np.concatenate(self.convert(msg.orange_cones), self.convert(msg.big_orange_cones))

        midpoints = self.find_midpoints(blue_cones, yellow_cones)#, orange_cones)
        midpoints = self.sort_midpoints(midpoints)

        #self.get_logger().info(msg)


    def publish_path(self, midpoints):
        waypoint_array = WaypointArrayStamped()
        waypoint_array.header.frame_id = "base_footprint"
        waypoint_array.header.stamp = self.get_clock().now().to_msg()

        for p in midpoints:
            point = Point(x=p[0], y=p[1])
            waypoint = Waypoint(position=point)
            waypoint_array.waypoints.append(waypoint)

        self.track_line_pub.publish(waypoint_array)

    def find_midpoints(self, blue_cones, yellow_cones):#, orange_cones):
        """
        IMPLEMENT YOURSELF
        Find the midpoints along the track
        :param blue_cones: cone positions
        :param yellow_cones:
        :param orange_cones:
        :return: list of midpoints
        """
        yellow_with_distanct = []

        for p in yellow_cones:
            distance = sqrt(cone.point.x**2 + cone.point.y**2)


        return [(1,1)]

    def sort_midpoints(self, midpoints):
        """
        IMPLEMENT YOURSELF
        Sort the midpoints to so that each consecutive midpoints is further from the car along the path
        :param midpoints:
        :return: sorted midpoints as 2d array where each row is a midpoint
        """

        return midpoints


    def convert(self, cones, struct=''):
        """
        Converts a cone array message into a np array of complex or 2d np array
        :param cones: list of ConeWithCovariance
        :param struct: Type of output list
        :return:
        """
        if struct == "complex":
            return np.array([p.point.x + 1j * p.point.y for p in cones])
        else:
            return np.array([[p.point.x, p.point.y] for p in cones])


def main(args=None):
    rclpy.init(args=args)

    planner = Planner()

    rclpy.spin(planner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
