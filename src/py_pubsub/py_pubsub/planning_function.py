# listens to the /ground_truth/cones topic
# records positions of currently visible cones
'''
This node listens to the /ground_truth/cones topic for a ConeArrayWithCovariance msg.
Currently it just handles blue and yellow cones.
The node publishes to the /trajectory topic an array of midpoint as a WaypointArrayStamped msg.
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from eufs_msgs.msg import WaypointArrayStamped, Waypoint, ConeArrayWithCovariance, CarState, ConeWithCovariance
import numpy as np
from math import sqrt, asin, degrees
from matplotlib import pyplot as plt


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
        #orange_cones = np.concatenate(self.convert(msg.orange_cones), self.convert(msg.big_orange_cones))

        midpoints = self.find_midpoints(blue_cones, yellow_cones)#, orange_cones)
        midpoints = self.sort_midpoints(midpoints)

        # sort arrays by distance
        yellow_cones = sorted(
                yellow_cones, 
                key=lambda x: x[1]
            )

        blue_cones = sorted(
                blue_cones, 
                key=lambda x: x[1]
            )

        #self.map_cones(blue_cones, yellow_cones, midpoints)
        self.publish_path(midpoints)


    def publish_path(self, midpoints):
        waypoint_array = WaypointArrayStamped()
        waypoint_array.header.frame_id = "base_footprint"
        waypoint_array.header.stamp = self.get_clock().now().to_msg()

        for p in midpoints:
            point = Point(x=p[0], y=p[1])
            waypoint = Waypoint(position=point)
            waypoint_array.waypoints.append(waypoint)

        self.track_line_pub.publish(waypoint_array)
        print(waypoint_array)

    def find_midpoints(self, blue_cones, yellow_cones):#, orange_cones):

        yellow_with_distance = []
        blue_with_distance = []
        midpoints = []
        try:
            # calculate distance of cones
            for p in yellow_cones:
                distance = sqrt(p[0]**2 + p[1]**2)
                yellow_with_distance.append((p, distance))
                #print(p, distance)

            for p in blue_cones:
                distance = sqrt(p[0]**2 + p[1]**2)
                blue_with_distance.append((p, distance))
                #print(p, distance)

            # sort arrays by distance
            yellow_with_distance = sorted(
                    yellow_with_distance, 
                    key=lambda x: x[1]
                )

            blue_with_distance = sorted(
                    blue_with_distance, 
                    key=lambda x: x[1]
                )
            
            #print(yellow_with_distance, blue_with_distance)
            midpoints_count = 1
            if len(yellow_with_distance) < len(blue_with_distance):
                for i in range(len(yellow_with_distance)-1):
                    midpoints_count += 2
            else:
                for i in range(len(blue_with_distance)-1):
                    midpoints_count += 2
            #print(midpoints_count)
                
            increment_blue = False
            blue_index = 0
            yellow_index = 0

            for i in range(midpoints_count):
                bx = blue_with_distance[blue_index][0][0]
                yx = yellow_with_distance[yellow_index][0][0]
                by = blue_with_distance[blue_index][0][1]
                yy = yellow_with_distance[yellow_index][0][1]
                midpoint = ((bx+yx)/2, (by+yy)/2)

                midpoints.append(midpoint)
                #print(blue_with_distance[blue_index], yellow_with_distance[yellow_index], midpoint)
                if increment_blue:
                    blue_index += 1
                else:
                    yellow_index += 1

                increment_blue = (not increment_blue)
        except:
            pass

        return midpoints

    # midpoints should already be sorted I think
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
        
    # produces a 2D map based on current stored cone coordinates, this will halt the program until the map is closed
    def map_cones(self, b, y, m):
        plt.rcParams["figure.figsize"] = [7.00, 3.50]
        plt.rcParams["figure.autolayout"] = True
        plt.xlim(-2, 20)
        plt.ylim(-8, 8)
        plt.grid()
        plt.plot([i[0] for i in b], [i[1] for i in b], marker="o", markersize=10, markeredgecolor="red", markerfacecolor="blue")
        plt.plot([i[0] for i in y], [i[1] for i in y], marker="o", markersize=10, markeredgecolor="red", markerfacecolor="yellow")
        plt.plot([i[0] for i in m], [i[1] for i in m], marker="X", markersize=10, markeredgecolor="red", markerfacecolor="green")
        plt.plot([0], [0], marker=">", markersize=10, markeredgecolor="red", markerfacecolor="green")
        plt.show()


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
