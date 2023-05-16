# listens to the /ground_truth/cones topic
# records positions of currently visible cones

import datetime
import rclpy
from math import sqrt, asin, degrees
from rclpy.node import Node
from matplotlib import pyplot as plt

from eufs_msgs.msg._cone_array_with_covariance import ConeArrayWithCovariance

class ConeSubscriber(Node):

    def __init__(self):
        super().__init__('cone_subscriber')
        self.subscription = self.create_subscription(
            ConeArrayWithCovariance,
            '/ground_truth/cones',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.record = open('cone_recording.csv', 'w') # for recording cone locations at a given time
        self.x = []
        self.y = []

    # saves the positions of currently visible cones
    def listener_callback(self, msg):

        cones_array = []

        # loop over every visible cone
        for arr in [msg.blue_cones, msg.yellow_cones, msg.big_orange_cones]:
            for cone in arr:

                cones_array.append(self.parse_cone_data(cone))

        self.x = [i[0] for i in cones_array]
        self.y = [i[1] for i in cones_array]

        # sort the cone array by distance, nearest cones first
        cones_array = sorted(
                        cones_array, 
                        key=lambda x: x[2]
                    )

        #self.write_cones(cones_array)
        print(cones_array)
        #self.get_logger().info(msg)
        #self.map_cones()

    # create string of cone data and write it to file
    def write_cones(self, cones_array):
        save = f'{datetime.datetime.now()}'

        # get data for first (nearest) 10 cones, giving 0 values if none
        for i in range(10):
            if i < len(cones_array):
                save += f',{cones_array[i][2]},{cones_array[i][3]}'
            else:
                save += ',0,0'

        save += '\n'
        self.record.write(save)


    # get distance and angle from car from cone points
    def parse_cone_data(self, cone):
        distance = sqrt(cone.point.x**2 + cone.point.y**2)
        angle = 90 - degrees(asin(abs(cone.point.y) / distance))

        # convery cones to cars left to negative angles
        if cone.point.y > 0:
            angle = -angle

        return (cone.point.x, cone.point.y, distance, angle)
    
    # produces a 2D map based on current stored cone coordinates
    def map_cones(self):
        plt.rcParams["figure.figsize"] = [7.00, 3.50]
        plt.rcParams["figure.autolayout"] = True
        plt.xlim(0, 20)
        plt.ylim(-15, 15)
        plt.grid()
        plt.plot(self.x, self.y, marker="o", markersize=10, markeredgecolor="red", markerfacecolor="green")
        plt.show()


def main(args=None):
    rclpy.init(args=args)

    cone_subscriber = ConeSubscriber()

    rclpy.spin(cone_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cone_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
