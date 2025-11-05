#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import tf2_ros
from tf2_ros import TransformException
import time
import copy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from transforms3d._gohlketransforms import euler_from_quaternion, quaternion_from_euler

class TowerToMap(Node):

    def __init__(self):
        time.sleep(10)
        super().__init__('TowerToMapNode')
        # Used by the callback for the topic /tower/goal
        self.goal = None
        # TODO: Instantiate the Buffer and TransformListener

        # TODO: Goal publisher on topic /uav/input/goal

        # TODO: Tower goal subscriber to topic /tower/goal


        # start main loop
        self.rate = 2
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)

    #TODO: Callback for the tower goal subscriber

    def mainloop(self):
        if self.goal:
            try:
                pass
                #TODO: Lookup the tower to world transform

                #TODO: Convert the goal to a PointStamped

                #TODO: Use the do_transform_point function to convert the point using the transform

                #TODO: Convert the point back into a vector message containing integers

                #TODO: Publish the vector
                # self.get_logger().info(f'Publishing Transformed Goal: {msg.x}, {msg.y}')

                # The tower will automatically send you a new goal once the drone reaches the requested position.
                #TODO: Reset the goal

            except TransformException as ex:
                self.get_logger().info(f'Error getting the tower transformation')


def main():
    rclpy.init()
    try:
        rclpy.spin(TowerToMap())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()

# Main function
if __name__ == '__main__':
    main()