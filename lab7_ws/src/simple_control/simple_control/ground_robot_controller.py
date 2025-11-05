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

class GroundRobotController(Node):

    def __init__(self):
        time.sleep(20)  # takes longer for the other pieces to come up
        super().__init__('GroundRobotController')
        # Our goal is 0,0,0 *in the drone frame*
        self.goal = Vector3()
        # TODO: Instantiate the Buffer and TransformListener

        # TODO: set up publisher to /ground_robot/goal - this will publish *in the world frame*

        # start main loop
        self.rate = 2
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)

    def mainloop(self):
        if self.goal:
            try:
                pass
                # TODO: Lookup the drone to world transform

                # TODO: Convert the goal to a PointStamped

                # TODO: Use the do_transform_point function to convert the point using the transform

                # TODO: Convert the point back into a vector message containing integers

                # TODO: Publish the vector
                # self.get_logger().info(f'Publishing Transformed Goal for Ground Robot: {msg.x}, {msg.y}')

            except TransformException as ex:
                self.get_logger().info(f'Error getting the drone transformation')


def main():
    rclpy.init()
    try:
        rclpy.spin(GroundRobotController())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()

# Main function
if __name__ == '__main__':
    main()