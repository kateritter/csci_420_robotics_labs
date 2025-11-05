#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import tf2_ros
import time
import copy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from transforms3d._gohlketransforms import euler_from_quaternion, quaternion_from_euler

class WorldToDrone(Node):

    def __init__(self):
        time.sleep(10)
        super().__init__('WorldToDroneNode')
        # Used by the callback for the topic /tower/goal
        self.gps = None
        self.transform_stamped = TransformStamped()
        # TODO: Instantiate the Broadcaster

        # TODO: Drone GPS subscriber to topic /uav/sensors/gps

        # TODO: fill in the parent and child frames
        self.transform_stamped.header.frame_id = 'TODO'
        self.transform_stamped.child_frame_id = 'TODO'

        # start main loop
        self.rate = 2
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)

    #TODO: Callback for the tower goal subscriber

    def mainloop(self):
        if self.gps:
            # TODO: Use the GPS position to set up the transform
            self.transform_stamped.transform.translation = None  # TODO: set the Translation from the GPS
            self.transform_stamped.transform.rotation = None  # TODO: set the quaternion from the GPS

            # TODO: Update the header to the current timestamp
            self.transform_stamped.header.stamp = 0  # TODO: set to the current time using self.get_clock().now().to_msg()
            # TODO: Broadcast the transform


def main():
    rclpy.init()
    try:
        rclpy.spin(WorldToDrone())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()

# Main function
if __name__ == '__main__':
    main()