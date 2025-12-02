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

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TowerToMap(Node):

    def __init__(self):
        time.sleep(10)
        super().__init__('TowerToMapNode')
        # Used by the callback for the topic /tower/goal
        self.goal = None
        # TODO: Instantiate the Buffer and TransformListener
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self)

        # TODO: Goal publisher on topic /uav/input/goal
        self.goal_pub = self.create_publisher(Vector3, '/uav/input/goal', 10)

        # TODO: Tower goal subscriber to topic /tower/goal
        self.tower_sub = self.create_subscription(Vector3, '/tower/goal', self.tower_goal_cb, 10)


        # start main loop
        self.rate = 2
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)

    #TODO: Callback for the tower goal subscriber
    def tower_goal_cb(self, msg):
        # store incoming goal (in tower frame)
        self.goal = msg
        
    def mainloop(self):
        if self.goal:
            try:
                #TODO: Lookup the tower to world transform
                transform = self.tfBuffer.lookup_transform('world', 'tower', rclpy.time.Time())
                #TODO: Convert the goal to a PointStamped
                point = PointStamped()
                point.header.frame_id = 'tower'
                point.header.stamp = self.get_clock().now().to_msg()
                point.point.x = float(self.goal.x)
                point.point.y = float(self.goal.y)
                point.point.z = float(self.goal.z)
                #TODO: Use the do_transform_point function to convert the point using the transform
                new_point = do_transform_point(point, transform)
                #TODO: Convert the point back into a vector message containing integers
                goal_msg = Vector3()
                goal_msg.x = float(new_point.point.x)
                goal_msg.y = float(new_point.point.y)
                goal_msg.z = float(new_point.point.z)
                self.goal_pub.publish(goal_msg)
                self.get_logger().info(f'Publishing Transformed Goal: {goal_msg.x}, {goal_msg.y}')

                #TODO: Publish the vector
                # self.get_logger().info(f'Publishing Transformed Goal: {goal_msg.x}, {goal_msg.y}')

                # The tower will automatically send you a new goal once the drone reaches the requested position.
                #TODO: Reset the goal
                self.goal = None

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
