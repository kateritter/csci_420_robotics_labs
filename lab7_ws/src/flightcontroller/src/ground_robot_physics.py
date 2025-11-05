#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import tf2_ros
import time
import copy
import math
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_point


class GroundRobotPhysics(Node):

    def __init__(self):
        time.sleep(10)
        super().__init__('GroundRobotPhysics')
        # Used by the callback for the topic /tower/goal
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'world'
        self.goal = None
        self.max_speed = 0.5
        self.goal_sub = self.create_subscription(Vector3, "/ground_robot/goal", self.get_goal, 1)
        self.gps_pub = self.create_publisher(PoseStamped, "/ground_robot/gps", 1)
        # start main loop
        self.rate = 5
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)

    def get_goal(self, msg):
        self.goal = copy.copy(msg)

    def mainloop(self):
        if self.goal is not None:
            dx = self.goal.x - self.pose.pose.position.x
            dy = self.goal.y - self.pose.pose.position.y
            angle = math.atan2(dy, dx)
            # normalize so that we move either to the goal or at max speed, whichever is less
            to_move_x = self.max_speed * math.cos(angle)
            to_move_y = self.max_speed * math.sin(angle)
            if abs(dx) < abs(to_move_x):
                to_move_x = dx
            if abs(dy) < abs(to_move_y):
                to_move_y = dy
            self.pose.pose.position.x += to_move_x
            self.pose.pose.position.y += to_move_y
            self.pose.header.stamp = self.get_clock().now().to_msg()
            # the pose pub is locked behind the goal being set because we don't want the robot to be visible until it has a goal
            self.gps_pub.publish(self.pose)

def main():
    rclpy.init()
    try:
        rclpy.spin(GroundRobotPhysics())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()

# Main function
if __name__ == '__main__':
    main()