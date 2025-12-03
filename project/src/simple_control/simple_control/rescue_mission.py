#!/usr/bin/env python3
"""Rescue mission node (checkpoint 1)

This node implements a simple occupancy grid generator for the project first
checkpoint. It subscribes to /uav/sensors/gps and /uav/sensors/lidar and
publishes an initial occupancy grid on /map (values initialized at 50).

Behavior implemented here is intentionally minimal to get the first
checkpoint working: we fuse the first few LIDAR scans into the map, marking
cells along each beam as free (decrease value) and marking the hit cell as
occupied (increase value). The node repeatedly publishes the current map so
the GUI can visualize it.
"""

from __future__ import annotations

import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan


class RescueMission(Node):
	"""Simple ROS2 node producing an OccupancyGrid from LIDAR and GPS.

	This implementation aims to provide a reliable first checkpoint: the
	occupancy grid is created, initialized to 50 (unknown) and updated when
	LIDAR scans arrive. It publishes `/map` at a slow rate so the GUI and
	environment controller can see the result.
	"""

	def __init__(self):
		super().__init__('rescue_mission_node')

		# Map parameters
		default_w = 23
		default_h = 23
		self.declare_parameter('map_width', default_w)
		self.declare_parameter('map_height', default_h)

		self.map_width = int(self.get_parameter('map_width').get_parameter_value().integer_value)
		self.map_height = int(self.get_parameter('map_height').get_parameter_value().integer_value)

		# We use 1 meter per cell (resolution 1.0)
		self.resolution = 1.0

		# Map origin: bottom-left in world coordinates; drone starts at (0,0)
		self.origin_x = -self.map_width / 2.0
		self.origin_y = -self.map_height / 2.0

		# Occupancy grid stored as flat list in row-major (x,y) to match GUI
		self.grid = [50] * (self.map_width * self.map_height)

		# ROS publishers/subscribers
		self.map_pub = self.create_publisher(OccupancyGrid, '/map', 1)
		self.gps_sub = self.create_subscription(PoseStamped, '/uav/sensors/gps', self.gps_callback, 10)
		self.lidar_sub = self.create_subscription(LaserScan, '/uav/sensors/lidar', self.lidar_callback, 10)

		# Local state
		self.has_gps = False
		self.drone_x = 0.0
		self.drone_y = 0.0

		# Publish map periodically so GUI picks it up
		self.create_timer(0.5, self.publish_map)

		# Mark start cell as free (drone starts at 0,0)
		sx, sy = self.world_to_map(0.0, 0.0)
		self.set_cell(sx, sy, 0)

		self.get_logger().info(f'rescue_mission started: map {self.map_width}x{self.map_height}')

	# ----------------------- helper methods -----------------------
	def world_to_map(self, wx: float, wy: float) -> tuple[int, int]:
		"""Convert world coordinates to integer map indices."""
		mx = int(wx - self.origin_x)
		my = int(wy - self.origin_y)
		return mx, my

	def map_index(self, mx: int, my: int) -> int:
		if mx < 0 or my < 0 or mx >= self.map_width or my >= self.map_height:
			return -1
		return int(my + mx * self.map_height)

	def get_cell(self, mx: int, my: int) -> int | None:
		idx = self.map_index(mx, my)
		if idx < 0:
			return None
		return self.grid[idx]

	def set_cell(self, mx: int, my: int, value: int) -> bool:
		idx = self.map_index(mx, my)
		if idx < 0:
			return False
		# clamp and store
		self.grid[idx] = max(0, min(100, int(value)))
		return True

	# ----------------------- callbacks -----------------------
	def gps_callback(self, msg: PoseStamped):
		self.drone_x = float(msg.pose.position.x)
		self.drone_y = float(msg.pose.position.y)
		self.has_gps = True

	def lidar_callback(self, msg: LaserScan):
		# Ignore scans until we have GPS (we need the origin)
		if not self.has_gps:
			return

		angle = msg.angle_min
		for r in msg.ranges:
			# Skip invalid negative values
			if math.isfinite(r) and r <= 0:
				angle += msg.angle_increment
				continue

			# For infinite, treat as far range
			max_range = msg.range_max if math.isfinite(msg.range_max) else 5.0
			dist = r if math.isfinite(r) else max_range

			# Trace along the beam in small steps and mark free cells
			# Step length smaller than resolution to ensure we hit each cell
			step = max(0.1, self.resolution / 4.0)
			t = 0.0
			last_mx_my = None
			while t < dist:
				wx = self.drone_x + t * math.cos(angle)
				wy = self.drone_y + t * math.sin(angle)
				mx, my = self.world_to_map(wx, wy)
				if (mx, my) != last_mx_my:
					# mark as more likely free
					cur = self.get_cell(mx, my)
					if cur is not None and cur != -1 and cur != -2 and cur != -3:
						# reduce occupancy confidence by 15
						self.set_cell(mx, my, (cur - 15))
					last_mx_my = (mx, my)
				t += step

			# mark the hit cell as occupied if the range was finite
			if math.isfinite(r) and dist > 0:
				hx = self.drone_x + dist * math.cos(angle)
				hy = self.drone_y + dist * math.sin(angle)
				hm_x, hm_y = self.world_to_map(hx, hy)
				cur = self.get_cell(hm_x, hm_y)
				if cur is not None:
					# increase occupancy confidence by 30
					self.set_cell(hm_x, hm_y, (cur + 30))

			angle += msg.angle_increment

	# ----------------------- publishing -----------------------
	def publish_map(self):
		msg = OccupancyGrid()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = 'map'

		info = MapMetaData()
		info.resolution = float(self.resolution)
		info.width = int(self.map_width)
		info.height = int(self.map_height)
		info.origin.position.x = float(self.origin_x)
		info.origin.position.y = float(self.origin_y)
		info.origin.position.z = 0.0
		msg.info = info

		# copy our internal grid
		msg.data = [int(g) for g in self.grid]

		self.map_pub.publish(msg)


def main(args=None):
	rclpy.init(args=args)
	try:
		node = RescueMission()
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		rclpy.try_shutdown()


if __name__ == '__main__':
	main()

