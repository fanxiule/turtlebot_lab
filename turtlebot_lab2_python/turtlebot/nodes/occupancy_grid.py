#!/usr/bin/env python
import sys
import rospy
import tf

import numpy as np 
from math import floor, log, sin, cos
from nav_msgs.msg import OccupancyGrid, MapMetaData	
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from sensor_msgs.msg import LaserScan

#some constants
O_0 = 0.5
O_occ = 1
O_free = 0
l_0 = log(O_0/(1-O_0))


class mapMaker():

	def __init__(self):
		rospy.init_node('turtlebot', anonymous=True)

		# initialize a publisher for occupancy grid
		self.occupancy_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
		self.occupancy_grid = OccupancyGrid()
		metadata = MapMetaData()
		metadata.map_load_time = None
		metadata.resolution = resolution = rospy.get_param("~grid_resolution", .10)
		metadata.width = int(rospy.get_param("~grid_width", 1000))
		metadata.height = int(rospy.get_param("~grid_height", 1000))
		pos = np.array([-metadata.width * resolution / 2, -metadata.height * resolution / 2, 0])
		metadata.origin = Pose()
		metadata.origin.position.x, metadata.origin.position.y = pos[:2]

		self.map_metadata = metadata
		self.laser_data = None
		self.position = None
		self.orientation = None
		self.covariance = None

		# initialize a subscriber to receive the estimated pose and the map
		rospy.Subscriber("/scan", LaserScan, self.laser_callback)

		# initialize a subscriber to receiver estimated posiition from the kalman filter output
		rospy.Subscriber('/indoor_pos', PoseWithCovarianceStamped, self.IPS_callback)

		rate = rospy.Rate(20)  # hz

		# while ROS is still running
		while not rospy.is_shutdown():

			"""
			# movement forward (X-axis only)
			msg.linear.x = 0.1
			msg.linear.y = 0
			msg.linear.z = 0

			# and angular rotation
			msg.angular.x = 0
			msg.angular.y = 0
			msg.angular.z = 0.3
			"""
			
			#pub.publish(msg)
			rate.sleep()


	def laser_callback(self, msg):
		self.laser_data = msg
		angle = self.laser_data.angle_min
		self.angles = []
		self.ranges = []
		i = 0
		while angle <= self.laser_data.angle_max: 
			if not (self.laser_data.ranges[i] < self.laser_data.range_min or self.laser_data.ranges[i] > self.laser_data.range_max):
				self.angles.append(angle)
				self.ranges.append(self.laser_data.ranges[i])
			return
		

	def IPS_callback(self, msg):
		self.position = msg.pose.pose.position
		quaternion = msg.pose.pose.orientation
		self.orientation = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
		self.covariance = msg.pose.covariance


	def update (self) :
		x, y, z = self.position
		pitch, roll, yaw = self.orientation

		for r,a in self.angles, self.ranges:
			cx, cy, pr = inverseScanner(x, y, yaw, a, r)
			for i in length(cx):
				idx = cx*self.occupancy_grid.info.width + cy*self.occupancy_grid.info.height
				self.occupancy_grid[idx] = self.occupancy_grid[idx] + log(pr/(1-pr)) - l_0
		self.publish_data()

	def inverseScanner(self,x_robot,y_robot,theta_robot,theta_scan,range_scan):
		#convert scans to cartesian coordinates of end points

		#call bresenham algorithm from start point to end point to receive list of all points on the line
		#for every point calculate occupancy probability
		#return cx, cy, pr (lists most likely)
		x_obj = x_robot + range_scan * sin(theta_robot + theta_scan)
		y_obj = y_robot + range_scan * cos(theta_robot + theta_scan)
		x_array, y_array = self.bresenham(x_robot, y_robot, x_obj, y_obj)
		pr = np.zeros(len(x_array))
		pr[-1] = 1
		return x_obj , y_obj, pr

	def publish_data(self):
		occGrid = OccupancyGrid()
		occGrid.header.frame_id = '/world'
		occGrid.info = self.map_metadata
		occGrid.data = ()
		self.occupancy_pub.publish(self.occupancy_grid) 


	def bresenham(self, x1, y1, x2,y2):
		x1 = round(x1)
		x2 = round(x2)
		y1 = round(y1)
		y2 = round(y2)

		dx = abs(x2-x1)
		dy = abs(y2-y1)

		steep = dy > dx
		if steep:
			t = dx
			dx = dy
			dy =t

		if dy == 0:
			q = np.zeros([int(dx + 1), 1])
		else:
			q = [a for a in np.arange(floor(dx/2), -dy * dx + floor(dx/2) - dy, -dy)]
			q = np.hstack([0, (np.diff(np.mod(q, dx)) >= 0).astype(float)])

		if steep:
			if y1 <= y2:
				y = np.arange(y1, y2+1)
			else:
				y = np.arange(y1, y2-1, -1)
			if x1 <= x2:
				x = x1+np.cumsum(q)
			else:
				x = x1-np.cumsum(q)
		else:
			if x1 <= x2:
				x = np.arange(x1, x2+1)
			else:
				x = np.arange(x1, x2-1, -1)
			if y1 <= y2:
				y = y1+np.cumsum(q)
			else:
				y = y1-np.cumsum(q)
		
		return x, y


def main():
	map = mapMaker()
	while True:
		continue




if __name__ == '__main__':
	try:
		sys.exit(main())
	except rospy.ROSInterruptException:
		pass