#!/usr/bin/env python
import sys
import rospy
import tf

import numpy as np 
from math import floor, log, sin, cos
from nav_msgs.msg import OccupancyGrid, MapMetaData	
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from sensor_msgs.msg import LaserScan

def prob_to_logit(prob):
	if prob <= 0.0:
		return -float('inf')
	if prob == 1.0:
		return float('inf')
	else:
		return log(prob/(1.0-prob))

#some constants
O_0 = 0.5
O_occ = 0.9
O_free = 0.1
l_0 = prob_to_logit(O_0)


class mapMaker():

	def __init__(self):
		rospy.init_node('turtlebot', anonymous=True)

		# initialize a publisher for occupancy grid
		self.occupancy_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
		# rospy.loginfo("got into initializaytion")
		self.occupancy_grid = OccupancyGrid()
		metadata = MapMetaData()
		# metadata.map_load_time = None
		metadata.resolution = resolution = rospy.get_param("~grid_resolution", 1)
		metadata.width = 250
		metadata.height = 250
		pos = np.array([-metadata.width * resolution / 2, -metadata.height * resolution / 2, 0])
		metadata.origin = Pose()
		metadata.origin.position.x, metadata.origin.position.y = pos[:2]
		self.conversion_m_to_xy = metadata.width/30
		#self.conversion_m_to_y = self.


		self.map_metadata = metadata
		self.occupancy_grid.info = self.map_metadata
		self.occupancy_grid.data = np.ones(metadata.height * metadata.width).astype(int)*O_0
		self.laser_data = None
		self.position = None
		self.orientation = None
		self.covariance = None

		# initialize a subscriber to receive the estimated pose and the map
		rospy.Subscriber("/scan", LaserScan, self.laser_callback)

		# initialize a subscriber to receiver estimated posiition from the kalman filter output
		rospy.Subscriber('/indoor_pos', PoseWithCovarianceStamped, self.IPS_callback)

		# rate = rospy.Rate(20)  # hz

		# while ROS is still running
		# while not rospy.is_shutdown():

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
		# rate.sleep()


	def laser_callback(self, msg):
		rospy.loginfo("got into laser callback")
		self.laser_data = msg
		angle = self.laser_data.angle_min
		self.angles = []																														
		self.ranges = []
		i = 0
		while angle <= self.laser_data.angle_max: 
			if not (self.laser_data.ranges[i] < self.laser_data.range_min or self.laser_data.ranges[i] > self.laser_data.range_max or np.isnan(self.laser_data.ranges[i])):
				self.angles.append(angle)
				if np.isnan(self.laser_data.ranges[i]):
					self.ranges.append(0)
				else :
					self.ranges.append(self.laser_data.ranges[i])
			angle += self.laser_data.angle_increment
			i +=1
		#rospy.loginfo("outtttttt the loop buddy")
		self.update()
		

	def IPS_callback(self, msg):
		self.position = msg.pose.pose.position
		quaternion = msg.pose.pose.orientation
		self.orientation = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
		self.covariance = msg.pose.covariance


	def update (self) :
		rospy.loginfo("In the update")
		#rospy.loginfo("position %s" % self.position)
		if self.position is None:
			return
		x = self.position.x
		y = self.position.y
		z = self.position.z
		pitch, roll, yaw = self.orientation
		#rospy.loginfo("got into update callback")

		# for r,a in self.angles, self.ranges:
		# rospy.loginfo("angles %s" % self.angles)

		for j in range(0,len(self.angles)):
			r = self.ranges[j]
			a = self.angles[j]
			
			cx, cy, pr = self.inverseScanner(x, y, yaw, a, r)
			# rospy.loginfo("x = %s, y = %s, yaw = %s, a = %s, r =%s" % (x,y,yaw,a,r))
			
			#rospy.loginfo("cx = %s, cy = %s, pr = %s" % (cx,cy,pr))
			for i in range(0,len(cx)):
				idx = int(cx[i])*self.occupancy_grid.info.width + int(cy[i])
				#rospy.loginfo("pr %s", pr)
				self.occupancy_grid.data[idx] = int(self.occupancy_grid.data[idx] + prob_to_logit(pr[i]) - l_0)
		self.publish_data()





	def inverseScanner(self,x_robot,y_robot,theta_robot,theta_scan,range_scan):
		#convert scans to cartesian coordinates of end points

		#call bresenham algorithm from start point to end point to receive list of all points on the line
		#for every point calculate occupancy probability
		#return cx, cy, pr (lists most likely)
		#rospy.loginfo("Tomas DEBUG x_robot = %s, y_robot = %s, theta_robot= %s, theta_scan = %s, range_scan =%s" % (x_robot,y_robot,theta_robot,theta_scan,range_scan))
		x_obj = x_robot + range_scan * sin(theta_robot + theta_scan)
		# rospy.loginfo("test")
		# rospy.loginfo(x_obj)
		y_obj = y_robot + range_scan * cos(theta_robot + theta_scan)
		x_array, y_array = self.bresenham(x_robot*self.conversion_m_to_xy, y_robot*self.conversion_m_to_xy, x_obj*self.conversion_m_to_xy, y_obj*self.conversion_m_to_xy)
		# rospy.loginfo('x_array = %s' % x_array)
		pr = np.ones(len(x_array))*O_free
		pr[-1] = O_occ
		return x_array , y_array, pr

	def publish_data(self):
		data_ = [np.int8(i*100) for i in self.occupancy_grid.data]
		occGrid = OccupancyGrid(info=self.map_metadata, data =data_)

		rospy.loginfo("about to publish: %s " % len(data_))
	
		self.occupancy_pub.publish(occGrid) 


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