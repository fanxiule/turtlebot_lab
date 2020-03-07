import numpy as np 
from math import floor, log, sin, cos
from nav_msgs.msg import OccupancyGrid, MapMetaData	

#some constants
O_0 = 0.5
O_occ = 1
O_free = 0
l_0 = log(O_0/(1-O_0))


class mapMaker(self):

	def __init__(self):
		rospy.init_node('turtlebot', anonymous=True)

		# initialize a publisher for occupancy grid
	    self.occupancy_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
	    self.occupancy_grid = OccupancyGrid()
	    metadata = MapMetaData
	    metadata.map_load_time
	    metadata.resolution = 
	    metadata.width = 
	    metadata.height = 
	    metadata.origin = 
	    metadata.origin.position.x = 
	    metadata.origin.orientation = 

	    self.map_metadata = metadata


	    # initialize a subscriber to receive the estimated pose and the map
	    rospy.Subscriber("/scan", LaserScan, laser_callback, ())

	    # initialize a subscriber to receiver estimated posiition from the kalman filter output
	    rospy.Subscriber('/pose', PoseStamped, kalman_callback, ())

	    rate = rospy.Rate(20)  # hz

	    # while ROS is still running
	    while not rospy.is_shutdown():

	        # movement forward (X-axis only)
	        msg.linear.x = 0.1
	        msg.linear.y = 0
	        msg.linear.z = 0

	        # and angular rotation
	        msg.angular.x = 0
	        msg.angular.y = 0
	        msg.angular.z = 0.3

	        pub.publish(msg)
	        rate.sleep()

	def laser_callback():
		#map correct data types and call update
		pass

	def kalman_callback(self, data):
		self.X_t = data
		pass


	def update (self, X_t, ranges, angles) :
		for each r,a in ranges, angles:
			cx, cy, pr = inverseScanner( X_t, rng=r, bearing=a)
			for i in length(cx):
				self.occupancy_grid(cx,cy) = self.occupancy_grid(cx,cy) + log(pr/(1-pr)) - l_0

	def inverseScanner(self,x_robot,y_robot,theta_robot,theta_scan,range_scan):
		#convert scans to cartesian coordinates of end points

		#call bresenham algorithm from start point to end point to receive list of all points on the line
		#for every point calculate occupancy probability
		#return cx, cy, pr (lists most likely)
		x_obj = x_robot + range_scan * sin(theta_robot + theta_scan)
		y_obj = y_robot + range_scan * cos(theta_robot + theta_scan)

		return x_obj , y_obj

	def publish_data(self):
		occGrid = OccupancyGrid()
		occGrid.header.frame_id = '/world'
		occGrid.info = self.map_metadata
		occGrid.data = ()
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