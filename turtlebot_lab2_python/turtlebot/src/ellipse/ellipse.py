
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from math import sqrt


class EllipPublisher(object):

    def __init__(self):
        self.pub = rospy.Publisher(
            '/visualization_marker', Marker, queue_size=1)

    def publish(self, x_eig, y_eig):
        x = 0
        y = 0
        steps = 50
        
        chi_area = 1.39  # chi square curve area, 0.50
        a = sqrt(chi_area**2*x_eig)
        b = sqrt(chi_area**2*y_eig)

        lines = Marker()
        # lines.header.frame_id = "/map"
        lines.header.frame_id = "/base_link"
        lines.id = 1  # each curve must have a unique id or you will overwrite an old ones
        lines.type = Marker.LINE_STRIP
        lines.action = Marker.ADD
        lines.ns = "ellipse"
        lines.scale.x = 0.01
        lines.color.r = 1.0
        lines.color.b = 0.2
        lines.color.a = 1.0

        # generate curve points
        for i in range(steps): # second quadrant
            p = Point()
            p.z = 0  # not used
            y = sqrt((1-x**2/a**2)*b**2)
            p.x = x
            p.y = y
            lines.points.append(p)
            x = x - a/steps

        x = x + a/steps
        for i in range(steps): # third quadrant
            p = Point()
            p.z = 0  # not used
            y = sqrt((1-x**2/a**2)*b**2)
            p.x = x
            p.y = -y
            lines.points.append(p)
            x = x + a/steps
        
        x = x - a/steps
        for i in range(steps): # fourth quadrant
            p = Point()
            p.z = 0  # not used
            y = sqrt((1-x**2/a**2)*b**2)
            p.x = x
            p.y = -y
            lines.points.append(p)
            x = x + a/steps
        
        x = x - a/steps
        for i in range(steps): # first quadrant
            p = Point()
            p.z = 0  # not used
            y = sqrt((1-x**2/a**2)*b**2)
            p.x = x
            p.y = y
            lines.points.append(p)
            x = x - a/steps

        # publish new curve
        self.pub.publish(lines)
