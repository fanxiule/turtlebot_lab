
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from math import sin


class CurvePublisher(object):

    def __init__(self):
        self.pub = rospy.Publisher(
            '/visualization_marker', Marker, queue_size=1)

    def publish(self, k):
        x = 0
        y = 0
        steps = 50

        lines = Marker()
        # lines.header.frame_id = "/map"
        lines.header.frame_id = "/base_link"
        lines.id = k  # each curve must have a unique id or you will overwrite an old ones
        lines.type = Marker.LINE_STRIP
        lines.action = Marker.ADD
        lines.ns = "curves"
        lines.scale.x = 0.1
        lines.color.r = 1.0
        lines.color.b = 0.2 * k
        lines.color.a = 1.0

        # generate curve points
        for i in range(steps):
            p = Point()
            p.x = x
            p.y = y
            p.z = 0  # not used
            lines.points.append(p)

            # curve model
            x = x + 0.1
            y = sin(0.1 * i * k)

        # publish new curve
        self.pub.publish(lines)
