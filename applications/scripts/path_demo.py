#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion, Pose, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, ColorRGBA
import math

class NavPath(object):
    def __init__(self, threshold=0.5):
        self._path = []
        self.threshold = threshold
        self.last_pos = None

        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        rospy.Subscriber('odom', Odometry, self.callback)

    def callback(self, msg):
        pos = msg.pose.pose.position
        if self.last_pos is None or self.distance(self.last_pos, pos) > self.threshold:
            self.last_pos = pos
            self._path.append(Point(pos.x, pos.y, pos.z))
            self.publish_path()

    def distance(self, p1, p2):
        """Calculate distance between two points."""
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)

    def publish_path(self):
        """Publish the path as a LINE_STRIP marker."""
        marker = Marker(
            type=Marker.LINE_STRIP,
            id=0,
            lifetime=rospy.Duration(0),
            scale=Vector3(0.05, 0, 0),  # width of the line
            header=Header(frame_id='odom'),
            color=ColorRGBA(0.0, 1.0, 0.0, 1.0),  # red color
            points=self._path
        )
        self.marker_pub.publish(marker)

def main():
    rospy.init_node('nav_path_node')
    nav_path = NavPath()
    rospy.spin()  # Keep the node alive to listen to callbacks

if __name__ == '__main__':
    main()
