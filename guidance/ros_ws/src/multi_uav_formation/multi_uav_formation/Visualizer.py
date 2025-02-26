import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class Visualizer:
    def __init__(self):
        self.marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
        self.markers = []

    def add_cylinder(self, position_xy, radius):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "cylinders"
        marker.id = len(self.markers)
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = position_xy[0]
        marker.pose.position.y = position_xy[1]
        marker.pose.position.z = 2.5
        marker.pose.orientation.w = 1.0

        marker.scale.x = 2 * radius
        marker.scale.y = 2 * radius
        marker.scale.z = 5.0

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration(0)

        self.markers.append(marker)

    def publish_once(self):
        marker_array = MarkerArray()

        if self.markers:
            for marker in self.markers:
                marker.header.stamp = rospy.Time.now()
                marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
