#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

class PathVisualizer:
    def __init__(self):
        rospy.init_node('path_visualizer')
        # What model name to track
        self.model_name = rospy.get_param('~model_name', 'automobile')
        # Path msg setup
        self.path = Path()
        self.path.header.frame_id = 'map'
        self.pub_path = rospy.Publisher('/ground_truth_path', Path, queue_size=1)
        # Marker setup
        self.marker = Marker()
        self.marker.header.frame_id = 'map'
        self.marker.ns = 'ground_truth'
        self.marker.id = 0
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.02   # line width
        self.marker.color.r = 1.0
        self.marker.color.a = 1.0
        self.pub_marker = rospy.Publisher('/ground_truth_marker', Marker, queue_size=1)

        # Subscribe
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.cb_states, queue_size=1)

    def cb_states(self, msg: ModelStates):
        # find the index of our model
         # time‐stamp using the current ROS time
        stamp = rospy.Time.now()
        try:
            idx = msg.name.index(self.model_name)
            print(f"Found {self.model_name} at index {idx}")
        except ValueError:
            return

        pose = msg.pose[idx]

        # 1) Append to Path
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = 'map'
        ps.pose = pose
        self.path.header.stamp = stamp
        self.path.poses.append(ps)
        self.pub_path.publish(self.path)

        # 2) Append to Marker
        pt = pose.position
        self.marker.header.stamp = stamp
        self.marker.points.append(pt)
        self.pub_marker.publish(self.marker)

if __name__=='__main__':
    PathVisualizer()
    rospy.spin()
