#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

if __name__=='__main__':
    rospy.init_node('track_mesh_publisher')
    pub = rospy.Publisher('/track_mesh_marker', Marker, queue_size=1, latch=True)
    rospy.sleep(0.5)

    m = Marker()
    m.header.frame_id    = 'map'
    m.header.stamp       = rospy.Time.now()
    m.ns                 = 'track'
    m.id                 = 0
    m.type               = Marker.MESH_RESOURCE
    m.action             = Marker.ADD
    # Replace this with the actual URI your Gazebo uses:
    m.mesh_resource      = 'model://your_track/meshes/track.dae'
    m.mesh_use_embedded_materials = True
    # position/orientation (identity, since mesh is already in world coords)
    m.pose.orientation.w = 1.0
    # scale is 1 if your mesh units match meters
    m.scale.x = m.scale.y = m.scale.z = 1.0
    # color multiplier (if not using embedded materials)
    m.color.r = m.color.g = m.color.b = 1.0
    m.color.a = 1.0

    pub.publish(m)
    rospy.loginfo("Published track mesh marker. Shutting down.")
    rospy.spin()
