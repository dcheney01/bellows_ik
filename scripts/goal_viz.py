#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# Author: Daniel Cheney
"""
This file allows to display a point in RVIZ that is controlled by an xbox controller
"""

global workspace
workspace = Marker()
workspace.type = 1
workspace.id = 1

workspace.header.frame_id = "map"

# Set the scale of the marker
workspace.scale.x = 2
workspace.scale.y = 2
workspace.scale.z = 1

# Set the color
workspace.color.r = 1.0
workspace.color.g = 0.0
workspace.color.b = 0.0
workspace.color.a = .2

workspace.pose.position.x = 0.0
workspace.pose.position.y = 0.0
workspace.pose.position.z = 0.5

workspace.pose.orientation.x = 0.0
workspace.pose.orientation.y = 0.0
workspace.pose.orientation.z = 0.0
workspace.pose.orientation.w = 1.0


def my_marker(id, color=[1,0]):
    marker = Marker()
    # This link was helpful:
        # https://answers.ros.org/question/373802/minimal-working-example-for-rviz-marker-publishing/
        
    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 1
    marker.id = id

    marker.header.frame_id = "map"

    # Set the scale of the marker
    scale = 1
    marker.scale.x = .15 * scale
    marker.scale.y = .15 * scale
    marker.scale.z = .15 * scale

    # Set the color
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    return marker


def update_marker_cb(msg):
    # Set the pose of the marker
    marker = my_marker(0)

    marker.pose.position.x = msg.x
    marker.pose.position.y = msg.y
    marker.pose.position.z = msg.z

    marker_pub.publish(marker)

def update_current_position_cb(msg):
    # Set the pose of the marker
    marker = my_marker(2, color=[0,1])

    marker.pose.position.x = msg.x
    marker.pose.position.y = msg.y
    marker.pose.position.z = msg.z

    marker_pub.publish(marker)

def start():
    global marker_pub
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=5)
    rospy.Subscriber("/goal_position", Point, update_marker_cb)
    rospy.Subscriber("/current_position", Point, update_current_position_cb)


if __name__ == '__main__':
    rospy.init_node('goal_viz')
    start()

    _control_rate = rospy.Rate(200.0)
    while not rospy.is_shutdown():
        marker_pub.publish(workspace)
        _control_rate.sleep()

