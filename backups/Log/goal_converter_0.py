#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Point

def goal_callback(msg):
    """ Converts PoseStamped to Point and publishes to Arduino """
    goal_point = Point()
    goal_point.x = msg.pose.position.x
    goal_point.y = msg.pose.position.y
    goal_point.z = 0.0  # Ignore Z for 2D navigation

    goal_pub.publish(goal_point)
    rospy.loginfo(f"Converted Goal: x={goal_point.x}, y={goal_point.y}")

# Initialize ROS Node
rospy.init_node("goal_converter", anonymous=True)

# Publisher (Arduino expects Point)
goal_pub = rospy.Publisher("/goal_to_arduino", Point, queue_size=10)

# Subscriber (RViz sends PoseStamped)
rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)

rospy.loginfo("Goal Converter Node Started")
rospy.spin()
