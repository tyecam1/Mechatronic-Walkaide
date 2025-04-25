#!/usr/bin/env python3

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

# ROS Publisher for obstacles
obstacle_pub = None
MAX_OBSTACLES = 5  # Send at most 5 obstacles to Arduino
DETECTION_RADIUS = 1.0  # Distance threshold for detecting obstacles (meters)
PUBLISH_RATE = 10

def scan_callback(scan_msg):
    """
    Processes LiDAR scan data to detect obstacles and send positions to Arduino.
    """
    global obstacle_pub

    rate = rospy.Rate(PUBLISH_RATE)

    obstacle_list = []
    angle_increment = scan_msg.angle_increment
    angle_min = scan_msg.angle_min
    ranges = np.array(scan_msg.ranges)

    for i in range(len(ranges)):
        distance = ranges[i]
        if distance > 0.1 and distance < DETECTION_RADIUS:  # Filter valid obstacles
            angle = angle_min + i * angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            obstacle_list.append((x, y))

    rate.sleep()

    # Sort obstacles by distance and limit to MAX_OBSTACLES
    obstacle_list.sort(key=lambda p: math.sqrt(p[0]**2 + p[1]**2))
    obstacle_list = obstacle_list[:MAX_OBSTACLES]

    # rospy.loginfo(f"Detected {len(obstacle_list)} obstacles")

    for x, y in obstacle_list:
        obstacle_msg = Point()
        obstacle_msg.x = x
        obstacle_msg.y = y
        obstacle_pub.publish(obstacle_msg)

def main():
    global obstacle_pub

    rospy.init_node("obstacle_detector")
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    obstacle_pub = rospy.Publisher("/obstacles_to_arduino", Point, queue_size=5)

    rospy.loginfo("Obstacle Detector Node Started")
    rospy.spin()

if __name__ == "__main__":
    main()
