#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
import json  # Use JSON instead of eval in production

# Global variables for sensor data
current_pose = None  # Will hold (x, y, yaw)
obstacles = []       # List of obstacle dictionaries, each with key "centroid" ([x, y, z])

# Hardcoded goal (x, y) in the map frame
GOAL = (0.0, 2.5)

def odom_callback(msg):
    global current_pose
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    quaternion = [ori.x, ori.y, ori.z, ori.w]
    (_, _, yaw) = euler_from_quaternion(quaternion)
    current_pose = (pos.x, pos.y, yaw)
    rospy.logdebug("Odom updated: x=%.2f, y=%.2f, yaw=%.2f", pos.x, pos.y, yaw)

def imu_callback(msg):
    # Not used in this simplified example; could be integrated for additional corrections.
    pass

def obstacle_callback(msg):
    global obstacles
    try:
        # In production, publish obstacles as JSON strings
        obstacles = json.loads(msg.data)
        rospy.logdebug("Obstacles updated: %s", obstacles)
    except Exception as e:
        rospy.logwarn("Failed to parse obstacle data: %s", e)
        obstacles = []

def compute_apf_command():
    """
    Compute the navigation command using an Artificial Potential Field (APF) approach.
    Attractive force pulls toward the goal, repulsive forces push away from obstacles.
    Returns a command string: "forward", "left", "right", or "stop".
    """
    if current_pose is None:
        return "stop"
    x, y, yaw = current_pose

    # Attractive force parameters
    k_attr = 1.0
    F_attr_x = k_attr * (GOAL[0] - x)
    F_attr_y = k_attr * (GOAL[1] - y)

    # Repulsive force parameters
    k_rep = 0.5
    d0 = 1.5  # influence distance (meters)
    F_rep_x = 0.0
    F_rep_y = 0.0
    for obs in obstacles:
        # Each obstacle is assumed to have a 'centroid': [x, y, z]
        obs_x = obs.get("centroid", [0,0,0])[0]
        obs_y = obs.get("centroid", [0,0,0])[1]
        dx = x - obs_x
        dy = y - obs_y
        dist = math.hypot(dx, dy)
        rospy.loginfo("Obstacle at (%.2f, %.2f) with distance: %.2f", obs_x, obs_y, dist)
        if 0 < dist < d0:
            rep_mag = k_rep * (1.0/dist - 1.0/d0) / (dist**2)
            F_rep_x += rep_mag * (dx/dist)
            F_rep_y += rep_mag * (dy/dist)

    # Total force vector
    F_total_x = F_attr_x + F_rep_x
    F_total_y = F_attr_y + F_rep_y

    # Desired heading is the angle of the total force
    desired_heading = math.atan2(F_total_y, F_total_x)
    heading_error = desired_heading - yaw
    # Normalize to [-pi, pi]
    heading_error = (heading_error + math.pi) % (2*math.pi) - math.pi

    # Debug output before returning
    rospy.loginfo("Attractive: (%.2f, %.2f), Repulsive: (%.2f, %.2f)", F_attr_x, F_attr_y, F_rep_x, F_rep_y)
    rospy.loginfo("Total: (%.2f, %.2f), Desired heading: %.2f, Yaw: %.2f, Error: %.2f", F_total_x, F_total_y, desired_heading, yaw, heading_error)

    # Determine command based on distance to goal and heading error
    distance_to_goal = math.hypot(GOAL[0]-x, GOAL[1]-y)
    if distance_to_goal < 0.5:
        return "stop"
    if abs(heading_error) < 0.2:
        return "forward"
    elif heading_error > 0:
        return "left"
    else:
        return "right"

def main():
    rospy.init_node("apf_navigation_node", anonymous=True)
    rospy.Subscriber("/rtabmap/odom", Odometry, odom_callback)
    rospy.Subscriber("/nano/imu_corrected", Imu, imu_callback)
    rospy.Subscriber("/obstacle_info", String, obstacle_callback)

    command_pub = rospy.Publisher("/arduino_commands", String, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz loop rate

    while not rospy.is_shutdown():
        command = compute_apf_command()
        command_pub.publish(command)
        rospy.loginfo("APF Command: %s", command)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

