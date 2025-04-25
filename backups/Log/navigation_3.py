#!/usr/bin/env python3
import rospy
import math
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
import json  # Use JSON instead of eval in production
from diagnostic_updater import Updater, DiagnosticStatusWrapper

# Diagnostics
updater = Updater()
updater.setHardwareID("APF Navigation Node")

def diagnostic_callback(stat):
    diag = DiagnosticStatusWrapper()
    diag.name = "APF Navigation Diagnostics"
    diag.level = DiagnosticStatusWrapper.OK
    diag.message = "Node running"
    diag.add("Processing Time (ms)", last_processing_time)
    diag.add("Heading Error", last_heading_error)
    stat.add(diag)

updater.add("APF Navigation Diagnostics", diagnostic_callback)

# Global variables for sensor data
current_pose = None  # (x, y, yaw) from odometry
imu_yaw = None       # Yaw from IMU
obstacles = []       # List of obstacle dictionaries, each with key "centroid" ([x, y, z])
rtabmap_info = {}    # SLAM metrics (e.g., loop closures, localization error)

# Additional publishers for APF forces and performance metrics
apf_forces_pub = rospy.Publisher("/apf_forces", String, queue_size=10)
perf_pub = rospy.Publisher("/navigation_perf", String, queue_size=10)

def odom_callback(msg):
    global current_pose, GOAL
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    quaternion = [ori.x, ori.y, ori.z, ori.w]
    (roll, pitch, yaw) = euler_from_quaternion(quaternion)
    current_pose = (pos.x, pos.y, pos.z, roll, pitch, yaw)
    
    # Update GOAL: 3m ahead in the direction the person is facing
    GOAL = (pos.x + 3.0 * math.cos(yaw), pos.y + 3.0 * math.sin(yaw))
    
    rospy.logdebug("Odom updated: x=%.2f, y=%.2f, yaw=%.2f, GOAL updated: (%.2f, %.2f)", 
                   pos.x, pos.y, yaw, GOAL[0], GOAL[1])


def obstacle_callback(msg):
    global obstacles
    try:
        data = json.loads(msg.data)
        msg_length = len(msg.data.encode('utf-8'))  # measure byte size
        rospy.logdebug("Received obstacle message (%d bytes)", msg_length)
        # In production, publish obstacles as JSON strings
        obstacles = data.get("obstacles",[])
        rospy.logdebug("Obstacles updated: %s", obstacles)
    except Exception as e:
        rospy.logwarn("Failed to parse obstacle data: %s", e)
        obstacles = []

def compute_apf_command():
    """
    Compute the navigation command using an Artificial Potential Field (APF) approach.
    Attractive force pulls toward the goal, repulsive forces push away from obstacles.
    Returns a tuple: (command, apf_forces, processing_time).
    """
    global last_processing_time, last_heading_error
    start_time = time.time()
    if current_pose is None:
        return ("stop", {}, 0.0)
    x, y, yaw = current_pose

    # Attractive force parameters
    #k_attr = 1 # 90* at 1m
    #k_attr = 4.5 # 45* at 1.5m
    k_attr = 1
    F_attr_x = k_attr * (GOAL[0] - x)
    F_attr_y = k_attr * (GOAL[1] - y)

    # Repulsive force parameters
    # k_rep = 0.39 * 2 (90* at 1m, d0= 2.5)
    k_rep = 1.6875 # 45* at 1.5m
    d0 = 3  # influence distance (meters)
    dStop = 0.4 # emergency dist
    F_rep_x = 0.0
    F_rep_y = 0.0
    for obs in obstacles:
        # Each obstacle is assumed to have a 'centroid': [x, y, z]
        obs_x = obs.get("centroid", [0,0,0])[0]
        obs_y = obs.get("centroid", [0,0,0])[1]
        obs_size = obs.get("size", [0, 0, 0])

        obs_area = (obs_size[0] * obs_size[1] * obs_size[2]) if obs_size[0] and obs_size[1]  else 1.0

        dx = x - obs_x
        dy = y - obs_y
        dist = math.hypot(dx, dy)

        if 0 < dist < d0:
            rospy.loginfo("Obstacle at (%.2f, %.2f) with distance: %.2f and area %.2f", obs_x, obs_y, dist, obs_area)
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
    #rospy.loginfo("Attractive: (%.2f, %.2f), Repulsive: (%.2f, %.2f)", F_attr_x, F_attr_y, F_rep_x, F_rep_y)
    #rospy.loginfo("Total: (%.2f, %.2f), Desired heading: %.2f, Yaw: %.2f, Error: %.2f", F_total_x, F_total_y, desired_heading, yaw, heading_error)

    # Package APF forces details
    apf_forces = {
        "F_attr": [F_attr_x, F_attr_y],
        "F_rep": [F_rep_x, F_rep_y],
        "F_total": [F_total_x, F_total_y],
        "heading_error": heading_error
    }

    # Determine command based on distance to goal and heading error
    distance_to_goal = math.hypot(GOAL[0]-x, GOAL[1]-y)
    if distance_to_goal < 0.5:
        command = "stop"
    elif abs(heading_error) < 0.2:
        command = "forward"
    elif abs(heading_error) < (math.pi / 4):  # Heading error less than 45° in radians
        if heading_error > 0:
            command = "halfleft"
        else:
            command = "halfright"
    else:
        if heading_error > 0:
            command = "left"
        else:
            command = "right"
    processing_time = time.time() - start_time
    return (command, apf_forces, processing_time)

def main():
    rospy.init_node("apf_navigation_node", anonymous=True)
    rospy.Subscriber("/rtabmap/odom", Odometry, odom_callback)
    rospy.Subscriber("/nano/imu_corrected", Imu, imu_callback)
    rospy.Subscriber("/obstacle_info", String, obstacle_callback)

    command_pub = rospy.Publisher("/arduino_commands", String, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz loop rate

    while not rospy.is_shutdown():
        command, apf_forces, processing_time = compute_apf_command()
        command_pub.publish(command)
        rospy.loginfo("APF Command: %s", command)
        # Publish APF forces as JSON
        apf_forces_pub.publish(json.dumps(apf_forces))
        updater.update()  # Publishes diagnostics
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

