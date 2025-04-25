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

rospy.init_node('apf_navigation_node', anonymous=False)

# Diagnostics
updater = Updater()
updater.setHardwareID("APF Navigation Node")

def diagnostic_callback(stat):
    # Set the overall summary status and message directly on 'stat'
    stat.summary(DiagnosticStatusWrapper.OK, "Node running")
    stat.add("Processing Time (ms)", last_processing_time)
    stat.add("Heading Error", last_heading_error)
    return stat

updater.add("APF Navigation Diagnostics", diagnostic_callback)

# Global variables for sensor data
current_pose = None  # (x, y, yaw) from odometry
imu_yaw = None       # Yaw from IMU
obstacles = []       # List of obstacle dictionaries, each with key "centroid" ([x, y, z])
last_processing_time = 0.0  # Initialise processing time
last_heading_error = 0.0  # Initiali2e heading error

# Additional publisher for APF forces and performance metrics
perf_pub = rospy.Publisher("/navigation_perf", String, queue_size=10)


def closest_point_on_box(robot_x, robot_y, obs_x, obs_y, obs_size, obs_orientation):
    # Compute half-sizes of the box (assuming obs_size = [size_x, size_y, size_z])
    half_width = obs_size[0] / 2.0
    half_depth = obs_size[1] / 2.0

    # Compute robot's position relative to obstacle center
    dx = robot_x - obs_x
    dy = robot_y - obs_y

    # Rotate robot position into the obstacle's frame
    local_x = dx * math.cos(obs_orientation) + dy * math.sin(obs_orientation)
    local_y = -dx * math.sin(obs_orientation) + dy * math.cos(obs_orientation)

    # Clamp the local coordinates to the bounds of the box
    clamped_x = max(-half_width, min(local_x, half_width))
    clamped_y = max(-half_depth, min(local_y, half_depth))

    # Transform the clamped point back to world coordinates
    world_x = obs_x + clamped_x * math.cos(obs_orientation) - clamped_y * math.sin(obs_orientation)
    world_y = obs_y + clamped_x * math.sin(obs_orientation) + clamped_y * math.cos(obs_orientation)
    return world_x, world_y

def odom_callback(msg):
    global current_pose, GOAL
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    quaternion = [ori.x, ori.y, ori.z, ori.w]
    (_, _, yaw) = euler_from_quaternion(quaternion)
    current_pose = (pos.x, pos.y, yaw)
    
    # Update GOAL: 3m ahead in the direction the person is facing
    GOAL = (pos.x + 3.0 * math.cos(yaw), pos.y + 3.0 * math.sin(yaw))
    
    rospy.logdebug("Odom updated: x=%.2f, y=%.2f, yaw=%.2f, GOAL updated: (%.2f, %.2f)", 
                   pos.x, pos.y, yaw, GOAL[0], GOAL[1])

def obstacle_callback(msg):
    global obstacles
    try:
        data = json.loads(msg.data)
        # In production, publish obstacles as JSON strings
        # If the data is already a list, use it; otherwise, try to get the "obstacles" key.
        obstacles = data if isinstance(data, list) else data.get("obstacles", [])
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
        return "stop"
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

    dStop = 0.4 # emergency dist
    F_rep_x = 0.0
    F_rep_y = 0.0

    mindist = float('inf')

    for obs in obstacles:
        # Retrieve obstacle properties
        obs_centroid = obs.get("centroid", [0, 0, 0])
        obs_x = obs_centroid[0]
        obs_y = obs_centroid[1]
        obs_size = obs.get("size", [1.0, 1.0, 1.0])
        obs_orientation = obs.get("orientation", 0.0)
        point_count = obs.get("point_count", 550)

        # Compute dynamic influence distance d0 based on point_count:
        # If point_count < 100, d0 = 2.0; if >= 1000, d0 = 4.0; else interpolate.
        if point_count < 100:
            d0_obs = 2.0
        elif point_count >= 1000:
            d0_obs = 4.0
        else:
            d0_obs = 2.0 + (point_count - 100) / 450.0

        # Compute the closest point on the obstacle's bounding box to the robot
        closest_x, closest_y = closest_point_on_box(x, y, obs_x, obs_y, obs_size, obs_orientation)

        # Compute the distance from the robot to that closest point
        dx = x - closest_x
        dy = y - closest_y
        dist = math.hypot(dx, dy)

        if 0 < dist < mindist:
            mindist = dist

        if 0 < dist < d0_obs:
            rospy.loginfo("Obstacle: centroid=(%.2f, %.2f), point_count=%d, d0=%.2f, closest_distance=%.2f",
                          obs_x, obs_y, point_count, d0_obs, dist)
            rep_mag = k_rep * (1.0 / dist - 1.0 / d0_obs) / (dist ** 2)
            F_rep_x += rep_mag * (dx / dist)
            F_rep_y += rep_mag * (dy / dist)

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
    if mindist < dStop:
        command = "recovery"
    else:
        if distance_to_goal < 0.5 :
            command = "stop"
        elif abs(heading_error) < 0.175:
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
    last_processing_time = processing_time

    # ? Publish Combined Navigation Performance Metrics
    navigation_perf = {
        "processing_time": processing_time * 1000,  # Convert to ms
        "heading_error": heading_error,
        "APF_total_force": [F_total_x, F_total_y],
        "APF_attractive_force": [F_attr_x, F_attr_y],
        "APF_repulsive_force": [F_rep_x, F_rep_y],
        "command_sent": command
    }
    perf_pub.publish(json.dumps(navigation_perf))

    return command

def main():
    rospy.Subscriber("/rtabmap/odom", Odometry, odom_callback)
    rospy.Subscriber("/obstacle_info", String, obstacle_callback)

    command_pub = rospy.Publisher("/arduino_commands", String, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz loop rate

    while not rospy.is_shutdown():
        command = compute_apf_command()
        command_pub.publish(command)
        rospy.loginfo("APF Command: %s", command)
        updater.update()  # Publishes diagnostics
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

