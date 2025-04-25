#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg
import tf
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32  # Assuming yaw from Arduino is published as a Float32 topic

# Global variable to store yaw from Arduino
arduino_yaw = 0.0

def yaw_callback(msg):
    """Callback function to receive yaw angle from Arduino (in degrees or radians)."""
    global arduino_yaw
    arduino_yaw = msg.data  # Update global yaw variable

def odom_callback(data, args):
    """Callback function to process odometry data and publish the corrected transform."""
    global arduino_yaw
    bc = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = args[0]  # Parent frame (e.g., /map)
    t.child_frame_id = args[1]   # Child frame (e.g., /base_link)

    # Use original position from odometry message
    t.transform.translation = data.pose.pose.position

    # Convert Arduino yaw (assumed to be in degrees) to quaternion
    corrected_quaternion = quaternion_from_euler(0, 0, arduino_yaw)  # (roll, pitch, yaw)

    # Use corrected yaw, but keep original roll and pitch
    t.transform.rotation.x = corrected_quaternion[0]
    t.transform.rotation.y = corrected_quaternion[1]
    t.transform.rotation.z = corrected_quaternion[2]
    t.transform.rotation.w = corrected_quaternion[3]

    # Publish the new transform
    bc.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("odomtransformer3")

    # Get parameters
    odomInput = rospy.get_param("~odom_input")  # Expected odometry topic (e.g., /scanmatch_odom)
    tfOutput  = rospy.get_param("~tf_output")   # Expected output frame (e.g., /base_link)
    yaw_topic = rospy.get_param("~yaw_input", "/arduino/yaw")  # Default to /arduino/yaw

    # Subscribe to odometry and yaw topics
    rospy.Subscriber(odomInput, nav_msgs.msg.Odometry, odom_callback, [odomInput, tfOutput])
    rospy.Subscriber(yaw_topic, Float32, yaw_callback)  # Listening to Arduino yaw

    rospy.spin()
