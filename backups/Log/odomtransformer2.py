#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler

class OdomTransformer:
    def __init__(self):
        rospy.init_node('odomtransformer', anonymous=True)
        
        # Subscribing to Hector SLAM odometry and Arduino yaw data
        self.odom_sub = rospy.Subscriber('/scanmatch_odom', Odometry, self.odom_callback)
        self.yaw_sub = rospy.Subscriber('/arduino/yaw', Float32, self.yaw_callback)
        
        # TF broadcaster
        self.br = tf.TransformBroadcaster()
        
        # Initialize yaw from Arduino
        self.current_yaw = 0.0
        self.last_odom = None  # To store the latest odom message

        rospy.loginfo("Odom Transformer Node Initialized")
    
    def yaw_callback(self, msg):
        """ Callback to receive yaw data from the Arduino IMU. """
        self.current_yaw = msg.data  # Arduino yaw in radians

    def odom_callback(self, msg):
        """ Callback to receive Hector SLAM odometry and publish corrected transform. """
        if self.last_odom is None:
            self.last_odom = msg
            return
        
        # Extract position from odometry
        position = msg.pose.pose.position

        # Correct orientation using the Arduino yaw
        corrected_orientation = quaternion_from_euler(0, 0, self.current_yaw)
        
        # Broadcast the transform (base_link relative to odom)
        self.br.sendTransform(
            (position.x, position.y, 0.0),
            corrected_orientation,
            rospy.Time.now(),
            "base_link",
            "odom"
        )
        rospy.loginfo_throttle(5, f"Broadcasting transform: x={position.x}, y={position.y}, yaw={self.current_yaw}")

if __name__ == '__main__':
    try:
        OdomTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

