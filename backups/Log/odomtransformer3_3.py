#!/usr/bin/env python3
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg
import std_msgs.msg  # Import for yaw message
import tf.transformations as tft

class OdomTransformer:
    def __init__(self):
        rospy.init_node("odomtransformer3")

        # Parameters
        self.odom_input = rospy.get_param("~odom_input", "scanmatch_odom")
        self.tf_output = rospy.get_param("~tf_output", "base_link")
        self.yaw_input = rospy.get_param("~yaw_input", "/arduino/yaw")  # Updated to yaw topic

        # IMU Data (only yaw)
        self.imu_yaw = 0.0

        # Subscribers
        self.odom_sub = rospy.Subscriber(self.odom_input, nav_msgs.msg.Odometry, self.odom_callback)
        self.yaw_sub = rospy.Subscriber(self.yaw_input, std_msgs.msg.Float32, self.yaw_callback)

        # Transform Broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()

    def yaw_callback(self, yaw_msg):
        """Update yaw from Arduino."""
        self.imu_yaw = yaw_msg.data  # Directly set yaw

    def odom_callback(self, odom_msg):
        """Publish transform using odometry but override yaw with Arduino yaw."""
        pos = odom_msg.pose.pose.position

        # Override rotation using IMU yaw only
        quat = tft.quaternion_from_euler(0, 0, self.imu_yaw)

        # Broadcast transform
        self.tf_broadcaster.sendTransform(
            (pos.x, pos.y, pos.z),
            (quat[0], quat[1], quat[2], quat[3]),
            rospy.Time.now(),
            self.tf_output,
            odom_msg.header.frame_id
        )

if __name__ == "__main__":
    try:
        OdomTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
