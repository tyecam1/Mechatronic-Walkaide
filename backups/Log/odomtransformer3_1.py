#!/usr/bin/env python3
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg
import tf.transformations as tft

class OdomTransformer:
    def __init__(self):
        rospy.init_node("odomtransformer3")

        # Parameters
        self.odom_input = rospy.get_param("~odom_input", "scanmatch_odom")
        self.tf_output = rospy.get_param("~tf_output", "base_link")
        self.imu_input = rospy.get_param("~imu_input", "/arduino/imu")

        # IMU Data
        self.imu_yaw = 0.0
        self.imu_ang_vel_z = 0.0

        # Subscribers
        self.odom_sub = rospy.Subscriber(self.odom_input, nav_msgs.msg.Odometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber(self.imu_input, sensor_msgs.msg.Imu, self.imu_callback)

        # Transform Broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()

    def imu_callback(self, imu_msg):
        """Extract yaw and angular velocity from IMU data."""
        quat = (imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w)
        _, _, self.imu_yaw = tft.euler_from_quaternion(quat)  # Extract yaw
        self.imu_ang_vel_z = imu_msg.angular_velocity.z  # Extract angular velocity

    def odom_callback(self, odom_msg):
        """Publish transform using odometry but override yaw with IMU data."""
        pos = odom_msg.pose.pose.position

        # Override rotation using IMU yaw
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
    OdomTransformer()
    rospy.spin()
