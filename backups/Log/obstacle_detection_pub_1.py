#!/usr/bin/env python3

import rospy
import pcl
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String  # We use JSON-like string messages
import json  # Import json module

# Initialize ROS Node
rospy.init_node("obstacle_clustering")

# Create a publisher for obstacle data (using a latched publisher is often helpful)
obstacle_pub = rospy.Publisher("/obstacle_info", String, queue_size=10, latch=True)

def ros_to_pcl(msg):
    """ Converts ROS PointCloud2 to PCL format """
    point_list = []
    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        point_list.append([point[0], point[1], point[2]])
    pcl_cloud = pcl.PointCloud()
    pcl_cloud.from_list(point_list)
    return pcl_cloud

def cloud_callback(msg):
    """ Processes incoming point clouds, clusters obstacles, and publishes their position & size """
    cloud = ros_to_pcl(msg)  # Convert ROS PointCloud2 message to PCL format
    tree = cloud.make_kdtree()

    # Euclidean Clustering for obstacle segmentation
    ec = cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.3)  # Clusters obstacles within 30 cm
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(10000)
    ec.set_SearchMethod(tree)

    cluster_indices = ec.Extract()
    obstacle_data = []

    for indices in cluster_indices:
        points = np.array([cloud[i] for i in indices])

        # Calculate obstacle centroid (mean position)
        centroid = np.mean(points, axis=0)

        # Calculate obstacle size (bounding box: max - min)
        size = np.max(points, axis=0) - np.min(points, axis=0)

        # Add to obstacle list
        obstacle_data.append({"centroid": centroid.tolist(), "size": size.tolist()})

    # Convert to JSON format and publish
    json_msg = json.dumps(obstacle_data)
    obstacle_pub.publish(json_msg)

    rospy.loginfo("Published %d obstacles.", len(obstacle_data))

# Subscribe to RTAB-Map's filtered obstacle cloud
rospy.Subscriber("/rtabmap/cloud_obstacles", PointCloud2, cloud_callback)

rospy.spin()

