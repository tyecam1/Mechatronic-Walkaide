#!/usr/bin/env python3

import rospy
import pcl
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import json  # JSON serialization

# Initialize ROS Node
rospy.init_node("obstacle_clustering")

# Create a publisher for obstacle data
obstacle_pub = rospy.Publisher("/obstacle_info", String, queue_size=10, latch=True)

# Processing Rate Control
PROCESS_RATE = 5  # Hz
last_processed_time = rospy.Time.now()

def ros_to_pcl(msg):
    """ Converts ROS PointCloud2 to PCL format """
    point_list = []
    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        point_list.append([point[0], point[1], point[2]])
    pcl_cloud = pcl.PointCloud()
    pcl_cloud.from_list(point_list)
    return pcl_cloud

def cloud_callback(msg):
    """ Processes point clouds, clusters obstacles, and publishes their position & size """
    global last_processed_time

    # Rate limiting
    current_time = rospy.Time.now()
    if (current_time - last_processed_time).to_sec() < 1.0 / PROCESS_RATE:
        return
    last_processed_time = current_time

    cloud = ros_to_pcl(msg)  # Convert ROS PointCloud2 message to PCL format

    tree = cloud.make_kdtree()

    # Euclidean Clustering
    ec = cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.2)  # Capture smaller objects
    ec.set_MinClusterSize(15)  # Capture smaller obstacles
    ec.set_MaxClusterSize(500)  # Allow large obstacles
    ec.set_SearchMethod(tree)

    cluster_indices = ec.Extract()
    obstacle_data = []

    for indices in cluster_indices:
        points = np.array([cloud[i] for i in indices])

        # Compute centroid
        centroid = np.mean(points, axis=0)

        # Compute bounding box size with robust percentiles
        min_bounds = np.percentile(points, 5, axis=0)  # Ignore bottom 5% of outliers
        max_bounds = np.percentile(points, 95, axis=0)  # Ignore top 5% of outliers
        size = max_bounds - min_bounds

        # Compute approximate area for filtering
        area = size[0] * size[1]

        # Compute cluster density (points per cubic meter)
        volume = size[0] * size[1] * size[2]
        density = len(points) / (volume + 1e-6)  # Avoid division by zero

        # Filter out unrealistic obstacles
        if area < 0.05 or area > 8.0:  # Allow smaller obstacles
            continue
        if density < 3.0:  # Ensure reasonable density
            continue

        # Add to obstacle list
        obstacle_data.append({"centroid": centroid.tolist(), "size": size.tolist()})

    # Convert to JSON and publish
    json_msg = json.dumps(obstacle_data)
    obstacle_pub.publish(json_msg)

    rospy.loginfo("Published %d obstacles.", len(obstacle_data))

# Subscribe to RTAB-Map's filtered obstacle cloud
rospy.Subscriber("/rtabmap/cloud_obstacles", PointCloud2, cloud_callback)

rospy.spin()

