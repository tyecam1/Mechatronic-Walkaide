#!/usr/bin/env python3

import rospy
import pcl
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import json
import time
from diagnostic_updater import Updater, DiagnosticStatusWrapper

# Initialize ROS Node
rospy.init_node("obstacle_clustering")

# Use a latched publisher so subscribers always get the last published obstacle data
obstacle_pub = rospy.Publisher("/obstacle_info", String, queue_size=10, latch=True)

# Initialize diagnostics updater
updater = Updater()
updater.setHardwareID("Obstacle Detection Node")

# Processing time tracking and obstacle state storage
last_processing_time = 0.0
last_obstacle_count = 0
last_obstacle_data = None  # Stores the last published JSON string


def ros_to_pcl(msg):
    """Converts ROS PointCloud2 to PCL format."""
    point_list = []
    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        point_list.append([point[0], point[1], point[2]])
    pcl_cloud = pcl.PointCloud()
    pcl_cloud.from_list(point_list)
    return pcl_cloud


def diagnostic_callback(stat):
    stat.summary(DiagnosticStatusWrapper.OK, "Processing normally")
    stat.add("Processing Time (ms)", last_processing_time * 1000)
    stat.add("Obstacle Count", last_obstacle_count)
    return stat


updater.add("Obstacle Detection Status", diagnostic_callback)


def cloud_callback(msg):
    global last_processing_time, last_obstacle_count, last_obstacle_data
    start_time = time.time()

    # Convert the incoming point cloud
    cloud = ros_to_pcl(msg)

    # ----- Downsampling (VoxelGrid Filter) -----
    voxel = cloud.make_voxel_grid_filter()
    leaf_size = 0.05  # Reduced leaf size for more detail
    voxel.set_leaf_size(leaf_size, leaf_size, leaf_size)
    downsampled_cloud = voxel.filter()

    # ----- Outlier Removal (Statistical Filter) -----
    sor = downsampled_cloud.make_statistical_outlier_filter()
    sor.set_mean_k(30)
    sor.set_std_dev_mul_thresh(1.5)
    filtered_cloud = sor.filter()

    # Build a kd-tree for clustering
    tree = filtered_cloud.make_kdtree()

    # ----- Euclidean Clustering -----
    ec = filtered_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.14)  # Tighter tolerance to keep clusters distinct
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(2000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    obstacle_list = []
    for indices in cluster_indices:
        points = np.array([filtered_cloud[i] for i in indices])
        centroid = np.mean(points, axis=0)
        size = np.max(points, axis=0) - np.min(points, axis=0)

        # ----- Compute Orientation via PCA on the Horizontal Plane -----
        points_2d = points[:, :2]
        cov_matrix = np.cov(points_2d.T)
        eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
        principal_vector = eigenvectors[:, np.argmax(eigenvalues)]
        orientation = np.arctan2(principal_vector[1], principal_vector[0])
        # ---------------------------------------------------------------

        obstacle_list.append({
            "centroid": centroid.tolist(),
            "size": size.tolist(),
            "orientation": orientation,  # in radians
            "point_count": len(points)
        })

    # Convert obstacle data to JSON
    obstacle_json = json.dumps(obstacle_list)

    # Publish only if the obstacle data has significantly changed
    if last_obstacle_data is None or obstacle_json != last_obstacle_data:
        obstacle_pub.publish(obstacle_json)
        last_obstacle_data = obstacle_json

    last_processing_time = time.time() - start_time
    last_obstacle_count = len(obstacle_list)
    updater.update()
    rospy.loginfo("Published %d obstacles in %.2f ms", last_obstacle_count, last_processing_time * 1000)


# Subscribe to the filtered point cloud topic
rospy.Subscriber("/rtabmap/cloud_obstacles", PointCloud2, cloud_callback)

rospy.spin()
