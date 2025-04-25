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

rospy.init_node("obstacle_clustering")
obstacle_pub = rospy.Publisher("/obstacle_info", String, queue_size=10, latch=True)
updater = Updater()
updater.setHardwareID("Obstacle Detection Node")

last_processing_time = 0.0
last_obstacle_count = 0
last_obstacle_data = None

def ros_to_pcl(msg):
    """Converts ROS PointCloud2 to PCL format, filtering out points above a given height."""
    point_list = []
    ceiling_threshold = 0.5  # Adjust this value as necessary
    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        # Filter out points above the threshold (assumed to be ceiling)
        if point[2] > ceiling_threshold:
            continue
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

    # Convert ROS point cloud to PCL cloud
    cloud = ros_to_pcl(msg)

    # ---------------------------
    # Downsampling (Voxel Grid Filter)
    # ---------------------------
    # Use different leaf sizes horizontally and vertically if desired.
    leaf_size_horizontal = 0.03  # for x,y details
    leaf_size_vertical = 0.12    # for z; less detail needed
    voxel = cloud.make_voxel_grid_filter()
    voxel.set_leaf_size(leaf_size_horizontal, leaf_size_horizontal, leaf_size_vertical)
    downsampled_cloud = voxel.filter()
    rospy.loginfo("Downsampled cloud has %d points", len(np.array(downsampled_cloud)))

    # ---------------------------
    # Outlier Removal (Statistical Filter)
    # ---------------------------
    sor = downsampled_cloud.make_statistical_outlier_filter()
    sor.set_mean_k(40)             # Higher number of neighbors for robust statistics
    sor.set_std_dev_mul_thresh(1.0)  # Lower threshold removes more outliers
    filtered_cloud = sor.filter()
    filtered_points = np.array(filtered_cloud)
    rospy.loginfo("Filtered cloud has %d points", filtered_points.shape[0])

    # ---------------------------
    # Iterative RANSAC for Wall Extraction
    # ---------------------------
    # Parameters for iterative segmentation:
    min_wall_inliers = 2250   # Minimum points to qualify as a wall plane
    c_threshold = 0.33        # For 15� tolerance (|c| < 0.26 means near vertical wall)
    ransac_threshold = 0.05  # Distance threshold for RANSAC
    wall_clouds = []          # List to store each detected wall cloud

    # Start with all filtered points
    remaining_cloud = filtered_cloud
    remaining_points = np.array(remaining_cloud)
    iteration = 0
    while remaining_points.shape[0] > min_wall_inliers:
        seg = remaining_cloud.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(ransac_threshold)
        indices, model = seg.segment()
        rospy.loginfo("Iteration %d: RANSAC found %d inliers", iteration, len(indices))
        if len(indices) < min_wall_inliers:
            rospy.loginfo("Iteration %d: Not enough inliers, stopping wall extraction.", iteration)
            break
        # Check orientation: model is [a, b, c, d]. For a vertical wall, |c| < c_threshold.
        if abs(model[2]) < c_threshold:
            current_wall = remaining_cloud.extract(indices, negative=False)
            wall_clouds.append(current_wall)
            rospy.loginfo("Iteration %d: Extracted a wall with %d points", iteration, len(np.array(current_wall)))
            # Remove these wall points from the remaining cloud
            remaining_cloud = remaining_cloud.extract(indices, negative=True)
            remaining_points = np.array(remaining_cloud)
        else:
            rospy.loginfo("Iteration %d: Plane did not meet wall orientation criteria.", iteration)
            break
        iteration += 1

    # ---------------------------
    # Use remaining cloud as non-wall points for obstacle clustering
    # ---------------------------
    non_wall_cloud = remaining_cloud
    non_wall_points = np.array(non_wall_cloud)
    rospy.loginfo("After wall extraction, non-wall cloud has %d points", non_wall_points.shape[0])

    # ---------------------------
    # Clustering on Non-Wall Points (Obstacles)
    # ---------------------------
    obstacle_list = []
    if non_wall_points.shape[0] > 0:
        tree = non_wall_cloud.make_kdtree()
        ec = non_wall_cloud.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.08)  # Tune based on environment: lower if obstacles merge
        ec.set_MinClusterSize(50)
        ec.set_MaxClusterSize(3000)
        ec.set_SearchMethod(tree)
        cluster_indices = ec.Extract()
        rospy.loginfo("Found %d clusters in non-wall cloud", len(cluster_indices))
        for indices in cluster_indices:
            points = np.array([non_wall_cloud[i] for i in indices])
            centroid = np.mean(points, axis=0)
            size = np.max(points, axis=0) - np.min(points, axis=0)
            # Orientation via PCA on XY plane
            points_2d = points[:, :2]
            cov_matrix = np.cov(points_2d.T)
            eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
            principal_vector = eigenvectors[:, np.argmax(eigenvalues)]
            orientation = np.arctan2(principal_vector[1], principal_vector[0])
            obstacle_list.append({
                "centroid": centroid.tolist(),
                "size": size.tolist(),
                "orientation": orientation,
                "point_count": len(points),
                "type": "obstacle"
            })
    else:
        rospy.logwarn("Non-wall cloud is empty; skipping clustering for obstacles.")

    # ---------------------------
    # Process each extracted wall cloud separately
    # ---------------------------
    for wall_cloud in wall_clouds:
        wall_points = np.array(wall_cloud)
        rospy.loginfo("Processing wall cloud with %d points", wall_points.shape[0])
        # Only process if wall has enough points; otherwise, it might be merged with obstacles.
        min_wall_points = 2250  # Tune this to avoid misclassifying moderately large obstacles as walls.
        if wall_points.shape[0] > min_wall_points:
            centroid = np.mean(wall_points, axis=0)
            size = np.max(wall_points, axis=0) - np.min(wall_points, axis=0)
            points_2d = wall_points[:, :2]
            cov_matrix = np.cov(points_2d.T)
            eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
            principal_vector = eigenvectors[:, np.argmax(eigenvalues)]
            orientation = np.arctan2(principal_vector[1], principal_vector[0])
            obstacle_list.append({
                "centroid": centroid.tolist(),
                "size": size.tolist(),
                "orientation": orientation,
                "point_count": int(wall_points.shape[0]),
                "type": "wall"
            })
        else:
            rospy.loginfo("Wall cloud too small (%d points); not processing as wall", wall_points.shape[0])

    # ---------------------------
    # Publish Final Obstacle Information
    # ---------------------------
    json_msg = json.dumps(obstacle_list)
    obstacle_pub.publish(json_msg)
    last_processing_time = time.time() - start_time
    last_obstacle_count = len(obstacle_list)
    updater.update()
    rospy.loginfo("Published %d obstacles in %.2f ms", last_obstacle_count, last_processing_time * 1000)

rospy.Subscriber("/rtabmap/cloud_obstacles", PointCloud2, cloud_callback)
rospy.spin()

