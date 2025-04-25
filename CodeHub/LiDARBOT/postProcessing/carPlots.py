#!/usr/bin/env python3
import os
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import math
import cv2 
from sensor_msgs.msg import LaserScan
from pathlib import Path 

# Folder containing the bag files
BAG_DIR = "CarBagFiles"

# Topics
POSE_TOPIC = "/slam_out_pose"
SCAN_TOPIC = "/scan"

def quaternion_to_yaw(q):
    """Convert quaternion (x,y,z,w) to yaw (radians)."""
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

def load_robot_poses(bag):
    """
    Returns a sorted list of (timestamp, x, y, yaw) from /slam_out_pose.
    """
    poses = []
    for _, msg, t in bag.read_messages(topics=[POSE_TOPIC]):
        x = msg.pose.position.x
        y = msg.pose.position.y
        # Convert quaternion to yaw
        w = msg.pose.orientation.w
        x_q = msg.pose.orientation.x
        y_q = msg.pose.orientation.y
        z_q = msg.pose.orientation.z
        yaw = quaternion_to_yaw(type("Q", (), dict(x=x_q, y=y_q, z=z_q, w=w)))
        poses.append((t.to_sec(), x, y, yaw))
    return sorted(poses, key=lambda p: p[0])

def load_scans(bag):
    """
    Returns a list of (timestamp, LaserScan) from /scan.
    """
    scans = []
    for _, msg, t in bag.read_messages(topics=[SCAN_TOPIC]):
        scans.append((t.to_sec(), msg))
    return scans

def find_nearest_pose(timestamp, poses):
    """
    Finds the pose (x, y, yaw) whose timestamp is closest to 'timestamp'.
    """
    if not poses:
        return None
    nearest = min(poses, key=lambda p: abs(p[0] - timestamp))
    return (nearest[1], nearest[2], nearest[3])  # (x, y, yaw)

def bresenham(x0, y0, x1, y1):
    """
    Bresenham's line algorithm for grid cells on a line between (x0, y0) and (x1, y1).
    """
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1

    if dx > dy:
        err = dx / 2.0
        while x != x1:
            cells.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            cells.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    cells.append((x1, y1))
    return cells

def build_occupancy_grid(poses, scans, resolution=0.05, margin=1.0):
    """
    Builds a naive occupancy grid using Bresenham line tracing:
      - -1 => unknown
      -  0 => free
      - 100 => occupied
    """
    # Gather all x,y for map bounds
    xs, ys = [], []
    for _, x, y, _ in poses:
        xs.append(x)
        ys.append(y)

    # Include laser endpoints
    for stamp, scan in scans:
        robot_pose = find_nearest_pose(stamp, poses)
        if robot_pose is None:
            continue
        rx, ry, ryaw = robot_pose
        angle = scan.angle_min
        for r in scan.ranges:
            if r < scan.range_min or r > scan.range_max:
                angle += scan.angle_increment
                continue
            ex = rx + r * math.cos(ryaw + angle)
            ey = ry + r * math.sin(ryaw + angle)
            xs.append(ex)
            ys.append(ey)
            angle += scan.angle_increment

    min_x = min(xs) - margin
    max_x = max(xs) + margin
    min_y = min(ys) - margin
    max_y = max(ys) + margin

    width = int(math.ceil((max_x - min_x) / resolution))
    height = int(math.ceil((max_y - min_y) / resolution))

    # Initialize grid
    grid = -1 * np.ones((height, width), dtype=np.int8)

    # Populate the grid
    for stamp, scan in scans:
        robot_pose = find_nearest_pose(stamp, poses)
        if robot_pose is None:
            continue
        rx, ry, ryaw = robot_pose

        robot_i = int((ry - min_y) / resolution)
        robot_j = int((rx - min_x) / resolution)

        angle = scan.angle_min
        for r in scan.ranges:
            if r < scan.range_min or r > scan.range_max:
                angle += scan.angle_increment
                continue
            ex = rx + r * math.cos(ryaw + angle)
            ey = ry + r * math.sin(ryaw + angle)

            end_i = int((ey - min_y) / resolution)
            end_j = int((ex - min_x) / resolution)

            # Mark free along the ray
            cells = bresenham(robot_i, robot_j, end_i, end_j)
            for (ci, cj) in cells[:-1]:
                if 0 <= ci < height and 0 <= cj < width:
                    grid[ci, cj] = 0
            # Mark endpoint as occupied
            if 0 <= end_i < height and 0 <= end_j < width:
                grid[end_i, end_j] = 100

            angle += scan.angle_increment

    return grid, min_x, min_y, resolution

def remove_small_clusters(grid, min_size=10):
    """
    Removes very small clusters of occupied cells (size < min_size).
    This helps clear random specks without merging or distorting bigger objects.
    """
    # connectedComponents on the "occupied" mask
    occ_mask = (grid == 100).astype(np.uint8)
    num_labels, labels = cv2.connectedComponents(occ_mask, connectivity=8)

    for label_val in range(1, num_labels):
        # Count how many pixels belong to this label
        size = np.sum(labels == label_val)
        if size < min_size:
            # Mark them as free
            grid[labels == label_val] = 0

    return grid

def grid_to_display(grid):
    """
    Convert occupancy grid to a grayscale image:
      - -1 => 0.5 (gray for unknown)
      -  0 => 1.0 (white for free)
      - 100 => 0.0 (black for occupied)
    """
    disp = grid.astype(float)
    disp[disp == -1] = 0.5
    disp[disp == 0]  = 1.0
    disp[disp == 100] = 0.0
    # Make it 3-channel so we can use imshow
    disp_3c = np.repeat(disp[:, :, np.newaxis], 3, axis=2)
    return disp_3c

def plot_occupancy_grid(grid, min_x, min_y, resolution, poses, title="Occupancy Grid Map", save_path=None):
    """
    Plots the occupancy grid in grayscale, with the robot path overlaid.
    Optionally saves the figure as a high-res PNG if save_path is provided.
    """
    h, w = grid.shape
    extent = [min_x, min_x + w*resolution, min_y, min_y + h*resolution]

    # Convert to display image
    disp_img = grid_to_display(grid)

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.imshow(disp_img, origin='lower', extent=extent)

    # Overlay path
    if poses:
        path = np.array([[p[1], p[2]] for p in poses])
        ax.plot(path[:, 0], path[:, 1], 'r-', linewidth=2, label="Robot Path")
        ax.scatter(path[0, 0], path[0, 1], color='green', s=50, label="Start")
        ax.scatter(path[-1, 0], path[-1, 1], color='blue', s=50, label="End")

    ax.set_title(title)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend()
    ax.grid(True)
    ax.set_aspect('equal')
    plt.tight_layout()

    # Save if path provided
    if save_path:
        print(f"Saving figure to: {save_path}")
        fig.savefig(save_path, dpi=300, bbox_inches='tight')

    plt.show()


def process_bag_file(bag_path):
    print(f"Processing {bag_path}...")
    with rosbag.Bag(bag_path, 'r') as bag:
        poses = load_robot_poses(bag)
        scans = load_scans(bag)
        if not poses or not scans:
            print("Insufficient data in", bag_path)
            return

        # Build raw occupancy grid
        grid, min_x, min_y, resolution = build_occupancy_grid(poses, scans)

        # Remove small clusters (e.g. <5 occupied cells)
        grid_cleaned = remove_small_clusters(grid, min_size=5)

        # Plot final map
        # Build output filename (PNG with same name as bag)
        output_name = os.path.splitext(os.path.basename(bag_path))[0] + ".png"
        output_path = os.path.join("ExportedPlots", output_name)

        # Ensure directory exists
        os.makedirs("ExportedPlots", exist_ok=True)

        # Plot and save
        plot_occupancy_grid(
            grid_cleaned,
            min_x, min_y,
            resolution,
            poses,
            title=os.path.basename(bag_path),
            save_path=output_path
        )


def main():
    if not os.path.isdir(BAG_DIR):
        print(f"Directory '{BAG_DIR}' not found.")
        return

    bag_files = sorted([f for f in os.listdir(BAG_DIR) if f.endswith(".bag")])
    if not bag_files:
        print("No .bag files found in", BAG_DIR)
        return

    for bag_filename in bag_files:
        bag_path = os.path.join(BAG_DIR, bag_filename)
        process_bag_file(bag_path)

if __name__ == "__main__":
    main()
