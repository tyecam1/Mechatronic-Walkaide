#!/usr/bin/env python3
import os
import math
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.cm as cm
from matplotlib.animation import FuncAnimation
import re
import ast, struct
import plotly.graph_objects as go
from bagpy import bagreader
import glob
import imageio

# REMAPS
COMMAND_MAP = {"left": -90, "halfleft": -45, "right": 90, "halfright":45, "forward": 0, "recovery": 180}
NAME_MAP = {"oscar": "Person 1 ", "sam": "Person 2 ", "dylan": "Person 3 ", "trystan": "Person 4 "}
DEVICE_MAP = {"pulley": "Pulley Device ", "clock": "Clock Device "}
ATTEMPT_MAP = {"1": "Attempt 1:", "2": "Attempt 2:", "3": "Attempt 3:"}

# -------------------------------------------------------------------
# Helper Functions
# -------------------------------------------------------------------
def write_csvs_from_bag(bag_file, output_folder):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    print(f"[INFO] Converting {bag_file} to CSVs in {output_folder}...")
    try:
        b = bagreader(bag_file)
    except Exception as e:
        print(f"Error opening bag file {bag_file}: {e}")
        return
    topics = {
        "/rtabmap/odom": "odom_data.csv",
        "/nano/imu_corrected": "imu_data.csv",
        "/obstacle_info": "obstacle_info.csv",
        "/arduino_commands": "arduino_commands.csv",
        "/navigation_perf": "navigation_perf.csv",
        "/diagnostics": "diagnostics.csv",
        "/tf": "tf.csv",
        "/tf_static": "tf_static.csv",
        "/rtabmap/cloud_obstacles": "cloud_obstacles.csv",
        "/rtabmap/cloud_map": "cloud_map.csv",
        "/rtabmap/mapData": "mapData.csv",
        "/rtabmap/localization_pose": "localization_pose.csv",
        "/camera/color/camera_info": "camera_info.csv",
        "/rtabmap/info": "rtabmap_info.csv",
        "/system_diagnostics": "system_diagnostics.csv"
    }
    for topic, csv_name in topics.items():
        try:
            csv_path = b.message_by_topic(topic)
            dest_path = os.path.join(output_folder, csv_name)
            os.replace(csv_path, dest_path)
            print(f"Saved topic {topic} as {dest_path}")
        except Exception as e:
            print(f"Topic {topic} is not available or extraction failed: {e}")

def sanitize_filename(name):
    # Remove spaces and invalid Windows filename characters (like : * ? " < > |)
    # This regex keeps only alphanumeric characters and underscores.
    return re.sub(r'[^A-Za-z0-9_]', '', name)

def get_report_title(filename):
    lower_filename = filename.lower()
    name_part = ""
    device_part = ""
    attempt_part = ""
    for key, mapping in NAME_MAP.items():
        if key in lower_filename:
            name_part = mapping
            break
    for key, mapping in DEVICE_MAP.items():
        if key in lower_filename:
            device_part = mapping
            break
    for key, mapping in ATTEMPT_MAP.items():
        if key in lower_filename:
            attempt_part = mapping
            break
    if name_part or device_part or attempt_part:
        return name_part + device_part + attempt_part
    else:
        return "Project Report"

def convert_time(df, column):
    if column in df.columns:
        try:
            df[column] = pd.to_numeric(df[column], errors='coerce')
            if not df[column].empty:
                start_time = df[column].iloc[0]
                df[column] = df[column] - start_time
        except Exception as e:
            print("Error converting time:", e)
    return df

def merge_data(data):
    odom_df = data.get("odom_data.csv", pd.DataFrame())
    if not odom_df.empty:
        odom_df = convert_time(odom_df, "Time")
        odom_df = odom_df.rename(columns={
            "pose.pose.position.x": "x",
            "pose.pose.position.y": "y"
        })
    return odom_df

# -------------------------------------------------------------------
# Helper Functions for Obstacle Extraction
# -------------------------------------------------------------------
def load_data(csv_folder):
    filenames = ["navigation_perf.csv", "obstacle_info.csv", "odom_data.csv", "imu_data.csv",
                 "arduino_commands.csv", "system_diagnostics.csv", "rtabmap_info.csv",
                 "cloud_map.csv", "cloud_obstacles.csv"]
    data = {}
    for f in filenames:
        path = os.path.join(csv_folder, f)
        try:
            df = pd.read_csv(path, on_bad_lines="skip", low_memory=False)
            df = convert_time(df, "Time")
            data[f] = df
        except Exception as e:
            print(f"Error loading {f}: {e}")
            data[f] = pd.DataFrame()
    return data

def parse_obstacles(s):
    if not isinstance(s, str):
        return []
    try:
        arr = json.loads(s)
        out = []
        for obs in arr:
            c = obs.get("centroid", [np.nan, np.nan, np.nan])
            sz = obs.get("size", [0, 0, 0])
            out.append({
                "obs_x": c[0],
                "obs_y": c[1],
                "obs_z": c[2],
                "size_x": sz[0],
                "size_y": sz[1],
                "size_z": sz[2],
                "orientation": obs.get("orientation", 0.0),
                "point_count": obs.get("point_count", 0),
                "type": obs.get("type","obstacle"),
                "time_s": np.nan
            })
        return out
    except Exception as e:
        print("Error parsing obstacles:", e)
        return []

def extract_obstacles(data):
    df_obs = data.get("obstacle_info.csv", pd.DataFrame()).copy()
    obs_rows = []
    if not df_obs.empty and "data" in df_obs.columns:
        for _, row in df_obs.iterrows():
            t_s = row.get("Time", np.nan)
            try:
                obs_list = parse_obstacles(row["data"])
                for obs in obs_list:
                    obs["time_s"] = t_s
                    obs_rows.append(obs)
            except Exception as e:
                print("Error parsing obstacles:", e)
    if obs_rows:
        return pd.DataFrame(obs_rows).sort_values("time_s").reset_index(drop=True)
    else:
        return pd.DataFrame()

def get_rotated_rect_coords(cx, cy, width, height, angle):
    hw, hh = width / 2, height / 2
    corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
    out = []
    for sx, sy in corners:
        rx = sx * math.cos(angle) - sy * math.sin(angle) + cx
        ry = sx * math.sin(angle) + sy * math.cos(angle) + cy
        out.append((rx, ry))
    return out

# -------------------------------------------------------------------
# Compute distance from a point to a polygon (list of (x,y))
# -------------------------------------------------------------------
def point_to_polygon_distance(pt, poly):
    px, py = pt
    min_dist = float('inf')
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i+1) % len(poly)]
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0 and dy == 0:
            dist = math.hypot(px - x1, py - y1)
        else:
            t = ((px - x1) * dx + (py - y1) * dy) / (dx*dx + dy*dy)
            t = max(0, min(1, t))
            proj_x = x1 + t * dx
            proj_y = y1 + t * dy
            dist = math.hypot(px - proj_x, py - proj_y)
        if dist < min_dist:
            min_dist = dist
    return min_dist

# -------------------------------------------------------------------
# Helper: Extract heading error from navigation_perf.csv
# -------------------------------------------------------------------
def extract_heading_error(df_nav):
    he = []
    for i, row in df_nav.iterrows():
        try:
            d = json.loads(row["data"])
            he.append(d.get("heading_error", np.nan))
        except Exception as e:
            he.append(np.nan)
    df_nav["heading_error"] = he
    return df_nav

def extract_processing_time(df_nav):
    pt = []
    for i, row in df_nav.iterrows():
        try:
            d = json.loads(row["data"])
            pt.append(d.get("processing_time", np.nan))
        except Exception as e:
            pt.append(np.nan)
    df_nav["processing_time"] = pt
    return df_nav

def decode_pointcloud2_csv(row):
    try:
        point_step = int(row["point_step"])
        width = int(row["width"])
        height = int(row["height"])
        is_bigend = bool(row["is_bigendian"])
        bin_data_str = row["data"]
        bin_data = ast.literal_eval(bin_data_str)
        n_points = width * height
        fmt = '<f' if not is_bigend else '>f'
        px, py, pz = [], [], []
        for i in range(n_points):
            start = i * point_step
            px.append(struct.unpack_from(fmt, bin_data, start)[0])
            py.append(struct.unpack_from(fmt, bin_data, start + 4)[0])
            pz.append(struct.unpack_from(fmt, bin_data, start + 8)[0])
        return pd.DataFrame({"mx": px, "my": py, "mz": pz})
    except Exception as e:
        print("Error in decode_pointcloud2_csv:", e)
        return pd.DataFrame()

# Helper functions to compute lost/regained times from odometry.
def segment_indices(df):
    valid = (~((df["x"] == 0) & (df["y"] == 0))).values
    segments = []
    current = []
    for i, v in enumerate(valid):
        if v:
            current.append(i)
        else:
            if current:
                segments.append(current)
                current = []
    if current:
        segments.append(current)
    return segments

def lost_regained_times(df):
    segments = segment_indices(df)
    lost = []
    regained = []
    if len(segments) > 1:
        for i in range(len(segments) - 1):
            lost.append(df["time_s"].iloc[segments[i][-1]])
            regained.append(df["time_s"].iloc[segments[i + 1][0]])
    return lost, regained


##############################################
# Helper Function to Create the Animation DataFrame
##############################################
def create_anim_df(df_path, nav_df):
    """
    Create an animation DataFrame (anim_df) that uses the same time base as the other plots,
    by merging the odometry/path data (df_path) with the APF forces extracted from nav_df.

    Expected:
      - df_path: DataFrame from odometry data, containing at least 'Time', 'x', 'y'.
      - nav_df: DataFrame from navigation_perf.csv, with columns "Time" and "data", where
                the "data" column is a JSON string containing keys:
                "APF_total_force", "APF_attractive_force", "APF_repulsive_force".

    The resulting anim_df will have:
         "Time", "x", "y", "APF_total_force", "APF_attractive_force", "APF_repulsive_force"
    """
    forces_list = []
    for idx, row in nav_df.iterrows():
        try:
            data_json = json.loads(row["data"])
        except Exception as e:
            data_json = {}
        forces_list.append({
            "Time": row["Time"],
            "APF_total_force": data_json.get("APF_total_force", [0, 0]),
            "APF_attractive_force": data_json.get("APF_attractive_force", [0, 0]),
            "APF_repulsive_force": data_json.get("APF_repulsive_force", [0, 0])
        })
    forces_df = pd.DataFrame(forces_list)
    # Ensure both DataFrames are sorted by Time before merging.
    df_path_sorted = df_path.sort_values("Time")
    forces_df_sorted = forces_df.sort_values("Time")
    anim_df = pd.merge_asof(df_path_sorted, forces_df_sorted, on="Time")
    return anim_df

# -------------------------------------------------------------------
# 3D Obstacle Plot Helper
# -------------------------------------------------------------------
def oriented_box_3d(obs_x, obs_y, obs_z, size_x, size_y, size_z, orientation_rads):
    theta = orientation_rads
    half_x, half_y, half_z = size_x / 2, size_y / 2, size_z / 2
    corners_local = np.array([
        [-half_x, -half_y, -half_z],
        [ half_x, -half_y, -half_z],
        [ half_x,  half_y, -half_z],
        [-half_x,  half_y, -half_z],
        [-half_x, -half_y,  half_z],
        [ half_x, -half_y,  half_z],
        [ half_x,  half_y,  half_z],
        [-half_x,  half_y,  half_z],
    ])
    cosT, sinT = math.cos(theta), math.sin(theta)
    Rz = np.array([[cosT, -sinT, 0],
                   [sinT,  cosT, 0],
                   [0,     0,    1]])
    corners_rot = corners_local.dot(Rz.T)
    corners_rot[:, 0] += obs_x
    corners_rot[:, 1] += obs_y
    corners_rot[:, 2] += obs_z
    faces = np.array([[0, 1, 2], [0, 2, 3],
                      [4, 5, 6], [4, 6, 7],
                      [0, 1, 5], [0, 5, 4],
                      [1, 2, 6], [1, 6, 5],
                      [2, 3, 7], [2, 7, 6],
                      [3, 0, 4], [3, 4, 7]])
    return go.Mesh3d(
        x=corners_rot[:, 0],
        y=corners_rot[:, 1],
        z=corners_rot[:, 2],
        i=faces[:, 0], j=faces[:, 1], k=faces[:, 2],
        color="rgba(255,0,0,0.5)",
        opacity=0.5,
        name="ObstacleBox"
    )

# -------------------------------------------------------------------
# Interactive Birdseye Plot Function with Global Map Editing
# Returns (updated_obstacles_dataframe, final_goal_center)
# After manual adjustment, the final plot is saved as PNG in results_folder.
# -------------------------------------------------------------------
def interactive_birdseye_plot(df_path, obstacles_df, title, results_folder):
    MIN_MOVE_THRESHOLD = 0.1
    GOAL_RADIUS = 3

    df_path = df_path[(df_path["x"].abs() > MIN_MOVE_THRESHOLD) | (df_path["y"].abs() > MIN_MOVE_THRESHOLD)]
    global_rotation = 0.0
    global_dx = 0.0
    global_dy = 0.0
    orig_path = df_path.copy()
    global_obstacles = obstacles_df.to_dict(orient="records")

    all_orig_pts = [(x, y) for x, y in zip(orig_path["x"], orig_path["y"])]
    for obs in global_obstacles:
        all_orig_pts.append((obs["obs_x"], obs["obs_y"]))
    if all_orig_pts:
        xs, ys = zip(*all_orig_pts)
        orig_x_min, orig_x_max = min(xs), max(xs)
        orig_y_min, orig_y_max = min(ys), max(ys)
        orig_corners = [(orig_x_min, orig_y_min), (orig_x_min, orig_y_max),
                        (orig_x_max, orig_y_min), (orig_x_max, orig_y_max)]
        orig_goal_center = max(orig_corners, key=lambda pt: math.hypot(pt[0], pt[1]))
    else:
        orig_goal_center = (0, 0)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title(title)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")

    line, = ax.plot([], [], "b-", label="Path")
    obstacle_patches = []
    goal_circle = None
    undo_stack = []

    def transform_point(x, y, rot, dx, dy):
        new_x = math.cos(rot) * x - math.sin(rot) * y + dx
        new_y = math.sin(rot) * x + math.cos(rot) * y + dy
        return new_x, new_y

    def update_plot():
        nonlocal goal_circle
        tx, ty = [], []
        for x, y in zip(orig_path["x"], orig_path["y"]):
            new_x, new_y = transform_point(x, y, global_rotation, global_dx, global_dy)
            tx.append(new_x)
            ty.append(new_y)
        line.set_data(tx, ty)
        for patch in obstacle_patches:
            patch.remove()
        obstacle_patches.clear()
        times = [obs["time_s"] for obs in global_obstacles if not pd.isnull(obs.get("time_s"))]
        if times:
            t_min, t_max = min(times), max(times)
        else:
            t_min, t_max = 0, 0
        for obs in global_obstacles:
            center_x, center_y = transform_point(obs["obs_x"], obs["obs_y"], global_rotation, global_dx, global_dy)
            new_angle = obs["orientation"] + global_rotation
            # get_rotated_rect_coords must be defined elsewhere.
            coords = get_rotated_rect_coords(center_x, center_y, obs["size_x"], obs["size_y"], new_angle)
            norm_val = (obs["time_s"] - t_min) / (t_max - t_min) if t_max > t_min else 0.5
            color = cm.Reds(norm_val)
            poly = patches.Polygon(coords, closed=True, facecolor=color, edgecolor="yellow", picker=10)
            ax.add_patch(poly)
            obstacle_patches.append(poly)
        new_goal_center = transform_point(orig_goal_center[0], orig_goal_center[1],
                                          global_rotation, global_dx, global_dy)
        if goal_circle:
            goal_circle.center = new_goal_center
        else:
            goal_circle = plt.Circle(new_goal_center, GOAL_RADIUS, color="purple", fill=False, linewidth=2)
            ax.add_patch(goal_circle)
        fig.canvas.draw()
        plt.pause(0.0001)

    update_plot()

    selected = {"index": None}

    def on_pick(event):
        artist = event.artist
        for i, patch in enumerate(obstacle_patches):
            if patch == artist:
                selected["index"] = i
                print("Selected obstacle for editing.")
                break

    def on_key(event):
        nonlocal global_rotation, global_dx, global_dy
        if event.key == "R":
            dTheta = math.radians(1)
        elif event.key == "E":
            dTheta = -math.radians(1)
        elif event.key == "left":
            global_dx -= 0.1
            print("Global map translated left.")
            update_plot()
            return
        elif event.key == "right":
            global_dx += 0.1
            print("Global map translated right.")
            update_plot()
            return
        elif event.key == "up":
            global_dy += 0.1
            print("Global map translated up.")
            update_plot()
            return
        elif event.key == "down":
            global_dy -= 0.1
            print("Global map translated down.")
            update_plot()
            return
        elif event.key == "u":
            if undo_stack:
                restored = undo_stack.pop()
                global_obstacles.append(restored)
                print("Undo: Restored last deleted obstacle.")
                update_plot()
            return
        elif event.key == "r":
            idx = selected.get("index")
            if idx is not None and 0 <= idx < len(global_obstacles):
                global_obstacles[idx]["orientation"] += math.pi / 2
                print("Rotated selected obstacle by 90°.")
                update_plot()
            return
        elif event.key == "d":
            idx = selected.get("index")
            if idx is not None and 0 <= idx < len(global_obstacles):
                undo_stack.append(global_obstacles[idx])
                del global_obstacles[idx]
                selected["index"] = None
                print("Deleted selected obstacle.")
                update_plot()
            return
        else:
            return

        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        cx = (xlim[0] + xlim[1]) / 2
        cy = (ylim[0] + ylim[1]) / 2
        global_rotation += dTheta
        new_tx = math.cos(dTheta) * (global_dx - cx) - math.sin(dTheta) * (global_dy - cy) + cx
        new_ty = math.sin(dTheta) * (global_dx - cx) + math.cos(dTheta) * (global_dy - cy) + cy
        global_dx, global_dy = new_tx, new_ty
        direction = "anticlockwise" if dTheta > 0 else "clockwise"
        print(f"Global map rotated by {math.degrees(dTheta):.1f}° {direction} about the center.")
        update_plot()

    fig.canvas.mpl_connect("pick_event", on_pick)
    fig.canvas.mpl_connect("key_press_event", on_key)

    # Open the interactive window.
    plt.show()

    # After the window is closed, ensure the figure is drawn and then save it.
    fig.canvas.draw()
    os.makedirs(results_folder, exist_ok=True)
    safe_title = sanitize_filename(title)
    filename = os.path.join(results_folder, safe_title + "_Interactive_Birdseye.png")
    try:
        fig.savefig(filename, dpi=300)
        print(f"Saved Interactive Birdseye Plot as {filename}")
    except Exception as e:
        print(f"Error saving interactive plot: {e}")

    return pd.DataFrame(global_obstacles), goal_circle.center

def non_interactive_birdseye_plot(df_path, obstacles_df, title, results_folder):
    """ Create a non-interactive birdseye plot: - The plot is transformed so that the origin (0,0)
    is in one corner and the goal centroid (computed as the bounding-box corner farthest from the
    origin) is in the opposite corner. - Obstacles are filtered so that: • "obstacle" type obstacles
    with size_x or size_y less than 0.2 m are removed. • For each type ("obstacle" and "wall"), obstacles
    are clustered separately based on centroid proximity (1.2 m threshold for "obstacle" and 3 m threshold
    for "wall") and only clusters with at least 5 members are retained. From each cluster, the obstacle
    with the most recent timestamp is kept. • Remaining "wall" obstacles are checked against expected
    orientations: if an obstacle is near the top or bottom edge it should be horizontal (orientation
    ≈ 0 mod π), and if near the left or right edge it should be vertical (orientation ≈ π/2 mod π);
    if not, its orientation is rotated by 90°. - The function saves the plot as a PNG file and returns
    a DataFrame of final obstacles and the goal centroid. """
    MIN_MOVE_THRESHOLD = 0.1
    GOAL_RADIUS = 3
    # Filter out insignificant path movements
    df_path = df_path[(df_path["x"].abs() > MIN_MOVE_THRESHOLD) | (df_path["y"].abs() > MIN_MOVE_THRESHOLD)]
    orig_path = df_path.copy()

    # Convert obstacles_df to list of dictionaries and ensure required keys exist.
    obstacles_list = obstacles_df.to_dict(orient="records")
    for obs in obstacles_list:
        # If "centroid" exists but "obs_x"/"obs_y" do not, add them.
        if "obs_x" not in obs and "centroid" in obs:
            obs["obs_x"] = obs["centroid"][0]
            obs["obs_y"] = obs["centroid"][1]
        # Similarly, if "size" exists but size_x/size_y are missing, add them.
        if "size_x" not in obs and "size" in obs:
            obs["size_x"] = obs["size"][0]
            obs["size_y"] = obs["size"][1]

    # Compute the overall bounding points from the path and obstacles.
    all_orig_pts = list(zip(orig_path["x"], orig_path["y"]))
    for obs in obstacles_list:
        all_orig_pts.append((obs["obs_x"], obs["obs_y"]))
    if all_orig_pts:
        xs, ys = zip(*all_orig_pts)
        orig_x_min, orig_x_max = min(xs), max(xs)
        orig_y_min, orig_y_max = min(ys), max(ys)
        orig_corners = [(orig_x_min, orig_y_min), (orig_x_min, orig_y_max),
                        (orig_x_max, orig_y_min), (orig_x_max, orig_y_max)]
        # Select the corner farthest from the origin as the goal.
        orig_goal_center = max(orig_corners, key=lambda pt: math.hypot(pt[0], pt[1]))
    else:
        orig_goal_center = (0, 0)

    # Step 1: Remove small "obstacle" type obstacles.
    filtered_obstacles = []
    for obs in obstacles_list:
        if obs.get("type") == "obstacle":
            if obs.get("size_x", 0) < 0.5 or obs.get("size_y", 0) < 0.5:
                continue
        filtered_obstacles.append(obs)

    # Step 2: Cluster obstacles by type separately.
    def cluster_obstacles(obstacles, threshold):
        clusters = []
        for obs in obstacles:
            added = False
            for cluster in clusters:
                rep = cluster[0]
                dx = obs["obs_x"] - rep["obs_x"]
                dy = obs["obs_y"] - rep["obs_y"]
                if math.hypot(dx, dy) <= threshold:
                    cluster.append(obs)
                    added = True
                    break
            if not added:
                clusters.append([obs])
        return clusters

    # Separate obstacles by type.
    obstacles_obstacle = [obs for obs in filtered_obstacles if obs.get("type") == "obstacle"]
    obstacles_wall = [obs for obs in filtered_obstacles if obs.get("type") == "wall"]

    clusters_obstacle = cluster_obstacles(obstacles_obstacle, threshold=0.5)
    clusters_wall = cluster_obstacles(obstacles_wall, threshold=1.5)

    # Only keep clusters that have at least 5 obstacles, and select the obstacle with the most recent timestamp.
    final_obstacles = []
    for cluster in clusters_obstacle + clusters_wall:
        if len(cluster) >= 2:
            best = max(cluster, key=lambda o: o.get("time_s", 0))
            final_obstacles.append(best)

    # Step 3: Determine plot limits.
    if orig_goal_center[0] >= 0:
        xlim = (0, orig_goal_center[0])
    else:
        xlim = (orig_goal_center[0], 0)
    if orig_goal_center[1] >= 0:
        ylim = (0, orig_goal_center[1])
    else:
        ylim = (orig_goal_center[1], 0)

    # Step 4: For each remaining "wall" obstacle, adjust orientation if needed.
    for obs in final_obstacles:
        if obs.get("type") == "wall":
            x = obs["obs_x"]
            y = obs["obs_y"]
            d_left = abs(x - xlim[0])
            d_right = abs(xlim[1] - x)
            d_bottom = abs(y - ylim[0])
            d_top = abs(ylim[1] - y)
            distances = {"left": d_left, "right": d_right, "bottom": d_bottom, "top": d_top}
            nearest_edge = min(distances, key=distances.get)
            orient = obs.get("orientation", 0)
            norm_orient = abs(orient) % math.pi
            tolerance = math.radians(20)
            if nearest_edge in ["top", "bottom"]:
                if norm_orient > tolerance and abs(norm_orient - math.pi) > tolerance:
                    obs["orientation"] = orient + math.pi / 2
            elif nearest_edge in ["left", "right"]:
                if abs(norm_orient - math.pi / 2) > tolerance:
                    obs["orientation"] = orient + math.pi / 2

    # Create the plot.
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title(title)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")

    # Plot the path (filtering out points where both x and y are zero).
    filtered_path = orig_path[(orig_path["x"] != 0) & (orig_path["y"] != 0)]
    ax.plot(filtered_path["x"], filtered_path["y"], "b-", label="Path")

    # Determine color normalization based on obstacle timestamps.
    times = [obs.get("time_s", 0) for obs in final_obstacles if obs.get("time_s") is not None]
    if times:
        t_min, t_max = min(times), max(times)
    else:
        t_min, t_max = 0, 0

    # Plot each remaining obstacle.
    for obs in final_obstacles:
        center_x = obs["obs_x"]
        center_y = obs["obs_y"]
        angle = obs.get("orientation", 0)
        coords = get_rotated_rect_coords(center_x, center_y, obs.get("size_x", 0), obs.get("size_y", 0), angle)
        if t_max > t_min:
            norm_val = (obs.get("time_s", 0) - t_min) / (t_max - t_min)
        else:
            norm_val = 0.5
        color = cm.Reds(norm_val)
        poly = patches.Polygon(coords, closed=True, facecolor=color, edgecolor="yellow")
        ax.add_patch(poly)

    # Mark the goal with a purple circle.
    goal_circle = plt.Circle(orig_goal_center, GOAL_RADIUS, color="purple", fill=False, linewidth=2)
    ax.add_patch(goal_circle)

    # Set the axis limits.
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)

    # Save the figure.
    os.makedirs(results_folder, exist_ok=True)
    safe_title = sanitize_filename(title)
    filename = os.path.join(results_folder, safe_title + "_Birdseye.png")
    try:
        fig.savefig(filename, dpi=300)
        print(f"Saved Birdseye Plot as {filename}")
    except Exception as e:
        print(f"Error saving plot: {e}")

    plt.close(fig)
    return pd.DataFrame(final_obstacles), orig_goal_center


# -------------------------------------------------------------------
# Heading Error and Distance Plot Function (saves automatically)
# -------------------------------------------------------------------
def plot_heading_error_with_distance(df, final_goal_center, updated_obstacles, title, results_folder):
    GOAL_RADIUS = 3
    if "Time" in df.columns:
        df = df.rename(columns={"Time": "time_s"})
    goal_dist = np.hypot(df["x"] - final_goal_center[0], df["y"] - final_goal_center[1]) - GOAL_RADIUS
    obst_polygons = []
    for idx, row in updated_obstacles.iterrows():
        poly = get_rotated_rect_coords(row["obs_x"], row["obs_y"], row["size_x"], row["size_y"], row["orientation"])
        obst_polygons.append(poly)
    min_obst_dist = []
    for idx, row in df.iterrows():
        pt = (row["x"], row["y"])
        if obst_polygons:
            dists = [point_to_polygon_distance(pt, poly) for poly in obst_polygons]
            min_obst_dist.append(min(dists))
        else:
            min_obst_dist.append(np.nan)
    abs_heading_error = df["heading_error"].abs()
    segments = segment_indices(df)
    fig, ax1 = plt.subplots(figsize=(10, 5))
    ax2 = ax1.twinx()
    for seg in segments:
        seg_times = df["time_s"].iloc[seg]
        seg_goal = goal_dist[seg]
        seg_obst = np.array(min_obst_dist)[seg]
        seg_heading = abs_heading_error.iloc[seg]
        ax1.plot(seg_times, seg_goal, 'k-', label="Distance from Goal Edge (m)")
        ax1.plot(seg_times, seg_obst, 'g-', label="Min Distance from Obstacles (m)")
        ax2.plot(seg_times, seg_heading, 'b-', label="Absolute Heading Error (rad)")
    if len(segments) > 1:
        for i in range(len(segments)-1):
            last_idx = segments[i][-1]
            first_idx = segments[i+1][0]
            lost_time = df["time_s"].iloc[last_idx]
            regained_time = df["time_s"].iloc[first_idx]
            ax1.axvline(x=lost_time, color='red', linestyle='--', label="Localisation Lost")
            ax1.axvline(x=regained_time, color='green', linestyle='--', label="Localisation Regained")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Distance (m)")
    ax2.set_ylabel("Absolute Heading Error (rad)", color="b")
    ax2.tick_params(axis="y", labelcolor="b")
    handles1, labels1 = ax1.get_legend_handles_labels()
    handles2, labels2 = ax2.get_legend_handles_labels()
    unique = {}
    for h, l in zip(handles1 + handles2, labels1 + labels2):
        if l not in unique:
            unique[l] = h
    ax1.legend(list(unique.values()), list(unique.keys()), loc="upper right")
    plt.title(title)
    plt.grid(True)
    safe_title = sanitize_filename(title)
    filename = os.path.join(results_folder, safe_title + ".png")
    plt.savefig(filename, dpi=300)
    plt.close()
    print(f"Saved Heading Error and Distance Plot as {filename}")

# -------------------------------------------------------------------
# Command vs. Heading Error Plot Function (saves automatically)
# -------------------------------------------------------------------
def plot_command_heading_error(arduino_df, nav_df, df_odom, title, results_folder):
    title = re.sub(r"Unknown", "", title, flags=re.IGNORECASE).strip()
    if "command" not in arduino_df.columns and "data" in arduino_df.columns:
        arduino_df = arduino_df.rename(columns={"data": "command"})
    if "Time" in arduino_df.columns:
        arduino_df = arduino_df.rename(columns={"Time": "time_s"})
    if "Time" in nav_df.columns:
        nav_df = nav_df.rename(columns={"Time": "time_s"})
    if "Time" in df_odom.columns:
        df_odom = df_odom.rename(columns={"Time": "time_s"})

    merged = pd.merge_asof(arduino_df.sort_values("time_s"),
                           df_odom.sort_values("time_s"),
                           on="time_s", direction="nearest")
    nav_merge = pd.merge_asof(arduino_df.sort_values("time_s"),
                              nav_df.sort_values("time_s"),
                              on="time_s", direction="nearest")
    merged["heading_error"] = nav_merge["heading_error"]
    try:
        merged["heading_error_deg"] = -merged["heading_error"] * (180 / math.pi)
        merged["command_angle"] = merged["command"].str.lower().map(COMMAND_MAP)
        merged["diff"] = np.abs(merged["heading_error_deg"] - merged["command_angle"])
        avg_diff = merged["diff"].mean()
        print(f"Average absolute difference (deg): {avg_diff:.2f}")
    except Exception as e:
        print("Error mapping commands to angles:", e)
        return

    tol = 1e-6
    merged = merged[(merged["x"].abs() > tol) | (merged["y"].abs() > tol)]

    segments = []
    current = []
    valid = (~((merged["x"] == 0) & (merged["y"] == 0))).values
    for i, v in enumerate(valid):
        if v:
            current.append(i)
        else:
            if current:
                segments.append(current)
                current = []
    if current:
        segments.append(current)

    plt.figure(figsize=(10, 5))
    for seg in segments:
        seg_times = merged["time_s"].iloc[seg]
        seg_heading = merged["heading_error_deg"].iloc[seg]
        seg_command = merged["command_angle"].iloc[seg]
        seg_diff = merged["diff"].iloc[seg]
        plt.plot(seg_times, seg_heading, label="Negative Heading Error (deg)")
        plt.plot(seg_times, seg_command, label="Command Angle (deg)")
        plt.plot(seg_times, seg_diff, label="Absolute Difference (deg)")
    plt.axhline(avg_diff, color="gray", linestyle="--", label=f"Avg Abs Diff = {avg_diff:.1f}°")

    lost_times, regained_times = lost_regained_times(df_odom)
    for i, lt in enumerate(lost_times):
        if i == 0:
            plt.axvline(x=lt, color="red", linestyle="--", label="Localisation Lost")
        else:
            plt.axvline(x=lt, color="red", linestyle="--")
    for i, rt in enumerate(regained_times):
        if i == 0:
            plt.axvline(x=rt, color="green", linestyle="--", label="Localisation Regained")
        else:
            plt.axvline(x=rt, color="green", linestyle="--")

    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.title(title + " - Command vs Heading Error")
    handles, labels = plt.gca().get_legend_handles_labels()
    unique = {}
    for h, l in zip(handles, labels):
        if l not in unique:
            unique[l] = h
    plt.legend(list(unique.values()), list(unique.keys()), loc="upper right")
    plt.grid(True)
    safe_title = sanitize_filename(title)
    filename = os.path.join(results_folder, safe_title + ".png")
    plt.savefig(filename, dpi=300)
    plt.close()
    print(f"Saved Command vs Heading Error Plot as {filename}")

# -------------------------------------------------------------------
# Diagnostics Plot Function (Combined CPU and Memory, saves automatically)
# -------------------------------------------------------------------
def plot_diagnostics(diag_df, odom_df, title, results_folder):
    if "data" in diag_df.columns and not {"machine", "cpu", "memory"}.issubset(diag_df.columns):
        machines = []
        cpus = []
        memories = []
        for i, row in diag_df.iterrows():
            try:
                d = json.loads(row["data"])
                machines.append(d.get("machine"))
                cpus.append(d.get("cpu"))
                memories.append(d.get("memory"))
            except Exception as e:
                machines.append(None)
                cpus.append(None)
                memories.append(None)
        diag_df["machine"] = machines
        diag_df["cpu"] = cpus
        diag_df["memory"] = memories

    if not {"machine", "cpu", "memory"}.issubset(diag_df.columns):
        print("Diagnostics data insufficient for plotting.")
        return

    if "time_s" not in diag_df.columns and "Time" in diag_df.columns:
        diag_df = diag_df.rename(columns={"Time": "time_s"})
    if "time_s" not in odom_df.columns and "Time" in odom_df.columns:
        odom_df = odom_df.rename(columns={"Time": "time_s"})

    machines = diag_df["machine"].unique()
    valid_odom = odom_df[(odom_df["x"] != 0) | (odom_df["y"] != 0)]
    new_end_time = valid_odom["time_s"].max() if not valid_odom.empty else diag_df["time_s"].max()
    lost_times, regained_times = lost_regained_times(odom_df)
    diag_df = diag_df[diag_df["time_s"] <= new_end_time]

    fig, (ax_cpu, ax_mem) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for m in machines:
        sub = diag_df[diag_df["machine"] == m]
        ax_cpu.plot(sub["time_s"], sub["cpu"], label=f"CPU {m}")
    ax_cpu.set_title(title + " - Diagnostics")
    ax_cpu.set_ylabel("CPU (%)")
    ax_cpu.grid(True)
    for i, lt in enumerate(lost_times):
        if i == 0:
            ax_cpu.axvline(x=lt, color="red", linestyle="--", label="Localisation Lost")
        else:
            ax_cpu.axvline(x=lt, color="red", linestyle="--")
    for i, rt in enumerate(regained_times):
        if i == 0:
            ax_cpu.axvline(x=rt, color="green", linestyle="--", label="Localisation Regained")
        else:
            ax_cpu.axvline(x=rt, color="green", linestyle="--")
    for m in machines:
        sub = diag_df[diag_df["machine"] == m]
        ax_mem.plot(sub["time_s"], sub["memory"], label=f"Memory {m}")
    ax_mem.set_xlabel("Time (s)")
    ax_mem.set_ylabel("Memory (%)")
    ax_mem.grid(True)
    for i, lt in enumerate(lost_times):
        if i == 0:
            ax_mem.axvline(x=lt, color="red", linestyle="--", label="Localisation Lost")
        else:
            ax_mem.axvline(x=lt, color="red", linestyle="--")
    for i, rt in enumerate(regained_times):
        if i == 0:
            ax_mem.axvline(x=rt, color="green", linestyle="--", label="Localisation Regained")
        else:
            ax_mem.axvline(x=rt, color="green", linestyle="--")
    ax_cpu.legend(loc="upper right")
    ax_mem.legend(loc="upper right")
    ax_cpu.set_xlim(0, new_end_time)
    ax_mem.set_xlim(0, new_end_time)
    plt.tight_layout()
    safe_title = sanitize_filename(title)
    filename = os.path.join(results_folder, safe_title + ".png")
    plt.savefig(filename, dpi=300)
    plt.close()
    print(f"Saved Diagnostics Plot as {filename}")

# -------------------------------------------------------------------
# Processing Time per Command Plot Function (saves automatically)
# -------------------------------------------------------------------
def plot_processing_time_per_command(arduino_df, nav_df, df_odom, updated_obstacles, title, results_folder):
    if "command" not in arduino_df.columns and "data" in arduino_df.columns:
        arduino_df = arduino_df.rename(columns={"data": "command"})
    if "Time" in arduino_df.columns:
        arduino_df = arduino_df.rename(columns={"Time": "time_s"})
    if "Time" in nav_df.columns:
        nav_df = nav_df.rename(columns={"Time": "time_s"})
    if "Time" in df_odom.columns:
        df_odom = df_odom.rename(columns={"Time": "time_s"})

    merged = pd.merge_asof(arduino_df.sort_values("time_s"),
                           df_odom.sort_values("time_s"),
                           on="time_s", direction="nearest")
    nav_df = extract_processing_time(nav_df)
    nav_merge = pd.merge_asof(arduino_df.sort_values("time_s"),
                              nav_df.sort_values("time_s"),
                              on="time_s", direction="nearest")
    merged["processing_time"] = nav_merge["processing_time"]

    dt = merged["time_s"].diff(periods=-1)
    weighted_avg = (merged["processing_time"][:-1] * dt[:-1]).sum() / dt[:-1].sum()
    print(f"Integrated processing time (ms): {weighted_avg:.2f}")

    obst_polygons = []
    for idx, row in updated_obstacles.iterrows():
        poly = get_rotated_rect_coords(row["obs_x"], row["obs_y"],
                                       row["size_x"], row["size_y"],
                                       row["orientation"])
        obst_polygons.append(poly)
    min_distances = []
    for idx, row in merged.iterrows():
        pt = (row["x"], row["y"])
        if obst_polygons:
            dists = [point_to_polygon_distance(pt, poly) for poly in obst_polygons]
            min_distances.append(min(dists))
        else:
            min_distances.append(np.nan)
    merged["min_distance"] = min_distances

    tol = 1e-6
    merged = merged[(merged["x"].abs() > tol) | (merged["y"].abs() > tol)]
    lost_times, regained_times = lost_regained_times(df_odom)

    fig, ax1 = plt.subplots(figsize=(10, 5))
    ax2 = ax1.twinx()
    ax1.plot(merged["time_s"], merged["processing_time"], '-', color='blue', label="Processing Time (ms)")
    ax1.axhline(y=weighted_avg, color="gray", linestyle="--", label=f"Weighted Avg Proc = {weighted_avg:.1f} ms")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Processing Time (ms)", color='blue')
    ax1.tick_params(axis='y', labelcolor='blue')
    ax2.plot(merged["time_s"], merged["min_distance"], '-', color='green', label="Min Distance (m)")
    ax2.set_ylabel("Min Distance (m)", color='green')
    ax2.tick_params(axis='y', labelcolor='green')
    for i, lt in enumerate(lost_times):
        if i == 0:
            ax1.axvline(x=lt, color="red", linestyle="--", label="Localisation Lost")
        else:
            ax1.axvline(x=lt, color="red", linestyle="--")
    for i, rt in enumerate(regained_times):
        if i == 0:
            ax1.axvline(x=rt, color="green", linestyle="--", label="Localisation Regained")
        else:
            ax1.axvline(x=rt, color="green", linestyle="--")
    fig.legend(loc='upper right')
    plt.title(title + " - Processing Time per Command")
    plt.grid(True)
    safe_title = sanitize_filename(title)
    filename = os.path.join(results_folder, safe_title + ".png")
    plt.savefig(filename, dpi=300)
    plt.close()
    print(f"Saved Processing Time per Command Plot as {filename}")

# -------------------------------------------------------------------
# 3D Plot Function (saves automatically as PNG using Plotly)
# -------------------------------------------------------------------
def create_and_show_3d_plot(df, map_pc_df, obs_pc_df, obstacles_df, final_goal_center, report_name, results_folder):
    if "obs_z" not in obstacles_df.columns:
        obstacles_df["obs_z"] = 0.0
    if "size_z" not in obstacles_df.columns:
        obstacles_df["size_z"] = 0.0

    filtered_df = df[(df["x"] != 0) | (df["y"] != 0)]
    trace_path_3d = go.Scatter3d(
        x=filtered_df["x"],
        y=filtered_df["y"],
        z=filtered_df["z"] if "z" in filtered_df.columns else np.zeros(len(filtered_df)),
        mode="lines+markers",
        name="3D Path",
        marker=dict(size=4)
    )
    trace_map = go.Scatter3d(
        x=map_pc_df["mx"],
        y=map_pc_df["my"],
        z=map_pc_df["mz"],
        mode="markers",
        name="Map Env",
        marker=dict(size=1, color="gray", opacity=0.4)
    )
    trace_obs = go.Scatter3d(
        x=obs_pc_df["mx"],
        y=obs_pc_df["my"],
        z=obs_pc_df["mz"],
        mode="markers",
        name="Obst Env",
        marker=dict(size=2, color="red", opacity=0.5)
    )
    obstacle_meshes = []
    for idx, obs in obstacles_df.iterrows():
        try:
            box_3d = oriented_box_3d(
                obs["obs_x"], obs["obs_y"], obs["obs_z"],
                obs["size_x"], obs["size_y"], obs["size_z"],
                obs["orientation"]
            )
            obstacle_meshes.append(box_3d)
        except Exception as e:
            print(f"Error creating 3D box for obstacle {idx}: {e}")
    data_3d = [trace_map, trace_obs, trace_path_3d] + obstacle_meshes
    center = np.array([final_goal_center[0], final_goal_center[1], -2.5])
    eye = np.array([0, 0, 0.2])
    direction = np.array(center) - eye
    norm = np.linalg.norm(direction)
    if norm > 0:
        unit_direction = direction / norm
        new_eye = eye - unit_direction * 1.7
    else:
        new_eye = eye
    camera = dict(
        center=dict(x=center[0], y=center[1], z=center[2]),
        eye=dict(x=new_eye[0], y=new_eye[1], z=new_eye[2]),
        up=dict(x=0, y=0, z=0.2)
    )
    layout = go.Layout(
        title=report_name + " 3D View",
        scene=dict(camera=camera, xaxis_title="X", yaxis_title="Y", zaxis_title="Z")
    )
    fig = go.Figure(data=data_3d, layout=layout)
    safe_report = sanitize_filename(report_name)
    filename = os.path.join(results_folder, safe_report + "_3D_View.png")
    # Increase image quality by setting width, height, and scale.
    fig.write_image(filename, width=1200, height=900, scale=2)
    print(f"Saved 3D Plot as {filename}")


# -------------------------------------------------------------------
# Function to Create and Display the 2D Animation (saves as video)
# -------------------------------------------------------------------
import os
import math
import numpy as np
import imageio
import plotly.graph_objs as go

def display_2d_animation(df_anim, updated_obstacles, final_goal_center, results_folder,
                         predefined_force_length=1, frame_duration=100, resolution_scale=360):
    # Create static obstacle shapes.
    obstacle_shapes = []
    for idx, obs in updated_obstacles.iterrows():
        try:
            corners = get_rotated_rect_coords(obs["obs_x"], obs["obs_y"],
                                              obs["size_x"], obs["size_y"],
                                              obs["orientation"])
            path_str = "M " + " L ".join(f"{x} {y}" for x, y in corners) + " Z"
            shape = {
                "type": "path",
                "path": path_str,
                "fillcolor": "rgba(255,0,0,0.3)",
                "line": {"color": "yellow"},
                "xref": "x",
                "yref": "y"
            }
            obstacle_shapes.append(shape)
        except Exception as e:
            print(f"Error processing obstacle {idx}: {e}")

    # Create a goal circle with a fixed radius of 3, centered at final_goal_center.
    GOAL_RADIUS = 3
    goal_circle = {
        "type": "circle",
        "xref": "x", "yref": "y",
        "x0": final_goal_center[0] - GOAL_RADIUS,
        "y0": final_goal_center[1] - GOAL_RADIUS,
        "x1": final_goal_center[0] + GOAL_RADIUS,
        "y1": final_goal_center[1] + GOAL_RADIUS,
        "line": {"color": "purple"}
    }
    obstacle_shapes.append(goal_circle)

    # Helper: compute constant-length force vector.
    def force_vector(force, length=predefined_force_length):
        if force[0] == 0 and force[1] == 0:
            return [0, 0]
        theta = math.atan2(force[1], force[0])
        return [length * math.cos(theta), length * math.sin(theta)]

    # Group frames by rounded Time.
    df_anim = df_anim.copy()
    df_anim["Time_round"] = df_anim["Time"].round(1)
    unique_times = np.sort(df_anim["Time_round"].unique())
    frames = []
    for t in unique_times:
        up_to_now = df_anim[df_anim["Time_round"] <= t]
        path_xy = up_to_now[["x", "y"]].dropna().drop_duplicates()
        path_xy = path_xy[~((path_xy["x"] == 0) & (path_xy["y"] == 0))]
        frame_data = df_anim[df_anim["Time_round"] == t]
        if frame_data.empty:
            continue
        current_row = frame_data.iloc[-1]
        x, y = current_row["x"], current_row["y"]
        total = force_vector(current_row.get("APF_total_force", [0, 0]), predefined_force_length)
        attractive = force_vector(current_row.get("APF_attractive_force", [0, 0]), predefined_force_length)
        repulsive = force_vector(current_row.get("APF_repulsive_force", [0, 0]), predefined_force_length)

        trace_path = go.Scatter(
            x=path_xy["x"],
            y=path_xy["y"],
            mode="lines+markers",
            name="Path",
            line=dict(color="blue"),
            marker=dict(size=5)
        )
        trace_total = go.Scatter(
            x=[x, x + total[0]],
            y=[y, y + total[1]],
            mode="lines+markers",
            name="Total Force",
            line=dict(color="red"),
            marker=dict(size=5)
        )
        trace_attr = go.Scatter(
            x=[x, x + attractive[0]],
            y=[y, y + attractive[1]],
            mode="lines+markers",
            name="Attractive Force",
            line=dict(color="green"),
            marker=dict(size=5)
        )
        trace_rep = go.Scatter(
            x=[x, x + repulsive[0]],
            y=[y, y + repulsive[1]],
            mode="lines+markers",
            name="Repulsive Force",
            line=dict(color="orange"),
            marker=dict(size=5)
        )

        frame = go.Frame(
            data=[trace_path, trace_total, trace_attr, trace_rep],
            name=str(t),
            layout=go.Layout(shapes=obstacle_shapes)
        )
        frames.append(frame)

    # Hardcode axis bounds
    x_limits = [0, final_goal_center[0]*0.5]
    y_limits = [0, final_goal_center[1]*0.75]

    # Compute image dimensions based on the axis range and resolution_scale.
    width = int(abs((x_limits[1] - x_limits[0])) * resolution_scale)
    height = int(abs((y_limits[1] - y_limits[0])) * resolution_scale)

    # Build a fixed layout without extra UI elements.
    fixed_layout = go.Layout(
        xaxis=dict(range=x_limits, autorange=False, showgrid=False, zeroline=False, visible=False),
        yaxis=dict(range=y_limits, autorange="reversed", scaleanchor="x", scaleratio=1, showgrid=False, zeroline=False, visible=False),
        shapes=obstacle_shapes,
        margin=dict(l=0, r=0, t=0, b=0),
        showlegend=False,
        title=""
    )

    # Save each frame as a temporary high-resolution PNG.
    temp_filenames = []
    for i, frame in enumerate(frames):
        temp_fig = go.Figure(data=frame.data, layout=fixed_layout)
        temp_filename = os.path.join(results_folder, f"anim_frame_{i:04d}.png")
        temp_fig.write_image(temp_filename, width=width, height=height, scale=1)
        temp_filenames.append(temp_filename)

    # Compile the temporary PNG images into an MP4 video.
    video_filename = os.path.join(results_folder, "2D_Birdseye_Animation.mp4")
    fps = 1000 / frame_duration  # Convert frame duration (ms) to fps.
    writer = imageio.get_writer(video_filename, fps=fps)
    for fname in temp_filenames:
        image = imageio.imread(fname)
        writer.append_data(image)
    writer.close()

    # Clean up temporary image files.
    for fname in temp_filenames:
        os.remove(fname)

    print(f"Saved 2D Birdseye Animation as video: {video_filename}")

def compute_and_record_reaction_times(report_name, arduino_df, nav_df, output_csv="reaction_times.csv"):
    """ For each machine command (from arduino_df) compute: - The reaction start time: time elapsed until the heading error
     (in degrees) has changed by at least 10° toward the commanded angle. - The reaction completion time: time elapsed until
      the heading error passes through zero (i.e. changes sign) relative to the commanded angle.
    Both times are computed relative to the command's timestamp.

    The function prints the average reaction start and completion times (in seconds) and appends a new row
    to output_csv with columns: report_name, avg_reaction_start, avg_reaction_complete.

    Parameters:
      report_name : string, used to identify the report in the CSV.
      arduino_df : DataFrame containing the machine command events. It should have a timestamp column ("Time" or "time_s")
                   and a command column (either "command" or "data") that maps (via COMMAND_MAP) to an angle.
      nav_df : DataFrame from navigation_perf.csv that includes a "heading_error" column (in radians) and a timestamp column.
      output_csv : Path to the CSV file to which the results will be appended.
    """

    # Ensure timestamps are numeric and use column "time_s"
    if "Time" in arduino_df.columns:
        arduino_df = arduino_df.rename(columns={"Time": "time_s"})
    if "Time" in nav_df.columns:
        nav_df = nav_df.rename(columns={"Time": "time_s"})

    # Sort both dataframes by time
    arduino_df = arduino_df.sort_values("time_s")
    nav_df = nav_df.sort_values("time_s")

    reaction_start_times = []
    reaction_complete_times = []

    # Loop over each command event
    for idx, cmd_row in arduino_df.iterrows():
        T_cmd = cmd_row["time_s"]
        # Get the command string from "command" column (or "data" if needed)
        if "command" in cmd_row:
            cmd_str = cmd_row["command"]
        elif "data" in cmd_row:
            cmd_str = cmd_row["data"]
        else:
            continue

        cmd_str = str(cmd_str).lower().strip()
        if cmd_str not in COMMAND_MAP:
            continue  # Skip commands that don't map to an angle.
        commanded_angle = COMMAND_MAP[cmd_str]

        # Get nav data after the command event
        nav_after = nav_df[nav_df["time_s"] >= T_cmd]
        if nav_after.empty:
            continue

        # Use the first nav row after the command as the baseline.
        try:
            first_nav = nav_after.iloc[0]
            # Convert heading error from radians to degrees using the same conversion as in your plot (with a negative sign).
            H0 = - first_nav["heading_error"] * (180 / math.pi)
        except Exception as e:
            continue

        # Define the initial difference between the current heading and the commanded angle.
        initial_diff = abs(H0 - commanded_angle)
        # We only measure reaction start if the initial error is greater than 10°.
        reaction_start_time = None
        reaction_complete_time = None

        # Determine the sign of the initial error (relative to commanded angle)
        initial_error_sign = np.sign(H0 - commanded_angle)
        # Iterate over the navigation data after T_cmd.
        for i, nav_row in nav_after.iterrows():
            try:
                H = - nav_row["heading_error"] * (180 / math.pi)
            except Exception:
                continue
            current_diff = abs(H - commanded_angle)

            # Reaction start: detect when the difference has decreased by at least 10° compared to initial error.
            if initial_diff > 10 and reaction_start_time is None:
                if current_diff <= (initial_diff - 10):
                    reaction_start_time = nav_row["time_s"] - T_cmd

            # Reaction complete: detect when the error passes through zero (i.e. the sign of the difference changes).
            # Compute current sign
            current_error_sign = np.sign(H - commanded_angle)
            if reaction_complete_time is None and initial_error_sign != 0 and current_error_sign != 0:
                if current_error_sign != initial_error_sign:
                    reaction_complete_time = nav_row["time_s"] - T_cmd

            # Break early if both times have been recorded.
            if reaction_start_time is not None and reaction_complete_time is not None:
                break

        if reaction_start_time is not None:
            reaction_start_times.append(reaction_start_time)
        if reaction_complete_time is not None:
            reaction_complete_times.append(reaction_complete_time)

    # Compute average times.
    if reaction_start_times:
        avg_reaction_start = np.mean(reaction_start_times)
    else:
        avg_reaction_start = np.nan
    if reaction_complete_times:
        avg_reaction_complete = np.mean(reaction_complete_times)
    else:
        avg_reaction_complete = np.nan

    print(f"Average reaction start time (time to begin change): {avg_reaction_start:.2f} seconds")
    print(f"Average reaction completion time (time to complete change): {avg_reaction_complete:.2f} seconds")

    # Append results to CSV file.
    header = ["report_name", "avg_reaction_start", "avg_reaction_complete"]
    row_df = pd.DataFrame([[report_name, avg_reaction_start, avg_reaction_complete]], columns=header)
    if os.path.exists(output_csv):
        row_df.to_csv(output_csv, mode='a', header=False, index=False)
    else:
        row_df.to_csv(output_csv, mode='w', header=True, index=False)

    return avg_reaction_start, avg_reaction_complete

# -------------------------------------------------------------------
# Example Main Processing Function for a Bag File (using sanitized names)
# -------------------------------------------------------------------
def process_bag_file(bag_file):
    base_name = os.path.splitext(os.path.basename(bag_file))[0]
    report_name = get_report_title(base_name)
    safe_report = sanitize_filename(report_name)
    results_folder = os.path.join("results", safe_report)
    print(f"Processing bag file: {bag_file}")
    # Create CSV output folder and results folder.
    csv_out_folder = os.path.join("csv_", base_name)
    os.makedirs(csv_out_folder, exist_ok=True)
    os.makedirs(results_folder, exist_ok=True)
    write_csvs_from_bag(bag_file, csv_out_folder)
    data = load_data(csv_out_folder)
    if data.get("odom_data.csv", pd.DataFrame()).empty:
        print(f"Skipping {bag_file}: odom_data.csv is empty or missing.")
        return
    df_path = merge_data(data)
    arduino_df = data.get("arduino_commands.csv", pd.DataFrame())
    nav_df = data.get("navigation_perf.csv", pd.DataFrame())
    if not nav_df.empty:
        nav_df = extract_heading_error(nav_df)
        df_path = pd.merge_asof(df_path.sort_values("Time"), nav_df.sort_values("Time"), on="Time", suffixes=("", "_nav"))
    obstacles_df = extract_obstacles(data)
    title_interactive = f"{report_name} Birdseye Interactive Plot"
    updated_obstacles, final_goal_center = non_interactive_birdseye_plot(df_path, obstacles_df, title_interactive, results_folder)
    print("Updated Obstacles after editing:")
    print(updated_obstacles)
    plot_title = f"{report_name} Heading Error and Distance Plot"
    plot_heading_error_with_distance(df_path, final_goal_center, updated_obstacles, plot_title, results_folder)
    command_title = f"{report_name} Command vs Heading Error Plot"
    if not arduino_df.empty and not nav_df.empty:
        plot_command_heading_error(arduino_df, nav_df, df_path, command_title, results_folder)
    else:
        print("Insufficient data for Command vs Heading Error Plot.")
    diagnostics_df = data.get("system_diagnostics.csv", pd.DataFrame())
    if not diagnostics_df.empty:
        diagnostics_df = convert_time(diagnostics_df, "Time")
        diagnostics_df = diagnostics_df.rename(columns={"Time": "time_s"})
        odom_df = merge_data(data)
        if "Time" in odom_df.columns:
            odom_df = odom_df.rename(columns={"Time": "time_s"})
        plot_diagnostics(diagnostics_df, odom_df, f"{report_name} Diagnostics", results_folder)
    processing_title = f"{report_name} Processing Time per Command"
    if not arduino_df.empty and not nav_df.empty:
        plot_processing_time_per_command(arduino_df, nav_df, df_path, updated_obstacles, processing_title, results_folder)
    else:
        print("Insufficient data for Processing Time per Command Plot.")
    map_pc_df = data.get("cloud_map.csv", pd.DataFrame())
    obs_pc_df = data.get("cloud_obstacles.csv", pd.DataFrame())
    if not map_pc_df.empty and not obs_pc_df.empty:
        map_pc = decode_pointcloud2_csv(map_pc_df.iloc[0])
        obs_pc = decode_pointcloud2_csv(obs_pc_df.iloc[0])
        create_and_show_3d_plot(df_path, map_pc, obs_pc, updated_obstacles, final_goal_center, report_name, results_folder)
    else:
        print("Insufficient 3D point cloud data for reconstruction.")
    if not nav_df.empty:
        anim_df = create_anim_df(df_path, nav_df)
        display_2d_animation(anim_df, updated_obstacles, final_goal_center, results_folder, predefined_force_length=1, frame_duration=50)
    else:
        print("Insufficient navigation performance data for 2D animation.")

    lost_coords = []
    df_path_sorted = df_path.sort_values("Time")
    for i in range(len(df_path_sorted) - 1):
        current = df_path_sorted.iloc[i]
        nxt = df_path_sorted.iloc[i+1]
        if (nxt["x"] == 0 and nxt["y"] == 0) and not (current["x"] == 0 and current["y"] == 0):
            lost_coords.append({"time": current["Time"], "x": current["x"], "y": current["y"]})
    if lost_coords:
        lost_coords_df = pd.DataFrame(lost_coords)
        lost_filename = os.path.join(csv_out_folder, f"{base_name}_lost_localisation.csv")
        lost_coords_df.to_csv(lost_filename, index=False)
        print(f"Saved lost localisation coordinates as {lost_filename}")
    else:
        print("No lost localisation points detected.")
    compute_and_record_reaction_times(report_name, arduino_df, nav_df, output_csv="reaction_times.csv")
    plt.close('all')

# -------------------------------------------------------------------
# Main Entry Point: Loop through all bag files in folder "bagfiles"
# -------------------------------------------------------------------
if __name__ == "__main__":
    bag_files = glob.glob(os.path.join("bagfiles", "*.bag"))
    for bag_file in bag_files:
        process_bag_file(bag_file)