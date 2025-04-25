#!/usr/bin/env python3
"""
Revised Comprehensive Mechatronics Data Processing Script

This script processes all .bag files in the script’s folder and produces visuals:
  • 2D animated birdseye view with fixed room boundaries (–10 to +10 m),
    a hardcoded goal marker, persistent obstacle displays (only the final detection per unique obstacle is kept),
    and lost–localisation markers.
  • APF Metrics plot (total force capped at 10, heading error remains positive).
  • Arduino command vs. heading error plot (with heading error converted to negative for display)
    and a processing time plot with a second axis for minimum distance.
  • Birdseye plot of trajectory and obstacles (using the final persistent obstacle positions)
    with odometry loss markers and minimum–distance markers.
  • Heading error overlay plot that also shows distance from the goal.
  • A global localisation–loss heatmap accumulated over all bag files.
  • 3D interactive view showing the final path and obstacles.

Hardcoded parameters (e.g. NAME_MAP, GOAL_AREA, room dimensions, grid thresholds, etc.)
are defined near the top.
"""

import os, argparse, time, io, json, math, re, struct, ast
import pandas as pd, numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import plotly.graph_objects as go
import imageio.v2 as imageio  # using imageio.v2 to suppress deprecation warnings
from bagpy import bagreader

# ---------------------------
# Global Parameters
# ---------------------------
NAME_MAP = {
    "oscar": "Person 1",
    "sam": "Person 2",
    "dylan": "Person 3",
    "trystan": "Person 4"
}
OUTPUT_FOLDERS = {
    "csv_data": "csv_data",
    "birdseye": "birdseye_plots",
    "heading_error": "heading_error_plots",
    "arduino": "arduino_plots",
    "2d_anim": "2d_animation",
    "3d_anim": "3d_animation",
    "apf": "apf_plots",
    "diagnostics": "diagnostics_plots",
    "heatmap": "localisation_loss_heatmap",
    "arduino_proc": "arduino_processing"
}
GOAL_AREA = {'x': 2.0, 'y': 2.0, 'width': 1.0, 'height': 1.0, 'color': 'purple'}
ROOM_BOUNDS = (-10, 10)  # fixed room boundaries ±10 m
CEILING_Z = 0.5
# Arduino command mapping in degrees (for command plot)
COMMAND_MAP = {
    "forward": 0,
    "halfleft": -45,
    "left": -90,
    "halfright": 45,
    "right": 90,
    "stop": 0
}
REALTIME_FPS = 30  # for 2D animation (1 second of data = 1 second of animation)
CELL_SIZE_ANIM = 1.0  # threshold (m) for 2D animation obstacles
CELL_SIZE_BIRD = 1.5  # threshold (m) for birdseye obstacles
MIN_AREA = 0.5  # obstacles with area < 0.5 m^2 are dropped
# Global container for lost localisation events (for heatmap)
global_lost_coords = []


# ---------------------------
# Bag-to-CSV Conversion
# ---------------------------
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


# ---------------------------
# Data Preprocessing Functions
# ---------------------------
def convert_time(df, time_col="Time"):
    if time_col in df.columns and not df.empty:
        df["time_s"] = df[time_col] - df[time_col].iloc[0]
    else:
        df["time_s"] = np.nan
    return df


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


def fix_odom_teleport(df, tol=1e-3):
    prev_x, prev_y = None, None
    for idx, row in df.iterrows():
        x, y = row.get("x", 0), row.get("y", 0)
        if abs(x) < tol and abs(y) < tol and prev_x is not None:
            df.at[idx, "x"] = prev_x
            df.at[idx, "y"] = prev_y
        else:
            prev_x, prev_y = x, y
    return df


def parse_nav_json(s):
    if not isinstance(s, str):
        return None
    try:
        data = json.loads(s)
        return {
            "processing_time": data.get("processing_time", np.nan),
            "heading_error": data.get("heading_error", np.nan),
            "total_fx": data["APF_total_force"][0] if "APF_total_force" in data else np.nan,
            "total_fy": data["APF_total_force"][1] if "APF_total_force" in data else np.nan,
            "attr_fx": data["APF_attractive_force"][0] if "APF_attractive_force" in data else np.nan,
            "attr_fy": data["APF_attractive_force"][1] if "APF_attractive_force" in data else np.nan,
            "rep_fx": data["APF_repulsive_force"][0] if "APF_repulsive_force" in data else np.nan,
            "rep_fy": data["APF_repulsive_force"][1] if "APF_repulsive_force" in data else np.nan,
            "command_sent": data.get("command_sent", None)
        }
    except Exception as e:
        print("Error parsing navigation JSON:", e)
        return None


def parse_obstacles(s):
    if not isinstance(s, str):
        return []
    try:
        arr = json.loads(s)
        out = []
        for obs in arr:
            c = obs.get("centroid", [np.nan, np.nan, np.nan])
            size = obs.get("size", [0, 0, 0])
            out.append({
                "obs_x": c[0], "obs_y": c[1], "obs_z": c[2],
                "size_x": size[0],
                "size_y": size[1],
                "size_z": size[2],
                "orientation": obs.get("orientation", 0.0),
                "point_count": obs.get("point_count", 0)
            })
        return out
    except Exception as e:
        print("Error parsing obstacles:", e)
        return []


def parse_sysdiag_json(s):
    if not isinstance(s, str):
        return None
    try:
        data = json.loads(s)
        return {
            "machine": data.get("machine", "unknown"),
            "cpu": data.get("cpu", np.nan),
            "memory": data.get("memory", np.nan)
        }
    except Exception as e:
        print("Error parsing system diagnostics:", e)
        return None


# ---------------------------
# Obstacle Filtering Helpers
# ---------------------------
def filter_unique_obstacles(df_obs, distance_threshold):
    if df_obs.empty or "time_s" not in df_obs.columns:
        return df_obs
    df = df_obs.sort_values("time_s").copy()
    keep = []
    for idx, row in df.iterrows():
        cx, cy = row["obs_x"], row["obs_y"]
        area = row["size_x"] * row["size_y"]
        if area < MIN_AREA:
            continue
        remove = False
        for kept in keep:
            dist = math.hypot(cx - kept["obs_x"], cy - kept["obs_y"])
            if dist < distance_threshold:
                remove = True
                if row["time_s"] > kept["time_s"]:
                    keep.remove(kept)
                    keep.append(row.to_dict())
                break
        if not remove:
            keep.append(row.to_dict())
    if keep:
        return pd.DataFrame(keep).sort_values("time_s").reset_index(drop=True)
    else:
        return pd.DataFrame()


# ---------------------------
# Geometry and Obstacle Plot Helpers
# ---------------------------
def get_rotated_rect_path(cx, cy, width, height, angle):
    hw, hh = width / 2.0, height / 2.0
    corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
    rotated = []
    for x, y in corners:
        rx = x * math.cos(angle) - y * math.sin(angle) + cx
        ry = x * math.sin(angle) + y * math.cos(angle) + cy
        rotated.append((rx, ry))
    return rotated


def rotated_rect_and_gradient_square_2d(obs_x, obs_y, size_x, size_y, orientation_rads, point_count, layers=1):
    red_coords = get_rotated_rect_path(obs_x, obs_y, size_x, size_y, orientation_rads)
    yellow_coords = get_rotated_rect_path(obs_x, obs_y, size_x + 4, size_y + 4, orientation_rads)
    return red_coords, yellow_coords


def oriented_box_3d(obs_x, obs_y, obs_z, size_x, size_y, size_z, orientation_rads):
    theta = orientation_rads
    half_x, half_y, half_z = size_x / 2, size_y / 2, size_z / 2
    corners_local = np.array([
        [-half_x, -half_y, -half_z],
        [half_x, -half_y, -half_z],
        [half_x, half_y, -half_z],
        [-half_x, half_y, -half_z],
        [-half_x, -half_y, half_z],
        [half_x, -half_y, half_z],
        [half_x, half_y, half_z],
        [-half_x, half_y, half_z]
    ])
    cosT, sinT = math.cos(theta), math.sin(theta)
    Rz = np.array([[cosT, -sinT, 0],
                   [sinT, cosT, 0],
                   [0, 0, 1]])
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


# ---------------------------
# Plotting Functions
# ---------------------------
def plot_birdseye(df, obstacles_df, title, out_folder):
    plt.figure(figsize=(8, 8))
    df_plot = df.copy()
    lost_coords = []
    # Mark lost odometry: when pose resets to [0,0] (use previous valid pose)
    for idx in range(1, len(df_plot)):
        if df_plot.at[df_plot.index[idx], "x"] == 0 and df_plot.at[df_plot.index[idx], "y"] == 0:
            lost_coords.append((df_plot.at[df_plot.index[idx - 1], "x"], df_plot.at[df_plot.index[idx - 1], "y"]))
            df_plot.at[df_plot.index[idx], "x"] = np.nan
            df_plot.at[df_plot.index[idx], "y"] = np.nan
    plt.plot(df_plot["x"], df_plot["y"], 'b-', linewidth=1)
    plt.xlim(ROOM_BOUNDS)
    plt.ylim(ROOM_BOUNDS)
    plt.gca().set_aspect('equal', adjustable='box')
    for (lx, ly) in lost_coords:
        plt.plot(lx, ly, 'p', color='red', markersize=10)
    # Draw obstacles using persistent obstacles data
    for idx, obs in obstacles_df.iterrows():
        try:
            red_coords, yellow_coords = rotated_rect_and_gradient_square_2d(
                obs["obs_x"], obs["obs_y"], obs["size_x"], obs["size_y"],
                obs["orientation"], obs["point_count"], layers=1)
            red_poly = patches.Polygon(red_coords, closed=True, facecolor="red", edgecolor="red", alpha=0.3)
            plt.gca().add_patch(red_poly)
            yellow_poly = patches.Polygon(yellow_coords, closed=True, facecolor="yellow", edgecolor="yellow",
                                          alpha=0.04 / 5)
            plt.gca().add_patch(yellow_poly)
        except Exception as e:
            print(f"Error plotting obstacle {idx}: {e}")
    # Plot goal marker
    goal_marker = patches.Circle((GOAL_AREA["x"] + GOAL_AREA["width"] / 2, GOAL_AREA["y"] + GOAL_AREA["height"] / 2),
                                 radius=0.2, color="purple")
    plt.gca().add_patch(goal_marker)
    plt.title(title)
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.grid(True)
    fname = os.path.join(out_folder, title.replace(" ", "_") + "_birdseye.png")
    plt.savefig(fname)
    plt.close()
    print(f"Saved birdseye plot: {fname}")


def plot_heading_error_with_distance(df, title, out_folder):
    lost_times = []
    regained_times = []
    prev_lost = False
    goal_cx = GOAL_AREA["x"] + GOAL_AREA["width"] / 2
    goal_cy = GOAL_AREA["y"] + GOAL_AREA["height"] / 2
    goal_dist = []
    times = df["time_s"].values
    for i in range(1, len(df)):
        lost = (df.at[df.index[i], "x"] == 0 and df.at[df.index[i], "y"] == 0)
        if lost and not prev_lost:
            lost_times.append(df.at[df.index[i - 1], "time_s"])
        if prev_lost and not lost:
            regained_times.append(df.at[df.index[i], "time_s"])
        prev_lost = lost
        x, y = df.at[df.index[i], "x"], df.at[df.index[i], "y"]
        goal_dist.append(math.hypot(x - goal_cx, y - goal_cy))
    he = df["heading_error"].iloc[1:]
    fig, ax1 = plt.subplots(figsize=(10, 5))
    ax1.plot(df["time_s"].iloc[1:], he, 'b-', label="Heading Error (rad)")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Heading Error (rad)", color='b')
    ax1.tick_params(axis='y', labelcolor='b')
    ax2 = ax1.twinx()
    if "min_distance" in df.columns:
        ax2.plot(df["time_s"].iloc[1:], df["min_distance"].iloc[1:], 'g-', label="Min Distance (m)")
        ax2.set_ylabel("Min Distance (m)", color='g')
        ax2.tick_params(axis='y', labelcolor='g')
    ax1.plot(lost_times, np.interp(lost_times, df["time_s"], df["heading_error"]), 'r*', markersize=12,
             label="Odometry Lost")
    ax1.plot(regained_times, np.interp(regained_times, df["time_s"], df["heading_error"]), 'ms', markersize=12,
             label="Odometry Regained")
    ax1.plot(df["time_s"].iloc[1:], goal_dist, 'k-', label="Distance from Goal (m)")
    fig.legend(loc='upper right')
    plt.title(title)
    plt.grid(True)
    fname = os.path.join(out_folder, title.replace(" ", "_") + "_heading_error_overlay.png")
    plt.savefig(fname)
    plt.close()
    print(f"Saved heading error overlay plot: {fname}")


def plot_command_heading_error(arduino_df, nav_df, title, out_folder):
    title = re.sub(r"Unknown", "", title, flags=re.IGNORECASE).strip()
    if "command" not in arduino_df.columns and "data" in arduino_df.columns:
        arduino_df = arduino_df.rename(columns={"data": "command"})
    if arduino_df.empty or nav_df.empty:
        print("Insufficient data for command-heading error plot.")
        return
    merged = pd.merge_asof(arduino_df.sort_values("time_s"), nav_df.sort_values("time_s"),
                           on="time_s", direction="nearest")
    try:
        merged["heading_error_deg"] = -merged["heading_error"] * (180 / math.pi)  # negative as required
        merged["command_angle"] = merged["command"].str.lower().map(COMMAND_MAP)
        merged["diff"] = np.abs(merged["heading_error_deg"] - merged["command_angle"])
    except Exception as e:
        print("Error mapping commands to angles:", e)
        return
    plt.figure(figsize=(10, 5))
    plt.plot(merged["time_s"], merged["heading_error_deg"], label="Negative Heading Error (deg)", marker='o',
             markersize=2)
    plt.plot(merged["time_s"], merged["command_angle"], label="Command Angle (deg)", marker='x', markersize=2)
    plt.plot(merged["time_s"], merged["diff"], label="Absolute Difference (deg)", marker='s', markersize=2)
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.title(title + " - Command vs Heading Error")
    plt.legend()
    plt.grid(True)
    fname = os.path.join(out_folder, title.replace(" ", "_") + "_command_heading_error.png")
    plt.savefig(fname)
    plt.close()
    print(f"Saved command-heading error plot: {fname}")


def create_2d_animation_frames(df_anim, obstacles_df, force_scale=0.025):
    # Use the obstacles_df (already extracted from obstacle_info.csv)
    global_obstacles = filter_unique_obstacles(obstacles_df, CELL_SIZE_ANIM)
    df_anim["time_s_round"] = df_anim["time_s"].round(1)
    unique_times = np.sort(df_anim["time_s_round"].unique())
    frames = []
    for t in unique_times:
        frame_data = df_anim[df_anim["time_s_round"] == t]
        if frame_data.empty:
            continue
        up_to_now = df_anim[df_anim["time_s_round"] <= t]
        path_xy = up_to_now[["x", "y"]].dropna().drop_duplicates()
        row_i = frame_data.iloc[-1]
        x, y = row_i["x"], row_i["y"]
        fx_total = row_i.get("total_fx", 0.0) * force_scale
        fy_total = row_i.get("total_fy", 0.0) * force_scale
        fx_attr = row_i.get("attr_fx", 0.0) * force_scale
        fy_attr = row_i.get("attr_fy", 0.0) * force_scale
        fx_rep = row_i.get("rep_fx", 0.0) * force_scale
        fy_rep = row_i.get("rep_fy", 0.0) * force_scale
        trace_total = go.Scatter(x=[x, x + fx_total], y=[y, y + fy_total],
                                 mode="lines+markers", name="Total Force",
                                 line=dict(color="red"), marker=dict(size=5))
        trace_attr = go.Scatter(x=[x, x + fx_attr], y=[y, y + fy_attr],
                                mode="lines+markers", name="Attractive Force",
                                line=dict(color="green"), marker=dict(size=5))
        trace_rep = go.Scatter(x=[x, x + fx_rep], y=[y, y + fy_rep],
                               mode="lines+markers", name="Repulsive Force",
                               line=dict(color="orange"), marker=dict(size=5))
        yaw_imu = row_i.get("yaw_imu", np.nan)
        if not pd.isna(yaw_imu):
            heading_len = 0.5
            x2 = x + heading_len * math.cos(yaw_imu)
            y2 = y + heading_len * math.sin(yaw_imu)
            trace_imu = go.Scatter(x=[x, x2], y=[y, y2],
                                   mode="lines+markers", name="IMU Heading",
                                   line=dict(color="magenta"), marker=dict(size=6))
        else:
            trace_imu = go.Scatter(x=[], y=[], mode="lines+markers", name="IMU Heading")
        shapes = []
        for idx, obs in global_obstacles.iterrows():
            try:
                red_coords, yellow_coords = rotated_rect_and_gradient_square_2d(
                    obs["obs_x"], obs["obs_y"], obs["size_x"], obs["size_y"],
                    obs["orientation"], obs["point_count"], layers=1)
                red_path_str = "M " + " L ".join(f"{pt[0]},{pt[1]}" for pt in red_coords) + " Z"
                yellow_path_str = "M " + " L ".join(f"{pt[0]},{pt[1]}" for pt in yellow_coords) + " Z"
                shapes.append(dict(
                    type="path",
                    path=red_path_str,
                    xref="x", yref="y",
                    fillcolor="red",
                    opacity=0.3,
                    line=dict(color="red")
                ))
                shapes.append(dict(
                    type="path",
                    path=yellow_path_str,
                    xref="x", yref="y",
                    fillcolor="yellow",
                    opacity=0.04 / 5,
                    line=dict(color="yellow")
                ))
            except Exception as e:
                print(f"Error processing obstacle in global filter at time {t}: {e}")
        # Add fixed room boundary as a rectangle (using a transparent fill)
        room_rect = dict(
            type="rect",
            x0=ROOM_BOUNDS[0], y0=ROOM_BOUNDS[0],
            x1=ROOM_BOUNDS[1], y1=ROOM_BOUNDS[1],
            line=dict(color="black", dash="dash"),
            fillcolor="rgba(0,0,0,0)"
        )
        shapes.append(room_rect)
        frame = go.Frame(data=[
            go.Scatter(x=path_xy["x"], y=path_xy["y"], mode="lines+markers", name="Path", line=dict(color="blue"),
                       marker=dict(size=5)),
            trace_imu, trace_total, trace_attr, trace_rep
        ], layout=go.Layout(shapes=shapes))
        frames.append((t, frame))
    return frames


def save_2d_animation(frames_with_time, init_data, init_shapes, title, out_folder):
    times = [t for t, frame in frames_with_time]
    frames = [frame for t, frame in frames_with_time]
    replicated_frames = []
    for i, frame in enumerate(frames):
        dt = times[i] - (times[i - 1] if i > 0 else times[i])
        dt = max(dt, 1.0)
        repeat_count = int(round(dt * REALTIME_FPS))
        for _ in range(repeat_count):
            replicated_frames.append(frame)
    images = []
    for frame in replicated_frames:
        temp_fig = go.Figure(data=frame.data, layout=frame.layout)
        try:
            img_bytes = temp_fig.to_image(format="png", scale=3)
            img_arr = imageio.imread(io.BytesIO(img_bytes))
            if img_arr.ndim >= 2 and img_arr.shape[0] > 1 and img_arr.shape[1] > 1:
                images.append(img_arr)
            else:
                print("Generated image has insufficient dimensions, skipping frame.")
        except Exception as e:
            print(f"Error converting frame to image: {e}")
    if not images:
        print("No valid images generated for 2D animation.")
        return
    video_fname = os.path.join(out_folder, title.replace(" ", "_") + "_HD_animation.mp4")
    imageio.mimwrite(video_fname, images, fps=REALTIME_FPS)
    print(f"Saved HD 2D animation as {video_fname}")


def create_and_show_3d_animation(df, map_pc_df, obs_pc_df, obstacles_df):
    df_filtered = df[df["z"] <= CEILING_Z]
    trace_path = go.Scatter3d(x=df_filtered["x"], y=df_filtered["y"], z=df_filtered["z"],
                              mode="lines+markers", name="3D Path", marker=dict(size=4))
    trace_map = go.Scatter3d(x=map_pc_df["mx"], y=map_pc_df["my"], z=map_pc_df["mz"],
                             mode="markers", name="Map Env", marker=dict(size=1, color="gray", opacity=0.4))
    trace_obs = go.Scatter3d(x=obs_pc_df["mx"], y=obs_pc_df["my"], z=obs_pc_df["mz"],
                             mode="markers", name="Obst Env", marker=dict(size=2, color="red", opacity=0.5))
    obstacles_final = filter_unique_obstacles(obstacles_df, CELL_SIZE_BIRD)
    obstacle_meshes = []
    for idx, obs in obstacles_final.iterrows():
        try:
            box_3d = oriented_box_3d(obs["obs_x"], obs["obs_y"], obs["obs_z"],
                                     obs["size_x"], obs["size_y"], obs["size_z"],
                                     obs["orientation"])
            obstacle_meshes.append(box_3d)
        except Exception as e:
            print(f"Error creating 3D box for obstacle {idx}: {e}")
    data = [trace_map, trace_obs, trace_path] + obstacle_meshes
    camera = dict(eye=dict(x=10, y=0, z=10))
    layout = go.Layout(title="3D Interactive View",
                       scene=dict(camera=camera, xaxis_title="X", yaxis_title="Y", zaxis_title="Z"))
    fig = go.Figure(data=data, layout=layout)
    fig.show()


def plot_apf_forces(df, title, out_folder):
    df["total_fmag"] = np.sqrt(df["total_fx"] ** 2 + df["total_fy"] ** 2)
    df["total_fmag"] = np.clip(df["total_fmag"], None, 10)
    he = df["heading_error"]
    fig, ax1 = plt.subplots(figsize=(8, 4))
    ax1.plot(df["time_s"], df["total_fmag"], 'r-', label="Total Force")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Force Magnitude", color='r')
    ax1.tick_params(axis='y', labelcolor='r')
    ax2 = ax1.twinx()
    ax2.plot(df["time_s"], he, 'b-', label="Heading Error (rad)")
    ax2.set_ylabel("Heading Error (rad)", color='b')
    ax2.tick_params(axis='y', labelcolor='b')
    fig.legend(loc='upper right')
    plt.title(title + " - APF Metrics")
    plt.grid(True)
    fname = os.path.join(out_folder, title.replace(" ", "_") + "_apf_metrics.png")
    plt.savefig(fname)
    plt.close()
    print(f"Saved APF metrics plot: {fname}")


def plot_diagnostics(df, title, out_folder):
    if {"cpu", "memory", "machine"}.issubset(df.columns):
        machines = df["machine"].unique()
        plt.figure(figsize=(8, 4))
        for m in machines:
            sub = df[df["machine"] == m]
            plt.plot(sub["time_s"], sub["cpu"], label=f"CPU {m}")
        plt.title(title + " - CPU Usage")
        plt.xlabel("Time (s)")
        plt.ylabel("CPU (%)")
        plt.legend()
        plt.grid(True)
        fname_cpu = os.path.join(out_folder, title.replace(" ", "_") + "_cpu.png")
        plt.savefig(fname_cpu)
        plt.close()
        plt.figure(figsize=(8, 4))
        for m in machines:
            sub = df[df["machine"] == m]
            plt.plot(sub["time_s"], sub["memory"], label=f"Memory {m}")
        plt.title(title + " - Memory Usage")
        plt.xlabel("Time (s)")
        plt.ylabel("Memory (%)")
        plt.legend()
        plt.grid(True)
        fname_mem = os.path.join(out_folder, title.replace(" ", "_") + "_memory.png")
        plt.savefig(fname_mem)
        plt.close()
        print(f"Saved diagnostics plots: {fname_cpu} and {fname_mem}")
    else:
        print("Diagnostics data insufficient for plotting.")


def plot_localisation_loss_heatmap_global(lost_coords, title, out_folder):
    if not lost_coords:
        print("No lost localisation events for global heatmap.")
        return
    xs, ys = zip(*lost_coords)
    heatmap, xedges, yedges = np.histogram2d(xs, ys, bins=50, range=[[-5, 5], [-5, 5]])
    extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]
    plt.figure(figsize=(6, 6))
    plt.imshow(heatmap.T, extent=extent, origin='lower', cmap='hot_r', aspect='equal')
    plt.colorbar(label="Count")
    plt.title(title + " - Global Localisation Loss Heatmap")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    fname = os.path.join(out_folder, title.replace(" ", "_") + "_global_loss_heatmap.png")
    plt.savefig(fname)
    plt.close()
    print(f"Saved global localisation loss heatmap: {fname}")


def plot_processing_time_per_command(arduino_df, nav_df, title, out_folder):
    if "command" not in arduino_df.columns and "data" in arduino_df.columns:
        arduino_df = arduino_df.rename(columns={"data": "command"})
    if arduino_df.empty or nav_df.empty:
        print("Insufficient data for processing time per command plot.")
        return
    merged = pd.merge_asof(arduino_df.sort_values("time_s"), nav_df.sort_values("time_s"),
                           on="time_s", direction="nearest")
    fig, ax1 = plt.subplots(figsize=(10, 5))
    ax1.plot(merged["time_s"], merged["processing_time"], 'o-', markersize=3, label="Processing Time (ms)")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Processing Time (ms)", color='blue')
    ax1.tick_params(axis='y', labelcolor='blue')
    ax2 = ax1.twinx()
    if "min_distance" in merged.columns:
        ax2.plot(merged["time_s"], merged["min_distance"], 's-', markersize=3, color="green", label="Min Distance (m)")
        ax2.set_ylabel("Min Distance (m)", color='green')
        ax2.tick_params(axis='y', labelcolor='green')
    fig.legend(loc='upper right')
    plt.title(title + " - Processing Time per Command")
    plt.grid(True)
    fname = os.path.join(out_folder, title.replace(" ", "_") + "_proc_time.png")
    plt.savefig(fname)
    plt.close()
    print(f"Saved processing time per command plot: {fname}")


# ---------------------------
# Data Loading & Merging
# ---------------------------
def load_data(csv_folder):
    filenames = ["navigation_perf.csv", "obstacle_info.csv", "odom_data.csv", "imu_data.csv",
                 "arduino_commands.csv", "system_diagnostics.csv", "rtabmap_info.csv",
                 "cloud_map.csv", "cloud_obstacles.csv"]
    data = {}
    for fname in filenames:
        path = os.path.join(csv_folder, fname)
        try:
            df = pd.read_csv(path, on_bad_lines="skip")
            df = convert_time(df, "Time")
            data[fname] = df
        except Exception as e:
            print(f"Error loading {fname}: {e}")
            data[fname] = pd.DataFrame()
    return data


def merge_data(data):
    df_odom = data.get("odom_data.csv", pd.DataFrame()).copy()
    if not df_odom.empty:
        df_odom.rename(columns={
            "pose.pose.position.x": "x",
            "pose.pose.position.y": "y",
            "pose.pose.position.z": "z"
        }, inplace=True)
        df_odom.sort_values("time_s", inplace=True)
    df_nav = data.get("navigation_perf.csv", pd.DataFrame()).copy()
    df_nav.sort_values("time_s", inplace=True)
    df_merged = pd.merge_asof(df_nav, df_odom[["time_s", "x", "y", "z"]], on="time_s", direction="nearest")
    nav_parsed = df_merged["data"].apply(parse_nav_json)
    df_merged["heading_error"] = nav_parsed.apply(lambda d: d["heading_error"] if d else np.nan)
    df_merged["processing_time"] = nav_parsed.apply(lambda d: d["processing_time"] if d else np.nan)
    df_merged["total_fx"] = nav_parsed.apply(lambda d: d["total_fx"] if d else np.nan)
    df_merged["total_fy"] = nav_parsed.apply(lambda d: d["total_fy"] if d else np.nan)
    df_merged["attr_fx"] = nav_parsed.apply(lambda d: d["attr_fx"] if d else np.nan)
    df_merged["attr_fy"] = nav_parsed.apply(lambda d: d["attr_fy"] if d else np.nan)
    df_merged["rep_fx"] = nav_parsed.apply(lambda d: d["rep_fx"] if d else np.nan)
    df_merged["rep_fy"] = nav_parsed.apply(lambda d: d["rep_fy"] if d else np.nan)
    return df_merged


def extract_obstacles(data):
    df_obs = data.get("obstacle_info.csv", pd.DataFrame()).copy()
    obs_rows = []
    if not df_obs.empty and "data" in df_obs.columns:
        for idx, row in df_obs.iterrows():
            t_s = row.get("time_s", np.nan)
            try:
                obs_list = parse_obstacles(row["data"])
                for obs in obs_list:
                    obs["time_s"] = t_s
                    obs_rows.append(obs)
            except Exception as e:
                print("Error parsing obstacle row:", e)
    if obs_rows:
        return pd.DataFrame(obs_rows).sort_values("time_s").reset_index(drop=True)
    else:
        return pd.DataFrame()


# ---------------------------
# Main Processing Function per Bag File
# ---------------------------
def process_bag_file(bag_file, main_output_dir):
    base_name = os.path.splitext(os.path.basename(bag_file))[0]
    mapped_name = base_name
    for key, val in NAME_MAP.items():
        mapped_name = re.sub(key, val, mapped_name, flags=re.IGNORECASE)
    system_type = "Pulley" if "pulley" in base_name.lower() else "Clock" if "clock" in base_name.lower() else ""
    overall_title = f"{mapped_name} {system_type}".strip()
    print(f"\n[PROCESSING] {bag_file} as {overall_title}...")

    csv_out_folder = os.path.join(main_output_dir, OUTPUT_FOLDERS["csv_data"], base_name)
    os.makedirs(csv_out_folder, exist_ok=True)

    write_csvs_from_bag(bag_file, csv_out_folder)

    nav_csv_path = os.path.join(csv_out_folder, "navigation_perf.csv")
    if not os.path.exists(nav_csv_path):
        print(f"Skipping {bag_file}: Required CSV files were not generated (possibly unindexed bag).")
        return

    data = load_data(csv_out_folder)
    if data.get("navigation_perf.csv", pd.DataFrame()).empty:
        print(f"Skipping {bag_file}: navigation_perf.csv is empty or missing 'time_s'.")
        return

    df_merged = merge_data(data)
    df_obs = extract_obstacles(data)
    obs_for_birdseye = filter_unique_obstacles(df_obs, CELL_SIZE_BIRD)
    obs_for_anim = filter_unique_obstacles(df_obs, CELL_SIZE_ANIM)

    merged_csv_path = os.path.join(csv_out_folder, overall_title.replace(" ", "_") + "_merged.csv")
    df_merged.to_csv(merged_csv_path, index=False)
    print(f"Saved merged CSV data: {merged_csv_path}")

    # Birdseye plot
    plot_birdseye(df_merged, obs_for_birdseye, overall_title + " Birdseye Plot",
                  os.path.join(main_output_dir, OUTPUT_FOLDERS["birdseye"]))

    # Compute minimum distance
    if not df_obs.empty:
        def compute_min(row):
            dists = np.hypot(df_obs["obs_x"] - row["x"], df_obs["obs_y"] - row["y"])
            return dists.min() if len(dists) > 0 else np.nan

        df_merged["min_distance"] = df_merged.apply(compute_min, axis=1)
    plot_heading_error_with_distance(df_merged, overall_title + " Heading Error and Min Distance",
                                     os.path.join(main_output_dir, OUTPUT_FOLDERS["heading_error"]))

    # Arduino command-heading error plot
    arduino_df = data.get("arduino_commands.csv", pd.DataFrame())
    if not arduino_df.empty:
        if "command" not in arduino_df.columns and "data" in arduino_df.columns:
            arduino_df = arduino_df.rename(columns={"data": "command"})
        plot_command_heading_error(arduino_df, df_merged, overall_title + " Command vs Heading Error",
                                   os.path.join(main_output_dir, OUTPUT_FOLDERS["arduino"]))
    else:
        print("No Arduino command data available.")

    # 2D Animation – use obs_for_anim for obstacles
    df_anim = pd.merge_asof(extract_obstacles(data).sort_values("time_s"), df_merged.sort_values("time_s"),
                            on="time_s", direction="nearest")
    if not df_anim.empty:
        frames_with_time = create_2d_animation_frames(df_anim, obs_for_anim, force_scale=0.025)
        if frames_with_time:
            init_data_2d = frames_with_time[0][1].data
            init_shapes_2d = frames_with_time[0][1].layout.shapes
            save_2d_animation(frames_with_time, init_data_2d, init_shapes_2d,
                              overall_title + " 2D Animated Birdseye",
                              os.path.join(main_output_dir, OUTPUT_FOLDERS["2d_anim"]))
        else:
            print("No frames generated for 2D animation.")
    else:
        print("Insufficient data for 2D animation.")

    # 3D Interactive View
    if not data.get("cloud_map.csv", pd.DataFrame()).empty and not data.get("cloud_obstacles.csv",
                                                                            pd.DataFrame()).empty:
        try:
            df_map_parsed = decode_pointcloud2_csv(data.get("cloud_map.csv", pd.DataFrame()).iloc[0])
            df_obs_parsed = decode_pointcloud2_csv(data.get("cloud_obstacles.csv", pd.DataFrame()).iloc[0])
        except Exception as e:
            print("Error decoding 3D point clouds:", e)
            df_map_parsed = pd.DataFrame()
            df_obs_parsed = pd.DataFrame()
        if not df_map_parsed.empty and not df_obs_parsed.empty:
            create_and_show_3d_animation(df_merged, df_map_parsed, df_obs_parsed, obs_for_birdseye)
        else:
            print("Insufficient 3D point cloud data for 3D animation.")
    else:
        print("3D point cloud CSVs not available.")

    # APF Plots
    plot_apf_forces(df_merged, overall_title + " APF Metrics",
                    os.path.join(main_output_dir, OUTPUT_FOLDERS["apf"]))

    # Diagnostics Plots
    diag_df = data.get("system_diagnostics.csv", pd.DataFrame()).copy()
    if not diag_df.empty:
        sd_parsed = diag_df["data"].apply(parse_sysdiag_json)
        diag_df["machine"] = sd_parsed.apply(lambda d: d["machine"] if d else "unknown")
        diag_df["cpu"] = sd_parsed.apply(lambda d: d["cpu"] if d else np.nan)
        diag_df["memory"] = sd_parsed.apply(lambda d: d["memory"] if d else np.nan)
        plot_diagnostics(diag_df, overall_title + " System Diagnostics",
                         os.path.join(main_output_dir, OUTPUT_FOLDERS["diagnostics"]))
    else:
        print("No system diagnostics data available.")

    # Processing Time per Command Plot
    plot_processing_time_per_command(arduino_df, df_merged, overall_title + " Processing Time per Command",
                                     os.path.join(main_output_dir, OUTPUT_FOLDERS["arduino_proc"]))

    # Accumulate lost localisation events for global heatmap
    for idx in range(1, len(df_merged)):
        if df_merged.at[df_merged.index[idx], "x"] == 0 and df_merged.at[df_merged.index[idx], "y"] == 0:
            global_lost_coords.append((df_merged.at[df_merged.index[idx - 1], "x"],
                                       df_merged.at[df_merged.index[idx - 1], "y"]))


# ---------------------------
# Main Entry Point
# ---------------------------
def main():
    parser = argparse.ArgumentParser(description="Generate comprehensive visuals for Mechatronics report")
    parser.add_argument("--output_dir", default="results", help="Directory to store all outputs")
    args = parser.parse_args()

    bag_folder = os.path.dirname(os.path.abspath(__file__))
    print(f"Processing bag files from folder: {bag_folder}")

    for folder in OUTPUT_FOLDERS.values():
        os.makedirs(os.path.join(args.output_dir, folder), exist_ok=True)

    bag_files = [os.path.join(bag_folder, f) for f in os.listdir(bag_folder) if f.endswith(".bag")]
    if not bag_files:
        print("No .bag files found in the script's folder.")
        return
    for bag_file in bag_files:
        process_bag_file(bag_file, args.output_dir)

    # Global localisation loss heatmap (accumulated lost localisation events)
    plot_localisation_loss_heatmap_global(global_lost_coords, "Global Localisation Loss",
                                          os.path.join(args.output_dir, OUTPUT_FOLDERS["heatmap"]))

    print("\nAll processing complete.")


if __name__ == "__main__":
    main()
