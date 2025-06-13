import os

# Create the evaluation script file in /mnt/data
script_content = r'''#!/usr/bin/env python3
"""
evaluate_ekf_comparison.py

Offline evaluation script to compare:
  - Custom EKF path:    topic /ekf/path
  - robot_localization path: topic /robot_localization/path
  - Ground truth path: topic /ground_truth_path

Usage:
  1. Record a rosbag containing at least:
       /ground_truth_path
       /ekf/path
       /robot_localization/path
     e.g.:
       rosbag record /ground_truth_path /ekf/path /robot_localization/path
  2. Run simulation/experiment.
  3. Run this evaluation script:
       chmod +x evaluate_ekf_comparison.py
       ./evaluate_ekf_comparison.py /path/to/bagfile.bag
  4. It will produce metrics and matplotlib plots saved in the current directory:
       - trajectories.png
       - error_over_time.png
       - error_histogram.png
       - yaw_error_over_time.png
       - yaw_error_histogram.png
     Additionally, it prints summary RMSE, mean, max errors.

Script logic:
  - Read Path messages from each topic.
  - For each Path msg, extract the final PoseStamped (latest pose) at that timestamp.
  - Build time-series dataframes (timestamp, x, y, z, yaw) for each method.
  - Align times: for each ground-truth timestamp, find nearest estimate timestamp within tolerance.
  - Compute position error (Euclidean) and yaw error (angle difference).
  - Compute metrics (RMSE, mean, max) for each estimate vs ground truth.
  - Plot:
      * Trajectories overlay in XY plane.
      * Error vs time for both methods.
      * Histogram of position errors.
      * Yaw error vs time and histogram.
"""

import sys
import rosbag
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf
import math

def extract_path_series(bag, topic_name):
    """
    Extract time-series of final pose from Path messages on topic_name.
    Returns a pandas DataFrame with columns: ['time', 'x', 'y', 'z', 'yaw'].
    'time' is float seconds (ros time).
    """
    times = []
    xs = []
    ys = []
    zs = []
    yaws = []
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        # msg is nav_msgs/Path
        if not msg.poses:
            continue
        # Take the last pose in the path
        ps: PoseStamped = msg.poses[-1]
        # Timestamp: use ps.header.stamp (ros.Time)
        ts = ps.header.stamp.to_sec()
        x = ps.pose.position.x
        y = ps.pose.position.y
        z = ps.pose.position.z
        # Extract yaw from quaternion
        q = ps.pose.orientation
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = euler[2]
        times.append(ts)
        xs.append(x)
        ys.append(y)
        zs.append(z)
        yaws.append(yaw)
    if not times:
        print(f"Warning: no messages found on topic {topic_name}")
        return pd.DataFrame(columns=['time','x','y','z','yaw'])
    df = pd.DataFrame({
        'time': times,
        'x': xs,
        'y': ys,
        'z': zs,
        'yaw': yaws
    })
    # Sort by time
    df = df.sort_values('time').reset_index(drop=True)
    # Remove duplicates: keep first occurrence at each unique time
    df = df.drop_duplicates(subset=['time'])
    return df

def angle_diff(a, b):
    """Compute smallest difference between angles a and b (radians)."""
    d = a - b
    return (d + np.pi) % (2 * np.pi) - np.pi

def align_and_compute_errors(df_gt, df_est, tol=0.05):
    """
    Align estimate df_est to ground truth df_gt by nearest time.
    tol: maximum time difference (seconds) to consider a valid match.
    Returns DataFrame with columns: ['time', 'gt_x','gt_y','gt_z','gt_yaw',
                                    'est_x','est_y','est_z','est_yaw',
                                    'pos_error','yaw_error'].
    Only rows where time difference <= tol are kept.
    """
    # Use pandas.merge_asof: left=gt, right=est
    # Requires sorted by time
    df_gt_sorted = df_gt.sort_values('time').reset_index(drop=True)
    df_est_sorted = df_est.sort_values('time').reset_index(drop=True)
    merged = pd.merge_asof(df_gt_sorted, df_est_sorted,
                           on='time',
                           direction='nearest',
                           tolerance=tol,
                           suffixes=('_gt','_est'))
    # Drop rows where no match found (NaNs in est columns)
    merged = merged.dropna(subset=['x_est','y_est'])
    # Rename columns for clarity
    merged = merged.rename(columns={
        'x_gt':'gt_x', 'y_gt':'gt_y', 'z_gt':'gt_z', 'yaw_gt':'gt_yaw',
        'x_est':'est_x','y_est':'est_y','z_est':'est_z','yaw_est':'est_yaw'
    })
    # Compute errors
    merged['pos_error'] = np.sqrt((merged['gt_x'] - merged['est_x'])**2 +
                                  (merged['gt_y'] - merged['est_y'])**2 +
                                  (merged['gt_z'] - merged['est_z'])**2)
    merged['yaw_error'] = merged.apply(lambda row: abs(angle_diff(row['gt_yaw'], row['est_yaw'])), axis=1)
    # Add time column
    merged['time'] = merged['time']
    return merged

def compute_metrics(df_err):
    """
    Compute RMSE, mean, max for pos_error and yaw_error.
    Returns dict.
    """
    metrics = {}
    if df_err.empty:
        metrics['pos_rmse'] = np.nan
        metrics['pos_mean'] = np.nan
        metrics['pos_max'] = np.nan
        metrics['yaw_rmse'] = np.nan
        metrics['yaw_mean'] = np.nan
        metrics['yaw_max'] = np.nan
        return metrics
    pos_err = df_err['pos_error'].values
    yaw_err = df_err['yaw_error'].values
    metrics['pos_rmse'] = np.sqrt(np.mean(pos_err**2))
    metrics['pos_mean'] = np.mean(pos_err)
    metrics['pos_max'] = np.max(pos_err)
    metrics['yaw_rmse'] = np.sqrt(np.mean(yaw_err**2))
    metrics['yaw_mean'] = np.mean(yaw_err)
    metrics['yaw_max'] = np.max(yaw_err)
    return metrics

def plot_trajectories(df_gt, df_custom, df_rl):
    """
    Plot XY trajectories overlayed.
    """
    plt.figure(figsize=(8,6))
    plt.plot(df_gt['x'], df_gt['y'], label='Ground Truth', linewidth=2)
    plt.plot(df_custom['x'], df_custom['y'], label='Custom EKF', linestyle='--')
    plt.plot(df_rl['x'], df_rl['y'], label='robot_localization EKF', linestyle='-.')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Trajectories Comparison')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('trajectories.png')
    plt.close()

def plot_error_over_time(df_err_custom, df_err_rl):
    """
    Plot position error vs time, yaw error vs time for both methods.
    """
    plt.figure(figsize=(8,6))
    plt.plot(df_err_custom['time'], df_err_custom['pos_error'], label='Custom EKF Pos Error')
    plt.plot(df_err_rl['time'], df_err_rl['pos_error'], label='robot_localization Pos Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Position Error (m)')
    plt.title('Position Error Over Time')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('pos_error_over_time.png')
    plt.close()

    plt.figure(figsize=(8,6))
    plt.plot(df_err_custom['time'], df_err_custom['yaw_error'], label='Custom EKF Yaw Error')
    plt.plot(df_err_rl['time'], df_err_rl['yaw_error'], label='robot_localization Yaw Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw Error (rad)')
    plt.title('Yaw Error Over Time')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('yaw_error_over_time.png')
    plt.close()

def plot_error_histogram(df_err_custom, df_err_rl):
    """
    Plot histogram of position errors and yaw errors.
    """
    plt.figure(figsize=(8,6))
    plt.hist(df_err_custom['pos_error'], bins=30, alpha=0.6, label='Custom EKF')
    plt.hist(df_err_rl['pos_error'], bins=30, alpha=0.6, label='robot_localization EKF')
    plt.xlabel('Position Error (m)')
    plt.ylabel('Frequency')
    plt.title('Position Error Histogram')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('pos_error_histogram.png')
    plt.close()

    plt.figure(figsize=(8,6))
    plt.hist(df_err_custom['yaw_error'], bins=30, alpha=0.6, label='Custom EKF')
    plt.hist(df_err_rl['yaw_error'], bins=30, alpha=0.6, label='robot_localization EKF')
    plt.xlabel('Yaw Error (rad)')
    plt.ylabel('Frequency')
    plt.title('Yaw Error Histogram')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('yaw_error_histogram.png')
    plt.close()

def main():
    if len(sys.argv) < 2:
        print("Usage: evaluate_ekf_comparison.py <bagfile>")
        sys.exit(1)
    bagfile = sys.argv[1]
    if not os.path.isfile(bagfile):
        print(f"Bag file '{bagfile}' not found.")
        sys.exit(1)

    print("Opening bag:", bagfile)
    bag = rosbag.Bag(bagfile)

    # Extract series
    df_gt = extract_path_series(bag, '/ground_truth_path')
    df_custom = extract_path_series(bag, '/ekf/path')
    df_rl = extract_path_series(bag, '/robot_localization/path')

    bag.close()

    # Align and compute errors
    tol = 0.05  # seconds tolerance for timestamp alignment
    df_err_custom = align_and_compute_errors(df_gt, df_custom, tol)
    df_err_rl     = align_and_compute_errors(df_gt, df_rl, tol)

    # Compute metrics
    metrics_custom = compute_metrics(df_err_custom)
    metrics_rl     = compute_metrics(df_err_rl)

    # Print summary
    print("\n===== Evaluation Summary =====")
    print("Custom EKF vs Ground Truth:")
    for k,v in metrics_custom.items():
        print(f"  {k}: {v:.4f}")
    print("robot_localization EKF vs Ground Truth:")
    for k,v in metrics_rl.items():
        print(f"  {k}: {v:.4f}")

    # Save metrics table to CSV
    df_metrics = pd.DataFrame({
        'method': ['custom_ekf', 'robot_localization'],
        'pos_rmse': [metrics_custom['pos_rmse'], metrics_rl['pos_rmse']],
        'pos_mean': [metrics_custom['pos_mean'], metrics_rl['pos_mean']],
        'pos_max': [metrics_custom['pos_max'], metrics_rl['pos_max']],
        'yaw_rmse': [metrics_custom['yaw_rmse'], metrics_rl['yaw_rmse']],
        'yaw_mean': [metrics_custom['yaw_mean'], metrics_rl['yaw_mean']],
        'yaw_max': [metrics_custom['yaw_max'], metrics_rl['yaw_max']],
    })
    df_metrics.to_csv('ekf_comparison_metrics.csv', index=False)
    print("Saved metrics to ekf_comparison_metrics.csv")

    # Plot trajectories
    plot_trajectories(df_gt, df_custom, df_rl)
    print("Saved trajectories.png")
    # Plot error over time
    plot_error_over_time(df_err_custom, df_err_rl)
    print("Saved pos_error_over_time.png, yaw_error_over_time.png")
    # Plot histograms
    plot_error_histogram(df_err_custom, df_err_rl)
    print("Saved pos_error_histogram.png, yaw_error_histogram.png")

    print("Evaluation complete.")

if __name__ == '__main__':
    main()
'''

# Write the script to file
script_path = '/mnt/data/evaluate_ekf_comparison.py'
with open(script_path, 'w') as f:
    f.write(script_content)

# Make executable
os.chmod(script_path, 0o755)

script_path
