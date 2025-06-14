#!/usr/bin/env python3
"""
evaluate_ekf_comparison.py

Offline evaluation script to compare:
  - Custom EKF path:    topic /ekf/path
  - robot_localization path: topic /robot_localization/path
  - Ground truth path: topic /ground_truth_path

Usage:
  chmod +x evaluate_ekf_comparison.py
  ./evaluate_ekf_comparison.py /path/to/bagfile.bag
"""
import sys
import os
import time
import rosbag
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf
import math

def extract_path_series(bag, topic_name, print_progress=True):
    """
    Extracts the time-series of final poses from Path messages on topic_name.
    Returns a pandas DataFrame with columns: ['time', 'x', 'y', 'z', 'yaw'].
    Logs progress every so often.
    """
    times, xs, ys, zs, yaws = [], [], [], [], []
    # Attempt to get total message count for this topic (for progress)
    total_msgs = None
    try:
        info = bag.get_type_and_topic_info()
        if topic_name in info.topics:
            total_msgs = info.topics[topic_name].message_count
    except Exception:
        total_msgs = None

    count = 0
    start_t = time.time()
    if print_progress:
        if total_msgs is not None:
            print(f"[{topic_name}] Starting extraction: ~{total_msgs} messages expected.")
        else:
            print(f"[{topic_name}] Starting extraction: total message count unknown.")

    # Iterate messages
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        count += 1
        # Progress logging
        if print_progress:
            if total_msgs:
                # every ~5% or every 500 messages
                if count % max(1, total_msgs // 20) == 0:
                    print(f"[{topic_name}] processed {count}/{total_msgs} msgs...")
            else:
                if count % 500 == 0:
                    print(f"[{topic_name}] processed {count} msgs...")

        if not msg.poses:
            continue
        # Take last pose in Path
        ps: PoseStamped = msg.poses[-1]
        # Use header.stamp; if zero, skip
        ts = ps.header.stamp.to_sec()
        x = ps.pose.position.x
        y = ps.pose.position.y
        z = ps.pose.position.z
        q = ps.pose.orientation
        # extract yaw
        try:
            euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            yaw = euler[2]
        except Exception:
            yaw = 0.0
        times.append(ts); xs.append(x); ys.append(y); zs.append(z); yaws.append(yaw)
    duration = time.time() - start_t
    if print_progress:
        print(f"[{topic_name}] Extraction done: processed {count} msgs in {duration:.2f}s, extracted {len(times)} samples.")
    if not times:
        print(f"Warning: no messages found on topic {topic_name} or no valid poses.")
        return pd.DataFrame(columns=['time','x','y','z','yaw'])
    df = pd.DataFrame({'time': times, 'x': xs, 'y': ys, 'z': zs, 'yaw': yaws})
    df = df.sort_values('time').reset_index(drop=True)
    # Drop duplicate timestamps
    before = len(df)
    df = df.drop_duplicates(subset=['time'])
    after = len(df)
    if print_progress and before != after:
        print(f"[{topic_name}] Dropped {before - after} duplicate timestamps.")
    return df

def angle_diff(a, b):
    """Compute smallest signed difference between angles a and b (radians)."""
    d = a - b
    return (d + np.pi) % (2 * np.pi) - np.pi

def align_and_compute_errors(df_gt, df_est, tol=0.05, print_progress=True):
    """
    Align estimate df_est to ground truth df_gt by nearest time within tol seconds.
    Returns DataFrame with columns:
      ['time','gt_x','gt_y','gt_z','gt_yaw','est_x','est_y','est_z','est_yaw','pos_error','yaw_error'].
    Only rows where time difference <= tol are kept.
    """
    if df_gt.empty or df_est.empty:
        if print_progress:
            print("[align] One of the dataframes is empty; returning empty error dataframe.")
        return pd.DataFrame(columns=['time','gt_x','gt_y','gt_z','gt_yaw',
                                     'est_x','est_y','est_z','est_yaw','pos_error','yaw_error'])
    # Ensure sorted
    df_gt_sorted = df_gt.sort_values('time').reset_index(drop=True)
    df_est_sorted = df_est.sort_values('time').reset_index(drop=True)
    if print_progress:
        print(f"[align] Aligning {len(df_est_sorted)} estimate samples to {len(df_gt_sorted)} ground-truth samples with tol={tol}s.")
    merged = pd.merge_asof(df_gt_sorted, df_est_sorted,
                           on='time',
                           direction='nearest',
                           tolerance=tol,
                           suffixes=('_gt','_est'))
    before = len(merged)
    # Drop rows where est is NaN
    merged = merged.dropna(subset=['x_est','y_est'])
    after = len(merged)
    if print_progress:
        print(f"[align] After dropping unmatched: {after}/{before} aligned samples kept.")
    # Rename columns
    merged = merged.rename(columns={
        'x_gt':'gt_x', 'y_gt':'gt_y', 'z_gt':'gt_z', 'yaw_gt':'gt_yaw',
        'x_est':'est_x','y_est':'est_y','z_est':'est_z','yaw_est':'est_yaw'
    })
    # Compute errors
    dx = merged['gt_x'] - merged['est_x']
    dy = merged['gt_y'] - merged['est_y']
    dz = merged['gt_z'] - merged['est_z']
    merged['pos_error'] = np.sqrt(dx*dx + dy*dy + dz*dz)
    # yaw error: absolute smallest difference
    merged['yaw_error'] = np.abs(merged.apply(lambda row: angle_diff(row['gt_yaw'], row['est_yaw']), axis=1))
    # time column remains
    merged = merged[['time','gt_x','gt_y','gt_z','gt_yaw','est_x','est_y','est_z','est_yaw','pos_error','yaw_error']]
    return merged

def compute_metrics(df_err, name=""):
    """
    Compute RMSE, mean, max for pos_error and yaw_error.
    Returns dict. Logs summary if print_progress=True.
    """
    metrics = {}
    if df_err.empty:
        metrics = {k: np.nan for k in ['pos_rmse','pos_mean','pos_max','yaw_rmse','yaw_mean','yaw_max']}
        print(f"[metrics{name}] No aligned error samples; metrics are NaN.")
        return metrics
    pos_err = df_err['pos_error'].values
    yaw_err = df_err['yaw_error'].values
    metrics['pos_rmse'] = np.sqrt(np.mean(pos_err**2))
    metrics['pos_mean'] = np.mean(pos_err)
    metrics['pos_max'] = np.max(pos_err)
    metrics['yaw_rmse'] = np.sqrt(np.mean(yaw_err**2))
    metrics['yaw_mean'] = np.mean(yaw_err)
    metrics['yaw_max'] = np.max(yaw_err)
    print(f"[metrics{name}] pos_rmse={metrics['pos_rmse']:.4f}, pos_mean={metrics['pos_mean']:.4f}, pos_max={metrics['pos_max']:.4f}; "
          f"yaw_rmse={metrics['yaw_rmse']:.4f}, yaw_mean={metrics['yaw_mean']:.4f}, yaw_max={metrics['yaw_max']:.4f}")
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
    print("[plot] Saved trajectories.png")

def plot_error_over_time(df_err_custom, df_err_rl):
    """
    Plot position error vs time, yaw error vs time for both methods.
    """
    # Position error over time
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
    print("[plot] Saved pos_error_over_time.png")

    # Yaw error over time
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
    print("[plot] Saved yaw_error_over_time.png")

def plot_error_histogram_and_cdf(df_err_custom, df_err_rl):
    """
    Plot histogram and CDF of position errors and yaw errors for both methods.
    """
    # Position error histogram
    plt.figure(figsize=(8,6))
    plt.hist(df_err_custom['pos_error'], bins=30, alpha=0.6, label='Custom EKF', density=False)
    plt.hist(df_err_rl['pos_error'], bins=30, alpha=0.6, label='robot_localization EKF', density=False)
    plt.xlabel('Position Error (m)')
    plt.ylabel('Frequency')
    plt.title('Position Error Histogram')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('pos_error_histogram.png')
    plt.close()
    print("[plot] Saved pos_error_histogram.png")

    # Position error CDF
    plt.figure(figsize=(8,6))
    for df_err, label in [(df_err_custom, 'Custom EKF'), (df_err_rl, 'robot_localization EKF')]:
        errs = np.sort(df_err['pos_error'].values)
        if len(errs) == 0:
            continue
        cdf = np.arange(1, len(errs)+1) / len(errs)
        plt.plot(errs, cdf, label=label)
    plt.xlabel('Position Error (m)')
    plt.ylabel('CDF')
    plt.title('Position Error CDF')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('pos_error_cdf.png')
    plt.close()
    print("[plot] Saved pos_error_cdf.png")

    # Yaw error histogram
    plt.figure(figsize=(8,6))
    plt.hist(df_err_custom['yaw_error'], bins=30, alpha=0.6, label='Custom EKF', density=False)
    plt.hist(df_err_rl['yaw_error'], bins=30, alpha=0.6, label='robot_localization EKF', density=False)
    plt.xlabel('Yaw Error (rad)')
    plt.ylabel('Frequency')
    plt.title('Yaw Error Histogram')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('yaw_error_histogram.png')
    plt.close()
    print("[plot] Saved yaw_error_histogram.png")

    # Yaw error CDF
    plt.figure(figsize=(8,6))
    for df_err, label in [(df_err_custom, 'Custom EKF'), (df_err_rl, 'robot_localization EKF')]:
        errs = np.sort(df_err['yaw_error'].values)
        if len(errs) == 0:
            continue
        cdf = np.arange(1, len(errs)+1) / len(errs)
        plt.plot(errs, cdf, label=label)
    plt.xlabel('Yaw Error (rad)')
    plt.ylabel('CDF')
    plt.title('Yaw Error CDF')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('yaw_error_cdf.png')
    plt.close()
    print("[plot] Saved yaw_error_cdf.png")

def plot_error_on_trajectory(df_err, label, filename_prefix):
    """
    Scatter ground-truth trajectory colored by error magnitude for this method.
    df_err has columns including 'gt_x','gt_y','pos_error','yaw_error','time'.
    We plot GT positions (gt_x, gt_y), color by pos_error.
    """
    if df_err.empty:
        print(f"[plot] No data for error-on-trajectory for {label}.")
        return
    plt.figure(figsize=(8,6))
    sc = plt.scatter(df_err['gt_x'], df_err['gt_y'], c=df_err['pos_error'], cmap='viridis', s=20)
    plt.colorbar(sc, label='Position Error (m)')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title(f'GT Trajectory Colored by Pos Error ({label})')
    plt.axis('equal')
    plt.grid(True)
    plt.tight_layout()
    outname = f'{filename_prefix}_error_on_trajectory.png'
    plt.savefig(outname)
    plt.close()
    print(f"[plot] Saved {outname}")

def main():
    if len(sys.argv) < 2:
        print("Usage: evaluate_ekf_comparison.py <bagfile>")
        sys.exit(1)
    bagfile = sys.argv[1]
    if not os.path.isfile(bagfile):
        print(f"Bag file '{bagfile}' not found.")
        sys.exit(1)

    print("Opening bag:", bagfile)
    t0_all = time.time()
    try:
        bag = rosbag.Bag(bagfile)
    except Exception as e:
        print(f"Error opening bag: {e}")
        sys.exit(1)

    # Extract series with progress logging
    df_gt = extract_path_series(bag, '/ground_truth_path', print_progress=True)
    df_custom = extract_path_series(bag, '/ekf/path', print_progress=True)
    df_rl = extract_path_series(bag, '/robot_localization/path', print_progress=True)

    bag.close()
    t_extract = time.time() - t0_all
    print(f"Finished extraction in {t_extract:.2f}s. Data sizes: GT={len(df_gt)}, Custom={len(df_custom)}, RL={len(df_rl)}")

    # Align and compute errors
    tol = 0.05  # seconds tolerance for timestamp alignment
    df_err_custom = align_and_compute_errors(df_gt, df_custom, tol, print_progress=True)
    df_err_rl     = align_and_compute_errors(df_gt, df_rl, tol, print_progress=True)

    # Compute metrics
    print("\n===== Evaluation Summary =====")
    metrics_custom = compute_metrics(df_err_custom, name=" Custom")
    metrics_rl     = compute_metrics(df_err_rl, name=" RL")

    # Save metrics table to CSV
    df_metrics = pd.DataFrame({
        'method': ['custom_ekf', 'robot_localization'],
        'pos_rmse': [metrics_custom.get('pos_rmse', np.nan), metrics_rl.get('pos_rmse', np.nan)],
        'pos_mean': [metrics_custom.get('pos_mean', np.nan), metrics_rl.get('pos_mean', np.nan)],
        'pos_max': [metrics_custom.get('pos_max', np.nan), metrics_rl.get('pos_max', np.nan)],
        'yaw_rmse': [metrics_custom.get('yaw_rmse', np.nan), metrics_rl.get('yaw_rmse', np.nan)],
        'yaw_mean': [metrics_custom.get('yaw_mean', np.nan), metrics_rl.get('yaw_mean', np.nan)],
        'yaw_max': [metrics_custom.get('yaw_max', np.nan), metrics_rl.get('yaw_max', np.nan)],
    })
    metrics_csv = 'ekf_comparison_metrics.csv'
    df_metrics.to_csv(metrics_csv, index=False)
    print(f"Saved metrics to {metrics_csv}")

    # Plot trajectories overlay
    plot_trajectories(df_gt, df_custom, df_rl)

    # Plot error over time
    plot_error_over_time(df_err_custom, df_err_rl)

    # Plot histograms and CDFs
    plot_error_histogram_and_cdf(df_err_custom, df_err_rl)

    # Plot error-on-trajectory scatter
    plot_error_on_trajectory(df_err_custom, 'Custom EKF', 'custom')
    plot_error_on_trajectory(df_err_rl, 'robot_localization EKF', 'robot_localization')

    total_time = time.time() - t0_all
    print(f"Evaluation complete in {total_time:.2f}s.")

if __name__ == '__main__':
    main()
