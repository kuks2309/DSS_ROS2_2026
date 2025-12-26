#!/usr/bin/env python3
"""
RTAB-MAP Database Viewer (Matplotlib version)

Simple tool to view the point cloud map stored in an RTAB-MAP database.
Uses matplotlib for visualization (no open3d required).

Usage:
    python3 view_rtabmap_matplotlib.py                      # Uses default ~/.ros/rtabmap.db
    python3 view_rtabmap_matplotlib.py /path/to/rtabmap.db  # Uses specified database
"""

import sqlite3
import numpy as np
import sys
import os
import zlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def get_default_db_path():
    """Get default RTAB-MAP database path"""
    return os.path.expanduser("~/.ros/rtabmap.db")


def read_point_cloud_from_bytes(data):
    """Parse point cloud data from RTAB-MAP database format (XYZI - 4 floats per point)."""
    if data is None or len(data) == 0:
        return None

    try:
        # Decompress if zlib compressed
        try:
            decompressed = zlib.decompress(data)
            data = decompressed
        except zlib.error:
            pass

        # RTAB-MAP stores as XYZI (4 floats per point)
        num_points = len(data) // 16  # 4 floats * 4 bytes
        if num_points > 0:
            points_xyzi = np.frombuffer(data[:num_points * 16], dtype=np.float32).reshape(-1, 4)
            points = points_xyzi[:, :3]  # Extract XYZ only
            valid_mask = np.all(np.isfinite(points), axis=1)
            points = points[valid_mask]
            if len(points) > 0:
                return points
    except Exception:
        pass
    return None


def get_database_info(db_path):
    """Get information about the RTAB-MAP database"""
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    info = {}

    cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
    tables = [row[0] for row in cursor.fetchall()]
    info['tables'] = tables

    if 'Node' in tables:
        cursor.execute("SELECT COUNT(*) FROM Node")
        info['node_count'] = cursor.fetchone()[0]

    if 'Link' in tables:
        cursor.execute("SELECT COUNT(*) FROM Link")
        info['link_count'] = cursor.fetchone()[0]

    if 'Data' in tables:
        cursor.execute("SELECT COUNT(*) FROM Data")
        info['data_count'] = cursor.fetchone()[0]

    conn.close()
    return info


def parse_pose_matrix(pose_data):
    """Parse pose data into 4x4 transformation matrix"""
    if pose_data is None or len(pose_data) < 48:
        return None

    pose_array = np.frombuffer(pose_data, dtype=np.float32)
    if len(pose_array) >= 12:
        # RTAB-MAP stores 3x4 matrix in row-major order
        matrix = np.eye(4)
        matrix[0, :4] = [pose_array[0], pose_array[1], pose_array[2], pose_array[3]]
        matrix[1, :4] = [pose_array[4], pose_array[5], pose_array[6], pose_array[7]]
        matrix[2, :4] = [pose_array[8], pose_array[9], pose_array[10], pose_array[11]]
        return matrix
    return None


def transform_points(points, transform_matrix):
    """Transform points using 4x4 transformation matrix"""
    if points is None or len(points) == 0 or transform_matrix is None:
        return points

    # Add homogeneous coordinate
    ones = np.ones((len(points), 1))
    points_h = np.hstack([points, ones])

    # Transform
    transformed = (transform_matrix @ points_h.T).T

    return transformed[:, :3]


def extract_all_point_clouds(db_path, max_nodes=None):
    """Extract all point clouds from the database and transform to global coordinates"""
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    all_points = []

    # Get poses for each node
    poses_dict = {}
    try:
        cursor.execute("SELECT id, pose FROM Node WHERE pose IS NOT NULL")
        for row in cursor.fetchall():
            node_id, pose_data = row
            matrix = parse_pose_matrix(pose_data)
            if matrix is not None:
                poses_dict[node_id] = matrix
    except Exception as e:
        print(f"Error reading poses: {e}")

    query = "SELECT id, scan FROM Data WHERE scan IS NOT NULL"
    if max_nodes:
        query += f" LIMIT {max_nodes}"

    try:
        cursor.execute(query)
        rows = cursor.fetchall()

        for row in rows:
            node_id, scan_data = row
            if scan_data:
                points = read_point_cloud_from_bytes(scan_data)
                if points is not None and len(points) > 0:
                    # Transform points to global coordinates using node pose
                    if node_id in poses_dict:
                        points = transform_points(points, poses_dict[node_id])
                    all_points.append(points)
    except Exception as e:
        print(f"Error reading scan data: {e}")

    conn.close()

    if all_points:
        return np.vstack(all_points)
    return None


def extract_node_poses(db_path):
    """Extract node poses from the database"""
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    poses = []

    try:
        cursor.execute("SELECT id, pose FROM Node WHERE pose IS NOT NULL ORDER BY id")
        rows = cursor.fetchall()

        for row in rows:
            node_id, pose_data = row
            if pose_data:
                try:
                    pose_array = np.frombuffer(pose_data, dtype=np.float32)
                    if len(pose_array) >= 12:
                        # 3x4 or 4x4 matrix: translation in last column
                        x, y, z = pose_array[3], pose_array[7], pose_array[11]
                        poses.append([x, y, z])
                except Exception:
                    pass
    except Exception as e:
        print(f"Error extracting poses: {e}")

    conn.close()
    return np.array(poses) if poses else None


def visualize_2d(points, poses=None, title="RTAB-MAP Map (Top View)"):
    """2D visualization (top-down view)"""
    fig, ax = plt.subplots(figsize=(12, 10))

    if points is not None and len(points) > 0:
        # Subsample if too many points
        if len(points) > 50000:
            indices = np.random.choice(len(points), 50000, replace=False)
            points_plot = points[indices]
        else:
            points_plot = points

        # Color by Z height
        colors = points_plot[:, 2]
        scatter = ax.scatter(points_plot[:, 0], points_plot[:, 1],
                           c=colors, cmap='viridis', s=0.5, alpha=0.5)
        plt.colorbar(scatter, label='Height (Z)')

    if poses is not None and len(poses) > 1:
        ax.plot(poses[:, 0], poses[:, 1], 'r-', linewidth=2, label='Trajectory')
        ax.scatter(poses[0, 0], poses[0, 1], c='green', s=100, marker='o', label='Start', zorder=5)
        ax.scatter(poses[-1, 0], poses[-1, 1], c='red', s=100, marker='s', label='End', zorder=5)
        ax.legend()

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(title)
    ax.axis('equal')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def visualize_3d(points, poses=None, title="RTAB-MAP Map (3D View)"):
    """3D visualization"""
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    if points is not None and len(points) > 0:
        # Subsample for 3D (fewer points for performance)
        if len(points) > 20000:
            indices = np.random.choice(len(points), 20000, replace=False)
            points_plot = points[indices]
        else:
            points_plot = points

        colors = points_plot[:, 2]
        scatter = ax.scatter(points_plot[:, 0], points_plot[:, 1], points_plot[:, 2],
                           c=colors, cmap='viridis', s=0.5, alpha=0.5)
        fig.colorbar(scatter, label='Height (Z)', shrink=0.5)

    if poses is not None and len(poses) > 1:
        ax.plot(poses[:, 0], poses[:, 1], poses[:, 2], 'r-', linewidth=2, label='Trajectory')
        ax.scatter(poses[0, 0], poses[0, 1], poses[0, 2], c='green', s=100, marker='o', label='Start')
        ax.scatter(poses[-1, 0], poses[-1, 1], poses[-1, 2], c='red', s=100, marker='s', label='End')
        ax.legend()

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title)

    plt.tight_layout()
    return fig


def main():
    # Get database path
    if len(sys.argv) > 1:
        db_path = sys.argv[1]
    else:
        db_path = get_default_db_path()

    db_path = os.path.expanduser(db_path)

    print(f"RTAB-MAP Database Viewer (Matplotlib)")
    print("=" * 50)
    print(f"Database: {db_path}")

    if not os.path.exists(db_path):
        print(f"\nError: Database file not found: {db_path}")
        sys.exit(1)

    # Get database info
    print(f"\nDatabase Info:")
    print("-" * 30)
    info = get_database_info(db_path)
    for key, value in info.items():
        print(f"  {key}: {value}")

    # Extract data
    print(f"\nExtracting poses...")
    poses = extract_node_poses(db_path)
    if poses is not None:
        print(f"  Found {len(poses)} poses")

    print(f"\nExtracting point clouds...")
    points = extract_all_point_clouds(db_path)
    if points is not None:
        print(f"  Found {len(points)} points")

    if points is None and poses is None:
        print("\nNo data could be extracted from the database.")
        sys.exit(1)

    # Show statistics
    print(f"\nMap Statistics:")
    print("-" * 30)
    if points is not None:
        print(f"  Total points: {len(points)}")
        print(f"  X range: [{points[:, 0].min():.2f}, {points[:, 0].max():.2f}] m")
        print(f"  Y range: [{points[:, 1].min():.2f}, {points[:, 1].max():.2f}] m")
        print(f"  Z range: [{points[:, 2].min():.2f}, {points[:, 2].max():.2f}] m")

    if poses is not None:
        print(f"\nTrajectory Statistics:")
        print("-" * 30)
        print(f"  Total poses: {len(poses)}")
        if len(poses) > 1:
            distances = np.sqrt(np.sum(np.diff(poses, axis=0)**2, axis=1))
            print(f"  Total distance: {distances.sum():.2f} m")
            print(f"  X range: [{poses[:, 0].min():.2f}, {poses[:, 0].max():.2f}] m")
            print(f"  Y range: [{poses[:, 1].min():.2f}, {poses[:, 1].max():.2f}] m")

    # Visualize
    print("\nGenerating visualizations...")
    db_name = os.path.basename(db_path)

    # 2D view
    fig_2d = visualize_2d(points, poses, f"{db_name} - Top View")

    # 3D view
    fig_3d = visualize_3d(points, poses, f"{db_name} - 3D View")

    # Save figures
    output_dir = os.path.dirname(db_path)
    base_name = db_path.replace('.db', '')

    fig_2d.savefig(f"{base_name}_2d.png", dpi=150)
    print(f"Saved: {base_name}_2d.png")

    fig_3d.savefig(f"{base_name}_3d.png", dpi=150)
    print(f"Saved: {base_name}_3d.png")

    print("\nDisplaying plots... (close windows to exit)")
    plt.show()


if __name__ == '__main__':
    main()
