#!/usr/bin/env python3
"""
RTAB-Map Database Visualizer
Visualizes point cloud map and trajectory from RTAB-Map .db files
"""

import argparse
import sqlite3
import struct
import sys
from pathlib import Path

import numpy as np

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False

try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """Convert quaternion to 3x3 rotation matrix"""
    r00 = 1 - 2*(qy*qy + qz*qz)
    r01 = 2*(qx*qy - qz*qw)
    r02 = 2*(qx*qz + qy*qw)
    r10 = 2*(qx*qy + qz*qw)
    r11 = 1 - 2*(qx*qx + qz*qz)
    r12 = 2*(qy*qz - qx*qw)
    r20 = 2*(qx*qz - qy*qw)
    r21 = 2*(qy*qz + qx*qw)
    r22 = 1 - 2*(qx*qx + qy*qy)
    return np.array([
        [r00, r01, r02],
        [r10, r11, r12],
        [r20, r21, r22]
    ])


def parse_transform(data):
    """Parse 12 floats (3x4 transform matrix) from binary data"""
    if data is None or len(data) < 48:
        return None
    values = struct.unpack('12f', data[:48])
    transform = np.array([
        [values[0], values[1], values[2], values[3]],
        [values[4], values[5], values[6], values[7]],
        [values[8], values[9], values[10], values[11]],
        [0, 0, 0, 1]
    ])
    return transform


def load_rtabmap_db(db_path):
    """Load poses and point clouds from RTAB-Map database"""
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # Get optimized poses from Link table (type 0 = neighbor links with poses)
    poses = {}

    # First try to get poses from Node table
    cursor.execute("SELECT id, pose FROM Node WHERE pose IS NOT NULL")
    for row in cursor.fetchall():
        node_id = row[0]
        pose_data = row[1]
        if pose_data:
            transform = parse_transform(pose_data)
            if transform is not None:
                poses[node_id] = transform

    print(f"Loaded {len(poses)} poses from database")

    # Get point clouds from Data table
    point_clouds = {}
    cursor.execute("SELECT id, scan FROM Data WHERE scan IS NOT NULL")
    for row in cursor.fetchall():
        node_id = row[0]
        scan_data = row[1]
        if scan_data and len(scan_data) > 0:
            point_clouds[node_id] = scan_data

    print(f"Loaded {len(point_clouds)} point clouds from database")

    conn.close()
    return poses, point_clouds


def decompress_scan(scan_data):
    """Decompress and parse scan data"""
    try:
        import zlib
        # Try to decompress if compressed
        try:
            decompressed = zlib.decompress(scan_data)
            data = decompressed
        except:
            data = scan_data

        # Parse header (format may vary)
        if len(data) < 12:
            return None

        # Try to parse as raw float32 x,y,z points
        num_floats = len(data) // 4
        if num_floats >= 3 and num_floats % 3 == 0:
            points = np.frombuffer(data, dtype=np.float32).reshape(-1, 3)
            # Filter out invalid points
            valid = np.isfinite(points).all(axis=1)
            valid &= np.abs(points).max(axis=1) < 1000  # Remove outliers
            return points[valid]

        return None
    except Exception as e:
        return None


def visualize_with_open3d(poses, point_clouds, voxel_size=0.1):
    """Visualize using Open3D"""
    print("Visualizing with Open3D...")

    # Create combined point cloud
    all_points = []
    all_colors = []

    # Generate colors for each node
    node_ids = sorted(poses.keys())
    colors = plt.cm.viridis(np.linspace(0, 1, len(node_ids)))[:, :3]

    for i, node_id in enumerate(node_ids):
        if node_id in point_clouds:
            points = decompress_scan(point_clouds[node_id])
            if points is not None and len(points) > 0:
                # Transform points to world frame
                pose = poses[node_id]
                R = pose[:3, :3]
                t = pose[:3, 3]
                transformed = (R @ points.T).T + t
                all_points.append(transformed)
                all_colors.append(np.tile(colors[i], (len(transformed), 1)))

    if not all_points:
        print("No valid point clouds found!")
        return

    # Combine all points
    combined_points = np.vstack(all_points)
    combined_colors = np.vstack(all_colors)

    print(f"Total points: {len(combined_points)}")

    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(combined_points)
    pcd.colors = o3d.utility.Vector3dVector(combined_colors)

    # Downsample
    if voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size)
        print(f"After downsampling: {len(pcd.points)} points")

    # Create trajectory line
    trajectory_points = []
    for node_id in sorted(poses.keys()):
        pose = poses[node_id]
        trajectory_points.append(pose[:3, 3])

    trajectory = np.array(trajectory_points)

    # Create line set for trajectory
    lines = [[i, i+1] for i in range(len(trajectory)-1)]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(trajectory)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in lines])  # Red trajectory

    # Create coordinate frame
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0)

    # Visualize
    o3d.visualization.draw_geometries(
        [pcd, line_set, coord_frame],
        window_name="RTAB-Map Visualization",
        width=1280,
        height=720
    )


def visualize_with_matplotlib(poses, point_clouds, max_points=200000, z_min=-1.5, z_max=4.0):
    """Visualize using Matplotlib with 4-view layout (similar to rtabmap-export)"""
    print("Visualizing with Matplotlib...")

    fig = plt.figure(figsize=(16, 12))

    # Collect all points
    all_points = []
    node_ids = sorted(poses.keys())

    for node_id in node_ids:
        if node_id in point_clouds:
            points = decompress_scan(point_clouds[node_id])
            if points is not None and len(points) > 0:
                pose = poses[node_id]
                R = pose[:3, :3]
                t = pose[:3, 3]
                transformed = (R @ points.T).T + t
                all_points.append(transformed)

    # Get trajectory
    trajectory = np.array([poses[nid][:3, 3] for nid in sorted(poses.keys())])

    if all_points:
        combined = np.vstack(all_points)
        print(f"Total points before filtering: {len(combined)}")
        print(f"Z range: {combined[:, 2].min():.2f} to {combined[:, 2].max():.2f}")

        # Filter Z range to remove ground noise and ceiling/sky noise
        z_mask = (combined[:, 2] > z_min) & (combined[:, 2] < z_max)
        combined = combined[z_mask]
        print(f"Points after Z filtering ({z_min}m ~ {z_max}m): {len(combined)}")

        # Subsample if too many points
        if len(combined) > max_points:
            idx = np.random.choice(len(combined), max_points, replace=False)
            combined = combined[idx]

        # 1. 3D Point Cloud View (top-left)
        ax1 = fig.add_subplot(221, projection='3d')
        ax1.scatter(combined[:, 0], combined[:, 1], combined[:, 2],
                   s=0.1, c=combined[:, 2], cmap='viridis', alpha=0.5)
        ax1.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2],
                 'r-', linewidth=1, alpha=0.7)
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('3D Point Cloud View')

        # 2. Top-Down View X-Y (top-right)
        ax2 = fig.add_subplot(222)
        ax2.scatter(combined[:, 0], combined[:, 1], s=0.05, c='black', alpha=0.3)
        ax2.plot(trajectory[:, 0], trajectory[:, 1], 'r-', linewidth=1, alpha=0.7)
        ax2.scatter(trajectory[0, 0], trajectory[0, 1], c='green', s=50, marker='o', zorder=5)
        ax2.scatter(trajectory[-1, 0], trajectory[-1, 1], c='red', s=50, marker='x', zorder=5)
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_title(f'Top-Down View (X-Y) - {len(combined)} points')
        ax2.set_aspect('equal')
        ax2.grid(True, alpha=0.3)

        # 3. Side View X-Z (bottom-left)
        ax3 = fig.add_subplot(223)
        ax3.scatter(combined[:, 0], combined[:, 2], s=0.05, c='black', alpha=0.3)
        ax3.plot(trajectory[:, 0], trajectory[:, 2], 'r-', linewidth=1, alpha=0.7)
        ax3.set_xlabel('X (m)')
        ax3.set_ylabel('Z (m)')
        ax3.set_title('Side View (X-Z)')
        ax3.grid(True, alpha=0.3)

        # 4. Front View Y-Z (bottom-right)
        ax4 = fig.add_subplot(224)
        ax4.scatter(combined[:, 1], combined[:, 2], s=0.05, c='black', alpha=0.3)
        ax4.plot(trajectory[:, 1], trajectory[:, 2], 'r-', linewidth=1, alpha=0.7)
        ax4.set_xlabel('Y (m)')
        ax4.set_ylabel('Z (m)')
        ax4.set_title('Front View (Y-Z)')
        ax4.grid(True, alpha=0.3)

    else:
        print("No valid point clouds found!")
        # Just show trajectory
        ax1 = fig.add_subplot(221, projection='3d')
        ax1.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], 'r-', linewidth=2)
        ax1.set_title('3D View (No point cloud)')

        ax2 = fig.add_subplot(222)
        ax2.plot(trajectory[:, 0], trajectory[:, 1], 'r-', linewidth=2)
        ax2.set_title('Top-Down View (X-Y)')
        ax2.set_aspect('equal')

        ax3 = fig.add_subplot(223)
        ax3.plot(trajectory[:, 0], trajectory[:, 2], 'r-', linewidth=2)
        ax3.set_title('Side View (X-Z)')

        ax4 = fig.add_subplot(224)
        ax4.plot(trajectory[:, 1], trajectory[:, 2], 'r-', linewidth=2)
        ax4.set_title('Front View (Y-Z)')

    plt.tight_layout()
    plt.show()


def export_to_ply(poses, point_clouds, output_path, voxel_size=0.05):
    """Export combined point cloud to PLY file"""
    print(f"Exporting to {output_path}...")

    all_points = []
    node_ids = sorted(poses.keys())

    for node_id in node_ids:
        if node_id in point_clouds:
            points = decompress_scan(point_clouds[node_id])
            if points is not None and len(points) > 0:
                pose = poses[node_id]
                R = pose[:3, :3]
                t = pose[:3, 3]
                transformed = (R @ points.T).T + t
                all_points.append(transformed)

    if not all_points:
        print("No points to export!")
        return

    combined = np.vstack(all_points)
    print(f"Total points before downsampling: {len(combined)}")

    if HAS_OPEN3D and voxel_size > 0:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(combined)
        pcd = pcd.voxel_down_sample(voxel_size)
        combined = np.asarray(pcd.points)
        print(f"After downsampling: {len(combined)} points")

    # Write PLY file
    with open(output_path, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(combined)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for p in combined:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

    print(f"Exported {len(combined)} points to {output_path}")


def main():
    parser = argparse.ArgumentParser(description='Visualize RTAB-Map database')
    parser.add_argument('db_path', type=str, help='Path to RTAB-Map .db file')
    parser.add_argument('--backend', type=str, choices=['open3d', 'matplotlib', 'auto'],
                       default='auto', help='Visualization backend')
    parser.add_argument('--export', type=str, default=None,
                       help='Export point cloud to PLY file')
    parser.add_argument('--voxel-size', type=float, default=0.1,
                       help='Voxel size for downsampling (0 to disable)')

    args = parser.parse_args()

    # Check if file exists
    if not Path(args.db_path).exists():
        print(f"Error: File not found: {args.db_path}")
        sys.exit(1)

    # Load database
    print(f"Loading {args.db_path}...")
    poses, point_clouds = load_rtabmap_db(args.db_path)

    if not poses:
        print("Error: No poses found in database!")
        sys.exit(1)

    # Export if requested
    if args.export:
        export_to_ply(poses, point_clouds, args.export, args.voxel_size)

    # Visualize
    backend = args.backend
    if backend == 'auto':
        backend = 'open3d' if HAS_OPEN3D else 'matplotlib'

    if backend == 'open3d':
        if not HAS_OPEN3D:
            print("Open3D not available, falling back to Matplotlib")
            backend = 'matplotlib'
        else:
            visualize_with_open3d(poses, point_clouds, args.voxel_size)

    if backend == 'matplotlib':
        if not HAS_MATPLOTLIB:
            print("Error: Neither Open3D nor Matplotlib available!")
            sys.exit(1)
        visualize_with_matplotlib(poses, point_clouds)


if __name__ == '__main__':
    main()
