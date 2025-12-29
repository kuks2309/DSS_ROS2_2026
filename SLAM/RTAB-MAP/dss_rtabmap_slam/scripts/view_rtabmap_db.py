#!/usr/bin/env python3
"""
RTAB-MAP Database Viewer

Simple tool to view the point cloud map stored in an RTAB-MAP database.

Usage:
    python3 view_rtabmap_db.py                           # Uses default ~/.ros/rtabmap.db
    python3 view_rtabmap_db.py /path/to/rtabmap.db       # Uses specified database

Requirements:
    pip install open3d numpy sqlite3
"""

import sqlite3
import numpy as np
import sys
import os
import zlib
import struct
from pathlib import Path

# Try to import open3d for visualization
try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("Warning: open3d not installed. Will save to PLY file instead of visualizing.")
    print("Install with: pip install open3d")


def get_default_db_path():
    """Get default RTAB-MAP database path"""
    return os.path.expanduser("~/.ros/rtabmap.db")


def read_point_cloud_from_bytes(data):
    """
    Parse point cloud data from RTAB-MAP database format.
    RTAB-MAP stores point clouds in a compressed binary format (XYZI - 4 floats per point).
    """
    if data is None or len(data) == 0:
        return None

    try:
        # Decompress if zlib compressed
        try:
            decompressed = zlib.decompress(data)
            data = decompressed
        except zlib.error:
            pass  # Data might not be compressed

        # RTAB-MAP stores point clouds as XYZI (4 floats per point)
        num_points = len(data) // 16  # 4 floats * 4 bytes
        if num_points > 0:
            points_xyzi = np.frombuffer(data[:num_points * 16], dtype=np.float32).reshape(-1, 4)
            # Extract XYZ only
            points = points_xyzi[:, :3]
            # Filter out invalid points
            valid_mask = np.all(np.isfinite(points), axis=1)
            points = points[valid_mask]
            if len(points) > 0:
                return points
    except Exception as e:
        pass

    return None


def get_database_info(db_path):
    """Get information about the RTAB-MAP database"""
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    info = {}

    # Get table names
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
    tables = [row[0] for row in cursor.fetchall()]
    info['tables'] = tables

    # Get node count
    if 'Node' in tables:
        cursor.execute("SELECT COUNT(*) FROM Node")
        info['node_count'] = cursor.fetchone()[0]

    # Get link count
    if 'Link' in tables:
        cursor.execute("SELECT COUNT(*) FROM Link")
        info['link_count'] = cursor.fetchone()[0]

    # Get data count
    if 'Data' in tables:
        cursor.execute("SELECT COUNT(*) FROM Data")
        info['data_count'] = cursor.fetchone()[0]

    # Get statistics
    if 'Statistics' in tables:
        cursor.execute("SELECT COUNT(*) FROM Statistics")
        info['stats_count'] = cursor.fetchone()[0]

    conn.close()
    return info


def extract_all_point_clouds(db_path):
    """Extract all point clouds from the database"""
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    all_points = []

    # Check available tables
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
    tables = [row[0] for row in cursor.fetchall()]

    print(f"Tables in database: {tables}")

    # Try to get data from Data table (contains sensor data)
    if 'Data' in tables:
        # Get column names
        cursor.execute("PRAGMA table_info(Data)")
        columns = [col[1] for col in cursor.fetchall()]
        print(f"Data table columns: {columns}")

        # Look for scan/cloud data columns
        scan_columns = [c for c in columns if 'scan' in c.lower() or 'cloud' in c.lower() or 'laser' in c.lower()]
        print(f"Scan-related columns: {scan_columns}")

        # Try to extract scan data
        if 'scan_info' in columns or 'scan' in columns:
            try:
                cursor.execute("SELECT id, scan FROM Data WHERE scan IS NOT NULL LIMIT 100")
                rows = cursor.fetchall()
                print(f"Found {len(rows)} rows with scan data")

                for row in rows:
                    node_id, scan_data = row
                    if scan_data:
                        points = read_point_cloud_from_bytes(scan_data)
                        if points is not None and len(points) > 0:
                            all_points.append(points)
                            print(f"  Node {node_id}: {len(points)} points")
            except Exception as e:
                print(f"Error reading scan data: {e}")

    # Try Node table for pose information
    if 'Node' in tables:
        cursor.execute("PRAGMA table_info(Node)")
        columns = [col[1] for col in cursor.fetchall()]
        print(f"Node table columns: {columns}")

        cursor.execute("SELECT COUNT(*) FROM Node")
        count = cursor.fetchone()[0]
        print(f"Total nodes in database: {count}")

    conn.close()

    if all_points:
        return np.vstack(all_points)
    return None


def extract_node_poses(db_path):
    """Extract node poses from the database to visualize the trajectory"""
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    poses = []

    cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
    tables = [row[0] for row in cursor.fetchall()]

    if 'Node' in tables:
        try:
            cursor.execute("PRAGMA table_info(Node)")
            columns = [col[1] for col in cursor.fetchall()]

            # Node table typically has: id, map_id, weight, stamp, label, pose
            if 'pose' in columns:
                cursor.execute("SELECT id, pose FROM Node WHERE pose IS NOT NULL ORDER BY id")
                rows = cursor.fetchall()

                for row in rows:
                    node_id, pose_data = row
                    if pose_data:
                        try:
                            # Pose is typically stored as 12 floats (3x4 transformation matrix)
                            # or 16 floats (4x4 matrix) or 7 floats (x,y,z,qx,qy,qz,qw)
                            pose_array = np.frombuffer(pose_data, dtype=np.float32)
                            if len(pose_array) >= 3:
                                # Extract position (x, y, z)
                                if len(pose_array) == 12:
                                    # 3x4 matrix: last column is translation
                                    x, y, z = pose_array[3], pose_array[7], pose_array[11]
                                elif len(pose_array) == 16:
                                    # 4x4 matrix
                                    x, y, z = pose_array[3], pose_array[7], pose_array[11]
                                elif len(pose_array) == 7:
                                    # x, y, z, qx, qy, qz, qw
                                    x, y, z = pose_array[0], pose_array[1], pose_array[2]
                                else:
                                    x, y, z = pose_array[0], pose_array[1], pose_array[2]

                                poses.append([x, y, z])
                        except Exception as e:
                            pass

                print(f"Extracted {len(poses)} node poses")
        except Exception as e:
            print(f"Error extracting poses: {e}")

    conn.close()

    if poses:
        return np.array(poses)
    return None


def visualize_with_open3d(points, poses=None):
    """Visualize point cloud using Open3D"""
    if not HAS_OPEN3D:
        print("Open3D not available for visualization")
        return

    geometries = []

    # Create point cloud
    if points is not None and len(points) > 0:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Color by height (Z axis)
        z_min, z_max = points[:, 2].min(), points[:, 2].max()
        if z_max > z_min:
            colors = np.zeros((len(points), 3))
            z_normalized = (points[:, 2] - z_min) / (z_max - z_min)
            colors[:, 0] = z_normalized  # Red channel based on height
            colors[:, 2] = 1 - z_normalized  # Blue channel inverse
            pcd.colors = o3d.utility.Vector3dVector(colors)

        geometries.append(pcd)
        print(f"Point cloud: {len(points)} points")

    # Create trajectory line from poses
    if poses is not None and len(poses) > 1:
        # Create line set for trajectory
        lines = [[i, i + 1] for i in range(len(poses) - 1)]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(poses)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in lines])  # Red trajectory
        geometries.append(line_set)
        print(f"Trajectory: {len(poses)} poses")

    # Add coordinate frame
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    geometries.append(coord_frame)

    if geometries:
        print("\nVisualization controls:")
        print("  Mouse: Rotate view")
        print("  Scroll: Zoom")
        print("  Shift+Mouse: Pan")
        print("  Q: Quit")
        o3d.visualization.draw_geometries(geometries,
                                          window_name="RTAB-MAP Database Viewer",
                                          width=1200, height=800)


def save_to_ply(points, output_path):
    """Save point cloud to PLY file"""
    if points is None or len(points) == 0:
        print("No points to save")
        return

    with open(output_path, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")

        for p in points:
            f.write(f"{p[0]} {p[1]} {p[2]}\n")

    print(f"Saved {len(points)} points to {output_path}")


def main():
    # Get database path
    if len(sys.argv) > 1:
        db_path = sys.argv[1]
    else:
        db_path = get_default_db_path()

    # Expand path
    db_path = os.path.expanduser(db_path)

    print(f"RTAB-MAP Database Viewer")
    print(f"=" * 50)
    print(f"Database: {db_path}")

    # Check if file exists
    if not os.path.exists(db_path):
        print(f"\nError: Database file not found: {db_path}")
        print("\nUsage:")
        print(f"  python3 {sys.argv[0]} [database_path]")
        print(f"\nDefault path: {get_default_db_path()}")
        sys.exit(1)

    # Get database info
    print(f"\nDatabase Info:")
    print("-" * 30)
    info = get_database_info(db_path)
    for key, value in info.items():
        print(f"  {key}: {value}")

    # Extract poses
    print(f"\nExtracting poses...")
    poses = extract_node_poses(db_path)

    # Extract point clouds
    print(f"\nExtracting point clouds...")
    points = extract_all_point_clouds(db_path)

    if points is None and poses is None:
        print("\nNo data could be extracted from the database.")
        print("The database might be empty or in an unsupported format.")
        sys.exit(1)

    # Visualize or save
    if HAS_OPEN3D:
        print(f"\nOpening visualization...")
        visualize_with_open3d(points, poses)
    else:
        # Save to file
        output_path = db_path.replace('.db', '_cloud.ply')
        if points is not None:
            save_to_ply(points, output_path)

        if poses is not None:
            poses_path = db_path.replace('.db', '_trajectory.txt')
            np.savetxt(poses_path, poses, fmt='%.6f', header='x y z')
            print(f"Saved trajectory to {poses_path}")


if __name__ == '__main__':
    main()
