#!/usr/bin/env python3

import sys
import struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def read_pcd(filename):
    """Read PCD file and return point cloud as numpy array"""
    with open(filename, 'rb') as f:
        # Read header
        header = []
        while True:
            line = f.readline().decode('ascii').strip()
            header.append(line)
            if line.startswith('DATA'):
                break

        # Parse header
        points_line = [l for l in header if l.startswith('POINTS')][0]
        num_points = int(points_line.split()[1])

        fields_line = [l for l in header if l.startswith('FIELDS')][0]
        fields = fields_line.split()[1:]

        size_line = [l for l in header if l.startswith('SIZE')][0]
        sizes = [int(s) for s in size_line.split()[1:]]

        # Read binary data
        data_type = header[-1].split()[1]

        if data_type == 'binary':
            # Calculate point size
            point_size = sum(sizes)
            raw_data = f.read()

            # Parse points
            points = []
            for i in range(num_points):
                offset = i * point_size
                point_data = raw_data[offset:offset + point_size]

                # Extract x, y, z (assuming first 3 fields are x, y, z)
                x = struct.unpack('f', point_data[0:4])[0]
                y = struct.unpack('f', point_data[4:8])[0]
                z = struct.unpack('f', point_data[8:12])[0]
                points.append([x, y, z])

            return np.array(points)
        else:
            # ASCII format
            points = []
            for line in f:
                values = line.decode('ascii').strip().split()
                if len(values) >= 3:
                    x, y, z = float(values[0]), float(values[1]), float(values[2])
                    points.append([x, y, z])
            return np.array(points)

def visualize_pcd(points, downsample=1):
    """Visualize point cloud using matplotlib"""
    # Downsample for faster visualization
    if downsample > 1:
        points = points[::downsample]

    print(f"Visualizing {len(points)} points...")

    # Create 3D plot
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Color by height (z value)
    colors = points[:, 2]

    # Plot
    scatter = ax.scatter(points[:, 0], points[:, 1], points[:, 2],
                        c=colors, cmap='viridis', s=1, alpha=0.6)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'Point Cloud Visualization ({len(points)} points)')

    # Add colorbar
    plt.colorbar(scatter, ax=ax, label='Height (m)')

    # Set equal aspect ratio
    max_range = np.array([points[:, 0].max() - points[:, 0].min(),
                         points[:, 1].max() - points[:, 1].min(),
                         points[:, 2].max() - points[:, 2].min()]).max() / 2.0

    mid_x = (points[:, 0].max() + points[:, 0].min()) * 0.5
    mid_y = (points[:, 1].max() + points[:, 1].min()) * 0.5
    mid_z = (points[:, 2].max() + points[:, 2].min()) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 view_pcd.py <pcd_file> [downsample_factor]")
        print("Example: python3 view_pcd.py GlobalMap.pcd 10")
        sys.exit(1)

    pcd_file = sys.argv[1]
    downsample = int(sys.argv[2]) if len(sys.argv) > 2 else 10

    print(f"Loading PCD file: {pcd_file}")
    points = read_pcd(pcd_file)
    print(f"Loaded {len(points)} points")
    print(f"Point cloud bounds:")
    print(f"  X: [{points[:, 0].min():.2f}, {points[:, 0].max():.2f}]")
    print(f"  Y: [{points[:, 1].min():.2f}, {points[:, 1].max():.2f}]")
    print(f"  Z: [{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]")

    visualize_pcd(points, downsample)
