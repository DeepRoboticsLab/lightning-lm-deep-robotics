#!/home/msy/miniconda3/bin/python
import sys
import matplotlib.pyplot as plt
import numpy as np
import os
# python3 src/lightning-lm/scripts/visualize_trajectory.py data/path_20260313_133347.txt --angle 180

def plot_trajectory(file_path, rotation_angle=0.0):
    if not os.path.exists(file_path):
        print(f"Error: File '{file_path}' not found.")
        return

    try:
        # Assuming the format: timestamp x y z qx qy qz qw
        # Using a more robust way to load data manualy if numpy fails due to version mismatch
        x_raw = []
        y_raw = []
        with open(file_path, 'r') as f:
            for line in f:
                # Handle CSV format (comma separated) or Space separated
                parts = line.replace(',', ' ').split()
                if len(parts) >= 3:
                    try:
                        x_raw.append(float(parts[1]))
                        y_raw.append(float(parts[2]))
                    except ValueError:
                        continue
        
        if not x_raw:
            print(f"Error: No valid data found in '{file_path}'.")
            return

        x_raw = np.array(x_raw)
        y_raw = np.array(y_raw)

        # 应用旋转矩阵计算
        theta = np.radians(rotation_angle)
        c, s = np.cos(theta), np.sin(theta)
        # R = [[cos, -sin], [sin, cos]]
        x = c * x_raw - s * y_raw
        y = s * x_raw + c * y_raw

        plt.figure(figsize=(10, 8))
        plt.plot(x, y, label=f'Trajectory (Rotated {rotation_angle}°)', marker='.', markersize=4, linestyle='-',\
                  linewidth=2, alpha=1)
        plt.scatter(x[0], y[0], color='green', label='Start', s=100, zorder=5)
        plt.scatter(x[-1], y[-1], color='red', label='End', s=100, zorder=5)
        
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title(f'Trajectory Visualization: {os.path.basename(file_path)}')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        
        # Show plot
        plt.show()

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Visualize trajectory from a file with optional rotation.')
    parser.add_argument('file_path', type=str, help='Path to the trajectory file')
    parser.add_argument('--angle', type=float, default=0.0, help='Rotation angle in degrees (counter-clockwise)')
    
    args = parser.parse_args()

    # Resolve path relative to current working directory or as absolute path
    input_path = args.file_path
    if not os.path.isabs(input_path):
        input_path = os.path.join(os.getcwd(), input_path)
    
    plot_trajectory(input_path, args.angle)
