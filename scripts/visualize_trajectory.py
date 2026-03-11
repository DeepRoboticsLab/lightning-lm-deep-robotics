#!/home/msy/miniconda3/bin/python
import sys
import matplotlib.pyplot as plt
import numpy as np
import os

def plot_trajectory(file_path):
    if not os.path.exists(file_path):
        print(f"Error: File '{file_path}' not found.")
        return

    try:
        # Assuming the format: timestamp x y z qx qy qz qw
        # Using a more robust way to load data manualy if numpy fails due to version mismatch
        x = []
        y = []
        with open(file_path, 'r') as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 3:
                    try:
                        x.append(float(parts[1]))
                        y.append(float(parts[2]))
                    except ValueError:
                        continue
        
        if not x:
            print(f"Error: No valid data found in '{file_path}'.")
            return

        x = np.array(x)
        y = np.array(y)

        plt.figure(figsize=(10, 8))
        plt.plot(x, y, label='Trajectory', marker='.', markersize=4, linestyle='-',\
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
    # Get current script directory to resolve relative paths if needed
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # Assuming the script is in src/lightning-lm/scripts, and data is in workspace root
    workspace_root = os.path.abspath(os.path.join(current_dir, "../../.."))

    if len(sys.argv) < 2:
        print("Usage: python3 visualize_trajectory.py <path_to_trajectory_file>")
        # Default to a file in data/ if it exists and no argument provided
        default_file = os.path.join(current_dir, "data/path_20260311_102412.txt")
        if os.path.exists(default_file):
            print(f"No file provided. Visualizing default file: {default_file}")
            plot_trajectory(default_file)
    else:
        # Resolve path relative to current working directory or as absolute path
        input_path = sys.argv[1]
        if not os.path.isabs(input_path):
            input_path = os.path.join(os.getcwd(), input_path)
        plot_trajectory(input_path)
