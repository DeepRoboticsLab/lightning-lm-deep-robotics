#!/usr/bin/env python3

import os
import sys
import argparse
from datetime import datetime
from multiprocessing import Pool, cpu_count
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

# Usage: python3 src/lightning-lm/scripts/extract_imu_from_bag_ros2.py /mnt/e/data/libraryf/libraryf_0.db3

def process_message_batch(batch):
    results = []
    # Local import to avoid pickling issues in multiprocessing
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    
    # Cache message type
    msg_type = None
    
    for topic, data, timestamp, msg_type_name in batch:
        if msg_type is None:
            msg_type = get_message(msg_type_name)
        
        msg = deserialize_message(data, msg_type)
        
        # Extract timestamp (seconds)
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Extract angular velocities
        wx, wy, wz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        
        # Extract linear accelerations
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        
        # Extract orientation
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        
        line = f"{ts:.6f},{wx:.6f},{wy:.6f},{wz:.6f},{ax:.6f},{ay:.6f},{az:.6f},{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f}\n"
        results.append(line)
    return results

def get_rosbag_options(path, serialization_format='cdr'):
    # If a path to a .db3 file is provided instead of a folder, use it directly
    if path.endswith('.db3'):
        storage_uri = path
    else:
        storage_uri = path
        
    storage_options = rosbag2_py.StorageOptions(uri=storage_uri, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)
    return storage_options, converter_options

def extract_imu_data(bag_path):
    # Create output directory based on bag filename or folder name
    base_name = os.path.basename(os.path.normpath(bag_path))
    if base_name.endswith('.db3'):
        bag_folder_name = os.path.splitext(base_name)[0]
    else:
        bag_folder_name = base_name
        
    output_dir = os.path.join("data", bag_folder_name)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Define IMU topics and their configurations
    configs = [
        {"name": "main_imu", "topic": "/IMU", "columns": "timestamp,wx,wy,wz,ax,ay,az,qx,qy,qz,qw"},
    ]

    # Open CSV files for writing
    files = {}
    date_str = datetime.now().strftime("%Y%m%d")
    for config in configs:
        filepath = os.path.join(output_dir, f"{config['name']}_{date_str}.csv")
        f = open(filepath, 'w')
        f.write(config["columns"] + "\n")
        files[config["topic"]] = {
            "file": f,
            "name": config["name"],
            "path": filepath,
            "msg_type": "sensor_msgs/msg/Imu"
        }

    print(f"Processing bag: {bag_path}")
    print(f"Output directory: {output_dir}")

    storage_options, converter_options = get_rosbag_options(bag_path)
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Get topic types
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    # Verify if topic exists
    target_topic = "/IMU"
    if target_topic not in type_map:
        print(f"Error: Topic {target_topic} not found in bag file.")
        print(f"Available topics: {list(type_map.keys())}")
        return

    msg_type = get_message(type_map[target_topic])
    
    print(f"Streaming data from {target_topic} to CSV...")
    count = 0
    start_time = datetime.now()
    
    # Simple direct extraction for speed
    f = files[target_topic]["file"]
    
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        
        if topic == target_topic:
            msg = deserialize_message(data, msg_type)
            
            # Extract timestamp (seconds)
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # Extract angular velocities and linear accelerations
            wx, wy, wz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
            ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
            qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
            
            f.write(f"{ts:.6f},{wx:.6f},{wy:.6f},{wz:.6f},{ax:.6f},{ay:.6f},{az:.6f},{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f}\n")
            
            count += 1
            if count % 10000 == 0:
                elapsed = (datetime.now() - start_time).total_seconds()
                print(f"Processed {count} messages... ({count/elapsed:.0f} msg/s)", end='\r')

    print(f"\nExtraction completed successfully! Total: {count} messages.")
    for topic in files:
        f_info = files[topic]
        f_info["file"].close()
        print(f"Saved {f_info['name']} data to {f_info['path']}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Extract IMU data from ROS2 bag (.db3) to CSV')
    parser.add_argument('bag_path', type=str, help='Path to the ROS2 bag folder (containing .db3 and metadata.yaml)')
    args = parser.parse_args()

    if not os.path.exists(args.bag_path):
        print(f"Error: Path does not exist: {args.bag_path}")
        sys.exit(1)

    extract_imu_data(args.bag_path)
