import os.path

import sys
import rclpy
from orca.orca_platform import datadir
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions
import zstandard as zstd
from pathlib import Path
import shutil


def merge_bags(input_bags, output_bag):
    # 初始化写入器
    if os.path.exists(output_bag):
        shutil.rmtree(output_bag)

    writer = SequentialWriter()
    storage_options = StorageOptions(uri=output_bag, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    writer.open(storage_options, converter_options)

    # 创建话题类型缓存，避免重复添加
    known_topics = {}

    # 遍历所有输入包
    for input_bag in input_bags:
        # 初始化读取器
        reader = SequentialReader()
        print('converting ', input_bag)
        reader.open(StorageOptions(uri=input_bag, storage_id=''), ConverterOptions('', ''))

        # 如果这是第一个包，或者需要动态发现话题，可以在这里处理
        # 获取当前包的话题和类型
        topic_types = reader.get_all_topics_and_types()
        for topic_type in topic_types:
            if topic_type.name not in known_topics:
                known_topics[topic_type.name] = topic_type.type
                writer.create_topic(topic_type)

        # 读取并写入消息
        while reader.has_next():
            (topic, data, t) = reader.read_next()
            writer.write(topic, data, t)

    print("合并完成！")


def convert_ros1_bag(input_file, output_file=None):
    """
    转换ros1 bags

    Args:
        input_file: 输入的.zstd或.db3.zst文件路径
        output_file: 输出的解压文件路径，如果为None则自动生成
    """
    if output_file is None:
        if input_file.endswith('.bag'):
            output_file = input_file.rsplit('.', 1)[0]
        else:
            output_file = input_file + '.decompressed'

    try:
        # 读取压缩文件并解压
        print('rosbags-convert --src ' + input_file + ' --dst ' + output_file)
        os.system('rosbags-convert --src ' + input_file + ' --dst ' + output_file)

        print(f"转换成功: {input_file} -> {output_file}")
        return output_file

    except Exception as e:
        print(f"解压失败: {e}")
        return None


def decompress_db3_zstd(input_file, output_file=None):
    """
    解压zstd压缩的db3文件

    Args:
        input_file: 输入的.zstd或.db3.zst文件路径
        output_file: 输出的解压文件路径，如果为None则自动生成
    """
    if output_file is None:
        if input_file.endswith('.zstd') or input_file.endswith('.zst'):
            output_file = input_file.rsplit('.', 1)[0]
        else:
            output_file = input_file + '.decompressed'

    # 创建解压器
    dctx = zstd.ZstdDecompressor()

    try:
        # 读取压缩文件并解压
        with open(input_file, 'rb') as compressed_file:
            with open(output_file, 'wb') as decompressed_file:
                dctx.copy_stream(compressed_file, decompressed_file)

        print(f"解压成功: {input_file} -> {output_file}")
        return output_file

    except Exception as e:
        print(f"解压失败: {e}")
        return None


def find_ros1_bags(root_folder):
    """查找所有ROS 2数据包文件"""
    root_path = Path(root_folder)
    bag_files = []

    # 查找可能的ROS 2数据包文件
    patterns = ['*.bag']

    for pattern in patterns:
        for file_path in root_path.rglob(pattern):
            file_info = {
                'name': file_path.name,
                'path': str(file_path.absolute()),
                'size': file_path.stat().st_size,
                'type': 'rosbag'
            }
            bag_files.append(file_info)

    return bag_files


def find_ros2_bags(root_folder):
    """查找所有ROS 2数据包文件"""
    root_path = Path(root_folder)
    bag_files = []

    # 查找可能的ROS 2数据包文件
    patterns = ['*.db3.zstd']

    for pattern in patterns:
        for file_path in root_path.rglob(pattern):
            file_info = {
                'name': file_path.name,
                'path': str(file_path.absolute()),
                'size': file_path.stat().st_size,
                'type': 'rosbag'
            }
            bag_files.append(file_info)

    return bag_files


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Usage: python merge_bags.py path_to_your_dir')
        exit(1)

    if not os.path.exists(sys.argv[1]):
        print('目录不存在')
        exit(1)

    data_dir = sys.argv[1]

    bags = find_ros1_bags(data_dir)
    print('找到了', len(bags), '个数据包')

    merged_bags = []
    print('转换数据包中')
    for bag in bags:
        convert_ros1_bag(bag["path"])

        filename = os.path.splitext(os.path.basename(bag["path"]))[0]

        print(bag["path"].rsplit('.', 1)[0])
        db3_name = bag["path"].rsplit('.', 1)[0] + '/' + filename + '.db3'

        print('db3 should be ', db3_name)

        merged_bags.append(db3_name)

    sorted(merged_bags)
    merge_bags(merged_bags, data_dir + "/merged")
