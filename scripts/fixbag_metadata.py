import sqlite3
import os
import sys
import yaml
from pathlib import Path

# Usage: python3 src/lightning-lm/scripts/fixbag_metadata.py /mnt/e/data/libraryf/libraryf_0.db3

def main():
    if len(sys.argv) > 1:
        path_arg = sys.argv[1]
    else:
        print("❌ 错误：请提供 .db3 文件路径或包含该文件的目录路径")
        print("用法: python3 fixbag_metadata.py /path/to/libraryf_0.db3")
        return

    # 自动识别是文件还是目录
    if os.path.isfile(path_arg) and path_arg.endswith('.db3'):
        db_file = path_arg
        bag_dir = os.path.dirname(db_file)
        db_filename = os.path.basename(db_file)
    else:
        bag_dir = path_arg
        # 尝试在目录下找唯一的 .db3 文件
        db_files = [f for f in os.listdir(bag_dir) if f.endswith('.db3')]
        if not db_files:
            print(f"❌ 错误：在目录 '{bag_dir}' 中找不到 .db3 文件")
            return
        db_filename = db_files[0]
        db_file = os.path.join(bag_dir, db_filename)

    print(f"🔍 正在读取数据库: {db_file} ...")
    
    try:
        conn = sqlite3.connect(db_file)
        cursor = conn.cursor()
        
        # 1. 获取所有话题信息 (ROS2 sqlite3 存储结构)
        cursor.execute("SELECT id, name, type, serialization_format FROM topics")
        topics_rows = cursor.fetchall()
        
        if not topics_rows:
            print("❌ 数据库中未发现任何话题！")
            return

        topics_metadata = []
        global_start = None
        global_end = None
        total_messages = 0
        
        print(f"✅ 发现 {len(topics_rows)} 个话题:")

        for topic_id, name, msg_type, serialization in topics_rows:
            # 统计每个话题
            cursor.execute("SELECT count(*), min(timestamp), max(timestamp) FROM messages WHERE topic_id = ?", (topic_id,))
            res = cursor.fetchone()
            count = res[0] if res else 0
            start_ts = res[1] if res else 0
            end_ts = res[2] if res else 0
            
            if count and count > 0:
                print(f"   - {name} ({msg_type}): {count} 条消息")
                if global_start is None or start_ts < global_start: global_start = start_ts
                if global_end is None or end_ts > global_end: global_end = end_ts
                total_messages += count
                
                # 适配 ROS2 常见的 metadata 结构
                topics_metadata.append({
                    'topic_metadata': {
                        'name': name,
                        'type': msg_type,
                        'serialization_format': serialization,
                        'offered_qos_profiles': ''
                    },
                    'message_count': count
                })

        conn.close()

        if global_start is None:
            print("❌ 未找到有效消息数据。")
            return

        duration_ns = global_end - global_start

        # 3. 构建 metadata.yaml
        # ⚠️ 注意：如果你的 .db3 文件本身已经是解压后的（比如直接被修复出来的）
        # 或者你在复制过程中已经解压了文件名，请将 compression_format 留空。
        # 如果报错 "ZSTD decompression error", 说明 rosbag 尝试去解压一个其实没压缩的文件。
        metadata = {
            'rosbag2_bagfile_information': {
                'version': 5, 
                'storage_identifier': 'sqlite3',
                'duration': {'nanoseconds': int(duration_ns)},
                'starting_time': {'nanoseconds_since_epoch': int(global_start)},
                'message_count': total_messages,
                'topics_with_message_count': topics_metadata,
                # 'compression_format': 'zstd',  # 您使用的压缩格式
                # 'compression_mode': 'file',    # 您使用的压缩模式
                'compression_format': '',  # 改为空字符串。如果文件已经是 .db3 且 sqlite 能读取，说明文件已解压
                'compression_mode': '',    # 改为空字符串
                'relative_file_paths': [db_filename],
                'files': [
                    {
                        'path': db_filename,
                        'starting_time': {'nanoseconds_since_epoch': int(global_start)},
                        'duration': {'nanoseconds': int(duration_ns)},
                        'message_count': total_messages
                    }
                ]
            }
        }

        # 4. 写入文件
        output_path = os.path.join(bag_dir, 'metadata.yaml')
        with open(output_path, 'w') as f:
            yaml.dump(metadata, f, sort_keys=False, default_flow_style=False)
        
        print(f"\n🎉 成功生成: {output_path}")
        print(f"   总消息数: {total_messages}")
        print(f"   时长: {duration_ns / 1e9:.2f} 秒")
        print("\n💡 现在你可以尝试运行: ros2 bag play <目录名>")

    except Exception as e:
        print(f"❌ 发生错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
